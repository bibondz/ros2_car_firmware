/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
/**
 * The Wi-Fi networks this robot knows, kept on the ROBOT.
 *
 * WHY NOT IN THE HEADER
 *
 * network_secrets.h is a compile-time list, so adding a network today means an
 * edit, a rebuild and a flash. That is the wrong shape for the thing it holds:
 * networks change when the robot MOVES, which is exactly when a laptop with a
 * toolchain is least likely to be present. It also means the credentials live
 * in a build tree rather than on the machine that uses them.
 *
 * So the list lives in NVS on the board. The header becomes a FIRST-BOOT SEED
 * and nothing more: if NVS is empty the seed is copied in, and from then on NVS
 * is the only authority. A board configured in the field ignores whatever the
 * source was rebuilt with, and a fresh board still comes up able to join
 * something.
 *
 * ORDER IS PRIORITY
 *
 * Entry 0 is tried first. That matters where several known networks overlap -
 * a phone hotspot you brought deliberately should win over the building Wi-Fi
 * that happens to be in range, and the only way to express that is order.
 *
 * WHAT IS NOT STORED HERE
 *
 * Nothing is ever printed back out. A password can be SET and REPLACED but not
 * read: the API returns SSIDs and whether each has a password, never the
 * password itself. Anything else would put every credential one open web page
 * away from anyone on the network.
 */
#ifndef WIFI_STORE_H
#define WIFI_STORE_H

#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#define WIFI_STORE_MAX      6      // more than anyone carries; bounded on purpose
#define WIFI_SSID_MAX      33      // 32 + terminator, as 802.11 defines it
#define WIFI_PASS_MAX      65      // 64 + terminator

struct WifiEntry {
  char ssid[WIFI_SSID_MAX];
  char pass[WIFI_PASS_MAX];
};

class WifiStore {
 public:
  /**
   * Load from NVS, seeding once from the compiled-in list if NVS is empty.
   *
   * @param seed       the header's networks
   * @param seed_count how many
   */
  void begin(Preferences *store, const WifiEntry *seed, uint8_t seed_count) {
    store_ = store;
    count_ = 0;
    if (!load() && seed && seed_count) {
      // First boot on this board: copy the seed in, then never look at it
      // again. This is the ONLY moment the header matters.
      for (uint8_t i = 0; i < seed_count && count_ < WIFI_STORE_MAX; ++i) {
        if (!seed[i].ssid[0]) continue;
        copyEntry(entries_[count_++], seed[i].ssid, seed[i].pass);
      }
      save();
      seeded_ = true;
    }
  }

  uint8_t count() const { return count_; }
  bool seededFromHeader() const { return seeded_; }
  const char *ssid(uint8_t i) const { return i < count_ ? entries_[i].ssid : ""; }
  const char *pass(uint8_t i) const { return i < count_ ? entries_[i].pass : ""; }
  bool hasPassword(uint8_t i) const { return i < count_ && entries_[i].pass[0] != '\0'; }

  int16_t indexOf(const char *ssid) const {
    for (uint8_t i = 0; i < count_; ++i) {
      if (strncmp(entries_[i].ssid, ssid, WIFI_SSID_MAX) == 0) return (int16_t)i;
    }
    return -1;
  }

  /**
   * Add a network, or replace the password of one already known.
   *
   * Replacing rather than duplicating matters: a duplicate SSID with a stale
   * password would be tried first and fail, and the working entry further down
   * the list would look like the broken one.
   */
  bool set(const char *ssid, const char *pass) {
    if (!ssid || !ssid[0]) return false;
    const int16_t at = indexOf(ssid);
    if (at >= 0) {
      copyEntry(entries_[at], ssid, pass);
      return save();
    }
    if (count_ >= WIFI_STORE_MAX) return false;
    copyEntry(entries_[count_++], ssid, pass);
    return save();
  }

  bool remove(const char *ssid) {
    const int16_t at = indexOf(ssid);
    if (at < 0) return false;
    for (uint8_t i = (uint8_t)at; i + 1 < count_; ++i) entries_[i] = entries_[i + 1];
    --count_;
    return save();
  }

  /** Move an entry, because order is priority. */
  bool moveTo(const char *ssid, uint8_t to) {
    const int16_t from = indexOf(ssid);
    if (from < 0 || to >= count_) return false;
    WifiEntry held = entries_[from];
    if (to > (uint8_t)from) {
      for (uint8_t i = (uint8_t)from; i < to; ++i) entries_[i] = entries_[i + 1];
    } else {
      for (uint8_t i = (uint8_t)from; i > to; --i) entries_[i] = entries_[i - 1];
    }
    entries_[to] = held;
    return save();
  }

  bool clear() {
    count_ = 0;
    return save();
  }

 private:
  static void copyEntry(WifiEntry &dst, const char *ssid, const char *pass) {
    strncpy(dst.ssid, ssid ? ssid : "", WIFI_SSID_MAX - 1);
    dst.ssid[WIFI_SSID_MAX - 1] = '\0';
    strncpy(dst.pass, pass ? pass : "", WIFI_PASS_MAX - 1);
    dst.pass[WIFI_PASS_MAX - 1] = '\0';
  }

  /**
   * Stored as one blob rather than a key per field.
   *
   * NVS keys are capped at 15 characters and a per-field scheme needs names
   * like "wifi_3_pass" plus a count that can disagree with what is actually
   * there. One blob cannot be half-written into an inconsistent list.
   */
  bool save() {
    if (!store_) return true;
    store_->putUChar("n", count_);
    return store_->putBytes("list", entries_, sizeof(WifiEntry) * count_)
           == sizeof(WifiEntry) * count_ || count_ == 0;
  }

  bool load() {
    if (!store_) return false;
    if (!store_->isKey("n")) return false;
    const uint8_t n = store_->getUChar("n", 0);
    if (n == 0 || n > WIFI_STORE_MAX) return false;
    const size_t want = sizeof(WifiEntry) * n;
    if (store_->getBytesLength("list") != want) return false;
    store_->getBytes("list", entries_, want);
    // Terminate defensively: a blob from an older layout could otherwise leave
    // an unterminated string, and everything downstream treats these as C
    // strings.
    for (uint8_t i = 0; i < n; ++i) {
      entries_[i].ssid[WIFI_SSID_MAX - 1] = '\0';
      entries_[i].pass[WIFI_PASS_MAX - 1] = '\0';
    }
    count_ = n;
    return true;
  }

  Preferences *store_ = nullptr;
  WifiEntry    entries_[WIFI_STORE_MAX];
  uint8_t      count_ = 0;
  bool         seeded_ = false;
};

#endif  // WIFI_STORE_H
