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
 * Just enough Preferences for the host tests to compile.
 *
 * The registry takes a Preferences* and accepts nullptr, which is what the
 * tests pass - persistence is an ESP32 concern and there is no NVS here. This
 * exists so the header compiles, not so it behaves: a test that needs real
 * storage semantics would be testing NVS rather than our own logic.
 */
#ifndef MOCK_PREFERENCES_H
#define MOCK_PREFERENCES_H

#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <string>

class Preferences {
 public:
  bool begin(const char *, bool = false) { return true; }
  void end() {}
  bool isKey(const char *key) {
    return store_.count(key) + blobs_.count(key) + bytes_.count(key) > 0;
  }
  float getFloat(const char *key, float fallback = 0.0f) {
    auto it = store_.find(key);
    return it == store_.end() ? fallback : it->second;
  }
  size_t putFloat(const char *key, float value) {
    store_[key] = value;
    return sizeof(float);
  }
  bool remove(const char *key) {
    return store_.erase(key) + blobs_.erase(key) + bytes_.erase(key) > 0;
  }
  uint8_t getUChar(const char *key, uint8_t fallback = 0) {
    auto it = bytes_.find(key);
    return it == bytes_.end() ? fallback : it->second;
  }
  size_t putUChar(const char *key, uint8_t value) { bytes_[key] = value; return 1; }
  size_t getBytesLength(const char *key) {
    auto it = blobs_.find(key);
    return it == blobs_.end() ? 0 : it->second.size();
  }
  size_t putBytes(const char *key, const void *value, size_t len) {
    const unsigned char *p = static_cast<const unsigned char *>(value);
    blobs_[key] = std::vector<unsigned char>(p, p + len);
    return len;
  }
  size_t getBytes(const char *key, void *out, size_t len) {
    auto it = blobs_.find(key);
    if (it == blobs_.end()) return 0;
    size_t n = it->second.size() < len ? it->second.size() : len;
    memcpy(out, it->second.data(), n);
    return n;
  }
  bool clear() { store_.clear(); blobs_.clear(); bytes_.clear(); return true; }

 private:
  std::map<std::string, float> store_;
  std::map<std::string, std::vector<unsigned char>> blobs_;
  std::map<std::string, uint8_t> bytes_;
};

#endif  // MOCK_PREFERENCES_H
