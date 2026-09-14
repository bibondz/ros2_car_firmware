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
 * Every tunable the robot has, declared once, in a table.
 *
 * WHY
 *
 * The numbers that need changing on a real robot are exactly the ones that
 * cannot be known in advance: the diode forward drop, the winding resistance,
 * the wheel diameter after the tyres wore, where the GPS antenna ended up. Each
 * of them currently lives in a #define, which means measuring one costs a code
 * edit, a rebuild and a flash - so in practice nobody measures them, and the
 * robot runs on datasheet guesses.
 *
 * The diode drop is the case in point. It shipped at 0.45 V from a datasheet;
 * measured, it is 1.81 V and 1.90 V. At 12 V that is 15% of the rail missing
 * from the back-EMF model, and the derived wheel speed reads high whenever GPS
 * is unavailable and the model is the fallback.
 *
 * WHAT A DECLARATION BUYS
 *
 * One line per parameter gives the whole system what it needs:
 *
 *   the web UI     builds its settings page by ASKING the board what exists,
 *                  so adding a tunable later needs no UI work at all
 *   the board      enforces min/max itself, so a bad value over any transport
 *                  is refused rather than accepted and acted upon
 *   safety         marks which values may change while the robot is moving,
 *                  and refuses the rest with a reason
 *
 * The #define becomes the DEFAULT. NVS holds only what was actually changed, so
 * a fresh board behaves exactly as the headers say, and a configured one
 * ignores whatever the headers were rebuilt with.
 *
 * STORAGE
 *
 * Values live in NVS under a short key. NVS keys are limited to 15 characters,
 * which is shorter than readable names like "motor.a.diode_vf" - so the stored
 * key is an index-free hash of the name, and the readable name stays in this
 * table where a person reads it.
 */
#ifndef PARAM_REGISTRY_H
#define PARAM_REGISTRY_H

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

/** When a parameter may be changed. */
enum ParamWhen : uint8_t {
  PARAM_LIVE = 0,      // safe to change at any time, including while driving
  PARAM_STOPPED = 1,   // refused while the robot is moving
};

struct ParamDef {
  const char *key;     // "motor.a.diode_vf" - what a person reads and the API uses
  const char *unit;    // "V", "m", "ohm", "" - shown beside the value
  const char *group;   // "motor", "geometry" - how the settings page is grouped
  float       def;     // the compiled-in default, from the config headers
  float       min;
  float       max;
  ParamWhen   when;
  const char *help;    // one line: what it does, and how to measure it
};

/**
 * Parameters are stored under a hash of their name, because NVS keys are capped
 * at 15 characters and readable names are longer. FNV-1a, printed as hex - the
 * point is only that distinct names give distinct keys, not cryptography.
 */
inline void paramStorageKey(const char *name, char out[16]) {
  uint32_t h = 2166136261u;
  for (const char *p = name; *p; ++p) {
    h ^= (uint8_t)(*p);
    h *= 16777619u;
  }
  snprintf(out, 16, "p%08lx", (unsigned long)h);
}

/** Why a set was refused, so the caller can say something useful. */
enum ParamResult : uint8_t {
  PARAM_OK = 0,
  PARAM_UNKNOWN,        // no such key
  PARAM_OUT_OF_RANGE,
  PARAM_NEEDS_STOP,     // marked STOPPED and the robot is moving
  PARAM_NOT_FINITE,     // NaN or infinity - would poison every later use
  PARAM_STORE_FAILED,
};

inline const char *paramResultText(ParamResult r) {
  switch (r) {
    case PARAM_OK:            return "ok";
    case PARAM_UNKNOWN:       return "no parameter by that name";
    case PARAM_OUT_OF_RANGE:  return "outside the allowed range";
    case PARAM_NEEDS_STOP:    return "the robot must be stopped to change this";
    case PARAM_NOT_FINITE:    return "not a finite number";
    case PARAM_STORE_FAILED:  return "could not be saved";
  }
  return "unknown";
}

/**
 * The registry: the table, plus the NVS-backed overrides on top of it.
 *
 * Deliberately holds no dynamic memory. The table is static and the values are
 * a fixed array, so nothing here can fragment the heap on a board that runs for
 * hours - which is the same rule the rest of the firmware follows.
 */
template <uint8_t N>
class ParamRegistry {
 public:
  void begin(const ParamDef *table, uint8_t count, Preferences *store) {
    table_ = table;
    count_ = count < N ? count : N;
    store_ = store;
    for (uint8_t i = 0; i < count_; ++i) {
      value_[i] = table_[i].def;
      overridden_[i] = false;
    }
    load();
  }

  uint8_t count() const { return count_; }
  const ParamDef &def(uint8_t i) const { return table_[i]; }
  float value(uint8_t i) const { return value_[i]; }
  bool overridden(uint8_t i) const { return overridden_[i]; }

  int16_t indexOf(const char *key) const {
    for (uint8_t i = 0; i < count_; ++i) {
      if (strcmp(table_[i].key, key) == 0) return (int16_t)i;
    }
    return -1;
  }

  float get(const char *key, float fallback = 0.0f) const {
    const int16_t i = indexOf(key);
    return i < 0 ? fallback : value_[i];
  }

  /**
   * Change a parameter, or say precisely why not.
   *
   * @param moving  whether the robot is currently in motion. A STOPPED-only
   *                parameter is refused while it is, because changing the
   *                wheel diameter or a gain mid-drive changes what every
   *                control loop believes about the machine under it.
   */
  ParamResult set(const char *key, float v, bool moving) {
    const int16_t i = indexOf(key);
    if (i < 0) return PARAM_UNKNOWN;
    return setIndex((uint8_t)i, v, moving);
  }

  ParamResult setIndex(uint8_t i, float v, bool moving) {
    if (i >= count_) return PARAM_UNKNOWN;
    // NaN first: it compares false against every bound, so a range check alone
    // would let it through and it would then poison everything downstream.
    if (!isfinite(v)) return PARAM_NOT_FINITE;
    if (v < table_[i].min || v > table_[i].max) return PARAM_OUT_OF_RANGE;
    if (table_[i].when == PARAM_STOPPED && moving) return PARAM_NEEDS_STOP;

    value_[i] = v;
    overridden_[i] = true;
    if (!store()) return PARAM_STORE_FAILED;
    return PARAM_OK;
  }

  /** Back to the compiled-in default, and forget the override. */
  ParamResult reset(const char *key, bool moving) {
    const int16_t i = indexOf(key);
    if (i < 0) return PARAM_UNKNOWN;
    if (table_[i].when == PARAM_STOPPED && moving) return PARAM_NEEDS_STOP;
    value_[i] = table_[i].def;
    overridden_[i] = false;
    if (!store()) return PARAM_STORE_FAILED;
    return PARAM_OK;
  }

  void resetAll(bool moving) {
    for (uint8_t i = 0; i < count_; ++i) {
      if (table_[i].when == PARAM_STOPPED && moving) continue;
      value_[i] = table_[i].def;
      overridden_[i] = false;
    }
    store();
  }

 private:
  /**
   * Only OVERRIDES are stored, never the defaults.
   *
   * That is what lets a header change reach a board that was configured months
   * ago: anything the user never touched follows the firmware, and only the
   * values they deliberately set stay put. Storing everything would freeze a
   * board at whatever the defaults were on the day it was first configured.
   */
  bool store() {
    if (!store_) return true;                // no NVS in the host tests
    char key[16];
    for (uint8_t i = 0; i < count_; ++i) {
      paramStorageKey(table_[i].key, key);
      if (overridden_[i]) {
        store_->putFloat(key, value_[i]);
      } else if (store_->isKey(key)) {
        // Only erase a key that is actually there. Preferences logs an ERROR
        // line for every erase of a missing key, and store() walks the whole
        // table on every set - so one `set` printed an error per
        // non-overridden parameter and buried the reply in noise.
        store_->remove(key);
      }
    }
    return true;
  }

  void load() {
    if (!store_) return;
    char key[16];
    for (uint8_t i = 0; i < count_; ++i) {
      paramStorageKey(table_[i].key, key);
      if (!store_->isKey(key)) continue;
      const float v = store_->getFloat(key, table_[i].def);
      // A stored value that is out of range is NOT trusted. Bounds can tighten
      // between firmware versions, and a saved number from an older one must
      // not be able to reintroduce a value this build considers invalid.
      if (isfinite(v) && v >= table_[i].min && v <= table_[i].max) {
        value_[i] = v;
        overridden_[i] = true;
      }
    }
  }

  const ParamDef *table_ = nullptr;
  Preferences    *store_ = nullptr;
  uint8_t         count_ = 0;
  float           value_[N] = {0};
  bool            overridden_[N] = {false};
};

#endif  // PARAM_REGISTRY_H
