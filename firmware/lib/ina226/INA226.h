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
 * @file INA226.h
 * @brief Minimal INA226 voltage/current monitor driver (header only, no allocation).
 *
 * Current is derived straight from the shunt voltage register
 * (I = Vshunt / Rshunt) so a wrong calibration register can never scale the
 * reading. The calibration register is still programmed so the on-chip power
 * register and the ALERT comparator behave sensibly.
 *
 * Bus voltage LSB = 1.25 mV, shunt voltage LSB = 2.5 uV (datasheet).
 */
#ifndef INA226_H
#define INA226_H

#include <Arduino.h>
#include <Wire.h>

class INA226 {
public:
  static const uint8_t REG_CONFIG   = 0x00;
  static const uint8_t REG_SHUNT_V  = 0x01;
  static const uint8_t REG_BUS_V    = 0x02;
  static const uint8_t REG_CALIB    = 0x05;
  static const uint8_t REG_MASK     = 0x06;
  static const uint8_t REG_MFG_ID   = 0xFE;
  static const uint8_t REG_DIE_ID   = 0xFF;

  INA226() {}

  /**
   * @param addr       I2C address (0x40..0x4F depending on the A0/A1 jumpers)
   * @param shunt_ohm  shunt resistor fitted on the module
   * @param max_amp    largest current you expect to measure (sets the current LSB)
   */
  bool begin(TwoWire *wire, uint8_t addr, float shunt_ohm, float max_amp) {
    wire_      = wire;
    addr_      = addr;
    shunt_ohm_ = (shunt_ohm > 1e-6f) ? shunt_ohm : 0.1f;

    uint16_t mfg = 0;
    if (!read16(REG_MFG_ID, mfg) || mfg != 0x5449) {
      present_ = false;
      return false;
    }

    // AVG = 16 samples, Vbus CT = 1.1 ms, Vshunt CT = 1.1 ms, continuous mode
    if (!write16(REG_CONFIG, 0x0527)) { present_ = false; return false; }

    float current_lsb = max_amp / 32768.0f;
    if (current_lsb < 1e-9f) current_lsb = 1e-9f;
    uint32_t cal = (uint32_t)(0.00512f / (current_lsb * shunt_ohm_));
    if (cal > 0xFFFF) cal = 0xFFFF;
    write16(REG_CALIB, (uint16_t)cal);
    calib_ = (uint16_t)cal;          // kept so a recovered device can be restored

    present_ = true;
    errors_  = 0;
    return true;
  }

  /** Refresh bus voltage + current. Returns false if the device stopped answering. */
  bool read() {
    if (!present_) {
      // A sensor that stopped answering must be allowed to come back.
      //
      // present_ latched false after six consecutive errors and nothing ever
      // cleared it: begin() is called once, in setup(). So a single I2C
      // disturbance on a bus shared with the IMU, the compass and three other
      // INAs removed the pack measurement for the rest of the boot - and with
      // it the battery cutoff, because the safety manager treats an
      // unmeasurable pack as "not a flat battery" and declines to latch. The
      // robot then drives until the cells are flat with no protection at all.
      //
      // Retrying is cheap: one register read every INA226_RETRY_MS while the
      // device is absent, which costs nothing on a healthy bus and is the only
      // way protection returns without a power cycle.
      const uint32_t now = millis();
      if ((uint32_t)(now - last_retry_ms_) < INA226_RETRY_MS) return false;
      last_retry_ms_ = now;
      uint16_t probe = 0;
      if (!read16(REG_BUS_V, probe)) return false;
      if (calib_ != 0) write16(REG_CALIB, calib_);   // the device was likely reset
      present_ = true;
      errors_  = 0;
      recoveries_++;
    }

    uint16_t raw_bus = 0, raw_shunt = 0;
    if (!read16(REG_BUS_V, raw_bus))   { fail(); return false; }
    if (!read16(REG_SHUNT_V, raw_shunt)) { fail(); return false; }

    voltage_ = (float)raw_bus * 0.00125f;                       // 1.25 mV/LSB
    current_ = ((float)(int16_t)raw_shunt * 2.5e-6f) / shunt_ohm_;  // 2.5 uV/LSB
    errors_  = 0;
    return true;
  }

  bool  present() const { return present_; }
  /** How many times this device came back after being declared absent. */
  uint16_t recoveries() const { return recoveries_; }
  float voltage() const { return voltage_; }   // [V]
  float current() const { return current_; }   // [A], positive = flowing IN+ -> IN-
  float power()   const { return voltage_ * current_; }  // [W]

private:
  void fail() {
    if (++errors_ > 5) {
      present_ = false;
      voltage_ = 0.0f;
      current_ = 0.0f;
      // Retry immediately on the next read rather than waiting a full period:
      // the common case is one disturbed transaction, not a dead device.
      last_retry_ms_ = millis() - INA226_RETRY_MS;
    }
  }

  // Long enough that a genuinely absent device costs one transaction every two
  // seconds, short enough that protection returns while the pack still matters.
  static const uint32_t INA226_RETRY_MS = 2000;

  bool write16(uint8_t reg, uint16_t value) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write((uint8_t)(value >> 8));
    wire_->write((uint8_t)(value & 0xFF));
    return wire_->endTransmission() == 0;
  }

  bool read16(uint8_t reg, uint16_t &out) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((int)addr_, 2) != 2) return false;
    uint8_t hi = wire_->read();
    uint8_t lo = wire_->read();
    out = ((uint16_t)hi << 8) | lo;
    return true;
  }

  TwoWire *wire_   = nullptr;
  uint8_t  addr_   = 0x40;
  float    shunt_ohm_ = 0.1f;
  bool     present_ = false;
  uint16_t calib_ = 0;            // last calibration written, for recovery
  uint32_t last_retry_ms_ = 0;    // when the absent device was last probed
  uint16_t recoveries_ = 0;
  uint8_t  errors_  = 0;
  float    voltage_ = 0.0f;
  float    current_ = 0.0f;
};

#endif // INA226_H
