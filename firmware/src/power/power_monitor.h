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
 * @file power_monitor.h
 * @brief The four INA226 sensors, and what each one actually measures.
 *
 * This class exists because the mapping from sensor to meaning is not obvious
 * and getting it wrong is expensive. An INA226 reads bus voltage and shunt
 * current at points that need not be the same node, and on this board two of
 * them are not:
 *
 *   PROCESSOR  voltage = the ESP32 +5 V rail, AFTER the buck
 *              current = from the battery side, so it is the whole module's
 *                        draw INCLUDING that buck
 *              -> two different nodes. Their product is not a power figure.
 *              -> this is NOT the pack voltage. It reads about 5 V.
 *
 *   FAN        voltage = the fan +5 V rail, after its own buck
 *              current = from source, including that buck
 *
 *   MOTOR A/B  voltage = VCC EMER, the pack behind the fuse, mushroom switch
 *                        and relay - so this IS the pack voltage
 *              current = source -> driver -> motor, with the series diode
 *                        AFTER the sensor
 *
 * PACK VOLTAGE therefore comes from the motor rails. When the mushroom switch
 * or the relay is open those rails read near zero, which means "cannot
 * measure", not "flat battery". packValid() says which, and the safety manager
 * refuses to latch on an unmeasurable pack.
 *
 * Reading the +5 V rail as if it were the pack is what latched the robot into
 * e-stop three seconds after every boot: 5 V is below any sane 3S cutoff, and
 * back then the latch could only be released by a voltage the rail could never
 * reach. The cutoff is cleared by a power cycle now, so an unmeasurable pack
 * must never be allowed to latch it in the first place - which is exactly what
 * packValid() is for.
 */
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <Arduino.h>
#include <INA226.h>
#include <config.h>

class PowerMonitor {
public:
  /** Bring up all four sensors. Returns false if any did not answer; the
   *  robot still runs, degraded, and missing sensors are reported per rail. */
  bool begin(TwoWire *bus) {
    bool all = true;
    all &= processor_.begin(bus, INA226_ADDR_PROCESSOR,
                            INA226_SHUNT_PROCESSOR_OHM, INA226_MAXA_PROCESSOR);
    all &= motor_a_.begin(bus, INA226_ADDR_MOTOR_A,
                          INA226_SHUNT_MOTOR_OHM, INA226_MAXA_MOTOR);
    all &= motor_b_.begin(bus, INA226_ADDR_MOTOR_B,
                          INA226_SHUNT_MOTOR_OHM, INA226_MAXA_MOTOR);
    all &= fan_.begin(bus, INA226_ADDR_FAN,
                      INA226_SHUNT_FAN_OHM, INA226_MAXA_FAN);
    return all;
  }

  /** Sample every sensor and recompute the derived pack voltage. */
  void update() {
    if (processor_.read()) {
      esp32_rail_v_ = processor_.voltage();
      module_a_     = processor_.current();
      rail_valid_   = true;
    } else {
      rail_valid_ = false;
    }

    motor_a_ok_ = motor_a_.read();
    if (motor_a_ok_) { motor_a_v_ = motor_a_.voltage(); motor_a_a_ = motor_a_.current(); }
    motor_b_ok_ = motor_b_.read();
    if (motor_b_ok_) { motor_b_v_ = motor_b_.voltage(); motor_b_a_ = motor_b_.current(); }
    if (fan_.read()) { fan_v_ = fan_.voltage(); fan_a_ = fan_.current(); }

    // Pack voltage from whichever motor rail is alive AND powered. Both rails
    // are the same node, so average when both are healthy and one noisy shunt
    // cannot swing the reading.
    pack_valid_ = false;
    if (motor_a_ok_ && motor_a_v_ >= PACK_RAIL_MIN_VALID_V) {
      pack_v_ = motor_a_v_;
      pack_valid_ = true;
    }
    if (motor_b_ok_ && motor_b_v_ >= PACK_RAIL_MIN_VALID_V) {
      pack_v_ = pack_valid_ ? (0.5f * (pack_v_ + motor_b_v_)) : motor_b_v_;
      pack_valid_ = true;
    }
  }

  // --- pack, the only thing the battery protection may judge ---
  float packVoltage() const { return pack_v_; }
  bool  packValid()   const { return pack_valid_; }

  // --- module supply ---
  float esp32RailVoltage() const { return esp32_rail_v_; }
  float moduleCurrent()    const { return module_a_; }
  bool  railValid()        const { return rail_valid_; }
  /** The ESP32's own supply is out of its sanity window - 3V3 since the board
   *  was moved off the 5 V rail onto its own buck. Worth showing on the web UI;
   *  never a reason to stop the robot on its own. */
  bool  railOutOfRange() const {
    return rail_valid_ &&
           (esp32_rail_v_ < ESP32_RAIL_MIN_V || esp32_rail_v_ > ESP32_RAIL_MAX_V);
  }

  // --- motors, pre-diode; the back-EMF model subtracts the drop itself ---
  float motorAVoltage() const { return motor_a_v_; }
  float motorACurrent() const { return motor_a_a_; }
  bool  motorAOk()      const { return motor_a_ok_; }
  float motorBVoltage() const { return motor_b_v_; }
  float motorBCurrent() const { return motor_b_a_; }
  bool  motorBOk()      const { return motor_b_ok_; }

  // --- fan rail ---
  float fanVoltage() const { return fan_v_; }
  float fanCurrent() const { return fan_a_; }

private:
  INA226 processor_, motor_a_, motor_b_, fan_;

  float pack_v_ = 0.0f;
  bool  pack_valid_ = false;

  float esp32_rail_v_ = 0.0f, module_a_ = 0.0f;
  bool  rail_valid_ = false;

  float motor_a_v_ = 0.0f, motor_a_a_ = 0.0f;
  float motor_b_v_ = 0.0f, motor_b_a_ = 0.0f;
  bool  motor_a_ok_ = false, motor_b_ok_ = false;

  float fan_v_ = 0.0f, fan_a_ = 0.0f;
};

#endif  // POWER_MONITOR_H
