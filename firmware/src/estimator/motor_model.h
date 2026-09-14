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
 * @file motor_model.h
 * @brief Wheel speed from motor voltage and current (back-EMF estimate).
 *
 * Each motor has its own TB6612 board and its own INA226, so for each wheel we
 * know the rail voltage and the current going into it. A brushed DC motor is:
 *
 *      V_applied = I * R + Ke * omega                 (steady state)
 *
 * so, rearranged, the shaft speed is
 *
 *      omega = ((V_bus - V_diode) * duty  -  I * R_total) / Ke
 *
 *   V_bus    measured by the INA226 on that motor rail
 *   V_diode  drop of the series diode that sits BETWEEN the sensor and the
 *            driver (schematic draft_5): VF + I * slope resistance. The sensor
 *            reads the rail before it, the motor only ever sees what is left.
 *   duty     PWM duty we are commanding, signed
 *   R_total  motor winding + TB6612 on-resistance + wiring
 *   Ke       volts per output-shaft RPM (gearbox included):
 *            12 V / 100 rpm = 0.12 for the JGA25-370-100RPM
 *
 * WHAT IT IS GOOD FOR
 *   - a per-wheel speed that does not need an encoder and does not need GPS,
 *     available the instant the wheels turn (used as the fallback reference for
 *     the speed estimator instead of a naive "duty x max speed")
 *   - it is load aware: driving up a slope draws more current, the estimate
 *     drops accordingly, exactly like a real encoder would show
 *   - STALL detection: full duty, high current, no back-EMF = the wheel is
 *     blocked. Nothing else on this robot can see that.
 *   - OPEN detection: duty commanded but almost no current = broken wire,
 *     dead driver channel or a motor that fell off
 *
 * WHAT IT IS NOT
 *   Not an encoder. Expect 10-20 % error: R and Ke drift with temperature, the
 *   INA226 averages over ~18 ms, and at low duty the current is discontinuous.
 *   Below MIN_DUTY the estimate is not published at all.
 */
#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <Arduino.h>
#include <math.h>

class MotorModel {
public:
  struct Config {
    float ke_v_per_rpm   = 0.12f;   // 12 V / 100 rpm output shaft
    float resistance_ohm = 5.0f;    // winding + driver + wiring
    float diode_vf_v     = 0.45f;   // series diode between the INA226 and the driver
    float diode_r_ohm    = 0.03f;   // its slope resistance (0 = flat drop)
    float wheel_diameter_m = 0.081f;
    float min_duty       = 0.12f;   // below this the estimate is meaningless
    float stall_current_a = 0.9f;   // "a lot of current" for this gearmotor
    float open_current_a = 0.04f;   // "no current at all"
    float filter_tau_s   = 0.20f;   // smooths the INA226 sampling noise
  };

  void begin(const Config &cfg) { cfg_ = cfg; reset(); }
  Config &config() { return cfg_; }

  void reset() {
    rpm_ = 0.0f;
    valid_ = false;
    stalled_ = false;
    open_circuit_ = false;
    stall_since_ = 0;
  }

  /**
   * @param dt      loop period [s]
   * @param bus_v   rail voltage from the INA226 on this motor [V]
   * @param amps    current into this motor [A] (sign ignored, magnitude used)
   * @param duty    commanded PWM duty, -1..+1
   * @param sensor_ok the INA226 answered this cycle
   */
  void update(float dt, float bus_v, float amps, float duty, bool sensor_ok) {
    if (!sensor_ok || dt <= 0.0f) { valid_ = false; return; }

    const float mag_duty = fabsf(duty);
    const float current = fabsf(amps);

    if (mag_duty < cfg_.min_duty) {
      // coasting or crawling: no usable back-EMF reading
      valid_ = false;
      stalled_ = false;
      open_circuit_ = false;
      stall_since_ = 0;
      open_since_ = 0;
      rpm_ += (0.0f - rpm_) * (dt / (cfg_.filter_tau_s + dt));
      return;
    }

    // what is left after the series diode (draft_5: diode sits after the sensor)
    diode_drop_ = cfg_.diode_vf_v + current * cfg_.diode_r_ohm;
    float v_driver = bus_v - diode_drop_;
    if (v_driver < 0.0f) v_driver = 0.0f;

    const float v_applied = v_driver * mag_duty;
    const float v_backemf = v_applied - current * cfg_.resistance_ohm;
    float rpm = (cfg_.ke_v_per_rpm > 1e-6f) ? (v_backemf / cfg_.ke_v_per_rpm) : 0.0f;
    if (rpm < 0.0f) rpm = 0.0f;                 // back-EMF cannot oppose the drive
    if (duty < 0.0f) rpm = -rpm;

    rpm_ += (rpm - rpm_) * (dt / (cfg_.filter_tau_s + dt));
    valid_ = true;

    // --- fault detection -------------------------------------------------
    // Open circuit needs the same patience as stall, and for a sharper reason.
    // Duty crosses min_duty at the START of the PWM ramp, while the current is
    // still climbing from zero - and the INA226s are only read at 5 Hz, so the
    // current reading can still be the pre-command zero for up to 200 ms after
    // the duty is up. Judged instantly, that reads as "this motor is drawing
    // nothing" on a motor that is about to draw plenty, and the dashboard
    // flashed a false open-circuit on every start.
    const bool looks_open = (current < cfg_.open_current_a);
    if (looks_open) {
      if (open_since_ == 0) open_since_ = millis();
      open_circuit_ = (millis() - open_since_) > OPEN_CIRCUIT_HOLD_MS;
    } else {
      open_since_ = 0;
      open_circuit_ = false;
    }

    const bool looks_stalled = (current > cfg_.stall_current_a) && (fabsf(rpm_) < 10.0f);
    if (looks_stalled) {
      if (stall_since_ == 0) stall_since_ = millis();
      stalled_ = (millis() - stall_since_) > 400;   // ignore the start-up surge
    } else {
      stall_since_ = 0;
      stalled_ = false;
    }
  }

  bool  valid() const { return valid_; }
  float rpm() const { return rpm_; }                       // output shaft [rpm]
  float diodeDrop() const { return diode_drop_; }          // [V] last computed drop
  float mps() const { return rpm_ / 60.0f * (float)M_PI * cfg_.wheel_diameter_m; }
  bool  stalled() const { return stalled_; }
  bool  openCircuit() const { return open_circuit_; }

private:
  Config cfg_;
  float rpm_ = 0.0f;
  float diode_drop_ = 0.0f;
  bool  valid_ = false;
  bool  stalled_ = false;
  bool  open_circuit_ = false;
  uint32_t stall_since_ = 0;
  uint32_t open_since_  = 0;
  // Comfortably longer than the 200 ms INA226 sampling interval, so a
  // stale current reading cannot be mistaken for a disconnected motor.
  static const uint32_t OPEN_CIRCUIT_HOLD_MS = 400;
};

#endif // MOTOR_MODEL_H
