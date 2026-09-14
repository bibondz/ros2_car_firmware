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
 * @file sim_world.h
 * @brief A synthetic robot with a KNOWN true pose, for testing the estimator.
 *
 * WHY THIS EXISTS
 * The estimator's job is to recover the truth from imperfect sensors. You
 * cannot test that with clean inputs: feed it perfect data and every filter
 * looks correct. What matters is the error against a truth you already know,
 * with realistic noise, bias and dropouts in the way.
 *
 * So this integrates a true pose from commanded wheel speeds, then SYNTHESISES
 * each sensor reading from that truth with its own characteristic error:
 *
 *   GPS      2 m Gaussian position noise, 1 Hz, configurable dropout windows,
 *            course only meaningful above a walking pace
 *   IMU      yaw with a slow gyro bias drift, forward acceleration with noise
 *   compass  hard-iron offset, and a "disturbed" flag that can be raised
 *   INA226   volts and amps consistent with the commanded PWM, so the motor
 *            model produces a matching back-EMF speed
 *
 * Because the truth is known exactly, accuracy is MEASURED rather than
 * asserted. That is the whole point: a claim like "under 1 m repeatability"
 * has to be a number produced by a test, not a hope.
 *
 * Deterministic on purpose - a fixed seed, so a failure can be reproduced.
 * No <random>: a tiny LCG keeps this identical across toolchains, which
 * matters when the same test runs here and on someone else's machine.
 */
#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <math.h>
#include <stdint.h>

#include "state_estimator.h"

class SimWorld {
public:
  struct Config {
    float dt              = 0.01f;    // 100 Hz, the real control rate
    float track_m         = 0.30f;
    float gps_period_s    = 1.0f;     // a consumer receiver, not 10 Hz RTK
    float gps_noise_m     = 2.0f;     // 1 sigma, about right for GEP-M10
    float gps_speed_noise = 0.05f;
    float gyro_bias_dps   = 0.30f;    // slow drift the fusion has to absorb
    float acc_noise       = 0.05f;
    float mag_offset_deg  = 8.0f;     // hard iron, uncorrected
    float mag_noise_deg   = 1.5f;
    float course_min_mps  = 0.25f;    // below this, GPS course is meaningless
    float bus_v           = 12.0f;
    float motor_r_ohm     = 5.0f;
    float motor_ke        = 0.12f;
    float diode_vf        = 1.81f;    // measured on this board
  };

  void begin(const Config &cfg) {
    cfg_ = cfg;
    rng_ = 12345u;                    // fixed seed: failures must reproduce
    t_ = 0.0f;
    true_x_ = true_y_ = 0.0f;
    true_heading_ = 0.0f;
    true_speed_ = 0.0f;
    gyro_bias_ = 0.0f;
    last_gps_t_ = -1000.0f;
    gps_lat_x_ = gps_lat_y_ = 0.0f;
    gps_valid_ = false;
    dropout_from_ = dropout_to_ = -1.0f;
    mag_disturbed_ = false;
  }

  /** Hide GPS between these times, to test coasting. */
  void setGpsDropout(float from_s, float to_s) {
    dropout_from_ = from_s;
    dropout_to_ = to_s;
  }
  void setMagDisturbed(bool on) { mag_disturbed_ = on; }

  /** Drive one control step at a commanded speed and turn rate, and return the
   *  sensor readings an estimator would have seen this cycle. */
  StateEstimator::Inputs step(float cmd_speed_mps, float cmd_yaw_rate_dps) {
    const float dt = cfg_.dt;

    // --- truth ---------------------------------------------------------
    // A first-order lag on speed, so the robot cannot change velocity
    // instantly - otherwise the filters are being asked something physically
    // impossible and the test proves nothing.
    true_speed_ += (cmd_speed_mps - true_speed_) * (dt / (0.4f + dt));
    const float yaw_rate_rad = cmd_yaw_rate_dps * 0.017453293f;
    true_heading_ = wrap360(true_heading_ + cmd_yaw_rate_dps * dt);

    const float h = true_heading_ * 0.017453293f;
    // compass heading: 0 = north = +y, growing clockwise
    true_x_ += true_speed_ * sinf(h) * dt;
    true_y_ += true_speed_ * cosf(h) * dt;
    t_ += dt;

    // --- sensors -------------------------------------------------------
    StateEstimator::Inputs in;
    in.dt = dt;

    gyro_bias_ += cfg_.gyro_bias_dps * dt / 60.0f;      // drifts over a minute
    in.imu_ok   = true;
    in.imu_yaw  = wrap360(true_heading_ + gyro_bias_);
    in.yaw_rate = -yaw_rate_rad;                        // CCW positive, ROS sense
    in.acc_fwd  = ((cmd_speed_mps - true_speed_) / (0.4f + dt)) + gauss(cfg_.acc_noise);

    const bool in_dropout = (t_ >= dropout_from_ && t_ <= dropout_to_);
    if (!in_dropout && (t_ - last_gps_t_) >= cfg_.gps_period_s) {
      last_gps_t_ = t_;
      gps_lat_x_ = true_x_ + gauss(cfg_.gps_noise_m);
      gps_lat_y_ = true_y_ + gauss(cfg_.gps_noise_m);
      gps_valid_ = true;
    }
    if (in_dropout) gps_valid_ = false;

    in.gps_ok        = gps_valid_;
    in.gps_speed     = gps_valid_ ? fmaxf(0.0f, true_speed_ + gauss(cfg_.gps_speed_noise)) : 0.0f;
    in.gps_course    = wrap360(true_heading_ + gauss(2.0f));
    in.gps_course_ok = gps_valid_ && true_speed_ > cfg_.course_min_mps;

    in.mag_ok       = !mag_disturbed_;
    in.mag_heading  = wrap360(true_heading_ + cfg_.mag_offset_deg + gauss(cfg_.mag_noise_deg));
    in.imu_mag_ok   = true;
    in.imu_mag_heading = wrap360(true_heading_ + gauss(3.0f));

    // Back-EMF speed the motor model would infer from this bus voltage and
    // current, so the fallback path gets a physically consistent number.
    in.model_mps = motorModelSpeed(cmd_speed_mps);
    return in;
  }

  // --- truth, for measuring error against ---
  float trueX() const { return true_x_; }
  float trueY() const { return true_y_; }
  float trueHeading() const { return true_heading_; }
  float trueSpeed() const { return true_speed_; }
  float time() const { return t_; }

  /** Straight-line distance from the true pose to an estimate. */
  float positionErrorFrom(float x, float y) const {
    const float dx = x - true_x_, dy = y - true_y_;
    return sqrtf(dx * dx + dy * dy);
  }

  /** The GPS fix as it stands, so a test can compare raw GPS against fusion. */
  bool gpsValid() const { return gps_valid_; }
  float gpsX() const { return gps_lat_x_; }
  float gpsY() const { return gps_lat_y_; }

private:
  static float wrap360(float a) {
    float x = fmodf(a, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
  }

  /** Deterministic uniform, so every run is identical. */
  float uniform() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return (float)((rng_ >> 8) & 0xFFFFFF) / (float)0xFFFFFF;
  }

  /** Gaussian via the sum of twelve uniforms - close enough for sensor noise
   *  and far cheaper than Box-Muller on a test that runs thousands of steps. */
  float gauss(float sigma) {
    float s = 0.0f;
    for (int i = 0; i < 12; ++i) s += uniform();
    return (s - 6.0f) * sigma;
  }

  float motorModelSpeed(float cmd) const {
    const float duty = fminf(1.0f, fabsf(cmd) / 1.0f);
    const float v_applied = (cfg_.bus_v - cfg_.diode_vf) * duty;
    const float current = v_applied / cfg_.motor_r_ohm * 0.35f;
    const float bemf = v_applied - current * cfg_.motor_r_ohm;
    const float rpm = bemf / cfg_.motor_ke;
    return fmaxf(0.0f, rpm * 0.081f * 3.14159265f / 60.0f) * (cmd < 0 ? -1.0f : 1.0f);
  }

  Config cfg_;
  uint32_t rng_ = 12345u;
  float t_ = 0.0f;
  float true_x_ = 0.0f, true_y_ = 0.0f, true_heading_ = 0.0f, true_speed_ = 0.0f;
  float gyro_bias_ = 0.0f;
  float last_gps_t_ = -1000.0f;
  float gps_lat_x_ = 0.0f, gps_lat_y_ = 0.0f;
  bool  gps_valid_ = false;
  float dropout_from_ = -1.0f, dropout_to_ = -1.0f;
  bool  mag_disturbed_ = false;
};

#endif  // SIM_WORLD_H
