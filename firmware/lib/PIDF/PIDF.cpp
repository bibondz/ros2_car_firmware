/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
#include "PIDF.h"
#include <math.h>

PIDF::PIDF(float min_val, float max_val,
           float Kp_, float Ki_,
           float i_min_, float i_max_,
           float Kd_, float Kf_,
           float tol_)
: Kp(0), Ki(0), Kd(0), Kf(0),
  error_tolerance(0),
  out_min(min_val), out_max(max_val),
  i_min(i_min_), i_max(i_max_),
  Setpoint(0.0f), LastError(0.0f), Integral(0.0f),
  Dfilt(0.0f), d_fc_hz(0.0f), d_init(true),
  last_us(0)
{
  setPIDF(Kp_, Ki_, Kd_, Kf_, tol_);
}

void PIDF::setPIDF(float Kp_, float Ki_, float Kd_, float Kf_, float tol_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_; Kf = Kf_;
  error_tolerance = tol_;
}

void PIDF::setOutputLimits(float min_val, float max_val) {
  out_min = min_val; out_max = max_val;
}

void PIDF::setIClamp(float i_min_, float i_max_) {
  i_min = i_min_; i_max = i_max_;
}

void PIDF::setDFilterCutoffHz(float fc_hz) {
  d_fc_hz = (fc_hz < 0.0f) ? 0.0f : fc_hz;
  d_init  = true;
}

void PIDF::reset() {
  Integral  = 0.0f;
  LastError = 0.0f;
  Dfilt     = 0.0f;
  d_init    = true;
  last_us   = 0;
}

float PIDF::step_dt() {
  unsigned long now = micros();
  if (last_us == 0) { last_us = now; return 0.0f; }
  unsigned long du = now - last_us;
  last_us = now;
  const float dt_min = 1e-4f;   // 0.1 ms
  const float dt_max = 0.2f;    // clamp after a stall so the I/D terms do not explode
  float dt = du * 1e-6f;
  if (dt < dt_min) dt = dt_min;
  if (dt > dt_max) dt = dt_max;
  return dt;
}

float PIDF::compute(float setpoint, float measure) {
  Setpoint = setpoint;
  return compute_with_error(setpoint - measure);
}

float PIDF::compute_with_error(float error) {
  float dt = step_dt();

  if (Kf == 0.0f) {
    // deadband: stop chattering around the setpoint
    if (fabsf(error) <= error_tolerance) {
      Integral  = 0.0f;
      LastError = error;
      // Sitting inside the deadband IS a known derivative - it is zero. Seed
      // the filter with that and mark it initialised.
      //
      // Leaving d_init true here meant the next sample outside the deadband
      // took the "first sample" path and set Dfilt straight to the raw
      // derivative, so the very first D kick after a setpoint step passed
      // through completely unfiltered - the one spike the filter exists to
      // suppress. On the heading loop that is a jolt at the start of every
      // move.
      Dfilt  = d_init ? 0.0f : (Dfilt * 0.9f);
      d_init = false;
      return 0.0f;
    }
  }

  Integral += error * dt;
  if (!(i_max == -1.0f && i_min == -1.0f)) {
    Integral = clamp(Integral, i_min, i_max);
  }

  float D_raw = (dt > 0.0f) ? (error - LastError) / dt : 0.0f;
  float D_use = D_raw;
  if (Kd != 0.0f && d_fc_hz > 0.0f) {
    float alpha = expf(-2.0f * (float)M_PI * d_fc_hz * dt);
    alpha = clamp(alpha, 0.0f, 1.0f);
    if (d_init) { Dfilt = D_raw; d_init = false; }
    else        { Dfilt = alpha * Dfilt + (1.0f - alpha) * D_raw; }
    D_use = Dfilt;
  }

  float out = Kp * error + Ki * Integral + Kd * D_use + Kf * Setpoint;

  LastError = error;
  return clamp(out, out_min, out_max);
}
