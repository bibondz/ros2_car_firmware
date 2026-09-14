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
 * @file PIDF.h
 * @brief PID + feed-forward controller with clamped integrator and filtered D term.
 *
 * Carried over from the mor_luam prototype (proven on hardware) with the
 * derivative low-pass and integrator clamp kept as-is. No dynamic allocation:
 * every instance is a fixed-size object created once at start-up.
 */
#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
public:
  PIDF(float min_val, float max_val,
       float Kp = 0.0f, float Ki = 0.0f,
       float i_min = 0.0f, float i_max = 0.0f,
       float Kd = 0.0f, float Kf = 0.0f,
       float error_tolerance = 0.0f);

  void  setPIDF(float Kp, float Ki, float Kd, float Kf, float error_tolerance);
  void  setOutputLimits(float min_val, float max_val);
  void  setIClamp(float i_min, float i_max);
  void  setDFilterCutoffHz(float fc_hz);
  void  reset();

  float compute(float setpoint, float measure);
  float compute_with_error(float error);

  float kp() const { return Kp; }
  float ki() const { return Ki; }
  float kd() const { return Kd; }
  float kf() const { return Kf; }
  float tol() const { return error_tolerance; }

private:
  float Kp, Ki, Kd, Kf;
  float error_tolerance;

  float out_min, out_max;
  float i_min, i_max;

  float Setpoint;
  float LastError;
  float Integral;

  float Dfilt;
  float d_fc_hz;
  bool  d_init;

  unsigned long last_us;

  static inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
  float step_dt();
};

#endif
