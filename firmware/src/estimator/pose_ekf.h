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
 * @file pose_ekf.h
 * @brief Extended Kalman filter for position, so GPS actually corrects it.
 *
 * THE PROBLEM THIS SOLVES
 * StateEstimator fuses heading and speed well, but position was plain dead
 * reckoning:
 *
 *     x += speed * cos(heading) * dt;
 *
 * Nothing ever corrected it, so the error grew without bound - a slightly wrong
 * heading becomes metres of drift over a minute, and the robot confidently
 * drives to the wrong place.
 *
 * STATE
 *     x, y      position in the local ENU frame [m]
 *     heading   compass degrees, 0 = north, carried so its uncertainty can
 *               couple into position - a heading error IS a position error
 *               once you have moved
 *
 * Speed is an input rather than a state. StateEstimator already fuses it from
 * three sources and knows which one it is using; duplicating that here would
 * mean two filters disagreeing about the same quantity.
 *
 * WHY AN EKF AND NOT A COMPLEMENTARY FILTER
 * The measurement quality genuinely varies: HDOP moves, satellites drop, and a
 * fix quality of 4 or 5 (RTK) is orders of magnitude better than a 1. A filter
 * with a fixed gain has to be tuned for the worst case and then wastes the good
 * data. Carrying a covariance means the gain follows the receiver's own
 * reported accuracy - and the day an RTK receiver is fitted, the same code
 * tightens automatically with no redesign.
 *
 * HONEST LIMIT
 * This cannot beat the GNSS bias. A consumer receiver sits 1.5-2.5 m off in a
 * way no amount of filtering removes, because the error is common to every
 * sample. What this removes is the JITTER on top of that, which is what makes a
 * path repeatable and a heading usable.
 */
#ifndef POSE_EKF_H
#define POSE_EKF_H

#include <math.h>
#include <stdint.h>

class PoseEkf {
public:
  struct Config {
    // Process noise. How much we distrust the prediction per second - this is
    // what lets the filter move when the model is wrong.
    float q_pos_mps      = 0.05f;   // unmodelled position drift [m/sqrt(s)]
    float q_heading_dps  = 2.0f;    // heading random walk [deg/sqrt(s)]

    // Measurement noise floor. A receiver reporting HDOP 1.0 is not accurate
    // to a centimetre, so the computed variance never goes below this.
    float r_min_m        = 0.8f;
    float r_max_m        = 25.0f;

    // Outlier rejection. A multipath jump next to a building arrives as a
    // perfectly valid-looking fix several metres away; accepting it yanks the
    // estimate and the robot lurches.
    float gate_sigma     = 3.0f;    // Mahalanobis distance, in sigmas
    uint8_t gate_max_reject = 5;    // after this many, believe the GPS again

    float init_pos_var   = 100.0f;  // 10 m, before the first fix
    float init_head_var  = 400.0f;  // 20 deg
  };

  void begin(const Config &cfg) { cfg_ = cfg; reset(); }

  void reset() {
    x_ = y_ = 0.0f;
    heading_ = 0.0f;
    p_xx_ = p_yy_ = cfg_.init_pos_var;
    p_xy_ = 0.0f;
    p_hh_ = cfg_.init_head_var;
    have_fix_ = false;
    rejected_ = 0;
    last_innov_m_ = 0.0f;
  }

  /** Seed the filter from the first fix, or from the web UI's "set start pose".
   *  Without this the first update has to drag the estimate from the origin. */
  void setPose(float x, float y, float heading_deg, float pos_var = 4.0f) {
    x_ = x; y_ = y;
    heading_ = wrap360(heading_deg);
    p_xx_ = p_yy_ = pos_var;
    p_xy_ = 0.0f;
    have_fix_ = true;
    rejected_ = 0;
  }

  /**
   * Predict forward one control step.
   *
   * @param dt           step [s]
   * @param speed_mps    fused body speed from StateEstimator
   * @param heading_deg  fused heading from StateEstimator
   * @param speed_var    how much that speed is trusted [(m/s)^2]
   * @param heading_var  how much that heading is trusted [deg^2]
   */
  void predict(float dt, float speed_mps, float heading_deg,
               float speed_var, float heading_var) {
    if (dt <= 0.0f) return;
    heading_ = wrap360(heading_deg);

    const float h = heading_ * DEG2RAD;
    const float s = sinf(h), c = cosf(h);

    // Compass convention: 0 = north = +y, growing clockwise.
    x_ += speed_mps * s * dt;
    y_ += speed_mps * c * dt;

    // Position uncertainty grows from three things: the process noise, the
    // speed uncertainty along the direction of travel, and - the one that
    // matters most on a robot - the heading uncertainty swinging the travelled
    // distance sideways.
    const float d = speed_mps * dt;
    const float head_rad_var = heading_var * DEG2RAD * DEG2RAD;
    const float lateral = d * d * head_rad_var;      // arc length error
    const float along = speed_var * dt * dt;
    const float q = cfg_.q_pos_mps * cfg_.q_pos_mps * dt;

    p_xx_ += along * s * s + lateral * c * c + q;
    p_yy_ += along * c * c + lateral * s * s + q;
    p_xy_ += (along - lateral) * s * c;
    p_hh_ += cfg_.q_heading_dps * cfg_.q_heading_dps * dt;
    if (p_hh_ > heading_var + 1.0f) p_hh_ = heading_var + 1.0f;
  }

  /**
   * Correct with a GPS fix.
   *
   * @param gx,gy    fix in the same local ENU frame [m]
   * @param hdop     receiver's reported dilution of precision
   * @param fix_q    NMEA fix quality: 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float
   * @return         false when the fix was rejected as an outlier
   */
  bool updateGps(float gx, float gy, float hdop, uint8_t fix_q) {
    const float r = measurementVariance(hdop, fix_q);

    if (!have_fix_) {                     // first fix: take it, do not filter it
      setPose(gx, gy, heading_, r);
      return true;
    }

    const float ix = gx - x_;             // innovation
    const float iy = gy - y_;
    last_innov_m_ = sqrtf(ix * ix + iy * iy);

    // Innovation covariance S = P + R, and its inverse for the gate and gain.
    const float s_xx = p_xx_ + r;
    const float s_yy = p_yy_ + r;
    const float s_xy = p_xy_;
    const float det = s_xx * s_yy - s_xy * s_xy;
    if (det <= 1e-9f) return false;       // degenerate, do not touch the state

    const float inv_xx =  s_yy / det;
    const float inv_yy =  s_xx / det;
    const float inv_xy = -s_xy / det;

    // Mahalanobis distance: how surprising is this fix, in its own units?
    const float d2 = ix * ix * inv_xx + 2.0f * ix * iy * inv_xy + iy * iy * inv_yy;
    const float gate = cfg_.gate_sigma * cfg_.gate_sigma;
    if (d2 > gate) {
      // Reject - but not forever. If the receiver has genuinely moved to a new
      // solution, refusing every fix would strand the estimate at a stale
      // position that only looks confident. After a few rejections in a row,
      // accept and let the filter re-converge.
      if (++rejected_ < cfg_.gate_max_reject) return false;
      rejected_ = 0;
      setPose(gx, gy, heading_, r);
      return true;
    }
    rejected_ = 0;

    // K = P * S^-1
    const float k_xx = p_xx_ * inv_xx + p_xy_ * inv_xy;
    const float k_xy = p_xx_ * inv_xy + p_xy_ * inv_yy;
    const float k_yx = p_xy_ * inv_xx + p_yy_ * inv_xy;
    const float k_yy = p_xy_ * inv_xy + p_yy_ * inv_yy;

    x_ += k_xx * ix + k_xy * iy;
    y_ += k_yx * ix + k_yy * iy;

    // P = (I - K) P
    const float n_xx = (1.0f - k_xx) * p_xx_ - k_xy * p_xy_;
    const float n_xy = (1.0f - k_xx) * p_xy_ - k_xy * p_yy_;
    const float n_yy = (1.0f - k_yy) * p_yy_ - k_yx * p_xy_;
    // The floor here is deliberately tiny, and an audit finding that called it
    // "a 1 cm sigma this receiver can never earn" was WRONG - measured, not
    // argued. Raising it to 0.5 m made the filter follow each fix instead of
    // averaging across fixes, and the 30 s straight-line sim went from under
    // 2 m of final error to 3.44 m, worse than raw GPS at 2.73 m and far worse
    // than dead reckoning at 0.41 m. Averaging the noise down IS the filter's
    // job; the correlated part of the GPS error is handled by r_min_m on the
    // measurement side, which is where it belongs.
    p_xx_ = fmaxf(n_xx, 1e-4f);
    p_yy_ = fmaxf(n_yy, 1e-4f);
    p_xy_ = n_xy;
    return true;
  }

  float x() const { return x_; }
  float y() const { return y_; }
  bool  hasFix() const { return have_fix_; }

  /** One number for the UI: the 1-sigma position uncertainty in metres.
   *  Published so the web UI can show honest accuracy instead of a dot that
   *  looks equally confident while coasting blind. */
  float positionSigma() const { return sqrtf(fmaxf(p_xx_ + p_yy_, 0.0f) * 0.5f); }

  float lastInnovation() const { return last_innov_m_; }
  uint8_t rejectedInARow() const { return rejected_; }

private:
  static constexpr float DEG2RAD = 0.017453293f;

  static float wrap360(float a) {
    float x = fmodf(a, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
  }

  /** Trust the receiver's own accuracy estimate rather than a constant.
   *  This is what makes an RTK receiver an improvement with no code change. */
  float measurementVariance(float hdop, uint8_t fix_q) const {
    float base;
    switch (fix_q) {
      case 4:  base = 0.02f; break;   // RTK fixed, centimetres
      case 5:  base = 0.30f; break;   // RTK float
      case 2:  base = 1.0f;  break;   // DGPS / SBAS
      default: base = 2.5f;  break;   // plain GPS, what this robot has
    }
    float sigma = base * fmaxf(hdop, 0.8f);
    if (sigma < cfg_.r_min_m) sigma = cfg_.r_min_m;
    if (sigma > cfg_.r_max_m) sigma = cfg_.r_max_m;
    return sigma * sigma;
  }

  Config cfg_;
  float x_ = 0.0f, y_ = 0.0f, heading_ = 0.0f;
  float p_xx_ = 100.0f, p_yy_ = 100.0f, p_xy_ = 0.0f, p_hh_ = 400.0f;
  bool  have_fix_ = false;
  uint8_t rejected_ = 0;
  float last_innov_m_ = 0.0f;
};

#endif  // POSE_EKF_H
