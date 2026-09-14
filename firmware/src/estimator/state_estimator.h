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
 * @file state_estimator.h
 * @brief Heading and speed estimation for a robot with NO wheel encoders.
 *
 * HEADING
 *   The BNO085 game rotation vector gives a smooth but *relative* yaw (it has
 *   no idea where north is). GPS course over ground is absolute but only valid
 *   while the robot is actually moving and is noisy at low speed. We keep a
 *   slowly-updated offset between the two: heading = imu_yaw + offset, where
 *   the offset is dragged towards (gps_course - imu_yaw) with a first order
 *   filter, rate limited, and only while speed > align_min_mps with a good fix.
 *   Result: IMU responsiveness with GPS absolute reference.
 *
 * SPEED
 *   Complementary filter. Forward acceleration from the IMU (gravity removed)
 *   is integrated for the fast component; GPS ground speed pulls the estimate
 *   back to truth with time constant gps_tau. If GPS goes stale the estimate is
 *   pulled towards the open-loop motor model instead so the integrator can
 *   never run away.
 *
 * WHEEL SPEED (reported to ROS / web UI, derived - there is no encoder)
 *   v_left  = v - w * track / 2
 *   v_right = v + w * track / 2      w = IMU yaw rate [rad/s]
 */
#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Arduino.h>
#include <math.h>

static inline float wrap360f(float a) { float x = fmodf(a, 360.0f); return x < 0.0f ? x + 360.0f : x; }
static inline float angErrDeg(float target, float current) {
  return fmodf((wrap360f(target) - wrap360f(current) + 540.0f), 360.0f) - 180.0f;
}
static inline float wrapPif(float a) {
  float x = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
  if (x < 0.0f) x += 2.0f * (float)M_PI;
  return x - (float)M_PI;
}

// Below this the speed estimate is treated as standing still, provided the
// sources agree. See the note where it is applied: gating on the sources too
// is what stops it trapping the filter at zero.
#define SPEED_CREEP_FLOOR_MPS 0.01f

class StateEstimator {
public:
  struct Config {
    // 0 = BNO085 rotation vector (its own magnetometer)
    // 1 = IMU gyro + GPS course alignment only (relative until the robot moves)
    // 2 = GPS course only
    // 3 = IMU gyro + QMC5883L compass when still + GPS course when moving
    // 4 = AUTO (default): same as 3, but every source is checked for health and
    //     the best one still alive is used. Priority while moving:
    //       GPS course > QMC5883L > BNO085 magnetometer > gyro only
    //     Losing any single sensor degrades the heading, it never stops the robot.
    uint8_t heading_source   = 4;
    float   align_min_mps    = 0.25f;
    float   align_tau_s      = 6.0f;
    float   align_max_dps    = 15.0f;
    float   mag_tau_s        = 4.0f;   // magnetometer pull-in while standing still
    float   mag_still_mps    = 0.10f;  // "standing still" threshold
    bool    mag_enable       = true;
    // How far an absolute reference may disagree with the IMU's own fused yaw
    // before it is treated as disturbed rather than believed. See the gate in
    // update() for why this exists and why it has to be able to give up.
    float   agree_tol_deg    = 25.0f;
    float   agree_recover_s  = 8.0f;
    float   gps_tau_s        = 1.5f;
    float   model_tau_s      = 2.5f;
    float   acc_deadband     = 0.08f;
    float   max_speed_mps    = 1.5f;
    float   track_m          = 0.30f;
    float   wheel_diameter_m = 0.081f;
  };

  void begin(const Config &cfg) { cfg_ = cfg; reset(); }

  void reset() {
    speed_        = 0.0f;
    heading_deg_  = 0.0f;
    align_offset_ = 0.0f;
    aligned_      = false;
    heading_ref_  = REF_NONE;
    x_ = y_ = 0.0f;
    // The gate's memory goes too. Carrying a disagreement across a reset would
    // let a stale disturbance from before the reset open the tolerance
    // immediately afterwards, which is the one moment the gate matters most.
    heading_innov_deg_ = 0.0f;
    disagree_s_    = 0.0f;
    mag_rejecting_ = false;
  }

  /**
   * @param dt         loop period [s]
   * @param imu_ok     IMU produced fresh data
   * @param imu_yaw    yaw from the IMU [deg], already converted to compass sense
   * @param yaw_rate   yaw rate [rad/s], counter-clockwise positive (raw gyro Z)
   * @param acc_fwd    forward acceleration [m/s^2], gravity removed
   * @param gps_ok     the fix is good enough to take a ground SPEED from
   * @param gps_fix_ok the receiver has a position at all (a looser, separate test)
   * @param gps_speed  ground speed [m/s]
   * @param gps_course course over ground [deg], only used when gps_course_ok
   * @param model_mps  open-loop speed implied by the current PWM (fallback reference)
   */
  /** Everything the estimator can be told about the world this cycle.
   *  Any field may be missing (ok flag false) - the arbitration below picks
   *  the best source that is still alive. */
  struct Inputs {
    float dt = 0.01f;
    bool  imu_ok = false;          // gyro + game rotation vector are fresh
    float imu_yaw = 0.0f;          // relative yaw [deg], compass sense
    float yaw_rate = 0.0f;         // [rad/s], counter-clockwise positive
    float acc_fwd = 0.0f;          // forward acceleration [m/s^2], gravity removed
    // Good enough to believe the receiver's VELOCITY: strict, because a
    // marginal fix reports plausible-looking nonsense ground speeds.
    bool  gps_ok = false;
    // The receiver has a POSITION at all: fresh, real, enough satellites for a
    // 3D solution. Looser on purpose, and a different question - this is what
    // decides whether the GPS is reported as a working sensor. Defaults to
    // false so a caller that sets neither is told the GPS is missing rather
    // than being told it is fine.
    bool  gps_fix_ok = false;
    float gps_speed = 0.0f;
    float gps_course = 0.0f;       // course over ground [deg]
    bool  gps_course_ok = false;   // course is meaningful (robot actually moving)
    bool  mag_ok = false;          // QMC5883L in the GPS module, calibrated and undisturbed
    float mag_heading = 0.0f;
    bool  imu_mag_ok = false;
    // Is the BNO085's magnetometer-referenced report even being ASKED for?
    // When it is not, its absence is a choice, not a fault, and flagging it as
    // a missing sensor sends someone hunting a magnetometer that is fine.
    bool  imu_mag_used = true;      // BNO085 rotation vector, accuracy >= medium
    float imu_mag_heading = 0.0f;
    float model_mps = 0.0f;        // open-loop speed implied by the PWM
    // False when the motor rails are not MEASURED to be live. Not the same as
    // the commanded relay state - a coil with no supply behind it, no battery,
    // or a failed contactor all leave the rails dead while the firmware thinks
    // it closed them. The
    // back-EMF model is then reading a rail at ~0 V and a current at ~1 mA and
    // turning that noise into a speed. Seen on hardware: robot latched in
    // e-stop, wheels physically still, estimator reporting 0.127 m/s - which
    // also blocked Wi-Fi OTA, because the updater refuses to flash a robot it
    // believes is moving.
    bool  motors_powered = true;
  };

  void update(const Inputs &in) {
    const float dt = in.dt;
    if (dt <= 0.0f) return;
    imu_ok_ = in.imu_ok;
    gps_ok_ = in.gps_ok;

    //------------------------------ speed ------------------------------//
    // ladder: IMU integration corrected by GPS  ->  GPS alone  ->  motor model
    float a = in.acc_fwd;
    if (fabsf(a) < cfg_.acc_deadband) a = 0.0f;
    float v = speed_ + (in.imu_ok ? a * dt : 0.0f);

    if (in.gps_ok) {
      float k = dt / ((in.imu_ok ? cfg_.gps_tau_s : cfg_.gps_tau_s * 0.3f) + dt);
      v += k * (in.gps_speed - v);
      speed_source_ = in.imu_ok ? SPEED_IMU_GPS : SPEED_GPS;
    } else {
      // With the rails dead the model is not a measurement, it is noise. Pull
      // toward zero instead: the robot is not driving itself. It could still be
      // rolling - pushed, or on a slope - so this is a reference, not a hard
      // assignment, and the IMU term above still carries a real push.
      const float ref = in.motors_powered ? in.model_mps : 0.0f;
      float k = dt / (cfg_.model_tau_s + dt);
      v += k * (ref - v);
      speed_source_ = in.imu_ok ? SPEED_IMU_MODEL : SPEED_MODEL;
    }
    if (v >  cfg_.max_speed_mps) v =  cfg_.max_speed_mps;
    if (v < -cfg_.max_speed_mps) v = -cfg_.max_speed_mps;

    // Kill sensor-noise creep when standing still - but only when the SOURCES
    // also say we are stopped.
    //
    // Applied unconditionally this traps the filter at zero forever. Starting
    // from rest the first correction is k*(ref - 0) with k = dt/(tau+dt), which
    // for tau 1.5 s at 100 Hz is 0.0066 - so a 0.40 m/s reference moves the
    // estimate by 0.0027, below the 0.01 floor, and it is zeroed. The next
    // cycle starts from zero again and the estimate can never climb out. The
    // robot would have reported 0 m/s whenever acceleration was too small to
    // clear the floor on its own, which is exactly the low-speed creep the
    // waypoint controller cares about.
    const float reference = in.gps_ok ? in.gps_speed
                                     : (in.motors_powered ? in.model_mps : 0.0f);
    if (fabsf(v) < SPEED_CREEP_FLOOR_MPS && fabsf(reference) < SPEED_CREEP_FLOOR_MPS) {
      v = 0.0f;
    }
    speed_ = v;

    //----------------------------- heading -----------------------------//
    // yaw_rate_ is CCW positive; the compass heading grows the other way
    yaw_rate_ = in.yaw_rate;
    const float compass_rate_dps = -in.yaw_rate * 57.29578f;

    health_ = 0;
    if (!in.imu_ok)                       health_ |= HEALTH_NO_IMU;
    // The SENSOR is missing only when there is no position at all. A fix too
    // rough to take a velocity from is still a fix, and reporting it as a dead
    // sensor sent someone looking for a hardware fault that did not exist.
    if (!in.gps_fix_ok)                   health_ |= HEALTH_NO_GPS;
    if (!in.mag_ok)                       health_ |= HEALTH_NO_COMPASS;
    if (in.imu_mag_used && !in.imu_mag_ok) health_ |= HEALTH_NO_IMU_MAG;

    if (cfg_.heading_source == 2) {
      // GPS course only
      if (in.gps_course_ok && in.gps_ok && fabsf(speed_) > cfg_.align_min_mps) {
        heading_deg_ = wrap360f(in.gps_course);
        heading_ref_ = REF_GPS;
      } else {
        heading_deg_ = wrap360f(heading_deg_ + compass_rate_dps * dt);   // gyro carries on
        heading_ref_ = REF_NONE;
      }
      aligned_ = in.gps_course_ok;
    } else if (cfg_.heading_source == 0) {
      // BNO085 rotation vector: already absolute, it uses its own magnetometer
      if (in.imu_ok) { heading_deg_ = wrap360f(in.imu_yaw); heading_ref_ = REF_IMU_MAG; }
      else           { heading_ref_ = REF_NONE; }
      aligned_ = in.imu_ok;
    } else {
      // ---------------- fallback chain (modes 1, 3 and AUTO) ----------------
      // The IMU's OWN FUSED HEADING is the backbone - in.imu_yaw, the BNO085's
      // rotation vector, not a gyro rate integrated here. The sensor already
      // does that integration at its own rate and corrects the gyro's zero
      // drift against gravity; redoing it in this loop would be strictly worse,
      // because every late or dropped cycle is rotation that silently never
      // happened. What the sensor cannot do is tell you where north is, so an
      // absolute reference is eased onto it. Sources, best first:
      //
      //   1. GPS course over ground  - only while actually moving, immune to magnets
      //   2. QMC5883L on the GPS mast - works standing still, away from the motors
      //   3. BNO085 magnetometer      - same idea, but sits next to the motors
      //   4. nothing                  - run on the IMU's own yaw, flag as relative
      //
      // Any of them can disappear at any moment; the next one down takes over
      // without the robot stopping.
      const bool allow_mag = cfg_.mag_enable && cfg_.heading_source >= 3;
      const bool moving    = in.gps_course_ok && in.gps_ok && fabsf(speed_) > cfg_.align_min_mps;
      const bool standing  = fabsf(speed_) < cfg_.mag_still_mps;

      /* Two sensors that both claim to know north, checked against a third
       * that cannot be lied to.
       *
       * The compass and the BNO085's magnetometer are both absolute and both
       * fooled by the same things - a motor drawing current, a steel bench, a
       * reinforcing bar in the floor. The IMU's fused yaw is neither: it comes
       * from the gyroscope, which does not care about magnets at all. It drifts,
       * slowly, which is exactly why it needs an absolute reference - but over
       * the seconds that a disturbance lasts it is the honest one.
       *
       * So each absolute source is compared with the fused heading before it is
       * believed. Measured on this robot: the compass moved 9.5 degrees in
       * 0.3 s while the gyroscope reported about 1 deg/s. One of those was
       * lying, and it was not the gyroscope. Applied blind, that step went
       * straight into the heading and the manual hold then steered to it.
       *
       * THE GATE HAS TO BE ABLE TO GIVE UP. A pure rejection rule locks a
       * correct sensor out for ever: if the gyro really has drifted 40 degrees,
       * every compass reading looks like a disturbance and nothing can ever pull
       * it back. So a disagreement that persists is eventually accepted - after
       * `agree_recover_s` the tolerance opens up, on the reasoning that a
       * disturbance passes and a genuine offset does not.
       */
      auto gated = [&](float innov, bool source_ok) -> bool {
        if (!source_ok) return false;
        if (!in.imu_ok) return true;         // no referee: nothing to check against
        // Before the first alignment there is nothing to disagree WITH. The
        // fused heading is still an arbitrary zero, so every real compass
        // reading looks like a wild disturbance and the gate would refuse the
        // very sample that establishes north - the robot would never align at
        // all. The first one always goes in; pullOffset snaps it.
        if (!aligned_) return true;
        const float mag = fabsf(innov);
        if (mag <= cfg_.agree_tol_deg) { disagree_s_ = 0.0f; return true; }
        disagree_s_ += dt;
        return disagree_s_ >= cfg_.agree_recover_s;
      };

      if (moving) {
        pullOffset(angErrDeg(in.gps_course, in.imu_yaw), cfg_.align_tau_s, dt);
        heading_ref_ = REF_GPS;
        disagree_s_ = 0.0f;
      } else if (allow_mag && standing && in.mag_ok) {
        // first sample snaps, afterwards it eases in so a passing truck cannot
        // yank the heading around
        const float innov = angErrDeg(in.mag_heading, wrap360f(in.imu_yaw + align_offset_));
        heading_innov_deg_ = innov;
        if (gated(innov, true)) {
          pullOffset(angErrDeg(in.mag_heading, in.imu_yaw), cfg_.mag_tau_s, dt);
          heading_ref_ = REF_MAG;
          mag_rejecting_ = false;
        } else {
          // Disturbed, and the IMU is still carrying the heading, so nothing
          // has to happen except refusing to believe this reading.
          mag_rejecting_ = true;
          mag_rejects_++;
          heading_ref_ = aligned_ ? heading_ref_ : REF_NONE;
        }
      } else if (allow_mag && standing && in.imu_mag_ok && cfg_.heading_source == 4) {
        // compass gone: fall back to the IMU's own magnetometer, more slowly
        // because it is the noisier of the two
        const float innov = angErrDeg(in.imu_mag_heading, wrap360f(in.imu_yaw + align_offset_));
        heading_innov_deg_ = innov;
        if (gated(innov, true)) {
          pullOffset(angErrDeg(in.imu_mag_heading, in.imu_yaw), cfg_.mag_tau_s * 2.0f, dt);
          heading_ref_ = REF_IMU_MAG;
          mag_rejecting_ = false;
        } else {
          mag_rejecting_ = true;
          mag_rejects_++;
        }
      } else if (!aligned_) {
        heading_ref_ = REF_NONE;
      }

      if (in.imu_ok) {
        heading_deg_ = wrap360f(in.imu_yaw + align_offset_);
      } else if (in.gps_course_ok && in.gps_ok && fabsf(speed_) > cfg_.align_min_mps) {
        heading_deg_ = wrap360f(in.gps_course);      // gyro dead, drive on GPS course
        heading_ref_ = REF_GPS;
      } else if (allow_mag && in.mag_ok) {
        heading_deg_ = wrap360f(in.mag_heading);     // gyro and GPS dead, compass only
        heading_ref_ = REF_MAG;
      } else {
        // Last resort only: the IMU is gone, so its fused yaw is gone with it
        // and there is nothing left but a rate to integrate. Reached when the
        // IMU, GPS course and compass have ALL failed at once.
        //
        // Saying so is part of the job. heading_ref_ kept whatever it last held
        // here, so with every absolute source dead the estimator went on
        // reporting REF_GPS or REF_MAG, HEALTH_NO_ABS_HEADING was never raised,
        // and everything downstream believed the heading was still referenced
        // to something. It is not: this is a drifting rate and nothing else.
        heading_deg_ = wrap360f(heading_deg_ + compass_rate_dps * dt);
        heading_ref_ = REF_NONE;
      }
    }

    if (heading_ref_ == REF_NONE) health_ |= HEALTH_NO_ABS_HEADING;

    //---------------------------- odometry -----------------------------//
    // published in the ROS ENU frame: x = east, y = north, yaw CCW from east
    const float h_enu = headingEnuRad();
    x_ += speed_ * cosf(h_enu) * dt;
    y_ += speed_ * sinf(h_enu) * dt;
  }

  /** How far the absolute reference in use disagrees with the fused heading.
   *  Small means the two independent sources agree, which is the case worth
   *  trusting more than either alone. Large means one of them is disturbed. */
  float headingInnovDeg() const { return heading_innov_deg_; }
  /** True while an absolute reference is being refused for disagreeing. */
  bool  magRejecting()    const { return mag_rejecting_; }
  uint32_t magRejects()   const { return mag_rejects_; }

  void setOdom(float x, float y) { x_ = x; y_ = y; }
  void resetOdom() { x_ = 0.0f; y_ = 0.0f; }

  enum HeadingRef { REF_NONE = 0, REF_MAG = 1, REF_GPS = 2, REF_IMU_MAG = 3, REF_MANUAL = 4 };
  HeadingRef headingRef() const { return heading_ref_; }

  /**
   * Tell the robot which way it is pointing, by hand.
   *
   * Used by the "set start pose" tool in the web UI (drag an arrow on the map,
   * like the 2D Pose Estimate in RViz) when there is no compass and no GPS
   * course to lock onto - indoors, or as a backup. It pins the gyro's relative
   * yaw to the given compass heading; the gyro carries it from there and any
   * real reference that shows up later (compass, GPS course) takes over again.
   */
  void setHeading(float compass_deg, float imu_yaw) {
    align_offset_ = wrap360f(compass_deg - imu_yaw);
    heading_deg_  = wrap360f(compass_deg);
    aligned_      = true;
    heading_ref_  = REF_MANUAL;
  }

  // what the speed estimate is currently built from
  enum SpeedSource { SPEED_NONE = 0, SPEED_IMU_GPS = 1, SPEED_GPS = 2,
                     SPEED_IMU_MODEL = 3, SPEED_MODEL = 4 };

  // which sensors are missing right now (published in the telemetry array)
  static const uint8_t HEALTH_NO_IMU         = 1 << 0;
  static const uint8_t HEALTH_NO_GPS         = 1 << 1;
  static const uint8_t HEALTH_NO_COMPASS     = 1 << 2;
  static const uint8_t HEALTH_NO_IMU_MAG     = 1 << 3;
  static const uint8_t HEALTH_NO_ABS_HEADING = 1 << 4;   // heading is relative only
  uint8_t health() const { return health_; }

  float speed()      const { return speed_; }          // [m/s] body forward
  float headingDeg() const { return heading_deg_; }    // [deg] compass, 0 = north, CW
  /** Same heading expressed in the ROS ENU convention (0 = east, CCW). */
  float headingEnuRad() const { return wrapPif((90.0f - heading_deg_) * 0.01745329f); }
  float yawRate()    const { return yaw_rate_; }       // [rad/s] CCW positive (ROS convention)
  float x()          const { return x_; }
  float y()          const { return y_; }
  bool  aligned()    const { return aligned_; }
  uint8_t speedSource() const { return speed_source_; }  // see SpeedSource

  float wheelLeftMps()  const { return speed_ - yaw_rate_ * cfg_.track_m * 0.5f; }
  float wheelRightMps() const { return speed_ + yaw_rate_ * cfg_.track_m * 0.5f; }
  float wheelLeftRpm()  const { return mpsToRpm(wheelLeftMps()); }
  float wheelRightRpm() const { return mpsToRpm(wheelRightMps()); }

  float mpsToRpm(float mps) const {
    float circ = (float)M_PI * cfg_.wheel_diameter_m;
    return (circ > 1e-6f) ? (mps / circ) * 60.0f : 0.0f;
  }

  Config &config() { return cfg_; }

private:
  /** Ease align_offset_ towards `want` with time constant tau, rate limited. */
  void pullOffset(float want, float tau_s, float dt) {
    if (!aligned_) { align_offset_ = want; aligned_ = true; return; }
    float err  = angErrDeg(want, align_offset_);
    float step = err * (dt / (tau_s + dt));
    float lim  = cfg_.align_max_dps * dt;
    if (step >  lim) step =  lim;
    if (step < -lim) step = -lim;
    align_offset_ = wrap360f(align_offset_ + step);
  }

  Config cfg_;
  HeadingRef heading_ref_ = REF_NONE;
  float speed_ = 0.0f;
  float heading_deg_ = 0.0f;
  float align_offset_ = 0.0f;
  float yaw_rate_ = 0.0f;
  // Cross-check state. heading_innov_deg_ is how far the absolute reference in
  // use sits from the fused heading; disagree_s_ is how long it has been out,
  // which is what separates a passing disturbance from a real offset.
  float heading_innov_deg_ = 0.0f;
  float disagree_s_ = 0.0f;
  bool  mag_rejecting_ = false;
  uint32_t mag_rejects_ = 0;
  float x_ = 0.0f, y_ = 0.0f;
  bool  aligned_ = false;
  bool  imu_ok_ = false;
  bool  gps_ok_ = false;
  uint8_t speed_source_ = 0;
  uint8_t health_ = 0;
};

#endif // STATE_ESTIMATOR_H
