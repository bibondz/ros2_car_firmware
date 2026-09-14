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
 * @file drive_controller.h
 * @brief Heading-hold differential drive controller (no encoders).
 *
 * AUTO mode  : the ROS side sends "go to heading H at speed V".
 *              - heading PID (IMU) produces a differential PWM term
 *              - speed PI + feed-forward produces the common PWM term
 *              - if the heading error is large the robot first turns on the
 *                spot (counter-rotating wheels), then drives straight
 * MANUAL mode: the web UI sends forward speed + yaw rate, open loop with the
 *              same feed-forward mapping. Used for jogging the robot around.
 *
 * Everything is fixed point-free float math on stack variables; the two PIDF
 * objects are the only state and they are created once.
 */
#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include <Arduino.h>
#include <PIDF.h>
#include <config.h>
#include <state_estimator.h>

class DriveController {
public:
  DriveController()
  : heading_(-HEADING_OUT_MAX, HEADING_OUT_MAX,
             HEADING_KP, HEADING_KI, HEADING_I_MIN, HEADING_I_MAX,
             HEADING_KD, HEADING_KF, HEADING_ERROR_TOL_DEG),
    speed_(-(float)PWM_MAX, (float)PWM_MAX,
           SPEED_KP, SPEED_KI, SPEED_I_MIN, SPEED_I_MAX,
           SPEED_KD, SPEED_KF, SPEED_ERROR_TOL_MPS)
  {}

  void begin() {
    heading_.setDFilterCutoffHz(HEADING_D_FILTER_HZ);
    speed_.setDFilterCutoffHz(SPEED_D_FILTER_HZ);
    reset();
  }

  void reset() {
    heading_.reset();
    speed_.reset();
    speed_ramp_ = 0.0f;
    spinning_   = false;
    // A reset means "stop what you were doing". These were left alone, so a
    // reset during a turn-to-heading resumed the turn on the next cycle, and a
    // stale manual heading hold could steer a later straight command to a
    // heading nobody had asked for.
    turn_in_place_ = false;
    turn_done_     = false;
    turn_ok_ms_    = 0;
    hold_active_   = false;
    settle_s_      = 0.0f;
    pwm_l_ = pwm_r_ = 0;
    out_l_ = out_r_ = 0.0f;
  }

  /**
   * Motor acceleration ramp.
   * @param accel_s time to go from stopped to full PWM  (0 = instant)
   * @param decel_s time to come back down to zero
   * The ramp is applied to the PWM that actually reaches the driver, so it
   * limits current draw and gearbox shock whatever the control loop asks for.
   * An emergency stop bypasses it: main.cpp calls motor.disable() directly.
   */
  void setRamp(float accel_s, float decel_s) {
    accel_per_s_ = (accel_s > 0.01f) ? ((float)PWM_MAX / accel_s) : 0.0f;
    decel_per_s_ = (decel_s > 0.01f) ? ((float)PWM_MAX / decel_s) : 0.0f;
  }

  //---------------------------- commands ----------------------------//
  void setAuto(float speed_mps, float heading_deg) {
    target_speed_   = speed_mps;
    target_heading_ = wrap360f(heading_deg);
    manual_ = false;
    turn_in_place_ = false;
    // Leaving manual drops the hold with it. Otherwise holdingHeading() keeps
    // reporting true through an automatic mission, and a later manual straight
    // command re-uses a heading latched before the mission started.
    hold_active_ = false;
    settle_s_    = 0.0f;
  }

  void setManual(float speed_mps, float yaw_rate_dps) {
    target_speed_    = speed_mps;
    target_yaw_rate_ = yaw_rate_dps;
    manual_ = true;
    turn_in_place_ = false;
  }

  /**
   * "Turn to 90 degrees" / "turn to north" - rotate on the spot and stop.
   * Works without GPS (indoor test mode): the heading comes from the compass
   * and the gyro, no ground speed needed.
   */
  void setTurnTo(float heading_deg) {
    target_heading_  = wrap360f(heading_deg);
    target_speed_    = 0.0f;
    target_yaw_rate_ = 0.0f;
    speed_ramp_      = 0.0f;
    manual_          = false;
    turn_in_place_   = true;
    turn_done_       = false;
    turn_ok_ms_      = 0;
    turn_start_ms_   = millis();     // bounds the turn; see TURN_IN_PLACE_MAX_MS
    hold_active_     = false;
    settle_s_        = 0.0f;
    heading_.reset();
  }

  bool turningInPlace() const { return turn_in_place_; }
  bool turnDone() const { return turn_done_; }
  /** True when the last turn ended on its time limit instead of on the heading. */
  bool turnTimedOut() const { return turn_timed_out_; }

  void stop() {
    target_speed_ = 0.0f;
    target_yaw_rate_ = 0.0f;
    speed_ramp_ = 0.0f;
    spinning_ = false;
    turn_in_place_ = false;
    hold_active_ = false;
    settle_s_ = 0.0f;
    heading_.reset();
    speed_.reset();
    pwm_l_ = pwm_r_ = 0;
    out_l_ = out_r_ = 0.0f;
  }

  //----------------------------- update -----------------------------//
  /** Two stages: work out what the wheels should do, then ramp the PWM there. */
  void update(float dt, const StateEstimator &est) {
    computeTargets(dt, est);
    applyRamp(dt);
  }

private:
  void computeTargets(float dt, const StateEstimator &est) {
    // speed setpoint slew, so a step command does not slam the gearbox
    float max_step = SPEED_SLEW_MPS_S * dt;
    float d = target_speed_ - speed_ramp_;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    speed_ramp_ += d;

    if (manual_) { updateManual(dt, est); return; }

    heading_error_ = angErrDeg(target_heading_, est.headingDeg());

    if (turn_in_place_) { updateTurnInPlace(); return; }

    /* Large error: turn on the spot first, rather than driving a long arc.
     *
     * Below this threshold the robot ALREADY drives and turns at the same time -
     * the heading PID trims the wheel difference while the speed term drives
     * forward, which is an arc. Spinning only takes over when the target is so
     * far off the nose that arcing would cover a lot of ground going the wrong
     * way; at 90 degrees of error an arc travels roughly half as far again as
     * turning first and then driving straight.
     *
     * It is a parameter because the right answer depends on the space. In a
     * corridor or a car park, turning on the spot is tidier and more
     * predictable. In an open field, arcing the whole way is smoother and
     * faster - set this to 180 and the robot never stops to turn.
     */
    if (!spinning_ && fabsf(heading_error_) > spin_above_deg_ && fabsf(target_speed_) > 1e-3f) {
      spinning_ = true;
      speed_.reset();
      speed_ramp_ = 0.0f;
    } else if (spinning_ && (fabsf(heading_error_) < HEADING_SPIN_EXIT_DEG
                             || fabsf(target_speed_) < 1e-3f)) {
      // Either the heading came good, or the command went to zero. The second
      // case was missing: the spin branch below runs before the zero-speed
      // check, so `setAuto(0, heading)` - which is what a stop looks like from
      // the ROS side - left both motors driving at full spin power until the
      // target heading happened to be reached.
      spinning_ = false;
      heading_.reset();
    }

    if (spinning_) {
      // headings are compass degrees: a positive error means "turn clockwise",
      // i.e. the left wheel drives forward and the right wheel backwards
      int turn = (heading_error_ > 0.0f) ? HEADING_SPIN_PWM : -HEADING_SPIN_PWM;
      tgt_l_ =  turn;
      tgt_r_ = -turn;
      return;
    }

    if (fabsf(target_speed_) < 1e-3f) { stopOutputs(); return; }

    // common term: feed-forward from the motor model, plus a PI on the fused
    // speed ONLY when that speed is a real measurement.
    //
    // updateManual() has refused to close this loop on anything but GPS-backed
    // speed since it was measured chasing noise - PWM surging between 771 and
    // 464 on a constant command, the estimate reading negative while the robot
    // drove forward. The automatic path closed the same loop on the same
    // estimate with no guard at all, so every mission driven without a usable
    // fix had the behaviour the manual guard exists to prevent.
    float ff   = SPEED_FF_GAIN * (speed_ramp_ / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;
    const uint8_t src = est.speedSource();
    const bool speed_is_measured = (src == StateEstimator::SPEED_IMU_GPS ||
                                    src == StateEstimator::SPEED_GPS);
    float corr = 0.0f;
    if (speed_is_measured) {
      corr = speed_.compute(speed_ramp_, est.speed());
    } else {
      speed_.reset();
    }
    float base = ff + corr;
    base = clampf(base, -(float)PWM_MAX, (float)PWM_MAX);

    // differential term from the heading PID (compass error, + = turn right)
    float diff = heading_.compute_with_error(heading_error_);
    // NO sign flip when reversing, and the old `if (target_speed_ < 0) diff =
    // -diff;` here was a real inversion of this loop's own feedback. Yaw rate on
    // a differential drive is (v_right - v_left) / track, which does not change
    // sign with the direction of travel: a positive diff turns the robot
    // clockwise whether the common term is positive or negative. Negating it in
    // reverse therefore turned the heading PID into positive feedback, and the
    // error grew until the spin branch took over. updateManual() has always said
    // so in its own comment; this half disagreed with it.

    float left  = base + diff;
    float right = base - diff;

    // keep the ratio if one side saturates, so the robot does not veer
    float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > (float)PWM_MAX) {
      float k = (float)PWM_MAX / peak;
      left  *= k;
      right *= k;
    }

    tgt_l_ = applyDeadband((int)lroundf(left));
    tgt_r_ = applyDeadband((int)lroundf(right));
  }

public:
  int   pwmLeft()  const { return pwm_l_; }
  int   pwmRight() const { return pwm_r_; }
  float headingError() const { return heading_error_; }
  float targetHeading() const { return target_heading_; }
  float targetSpeed()   const { return target_speed_; }
  float targetYawRate() const { return target_yaw_rate_; }
  float rampedSpeed()   const { return speed_ramp_; }
  bool  spinning()      const { return spinning_; }
  bool  manual()        const { return manual_; }
  /** True while manual driving is holding a heading rather than steering. */
  bool  holdingHeading() const { return hold_active_; }
  /** True while the manual speed loop has a measurement worth closing on. */
  bool  speedClosed()    const { return speed_closed_; }
  float holdHeading()    const { return hold_heading_; }

  /** Open-loop speed implied by the current PWM - the estimator uses it when GPS is lost. */
  float modelSpeedMps() const {
    float duty = 0.5f * ((float)pwm_l_ + (float)pwm_r_) / (float)PWM_MAX;
    return duty * ROBOT_MAX_SPEED_MPS;
  }

  /** Heading error above which the robot turns on the spot instead of arcing.
   *  180 means it never stops to turn - it always drives and steers together. */
  void  setSpinAboveDeg(float deg) { spin_above_deg_ = deg; }
  float spinAboveDeg() const { return spin_above_deg_; }

  PIDF &headingPid() { return heading_; }
  PIDF &speedPid()   { return speed_; }

private:
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  void stopOutputs() {
    tgt_l_ = tgt_r_ = 0;
    speed_.reset();
    heading_.reset();
  }

  /**
   * Slew the applied PWM towards the target.
   *  - starting from rest the output jumps to PWM_MIN_MOVE first (breakaway
   *    torque) and ramps from there, otherwise the ramp is spent below the
   *    stiction point and the wheels just buzz
   *  - a direction reversal always ramps down through zero first
   *  - stopping uses the (faster) decel rate
   */
  void applyRamp(float dt) {
    pwm_l_ = rampAxis(out_l_, (float)tgt_l_, dt);
    pwm_r_ = rampAxis(out_r_, (float)tgt_r_, dt);
  }

  int rampAxis(float &current, float target, float dt) {
    if (accel_per_s_ <= 0.0f) { current = target; return (int)lroundf(current); }

    const bool reversing = (current > 1.0f && target < -1.0f) || (current < -1.0f && target > 1.0f);
    const float aim = reversing ? 0.0f : target;
    const bool slowing = fabsf(aim) < fabsf(current);
    const float rate = (slowing && decel_per_s_ > 0.0f) ? decel_per_s_ : accel_per_s_;

    if (fabsf(current) < 1.0f && fabsf(aim) > 1.0f) {
      current = (aim > 0.0f) ? (float)PWM_MIN_MOVE : -(float)PWM_MIN_MOVE;  // breakaway
      if (fabsf(current) > fabsf(aim)) current = aim;
    }

    const float step = rate * dt;
    const float diff = aim - current;
    current += (diff > step) ? step : (diff < -step ? -step : diff);

    if (fabsf(current) < 1.0f) current = 0.0f;
    return (int)lroundf(current);
  }

  static int applyDeadband(int pwm) {
    if (pwm == 0) return 0;
    if (abs(pwm) < PWM_MIN_MOVE) return (pwm > 0) ? PWM_MIN_MOVE : -PWM_MIN_MOVE;
    if (pwm >  PWM_MAX) return  PWM_MAX;
    if (pwm < -PWM_MAX) return -PWM_MAX;
    return pwm;
  }

  /** Rotate on the spot until the heading error is inside the tolerance, then stop. */
  void updateTurnInPlace() {
    const uint32_t now = millis();
    // The bound. A turn is a one-shot command that outlives the message asking
    // for it, so the command timeout does not apply - which left nothing at all
    // stopping a turn whose heading never reaches tolerance.
    // No "== 0 means unset" test here on purpose: millis() really is 0 for the
    // first millisecond after boot, and a sentinel that collides with a real
    // value disables the very bound it is guarding. turn_start_ms_ is always
    // assigned by setTurnTo(), and it is only read while a turn is running.
    if ((uint32_t)(now - turn_start_ms_) >= TURN_IN_PLACE_MAX_MS) {
      turn_in_place_ = false;
      turn_done_     = true;
      turn_timed_out_ = true;
      tgt_l_ = tgt_r_ = 0;
      heading_.reset();
      return;
    }
    if (fabsf(heading_error_) <= TURN_IN_PLACE_TOL_DEG) {
      if (turn_ok_ms_ == 0) turn_ok_ms_ = now;
      tgt_l_ = tgt_r_ = 0;
      if (now - turn_ok_ms_ >= TURN_IN_PLACE_SETTLE_MS) {
        turn_in_place_ = false;
        turn_done_ = true;
        heading_.reset();
      }
      return;
    }
    turn_ok_ms_ = 0;

    // proportional so it eases into the target instead of banging into it
    float mag = fabsf(heading_.compute_with_error(heading_error_));
    if (mag < (float)PWM_MIN_MOVE)   mag = (float)PWM_MIN_MOVE;
    if (mag > (float)HEADING_SPIN_PWM) mag = (float)HEADING_SPIN_PWM;

    const int turn = (heading_error_ > 0.0f) ? (int)lroundf(mag) : -(int)lroundf(mag);
    tgt_l_ =  turn;      // compass error > 0 = turn clockwise
    tgt_r_ = -turn;
  }

  /**
   * Manual driving, with the heading held while the command is straight.
   *
   * WHY: this used to be open loop end to end. The jog buttons gave a speed and
   * a turn rate, those became two PWM numbers, and nothing ever compared where
   * the robot pointed with where it was pointing a moment ago. Two motors are
   * never identical - this robot's measured diode drops differ by 0.09 V and
   * its drivers are not matched - so "forward" drove an arc and the operator
   * had to keep tapping a turn button to fight it.
   *
   * Now: commanding a turn steers exactly as before, and releasing it latches
   * the heading at that instant. While the command is straight, the heading PID
   * trims the wheel difference to keep it.
   *
   * The correction is clamped hard, because this is a trim and not a
   * controller. It has to take out a slow drift without ever fighting the
   * person holding the button - a hold that can out-pull the jog command would
   * be worse than no hold at all.
   *
   * The sign works for reverse without a special case: heading responds to the
   * wheel difference the same way whichever direction the robot is travelling.
   */
  void updateManual(float dt, const StateEstimator &est) {
    if (fabsf(speed_ramp_) < 1e-3f && fabsf(target_yaw_rate_) < 1e-3f) {
      hold_active_ = false;                       // stopped: nothing to hold
      speed_.reset();                             // no integral into the next move
      stopOutputs();
      return;
    }
    float base = (speed_ramp_ / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;

#if MANUAL_SPEED_PID
    /* Close the speed loop, so the number on the screen is what the robot does.
     *
     * The feed-forward above is a guess from PWM to speed. It is a reasonable
     * guess and a poor answer: the same duty gives one speed on a fresh battery,
     * another on a flat one, and another again up a slope or on carpet. The web
     * page has been claiming "the robot drives at exactly the speed set above"
     * the whole time, which was simply not true.
     *
     * The guard is the part that matters, and the first version of it was too
     * generous. It allowed SPEED_IMU_MODEL - the motor model corrected by
     * integrated IMU acceleration - on the grounds that the accelerometer is
     * independent of the PWM. Measured on the floor, that estimate is not good
     * enough to close a loop on: driving forward at a steady 0.18 m/s command
     * it reported anything from -0.36 to +0.34 m/s, NEGATIVE while the robot
     * was plainly moving forward, and the loop chased it - PWM surged between
     * 771 and 464 on a constant command. Integrated acceleration drifts, and at
     * these speeds the drift is larger than the signal.
     *
     * So the loop closes only on GPS-backed speed, which is an actual
     * measurement of ground movement. Without GPS the feed-forward alone is
     * what drives, which is honest: the speed is then a request rather than a
     * result, and a smooth wrong speed is far better to drive than a surging
     * one chasing a number that is noise. It also leaves the heading hold a
     * steady base to trim, which it cannot do while the PWM is jumping by 300.
     */
    const uint8_t src = est.speedSource();
    const bool speed_is_measured = (src == StateEstimator::SPEED_IMU_GPS ||
                                    src == StateEstimator::SPEED_GPS);
    if (speed_is_measured && fabsf(speed_ramp_) > MANUAL_HOLD_MIN_SPEED) {
      float scorr = speed_.compute(speed_ramp_, est.speed());
      if (scorr >  (float)MANUAL_SPEED_MAX_PWM) scorr =  (float)MANUAL_SPEED_MAX_PWM;
      if (scorr < -(float)MANUAL_SPEED_MAX_PWM) scorr = -(float)MANUAL_SPEED_MAX_PWM;
      base += scorr;
      base = clampf(base, -(float)PWM_MAX, (float)PWM_MAX);
      speed_closed_ = true;
    } else {
      speed_.reset();
      speed_closed_ = false;
    }
#endif
    // yaw rate [deg/s, clockwise positive] -> wheel speed difference -> PWM
    float dv   = (target_yaw_rate_ * 0.01745329f) * WHEEL_TRACK_M * 0.5f;
    float diff = (dv / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;

#if MANUAL_HOLD_HEADING
    const bool turning  = fabsf(target_yaw_rate_) > 1e-3f;
    const bool straight = !turning && fabsf(speed_ramp_) > MANUAL_HOLD_MIN_SPEED;
    if (!straight) {
      // Steering, or too slow to be driving. Drop the hold so the next straight
      // stretch latches the heading the operator actually left it pointing at,
      // rather than the one from before the turn.
      hold_active_ = false;
      settle_s_ = 0.0f;
    } else {
      if (!hold_active_) {
        /* Wait for the turn to be genuinely over before deciding what to hold.
         *
         * The fused heading lags the robot: the compass is eased in over
         * seconds and the IMU's own fusion has its own delay. Latching the
         * instant the turn command stopped therefore captured where the robot
         * HAD been, and the hold then steered back to it - a visible kick the
         * other way before it settled and drove on.
         *
         * A rate threshold alone does not fix it, because the estimate is still
         * catching up for a moment after the rotation has actually stopped.
         * Both conditions are needed: turning slowly enough, for long enough.
         */
        const float yaw_dps = fabsf(est.yawRate()) * 57.29578f;
        if (yaw_dps > MANUAL_HOLD_SETTLE_DPS) {
          settle_s_ = 0.0f;
        } else {
          settle_s_ += dt;
        }
        if (settle_s_ >= MANUAL_HOLD_SETTLE_S) {
          hold_heading_ = est.headingDeg();
          hold_active_  = true;
          settle_s_     = 0.0f;
          heading_.reset();        // no integral carried in from a previous hold
        }
      }
      // Until it has latched there is nothing to hold, so drive straight and
      // let the operator's last command stand.
      if (!hold_active_) {
        heading_error_ = 0.0f;
        tgt_l_ = applyDeadband((int)lroundf(base + diff));
        tgt_r_ = applyDeadband((int)lroundf(base - diff));
        return;
      }
      heading_error_ = angErrDeg(hold_heading_, est.headingDeg());
      float corr = heading_.compute_with_error(heading_error_);
      if (corr >  (float)MANUAL_HOLD_MAX_PWM) corr =  (float)MANUAL_HOLD_MAX_PWM;
      if (corr < -(float)MANUAL_HOLD_MAX_PWM) corr = -(float)MANUAL_HOLD_MAX_PWM;
      diff += corr;
    }
#endif

    float left  = base + diff;
    float right = base - diff;
    float peak  = fmaxf(fabsf(left), fabsf(right));
    if (peak > (float)PWM_MAX) { float k = (float)PWM_MAX / peak; left *= k; right *= k; }

    tgt_l_ = applyDeadband((int)lroundf(left));
    tgt_r_ = applyDeadband((int)lroundf(right));
  }

  PIDF  heading_;
  PIDF  speed_;
  float target_speed_    = 0.0f;
  float target_heading_  = 0.0f;
  float target_yaw_rate_ = 0.0f;
  float speed_ramp_      = 0.0f;
  float heading_error_   = 0.0f;
  bool  spinning_ = false;
  bool  manual_   = false;
  bool  hold_active_  = false;   // manual heading hold is latched
  bool  speed_closed_ = false;   // manual speed loop is running on a real measurement
  float spin_above_deg_ = HEADING_SPIN_ERR_DEG;
  float settle_s_ = 0.0f;        // how long the yaw rate has been near zero
  float hold_heading_ = 0.0f;    // the heading it is holding, degrees
  bool  turn_in_place_ = false;
  bool  turn_done_ = false;
  bool  turn_timed_out_ = false;
  uint32_t turn_ok_ms_ = 0;
  uint32_t turn_start_ms_ = 0;
  int   tgt_l_ = 0, tgt_r_ = 0;       // what the controller asks for
  float out_l_ = 0.0f, out_r_ = 0.0f; // what the ramp has actually applied
  int   pwm_l_ = 0, pwm_r_ = 0;       // rounded copy of out_*, sent to the driver
  float accel_per_s_ = (float)PWM_MAX / MOTOR_ACCEL_RAMP_S;
  float decel_per_s_ = (float)PWM_MAX / MOTOR_DECEL_RAMP_S;
};

#endif // DRIVE_CONTROLLER_H
