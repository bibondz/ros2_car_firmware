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
 * DriveController tests: heading hold, spin on the spot, turn-to-heading,
 * manual jog, PWM saturation and the motor acceleration ramp.
 */
#include <unity.h>
#include "Arduino.h"
#include "config.h"
#include "drive_controller.h"
#include "state_estimator.h"

/** Estimator pinned to a fixed heading/speed, so the controller is tested alone. */
static StateEstimator fixed_state(float heading_deg, float speed_mps) {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = 4;
    cfg.mag_tau_s = 0.001f;
    cfg.align_max_dps = 100000.0f;
    cfg.gps_tau_s = 0.001f;
    est.begin(cfg);

    StateEstimator::Inputs in;
    in.dt = 0.01f;
    in.imu_ok = true;
    in.mag_ok = true;
    in.mag_heading = heading_deg;
    in.gps_ok = true;
    in.gps_speed = speed_mps;
    for (int i = 0; i < 400; ++i) est.update(in);
    return est;
}

static DriveController make_drive() {
    DriveController drive;
    drive.begin();
    return drive;
}

static void spin_cycles(DriveController &drive, const StateEstimator &est, int n, float dt = 0.01f) {
    for (int i = 0; i < n; ++i) { mock_advance_ms((unsigned long)(dt * 1000.0f)); drive.update(dt, est); }
}

// ------------------------------------------------------------------- basics
void test_starts_stopped(void) {
    DriveController drive = make_drive();
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_zero_speed_keeps_motors_off(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setAuto(0.0f, 0.0f);
    spin_cycles(drive, est, 50);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_drives_forward_on_target_heading(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);
    drive.setAuto(0.25f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
    TEST_ASSERT_INT_WITHIN(120, drive.pwmLeft(), drive.pwmRight());   // roughly straight
}

void test_heading_error_steers_right(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);       // pointing north
    drive.setAuto(0.25f, 20.0f);                        // want north-east, small error
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > drive.pwmRight());   // left faster = turn clockwise
}

void test_heading_error_steers_left(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);
    drive.setAuto(0.25f, 340.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmRight() > drive.pwmLeft());
}

void test_large_error_spins_on_the_spot(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setAuto(0.25f, 120.0f);                        // 120 deg away
    spin_cycles(drive, est, 100);
    TEST_ASSERT_TRUE(drive.spinning());
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);              // counter rotating
}

void test_spin_direction_is_shortest_way(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(10.0f, 0.0f);
    drive.setAuto(0.25f, 300.0f);                        // shorter to go anticlockwise
    spin_cycles(drive, est, 100);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
    TEST_ASSERT_TRUE(drive.pwmLeft() < 0);
}

void test_pwm_never_exceeds_limit(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);        // measured speed 0, huge error
    drive.setAuto(0.42f, 0.0f);
    spin_cycles(drive, est, 500);
    TEST_ASSERT_TRUE(drive.pwmLeft() <= PWM_MAX);
    TEST_ASSERT_TRUE(drive.pwmRight() <= PWM_MAX);
    TEST_ASSERT_TRUE(drive.pwmLeft() >= -PWM_MAX);
}

// ------------------------------------------------------------ turn in place
void test_turn_to_heading_rotates(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(90.0f);
    TEST_ASSERT_TRUE(drive.turningInPlace());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);
}

void test_turn_to_heading_finishes_inside_tolerance(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(89.0f, 0.0f);       // already almost there
    drive.setTurnTo(90.0f);
    spin_cycles(drive, est, 100);                        // longer than the settle time
    TEST_ASSERT_FALSE(drive.turningInPlace());
    TEST_ASSERT_TRUE(drive.turnDone());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

void test_turn_to_heading_needs_no_speed_command(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drive.targetSpeed());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_TRUE(drive.pwmLeft() != 0);              // still rotates
}

void test_stop_cancels_turn(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(180.0f);
    drive.stop();
    TEST_ASSERT_FALSE(drive.turningInPlace());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

// ------------------------------------------------------------------ manual
void test_manual_forward(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.manual());
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_EQUAL_INT(drive.pwmLeft(), drive.pwmRight());
}

void test_manual_backward(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(-0.2f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() < 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);
}

void test_manual_turn_right(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, 45.0f);                        // clockwise
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > drive.pwmRight());
}

void test_manual_turn_left(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, -45.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmRight() > drive.pwmLeft());
}

void test_manual_zero_stops(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 100);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

// -------------------------------------------------------------- accel ramp
void test_ramp_does_not_jump_to_full_power(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 1);                          // one 10 ms tick
    TEST_ASSERT_TRUE(drive.pwmLeft() <= PWM_MIN_MOVE + 30);
}

void test_ramp_reaches_target_after_the_ramp_time(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);                        // 2 s > 0.6 s ramp
    TEST_ASSERT_TRUE(drive.pwmLeft() > PWM_MAX / 2);
}

void test_ramp_breakaway_avoids_stiction(void) {
    DriveController drive = make_drive();
    drive.setRamp(2.0f, 1.0f);                           // very slow ramp
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 1);
    TEST_ASSERT_TRUE(drive.pwmLeft() >= PWM_MIN_MOVE);   // jumped straight to breakaway
}

void test_ramp_down_to_zero(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 5);
    int mid = drive.pwmLeft();
    TEST_ASSERT_TRUE(mid > 0);                           // not an instant cut
    spin_cycles(drive, est, 200);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

void test_ramp_zero_time_is_instant(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.0f, 0.0f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > PWM_MAX / 2);
}

void test_direction_reversal_passes_through_zero(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    drive.setManual(-0.42f, 0.0f);
    bool saw_zero_or_less = false;
    for (int i = 0; i < 200; ++i) {
        spin_cycles(drive, est, 1);
        if (drive.pwmLeft() <= 0) { saw_zero_or_less = true; break; }
    }
    TEST_ASSERT_TRUE(saw_zero_or_less);
}

void test_model_speed_follows_pwm(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.0f, 0.0f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drive.modelSpeedMps());
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_TRUE(drive.modelSpeedMps() > 0.1f);
}

void test_reset_clears_outputs(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 100);
    drive.reset();
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_target_accessors(void) {
    DriveController drive = make_drive();
    drive.setAuto(0.3f, 123.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, drive.targetSpeed());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 123.0f, drive.targetHeading());
    drive.setManual(0.1f, 30.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, drive.targetYawRate());
}

void test_setauto_leaves_manual_mode(void) {
    DriveController drive = make_drive();
    drive.setManual(0.1f, 0.0f);
    TEST_ASSERT_TRUE(drive.manual());
    drive.setAuto(0.1f, 0.0f);
    TEST_ASSERT_FALSE(drive.manual());
}


/* ---------------------------------------------------------- manual hold ---
 * Manual driving was open loop end to end: a speed and a turn rate became two
 * PWM numbers and nothing ever looked at where the robot pointed. Two motors
 * are never identical, so "forward" drove an arc and the operator had to keep
 * tapping a turn button to fight it. These pin the behaviour that fixes it.
 */

void test_manual_straight_latches_the_heading(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 60);           // past the settle window
    TEST_ASSERT_TRUE(drive.holdingHeading());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, drive.holdHeading());
}

void test_manual_turning_releases_the_hold(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 60);
    TEST_ASSERT_TRUE(drive.holdingHeading());
    // A turn command is the operator steering; the hold must get out of the way.
    drive.setManual(0.2f, 30.0f);
    spin_cycles(drive, est, 2);
    TEST_ASSERT_FALSE(drive.holdingHeading());
}

void test_manual_relatches_after_a_turn(void) {
    DriveController drive = make_drive();
    StateEstimator straight_on = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, straight_on, 60);
    drive.setManual(0.2f, 30.0f);
    spin_cycles(drive, straight_on, 2);
    // Released the turn while pointing somewhere else: the hold must take the
    // NEW heading, not the one from before the turn, or letting go of a turn
    // would drag the robot back round.
    StateEstimator turned = fixed_state(120.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, turned, 60);
    TEST_ASSERT_TRUE(drive.holdingHeading());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 120.0f, drive.holdHeading());
}

void test_manual_stopped_holds_nothing(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(90.0f, 0.0f);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 5);
    TEST_ASSERT_FALSE(drive.holdingHeading());
}

void test_manual_drift_produces_a_correction(void) {
    DriveController drive = make_drive();
    StateEstimator on_course = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, on_course, 60);
    const int l_straight = drive.pwmLeft();
    const int r_straight = drive.pwmRight();
    TEST_ASSERT_EQUAL_INT(l_straight, r_straight);   // no error, no correction

    // Now the robot has drifted off the held heading. The wheels must differ.
    StateEstimator drifted = fixed_state(80.0f, 0.0f);
    spin_cycles(drive, drifted, 20);
    TEST_ASSERT_TRUE(drive.pwmLeft() != drive.pwmRight());
}

void test_manual_correction_cannot_overpower_the_operator(void) {
    DriveController drive = make_drive();
    // A large error, held long enough for any integral to wind all the way up.
    StateEstimator far_off = fixed_state(180.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, far_off, 60);
    spin_cycles(drive, far_off, 300);
    // The trim is clamped, so both wheels must still be driving FORWARD - a
    // hold that can reverse a wheel is steering, not trimming, and would fight
    // the person holding the button.
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
}


void test_manual_speed_loop_runs_on_a_measured_speed(void) {
    DriveController drive = make_drive();
    // fixed_state drives the estimator with IMU and GPS, so the speed it
    // reports is measured rather than derived from the PWM we just set.
    StateEstimator est = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 20);
    TEST_ASSERT_TRUE(drive.speedClosed());
}

void test_manual_speed_loop_refuses_to_close_on_the_motor_model(void) {
    DriveController drive = make_drive();
    // No GPS: the estimator falls back to the motor model, which is computed
    // FROM the PWM. Closing a loop on that compares the output with itself.
    // Measured on the floor, even the IMU-corrected version of it reported
    // -0.36 m/s while the robot drove forward, and the loop chased it hard
    // enough to swing the PWM by 300. The guard must refuse anything but GPS.
    StateEstimator est;
    StateEstimator::Config cfg;
    est.begin(cfg);
    StateEstimator::Inputs in;
    in.dt = 0.01f;
    in.imu_ok = false;
    in.gps_ok = false;
    in.model_mps = 0.2f;
    for (int i = 0; i < 400; ++i) est.update(in);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 20);
    TEST_ASSERT_FALSE(drive.speedClosed());
}

void test_manual_hold_works_in_reverse(void) {
    DriveController drive = make_drive();
    StateEstimator on_course = fixed_state(90.0f, 0.0f);
    drive.setManual(-0.2f, 0.0f);          // backwards, straight
    spin_cycles(drive, on_course, 60);
    TEST_ASSERT_TRUE(drive.holdingHeading());
    // Heading responds to the wheel difference the same way whichever
    // direction the robot travels, so the correction needs no sign flip -
    // unlike the auto path, where the flip is about which way the target lies.
    StateEstimator drifted = fixed_state(80.0f, 0.0f);
    spin_cycles(drive, drifted, 20);
    TEST_ASSERT_TRUE(drive.pwmLeft() != drive.pwmRight());
    TEST_ASSERT_TRUE(drive.pwmLeft() < 0);   // still reversing
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);
}


void test_auto_arcs_below_the_spin_threshold(void) {
    DriveController drive = make_drive();
    drive.setSpinAboveDeg(45.0f);
    // 30 degrees off: under the threshold, so it must drive AND steer, not stop
    // and turn. Both wheels forward, and different from each other.
    StateEstimator est = fixed_state(30.0f, 0.0f);
    drive.setAuto(0.2f, 0.0f);
    spin_cycles(drive, est, 40);
    TEST_ASSERT_FALSE(drive.spinning());
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
    TEST_ASSERT_TRUE(drive.pwmLeft() != drive.pwmRight());
}

void test_auto_spins_above_the_spin_threshold(void) {
    DriveController drive = make_drive();
    drive.setSpinAboveDeg(45.0f);
    StateEstimator est = fixed_state(120.0f, 0.0f);   // far off the nose
    drive.setAuto(0.2f, 0.0f);
    spin_cycles(drive, est, 40);
    TEST_ASSERT_TRUE(drive.spinning());
}

void test_spin_threshold_at_180_never_stops_to_turn(void) {
    DriveController drive = make_drive();
    // The open-field setting: arc the whole way round rather than ever stopping.
    drive.setSpinAboveDeg(180.0f);
    StateEstimator est = fixed_state(120.0f, 0.0f);
    drive.setAuto(0.2f, 0.0f);
    spin_cycles(drive, est, 40);
    TEST_ASSERT_FALSE(drive.spinning());
}


void test_hold_waits_for_the_turn_to_settle_before_latching(void) {
    DriveController drive = make_drive();
    // Still rotating: the fused heading lags the robot, so latching now would
    // capture where it HAD been and the hold would steer back to it - the kick
    // reported from the robot as "it turns back a little, then goes forward".
    StateEstimator turning = fixed_state(90.0f, 0.0f);
    StateEstimator::Inputs spinning = StateEstimator::Inputs();
    (void)spinning;
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, turning, 5);        // well inside the settle window
    TEST_ASSERT_FALSE(drive.holdingHeading());
}

void test_hold_does_not_steer_while_it_is_still_settling(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(90.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 5);
    // Nothing latched yet, so both wheels must be driving equally - no
    // correction toward a heading it has not chosen.
    TEST_ASSERT_FALSE(drive.holdingHeading());
    TEST_ASSERT_EQUAL_INT(drive.pwmLeft(), drive.pwmRight());
}


/**
 * Steering must not change sign when the robot drives backwards.
 *
 * Yaw rate on a differential drive is (v_right - v_left) / track, which does
 * not depend on the sign of the common velocity - so the wheel difference that
 * corrects a given heading error is the same going forwards and backwards. The
 * automatic path used to negate it in reverse, turning its own heading PID into
 * positive feedback; the error grew until the spin branch took over. All three
 * reviewers in the audit found this one independently.
 */
void test_auto_steering_keeps_its_sign_in_reverse(void) {
    StateEstimator est = fixed_state(0.0f, 0.0f);   // pointing north

    DriveController fwd = make_drive();
    fwd.setAuto(0.20f, 20.0f);                      // target 20 deg clockwise
    spin_cycles(fwd, est, 30);
    const int fwd_diff = fwd.pwmLeft() - fwd.pwmRight();

    DriveController rev = make_drive();
    rev.setAuto(-0.20f, 20.0f);                     // same error, driving backwards
    spin_cycles(rev, est, 30);
    const int rev_diff = rev.pwmLeft() - rev.pwmRight();

    TEST_ASSERT_TRUE(fwd_diff > 0);                 // left faster: turns clockwise
    TEST_ASSERT_TRUE_MESSAGE(rev_diff > 0,
        "reverse must correct the same way; negating it is positive feedback");
}

/** A zero-speed command ends an active spin instead of being ignored. */
void test_zero_speed_command_stops_an_active_spin(void) {
    StateEstimator est = fixed_state(0.0f, 0.0f);
    DriveController drive = make_drive();

    drive.setAuto(0.20f, 120.0f);                   // way off: it spins on the spot
    spin_cycles(drive, est, 30);
    TEST_ASSERT_TRUE(drive.spinning());

    drive.setAuto(0.0f, 120.0f);                    // what a stop looks like from ROS
    spin_cycles(drive, est, 60);
    TEST_ASSERT_FALSE_MESSAGE(drive.spinning(),
        "the spin branch ran before the zero-speed check, so a stop was ignored");
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

/** reset() means stop: a turn-to-heading must not resume after it. */
void test_reset_abandons_a_turn_in_place(void) {
    StateEstimator est = fixed_state(0.0f, 0.0f);
    DriveController drive = make_drive();

    drive.setTurnTo(90.0f);
    spin_cycles(drive, est, 10);
    TEST_ASSERT_TRUE(drive.turningInPlace());

    drive.reset();
    TEST_ASSERT_FALSE(drive.turningInPlace());
    spin_cycles(drive, est, 20);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

/** A turn that can never reach its heading still ends. */
void test_a_turn_that_never_arrives_gives_up(void) {
    StateEstimator est = fixed_state(0.0f, 0.0f);   // heading never changes
    DriveController drive = make_drive();

    drive.setTurnTo(90.0f);                         // it will never get there
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE_MESSAGE(drive.turningInPlace(),
        "still well inside the time limit, so it should still be trying");

    mock_advance_ms(TURN_IN_PLACE_MAX_MS);
    drive.update(0.01f, est);
    TEST_ASSERT_FALSE_MESSAGE(drive.turningInPlace(),
        "a one-shot turn has no command timeout, so this bound is all there is");
    TEST_ASSERT_TRUE(drive.turnTimedOut());

    // The PWM comes down the deceleration ramp rather than snapping to zero -
    // that ramp is deliberate, it is what protects the gearbox - so give it the
    // time it is entitled to and then require a full stop.
    spin_cycles(drive, est, 100);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

int run_drive_tests(void) {
    RUN_TEST(test_starts_stopped);
    RUN_TEST(test_zero_speed_keeps_motors_off);
    RUN_TEST(test_drives_forward_on_target_heading);
    RUN_TEST(test_heading_error_steers_right);
    RUN_TEST(test_heading_error_steers_left);
    RUN_TEST(test_large_error_spins_on_the_spot);
    RUN_TEST(test_spin_direction_is_shortest_way);
    RUN_TEST(test_pwm_never_exceeds_limit);
    RUN_TEST(test_turn_to_heading_rotates);
    RUN_TEST(test_turn_to_heading_finishes_inside_tolerance);
    RUN_TEST(test_turn_to_heading_needs_no_speed_command);
    RUN_TEST(test_stop_cancels_turn);
    RUN_TEST(test_manual_forward);
    RUN_TEST(test_manual_backward);
    RUN_TEST(test_manual_turn_right);
    RUN_TEST(test_manual_turn_left);
    RUN_TEST(test_manual_zero_stops);
    RUN_TEST(test_manual_straight_latches_the_heading);
    RUN_TEST(test_hold_waits_for_the_turn_to_settle_before_latching);
    RUN_TEST(test_hold_does_not_steer_while_it_is_still_settling);
    RUN_TEST(test_manual_turning_releases_the_hold);
    RUN_TEST(test_manual_relatches_after_a_turn);
    RUN_TEST(test_manual_stopped_holds_nothing);
    RUN_TEST(test_manual_drift_produces_a_correction);
    RUN_TEST(test_manual_correction_cannot_overpower_the_operator);
    RUN_TEST(test_manual_speed_loop_runs_on_a_measured_speed);
    RUN_TEST(test_manual_speed_loop_refuses_to_close_on_the_motor_model);
    RUN_TEST(test_manual_hold_works_in_reverse);
    RUN_TEST(test_auto_arcs_below_the_spin_threshold);
    RUN_TEST(test_auto_spins_above_the_spin_threshold);
    RUN_TEST(test_spin_threshold_at_180_never_stops_to_turn);
    RUN_TEST(test_ramp_does_not_jump_to_full_power);
    RUN_TEST(test_ramp_reaches_target_after_the_ramp_time);
    RUN_TEST(test_ramp_breakaway_avoids_stiction);
    RUN_TEST(test_ramp_down_to_zero);
    RUN_TEST(test_ramp_zero_time_is_instant);
    RUN_TEST(test_direction_reversal_passes_through_zero);
    RUN_TEST(test_model_speed_follows_pwm);
    RUN_TEST(test_reset_clears_outputs);
    RUN_TEST(test_target_accessors);
    RUN_TEST(test_setauto_leaves_manual_mode);
    RUN_TEST(test_auto_steering_keeps_its_sign_in_reverse);
    RUN_TEST(test_zero_speed_command_stops_an_active_spin);
    RUN_TEST(test_reset_abandons_a_turn_in_place);
    RUN_TEST(test_a_turn_that_never_arrives_gives_up);
    return 0;
}
