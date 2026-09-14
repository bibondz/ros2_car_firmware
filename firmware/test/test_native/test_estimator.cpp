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
 * StateEstimator tests: heading arbitration, sensor fallbacks, speed fusion,
 * wheel-speed derivation and odometry.
 */
#include <unity.h>
#include "Arduino.h"
#include "state_estimator.h"

static StateEstimator make(uint8_t source = 4) {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = source;
    cfg.align_min_mps = 0.25f;
    cfg.align_tau_s = 1.0f;
    cfg.align_max_dps = 180.0f;      // no rate limit in the tests
    cfg.mag_tau_s = 1.0f;
    cfg.mag_still_mps = 0.10f;
    cfg.gps_tau_s = 0.5f;
    cfg.model_tau_s = 0.5f;
    cfg.track_m = 0.30f;
    cfg.wheel_diameter_m = 0.081f;
    est.begin(cfg);
    return est;
}

static StateEstimator::Inputs base_inputs() {
    StateEstimator::Inputs in;
    in.dt = 0.01f;
    in.imu_ok = true;
    in.imu_yaw = 0.0f;
    in.yaw_rate = 0.0f;
    in.acc_fwd = 0.0f;
    return in;
}

static void run(StateEstimator &est, StateEstimator::Inputs in, int cycles) {
    for (int i = 0; i < cycles; ++i) est.update(in);
}

// ------------------------------------------------------------ angle helpers
void test_wrap360_positive(void) { TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, wrap360f(370.0f)); }
void test_wrap360_negative(void) { TEST_ASSERT_FLOAT_WITHIN(0.001f, 350.0f, wrap360f(-10.0f)); }
void test_ang_err_wraps_short_way(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, angErrDeg(350.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, angErrDeg(10.0f, 350.0f));
}
void test_wrap_pi_negative(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -(float)M_PI / 2.0f, wrapPif(3.0f * (float)M_PI / 2.0f));
}

// --------------------------------------------------------------- heading ref
void test_compass_gives_heading_while_standing_still(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, est.headingDeg());
    TEST_ASSERT_EQUAL(StateEstimator::REF_MAG, est.headingRef());
}

void test_gps_course_wins_while_moving(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;  in.mag_heading = 90.0f;
    in.gps_ok = true;  in.gps_course = 270.0f; in.gps_course_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    TEST_ASSERT_EQUAL(StateEstimator::REF_GPS, est.headingRef());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 270.0f, est.headingDeg());
}

void test_falls_back_to_imu_magnetometer_when_compass_dies(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = false;                       // QMC5883L gone
    in.imu_mag_ok = true; in.imu_mag_heading = 45.0f;
    run(est, in, 800);
    TEST_ASSERT_EQUAL(StateEstimator::REF_IMU_MAG, est.headingRef());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 45.0f, est.headingDeg());
}

void test_no_absolute_source_is_reported(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    run(est, in, 10);
    TEST_ASSERT_EQUAL(StateEstimator::REF_NONE, est.headingRef());
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_ABS_HEADING);
}

void test_health_flags_report_missing_sensors(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    run(est, in, 5);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_IMU);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_GPS);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_COMPASS);
}

/**
 * A fix too rough to take a SPEED from is still a fix.
 *
 * This is the bug it holds shut. `gps_ok` is a strict speed gate - 6 satellites
 * and HDOP under 1.5 - because a marginal fix reports plausible-looking nonsense
 * ground speeds. That same flag was also raising HEALTH_NO_GPS, so on a real,
 * usable 7-satellite fix at HDOP 1.8 the web UI drew the GPS as a missing
 * sensor while the robot was navigating on it.
 *
 * They are two questions and the estimator now takes two answers.
 */
void test_a_fix_too_rough_for_speed_is_still_a_working_sensor(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok     = false;     // 7 sats at HDOP 1.8: fails the speed gate
    in.gps_fix_ok = true;      // ... but the receiver has a position
    run(est, in, 5);
    TEST_ASSERT_FALSE(est.health() & StateEstimator::HEALTH_NO_GPS);
}

void test_no_position_at_all_is_reported_as_a_missing_gps(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok     = false;
    in.gps_fix_ok = false;
    run(est, in, 5);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_GPS);
}

/** The speed path must not have been loosened by the split. */
void test_a_rough_fix_is_still_refused_as_a_speed_source(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok     = false;       // the speed gate said no
    in.gps_fix_ok = true;        // even though there is a position
    in.gps_speed  = 2.0f;        // and the receiver is claiming this
    in.model_mps  = 0.0f;
    run(est, in, 50);
    TEST_ASSERT_TRUE(est.speed() < 0.2f);
}

void test_gyro_carries_heading_when_everything_absolute_is_gone(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 0.0f;
    run(est, in, 500);                       // learn 0 deg from the compass
    in.mag_ok = false;
    in.yaw_rate = -0.5f;                     // CCW negative -> compass heading grows
    in.imu_yaw = 0.0f;
    for (int i = 0; i < 100; ++i) { in.imu_yaw += 0.5f * 57.29578f * 0.01f; est.update(in); }
    TEST_ASSERT_TRUE(est.headingDeg() > 20.0f);   // it kept turning with the gyro
}

void test_imu_dead_uses_gps_course_directly(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    in.gps_ok = true; in.gps_course = 123.0f; in.gps_course_ok = true; in.gps_speed = 0.6f;
    run(est, in, 300);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 123.0f, est.headingDeg());
    TEST_ASSERT_EQUAL(StateEstimator::REF_GPS, est.headingRef());
}

void test_imu_and_gps_dead_uses_compass_directly(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    in.mag_ok = true; in.mag_heading = 200.0f;
    run(est, in, 50);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, est.headingDeg());
}

void test_mode0_uses_imu_yaw_as_absolute(void) {
    StateEstimator est = make(0);
    StateEstimator::Inputs in = base_inputs();
    in.imu_yaw = 137.0f;
    run(est, in, 5);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 137.0f, est.headingDeg());
}

void test_mode2_gps_only(void) {
    StateEstimator est = make(2);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_course = 80.0f; in.gps_course_ok = true; in.gps_speed = 0.8f;
    run(est, in, 100);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 80.0f, est.headingDeg());
}

void test_compass_ignored_while_driving_fast(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.6f; in.gps_course = 10.0f; in.gps_course_ok = true;
    in.mag_ok = true; in.mag_heading = 300.0f;      // disagrees on purpose
    run(est, in, 600);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 10.0f, est.headingDeg());
}

// ------------------------------------------------------------------- speed
void test_speed_follows_gps(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.40f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.40f, est.speed());
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_IMU_GPS, est.speedSource());
}

/**
 * A robot whose contactor is open is not driving itself, whatever the back-EMF
 * model says.
 *
 * With the relay open the motor rails sit near zero volts and about a milliamp
 * of leakage, and the model turns that noise into a speed. Found on hardware:
 * latched in e-stop, wheels physically still, estimator reporting 0.127 m/s.
 * That was not merely a wrong number on a screen - it blocked Wi-Fi OTA, since
 * the updater refuses to flash a robot it believes is moving.
 */
void test_open_contactor_does_not_invent_speed(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = false;
    in.model_mps = 0.35f;            // the noise dead rails produce
    in.motors_powered = false;
    run(est, in, 1500);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, est.speed());
}

/** ...and the same reading is still believed once the rails are actually live,
 *  or the robot would have no speed fallback at all when GPS is unavailable. */
void test_powered_contactor_still_trusts_the_motor_model(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = false;
    in.model_mps = 0.35f;
    in.motors_powered = true;
    run(est, in, 1500);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.35f, est.speed());
}

void test_speed_falls_back_to_model_without_gps(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = false; in.model_mps = 0.30f;
    // model_tau_s is 2.5 s, so 500 cycles at dt=0.01 is only 2 time constants
    // and a first-order lag has reached 86.5% - 0.259, outside the +-0.03 the
    // assertion allows. Run 4 time constants, where it is at 98.2%.
    run(est, in, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.30f, est.speed());
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_IMU_MODEL, est.speedSource());
}

void test_speed_source_without_imu(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false; in.gps_ok = true; in.gps_speed = 0.2f;
    run(est, in, 100);
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_GPS, est.speedSource());
}

void test_speed_is_clamped(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.acc_fwd = 50.0f;                      // absurd acceleration
    run(est, in, 1000);
    TEST_ASSERT_TRUE(est.speed() <= 1.51f);
}

void test_small_speed_is_squelched(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.001f;
    run(est, in, 200);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, est.speed());
}

void test_acceleration_deadband(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.acc_fwd = 0.05f;                      // below the 0.08 deadband
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
}

// ------------------------------------------------------- wheels and odometry
void test_wheel_speeds_equal_when_driving_straight(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.3f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, est.wheelLeftMps(), est.wheelRightMps());
}

void test_wheel_speeds_split_when_turning(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.3f;
    in.yaw_rate = 1.0f;                      // 1 rad/s counter-clockwise = turning left
    run(est, in, 500);
    TEST_ASSERT_TRUE(est.wheelRightMps() > est.wheelLeftMps());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.30f, est.wheelRightMps() - est.wheelLeftMps());  // w * track
}

void test_rpm_conversion(void) {
    StateEstimator est = make(4);
    // 0.2545 m per revolution (81 mm wheel) -> 1 m/s is about 236 rpm
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 235.8f, est.mpsToRpm(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.mpsToRpm(0.0f));
}

void test_odometry_moves_north_when_heading_north(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 0.0f;         // north
    in.gps_ok = true; in.gps_speed = 0.0f;
    run(est, in, 500);
    in.gps_speed = 1.0f;
    run(est, in, 500);                               // ~5 s at up to 1 m/s
    TEST_ASSERT_TRUE(est.y() > 1.0f);                // north = +y in ENU
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, est.x());
}

void test_odometry_moves_east_when_heading_east(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f;        // east
    in.gps_ok = true; in.gps_speed = 0.0f;
    run(est, in, 500);
    in.gps_speed = 1.0f;
    run(est, in, 500);
    TEST_ASSERT_TRUE(est.x() > 1.0f);                // east = +x
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, est.y());
}

void test_heading_enu_conversion(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, est.headingEnuRad());   // east = 0 rad in ENU
}

void test_reset_clears_everything(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f; in.gps_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    est.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.x());
    TEST_ASSERT_EQUAL(StateEstimator::REF_NONE, est.headingRef());
}

void test_reset_odom_keeps_heading(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f; in.gps_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    float heading = est.headingDeg();
    est.resetOdom();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, heading, est.headingDeg());
}

void test_zero_dt_is_ignored(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.dt = 0.0f;
    in.gps_ok = true; in.gps_speed = 1.0f;
    run(est, in, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
}


/**
 * A stationary robot must not be told it is doing 7 m/s.
 *
 * MEASURED ON THE REAL ROBOT, standing still indoors: the receiver reported
 * ground speeds of 1.6, 3.3, 4.8 and 7.28 m/s on a 3-satellite fix with HDOP
 * 1.3. Every one of those passed the old test - fix quality above zero and at
 * least one satellite - and the fused speed sat pinned at its 0.36 m/s ceiling
 * while the odometry wandered a hundred metres across a room where nothing had
 * moved. With a motor driver enabled the controller would have acted on it.
 *
 * The test that catches this is not about satellites at all: the robot cannot
 * outrun itself, so a reading far above its own maximum is wrong by definition.
 */
void test_gps_speed_the_robot_cannot_reach_is_rejected(void) {
    StateEstimator est;
    StateEstimator::Config &cfg = est.config();
    cfg.max_speed_mps = 0.36f;               // this robot's real ceiling

    // Feed the exact nonsense the receiver produced, with gps_ok already false
    // because the plausibility gate in main.cpp refused it.
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = false;                       // refused: 7.28 >> 2 x 0.36
    in.gps_speed = 7.28f;
    in.motors_powered = false;               // e-stop pressed, rails dead
    in.model_mps = 0.0f;
    for (int i = 0; i < 500; ++i) est.update(in);

    // With the bad reading refused, the only reference left is zero, and that
    // is where a stationary robot belongs.
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, est.speed());
}

/** And the same reading, wrongly accepted, is what the bug looked like. */
void test_believing_a_bad_gps_speed_pins_the_estimate(void) {
    StateEstimator est;
    StateEstimator::Config &cfg = est.config();
    cfg.max_speed_mps = 0.36f;

    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true;                        // the old, too-weak gate said yes
    in.gps_speed = 7.28f;
    for (int i = 0; i < 500; ++i) est.update(in);

    // Pinned at the ceiling - exactly what the robot reported. This test exists
    // to show the failure is real and to fail loudly if the gate is ever
    // loosened back to what it was.
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.36f, est.speed());
}



/* ------------------------------------------- compass / IMU cross-check ---
 * Two sensors that both claim to know north, refereed by one that cannot be
 * lied to. Measured on the robot: the compass moved 9.5 degrees in 0.3 s while
 * the gyroscope reported about 1 deg/s, and applied blind that step went
 * straight into the heading.
 */

void test_compass_and_imu_agreeing_is_believed(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    in.imu_yaw = 0.0f;
    run(est, in, 500);                       // settles onto the compass
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, est.headingDeg());
    TEST_ASSERT_FALSE(est.magRejecting());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, est.headingInnovDeg());
}

void test_a_compass_step_the_imu_does_not_corroborate_is_refused(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    in.imu_yaw = 0.0f;
    run(est, in, 500);
    const float settled = est.headingDeg();

    // The compass jumps 60 degrees while the IMU says the robot has not moved.
    // That is a disturbance, not a turn, and the heading must not follow it.
    in.mag_heading = 150.0f;
    run(est, in, 50);                        // half a second, well under recover
    TEST_ASSERT_TRUE(est.magRejecting());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, settled, est.headingDeg());
}

void test_a_real_turn_is_not_refused(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    in.imu_yaw = 0.0f;
    run(est, in, 500);

    // The robot really turns: the IMU's own yaw moves with the compass, so the
    // two still agree and nothing should be rejected.
    in.imu_yaw = 60.0f;
    in.mag_heading = 150.0f;
    run(est, in, 50);
    TEST_ASSERT_FALSE(est.magRejecting());
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 150.0f, est.headingDeg());
}

void test_a_lasting_disagreement_is_eventually_accepted(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    in.imu_yaw = 0.0f;
    run(est, in, 500);

    // A gate that can never give up locks a correct sensor out for ever: if the
    // gyro has genuinely drifted, every compass reading looks like a
    // disturbance and nothing can pull it back. A disagreement that outlasts
    // agree_recover_s has to be believed.
    in.mag_heading = 150.0f;
    run(est, in, 3000);                      // 30 s, well past the 8 s recovery
    TEST_ASSERT_FALSE(est.magRejecting());
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 150.0f, est.headingDeg());
}

void test_the_first_alignment_is_never_gated(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 200.0f;                 // far from the arbitrary zero
    in.imu_yaw = 0.0f;
    // Before the first alignment there is nothing to disagree WITH, so the
    // sample that establishes north must always go in - otherwise the robot
    // never aligns at all.
    run(est, in, 5);
    TEST_ASSERT_FALSE(est.magRejecting());
}

void test_no_imu_means_no_referee_so_the_compass_is_believed(void) {
    StateEstimator est = make(3);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    in.imu_yaw = 0.0f;
    run(est, in, 500);
    // IMU dies. There is nothing left to check the compass against, and a
    // compass is far better than nothing - refusing it here would leave the
    // robot with no absolute heading at all.
    in.imu_ok = false;
    in.mag_heading = 150.0f;
    run(est, in, 50);
    TEST_ASSERT_FALSE(est.magRejecting());
}

int run_estimator_tests(void) {
    RUN_TEST(test_compass_and_imu_agreeing_is_believed);
    RUN_TEST(test_a_compass_step_the_imu_does_not_corroborate_is_refused);
    RUN_TEST(test_a_real_turn_is_not_refused);
    RUN_TEST(test_a_lasting_disagreement_is_eventually_accepted);
    RUN_TEST(test_the_first_alignment_is_never_gated);
    RUN_TEST(test_no_imu_means_no_referee_so_the_compass_is_believed);
    RUN_TEST(test_gps_speed_the_robot_cannot_reach_is_rejected);
    RUN_TEST(test_believing_a_bad_gps_speed_pins_the_estimate);
    RUN_TEST(test_wrap360_positive);
    RUN_TEST(test_wrap360_negative);
    RUN_TEST(test_ang_err_wraps_short_way);
    RUN_TEST(test_wrap_pi_negative);
    RUN_TEST(test_compass_gives_heading_while_standing_still);
    RUN_TEST(test_gps_course_wins_while_moving);
    RUN_TEST(test_falls_back_to_imu_magnetometer_when_compass_dies);
    RUN_TEST(test_no_absolute_source_is_reported);
    RUN_TEST(test_health_flags_report_missing_sensors);
    RUN_TEST(test_a_fix_too_rough_for_speed_is_still_a_working_sensor);
    RUN_TEST(test_no_position_at_all_is_reported_as_a_missing_gps);
    RUN_TEST(test_a_rough_fix_is_still_refused_as_a_speed_source);
    RUN_TEST(test_gyro_carries_heading_when_everything_absolute_is_gone);
    RUN_TEST(test_imu_dead_uses_gps_course_directly);
    RUN_TEST(test_imu_and_gps_dead_uses_compass_directly);
    RUN_TEST(test_mode0_uses_imu_yaw_as_absolute);
    RUN_TEST(test_mode2_gps_only);
    RUN_TEST(test_compass_ignored_while_driving_fast);
    RUN_TEST(test_speed_follows_gps);
    RUN_TEST(test_speed_falls_back_to_model_without_gps);
    RUN_TEST(test_open_contactor_does_not_invent_speed);
    RUN_TEST(test_powered_contactor_still_trusts_the_motor_model);
    RUN_TEST(test_speed_source_without_imu);
    RUN_TEST(test_speed_is_clamped);
    RUN_TEST(test_small_speed_is_squelched);
    RUN_TEST(test_acceleration_deadband);
    RUN_TEST(test_wheel_speeds_equal_when_driving_straight);
    RUN_TEST(test_wheel_speeds_split_when_turning);
    RUN_TEST(test_rpm_conversion);
    RUN_TEST(test_odometry_moves_north_when_heading_north);
    RUN_TEST(test_odometry_moves_east_when_heading_east);
    RUN_TEST(test_heading_enu_conversion);
    RUN_TEST(test_reset_clears_everything);
    RUN_TEST(test_reset_odom_keeps_heading);
    RUN_TEST(test_zero_dt_is_ignored);
    return 0;
}
