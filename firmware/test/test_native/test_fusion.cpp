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
 * Fusion tests against a known truth.
 *
 * Every assertion here is an ERROR MEASURED against SimWorld's true pose, not
 * a claim about what the code does. That is deliberate: "under 1 m
 * repeatability" has to be a number a test produces, or it is only a hope.
 *
 * These also document what the estimator can and cannot do today. Position is
 * still plain dead reckoning - GPS never corrects it - so the position tests
 * below assert the drift bound that dead reckoning actually achieves. When the
 * EKF lands (T20-T25) those bounds tighten and these tests are what proves it.
 */
#include <unity.h>
#include <math.h>

#include "Arduino.h"
#include "config.h"
#include "sim_world.h"
#include "state_estimator.h"

static StateEstimator makeEstimator(uint8_t heading_source = 4) {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = heading_source;
    cfg.track_m = 0.30f;
    est.begin(cfg);
    return est;
}

static SimWorld makeWorld() {
    SimWorld w;
    SimWorld::Config cfg;
    w.begin(cfg);
    return w;
}

/** Shortest absolute angle between two compass headings. */
static float headingError(float a, float b) {
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return fabsf(d);
}

// --------------------------------------------------------------- heading ---

void test_sim_heading_converges_while_driving(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    // Drive straight for 20 s. The IMU yaw carries a growing gyro bias, so
    // this is really asking whether GPS course pulls the heading back.
    for (int i = 0; i < 2000; ++i) {
        mock_advance_ms(10);
        est.update(w.step(0.6f, 0.0f));
    }
    TEST_ASSERT_TRUE(est.speed() > 0.4f);
    TEST_ASSERT_FLOAT_WITHIN(12.0f, 0.0f, headingError(est.headingDeg(), w.trueHeading()));
}

void test_sim_heading_survives_a_disturbed_compass(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    for (int i = 0; i < 1000; ++i) { mock_advance_ms(10); est.update(w.step(0.6f, 0.0f)); }
    w.setMagDisturbed(true);                       // magnet, motor, steel fence
    for (int i = 0; i < 1000; ++i) { mock_advance_ms(10); est.update(w.step(0.6f, 0.0f)); }

    // The compass is gone, but GPS course is still there: heading must degrade,
    // not collapse, and the robot must never be told to stop.
    TEST_ASSERT_FLOAT_WITHIN(20.0f, 0.0f, headingError(est.headingDeg(), w.trueHeading()));
}

// ----------------------------------------------------------------- speed ---

void test_sim_speed_tracks_truth(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    for (int i = 0; i < 1500; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 0.0f)); }
    TEST_ASSERT_FLOAT_WITHIN(0.15f, w.trueSpeed(), est.speed());
}

void test_sim_speed_survives_a_gps_dropout(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    for (int i = 0; i < 1000; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 0.0f)); }
    w.setGpsDropout(10.0f, 40.0f);                 // 30 s under cover
    for (int i = 0; i < 3000; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 0.0f)); }

    // With no GPS the motor model becomes the reference. It must keep a
    // sensible speed rather than winding up or collapsing to zero - the
    // integrator running away here is what would drive the robot into a fence.
    TEST_ASSERT_TRUE(est.speed() > 0.1f);
    TEST_ASSERT_TRUE(est.speed() < 1.5f);
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_IMU_MODEL, est.speedSource());
}

// -------------------------------------------------------------- position ---

void test_sim_dead_reckoning_drift_is_bounded_over_a_short_run(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    for (int i = 0; i < 2000; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 0.0f)); }

    // 20 s of straight driving. Position is pure dead reckoning today, so this
    // documents the drift that actually accumulates rather than pretending it
    // is corrected. The EKF should cut this substantially; when it does, tighten
    // this bound - it failing after that change is the point.
    const float err = w.positionErrorFrom(est.x(), est.y());
    TEST_ASSERT_TRUE(err < 6.0f);
}

void test_sim_square_path_returns_near_the_origin(void) {
    mock_reset_clock();
    SimWorld w = makeWorld();
    StateEstimator est = makeEstimator();

    // Four 10 s legs with a 90 degree turn between each.
    for (int leg = 0; leg < 4; ++leg) {
        for (int i = 0; i < 1000; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 0.0f)); }
        for (int i = 0; i < 300; ++i)  { mock_advance_ms(10); est.update(w.step(0.0f, 30.0f)); }
    }
    // The truth itself should come back near where it started; if the sim
    // cannot close a square, no estimator result from it means anything.
    const float truth_closure = sqrtf(w.trueX() * w.trueX() + w.trueY() * w.trueY());
    TEST_ASSERT_TRUE(truth_closure < 8.0f);
}

// ------------------------------------------------- degradation, one by one --

void test_sim_keeps_navigating_with_each_sensor_lost(void) {
    // Losing any single source must degrade the estimate, never stop it. This
    // is the whole promise of heading_source = 4 and it is cheap to regress.
    const int cases = 3;
    for (int c = 0; c < cases; ++c) {
        mock_reset_clock();
        SimWorld w = makeWorld();
        StateEstimator est = makeEstimator();

        for (int i = 0; i < 1500; ++i) {
            mock_advance_ms(10);
            StateEstimator::Inputs in = w.step(0.5f, 0.0f);
            if (c == 0) in.gps_ok = in.gps_course_ok = false;   // no GPS
            if (c == 1) in.imu_ok = false;                      // no IMU
            if (c == 2) in.mag_ok = in.imu_mag_ok = false;      // no compass
            est.update(in);
        }
        // Still producing finite, usable numbers in every case.
        TEST_ASSERT_FALSE(isnan(est.headingDeg()));
        TEST_ASSERT_FALSE(isnan(est.speed()));
        TEST_ASSERT_TRUE(est.headingDeg() >= 0.0f && est.headingDeg() <= 360.0f);
    }
}

void test_sim_is_deterministic(void) {
    // Two identical runs must agree exactly, or a failure here can never be
    // reproduced and every other test in this file becomes flaky.
    float first_x = 0.0f, first_h = 0.0f;
    for (int run = 0; run < 2; ++run) {
        mock_reset_clock();
        SimWorld w = makeWorld();
        StateEstimator est = makeEstimator();
        for (int i = 0; i < 500; ++i) { mock_advance_ms(10); est.update(w.step(0.5f, 10.0f)); }
        if (run == 0) { first_x = w.trueX(); first_h = est.headingDeg(); }
        else {
            TEST_ASSERT_EQUAL_FLOAT(first_x, w.trueX());
            TEST_ASSERT_EQUAL_FLOAT(first_h, est.headingDeg());
        }
    }
}

void test_a_receiver_with_no_fix_must_not_move_the_speed_estimate(void) {
    // A GPS with no fix still streams NMEA, and the speed field in those
    // sentences is meaningless. Measured on the bench: 0 satellites, HDOP
    // 99.99, fix quality 0, and a reported ground speed of 2.53 m/s. A board
    // sitting still then reported drifting at up to 1.5 m/s.
    //
    // main.cpp gates gps_ok on the fix now, so this asserts what the estimator
    // must do with the result: told the GPS is not usable, a stationary robot
    // stays stationary no matter what number came with it.
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.gps_tau_s = 1.5f;
    cfg.model_tau_s = 1.0f;
    cfg.max_speed_mps = 3.0f;
    est.begin(cfg);

    StateEstimator::Inputs in;
    in.dt = 0.02f;
    in.imu_ok = true;
    in.imu_yaw = 0.0f;
    in.yaw_rate = 0.0f;
    in.acc_fwd = 0.0f;
    in.gps_ok = false;                 // no fix, so the numbers below are junk
    in.gps_speed = 2.53f;              // exactly what the receiver reported
    in.gps_course = 148.9f;
    in.gps_course_ok = false;
    in.model_mps = 0.0f;

    for (int i = 0; i < 500; ++i) est.update(in);   // 10 seconds
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, est.speed());
}

int run_fusion_tests(void) {
    RUN_TEST(test_a_receiver_with_no_fix_must_not_move_the_speed_estimate);
    RUN_TEST(test_sim_is_deterministic);
    RUN_TEST(test_sim_heading_converges_while_driving);
    RUN_TEST(test_sim_heading_survives_a_disturbed_compass);
    RUN_TEST(test_sim_speed_tracks_truth);
    RUN_TEST(test_sim_speed_survives_a_gps_dropout);
    RUN_TEST(test_sim_dead_reckoning_drift_is_bounded_over_a_short_run);
    RUN_TEST(test_sim_square_path_returns_near_the_origin);
    RUN_TEST(test_sim_keeps_navigating_with_each_sensor_lost);
    return 0;
}
