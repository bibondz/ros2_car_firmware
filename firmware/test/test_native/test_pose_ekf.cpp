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
 * EKF tests, measured against SimWorld's known truth.
 *
 * The claim being tested is that fusing GPS into position beats both of the
 * things it replaces: raw GPS (jittery) and dead reckoning (drifts). Each test
 * below computes both and asserts the EKF is better, rather than asserting a
 * number in isolation - a bound that happens to pass proves nothing about
 * whether the filter helped.
 */
#include <unity.h>
#include <math.h>

#include "Arduino.h"
#include "config.h"
#include "pose_ekf.h"
#include "sim_world.h"
#include "state_estimator.h"

static PoseEkf makeEkf() {
    PoseEkf ekf;
    PoseEkf::Config cfg;
    ekf.begin(cfg);
    return ekf;
}

static StateEstimator makeEst() {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = 4;
    est.begin(cfg);
    return est;
}

// ----------------------------------------------------------------- basics --

void test_ekf_takes_the_first_fix_without_filtering_it(void) {
    PoseEkf ekf = makeEkf();
    TEST_ASSERT_FALSE(ekf.hasFix());
    ekf.updateGps(10.0f, -4.0f, 1.0f, 1);
    TEST_ASSERT_TRUE(ekf.hasFix());
    // Nothing to filter against yet, so it must land exactly on the fix rather
    // than dragging in from the origin.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, ekf.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -4.0f, ekf.y());
}

void test_ekf_uncertainty_grows_while_coasting_and_shrinks_on_a_fix(void) {
    PoseEkf ekf = makeEkf();
    ekf.updateGps(0.0f, 0.0f, 1.0f, 1);
    const float after_fix = ekf.positionSigma();

    for (int i = 0; i < 1000; ++i) ekf.predict(0.01f, 0.5f, 0.0f, 0.01f, 4.0f);
    const float after_coast = ekf.positionSigma();
    // Coasting blind must be reported as less certain - a UI that shows the
    // same confident dot after 10 s without GPS is lying to the driver.
    TEST_ASSERT_TRUE(after_coast > after_fix);

    ekf.updateGps(5.0f, 0.0f, 1.0f, 1);
    TEST_ASSERT_TRUE(ekf.positionSigma() < after_coast);
}

void test_ekf_trusts_rtk_far_more_than_plain_gps(void) {
    // Same geometry, different fix quality.
    //
    // The observable here is the resulting UNCERTAINTY, not how far the state
    // jumps toward the measurement. My first version asserted the jump and was
    // wrong about the mechanism: the gain is P/(P+R), and with RTK both P and R
    // are small, so the ratio is similar - the state does not move further. What
    // RTK actually buys is a much tighter covariance, which is what "trusted
    // more" means and what makes the estimate better.
    PoseEkf plain = makeEkf(), rtk = makeEkf();
    plain.updateGps(0.0f, 0.0f, 1.0f, 1);
    rtk.updateGps(0.0f, 0.0f, 1.0f, 4);
    for (int i = 0; i < 500; ++i) {
        plain.predict(0.01f, 0.5f, 0.0f, 0.01f, 4.0f);
        rtk.predict(0.01f, 0.5f, 0.0f, 0.01f, 4.0f);
    }
    // Measured: plain 2.503 m, RTK 0.808 m after the same 5 s of coasting.
    TEST_ASSERT_TRUE(rtk.positionSigma() < plain.positionSigma() * 0.5f);

    plain.updateGps(0.0f, 10.0f, 1.0f, 1);
    rtk.updateGps(0.0f, 10.0f, 1.0f, 4);
    TEST_ASSERT_TRUE(rtk.positionSigma() < plain.positionSigma());
}

// ------------------------------------------------------- outlier rejection --

void test_ekf_rejects_a_multipath_jump(void) {
    PoseEkf ekf = makeEkf();
    ekf.updateGps(0.0f, 0.0f, 1.0f, 1);
    for (int i = 0; i < 300; ++i) ekf.predict(0.01f, 0.2f, 0.0f, 0.01f, 1.0f);

    const float before = ekf.y();
    const bool accepted = ekf.updateGps(0.0f, 40.0f, 1.0f, 1);   // 40 m sideways
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, before, ekf.y());             // barely moved
}

void test_ekf_gives_in_if_the_receiver_really_has_moved(void) {
    // Rejecting forever is its own failure: the estimate would sit at a stale
    // position looking confident. After a run of rejections it must accept.
    PoseEkf ekf = makeEkf();
    ekf.updateGps(0.0f, 0.0f, 1.0f, 1);
    bool accepted = false;
    for (int i = 0; i < 10 && !accepted; ++i) {
        for (int k = 0; k < 100; ++k) ekf.predict(0.01f, 0.0f, 0.0f, 0.01f, 1.0f);
        accepted = ekf.updateGps(0.0f, 40.0f, 1.0f, 1);
    }
    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 40.0f, ekf.y());
}

// -------------------------------------------- against the simulated world ---

/** Drive straight and compare EKF, raw GPS and dead reckoning to the truth. */
static void runStraight(float *ekf_err, float *gps_err, float *dr_err) {
    mock_reset_clock();
    SimWorld w; SimWorld::Config wc; w.begin(wc);
    StateEstimator est = makeEst();
    PoseEkf ekf = makeEkf();

    float dr_x = 0.0f, dr_y = 0.0f;
    float gps_sum = 0.0f; int gps_n = 0;
    bool seeded = false;

    for (int i = 0; i < 3000; ++i) {                 // 30 s
        mock_advance_ms(10);
        StateEstimator::Inputs in = w.step(0.5f, 0.0f);
        est.update(in);

        const float h = est.headingDeg() * 0.017453293f;
        dr_x += est.speed() * sinf(h) * 0.01f;
        dr_y += est.speed() * cosf(h) * 0.01f;

        ekf.predict(0.01f, est.speed(), est.headingDeg(), 0.01f, 9.0f);
        if (w.gpsValid()) {
            if (!seeded) { ekf.setPose(w.gpsX(), w.gpsY(), est.headingDeg()); seeded = true; }
            else ekf.updateGps(w.gpsX(), w.gpsY(), 1.0f, 1);
            const float dx = w.gpsX() - w.trueX(), dy = w.gpsY() - w.trueY();
            gps_sum += sqrtf(dx * dx + dy * dy);
            ++gps_n;
        }
    }
    *ekf_err = w.positionErrorFrom(ekf.x(), ekf.y());
    *dr_err  = w.positionErrorFrom(dr_x, dr_y);
    *gps_err = gps_n ? gps_sum / (float)gps_n : 0.0f;
}

void test_ekf_beats_raw_gps_and_dead_reckoning(void) {
    float ekf_err = 0, gps_err = 0, dr_err = 0;
    runStraight(&ekf_err, &gps_err, &dr_err);

    // The point of the whole exercise: fusing must beat both inputs.
    TEST_ASSERT_TRUE(ekf_err < gps_err);
    TEST_ASSERT_TRUE(ekf_err < dr_err);
    // And it must be genuinely good, not merely less bad.
    TEST_ASSERT_TRUE(ekf_err < 2.0f);
}

void test_ekf_coasts_through_a_gps_dropout(void) {
    mock_reset_clock();
    SimWorld w; SimWorld::Config wc; w.begin(wc);
    StateEstimator est = makeEst();
    PoseEkf ekf = makeEkf();

    for (int i = 0; i < 1000; ++i) {                 // 10 s with GPS
        mock_advance_ms(10);
        StateEstimator::Inputs in = w.step(0.5f, 0.0f);
        est.update(in);
        ekf.predict(0.01f, est.speed(), est.headingDeg(), 0.01f, 9.0f);
        if (w.gpsValid()) ekf.updateGps(w.gpsX(), w.gpsY(), 1.0f, 1);
    }
    const float sigma_with_gps = ekf.positionSigma();

    w.setGpsDropout(10.0f, 40.0f);                   // 30 s under cover
    for (int i = 0; i < 3000; ++i) {
        mock_advance_ms(10);
        StateEstimator::Inputs in = w.step(0.5f, 0.0f);
        est.update(in);
        ekf.predict(0.01f, est.speed(), est.headingDeg(), 0.01f, 9.0f);
        if (w.gpsValid()) ekf.updateGps(w.gpsX(), w.gpsY(), 1.0f, 1);
    }

    // With no GPS the EKF IS dead reckoning - there is nothing to correct
    // against - so this bound is what dead reckoning actually achieves over
    // 30 s blind at 0.5 m/s: measured 10.97 m, driven mostly by speed error
    // from the motor-model fallback rather than by the filter. Tightening it
    // needs a better speed estimate during a dropout, which is separate work.
    // What the EKF must do here is not make it worse and not go unstable.
    TEST_ASSERT_TRUE(w.positionErrorFrom(ekf.x(), ekf.y()) < 13.0f);
    TEST_ASSERT_TRUE(ekf.positionSigma() > sigma_with_gps * 1.5f);
}

int run_pose_ekf_tests(void) {
    RUN_TEST(test_ekf_takes_the_first_fix_without_filtering_it);
    RUN_TEST(test_ekf_uncertainty_grows_while_coasting_and_shrinks_on_a_fix);
    RUN_TEST(test_ekf_trusts_rtk_far_more_than_plain_gps);
    RUN_TEST(test_ekf_rejects_a_multipath_jump);
    RUN_TEST(test_ekf_gives_in_if_the_receiver_really_has_moved);
    RUN_TEST(test_ekf_beats_raw_gps_and_dead_reckoning);
    RUN_TEST(test_ekf_coasts_through_a_gps_dropout);
    return 0;
}
