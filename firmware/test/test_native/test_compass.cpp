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
 * Compass calibration - the part that can turn a working sensor into a dead one.
 *
 * The driver only applies its magnetic-disturbance check ONCE CALIBRATED. So a
 * calibration that comes out wrong does not merely tilt the heading a few
 * degrees: it switches on a test that then rejects every reading, and the
 * compass stops working entirely. That happened on the real robot - a healthy
 * compass went to "disturbed" on every frame, heading pinned at zero, the
 * moment a calibration was saved.
 *
 * The cause was in these sums, which is why they are worth testing directly.
 */

#include <unity.h>

#include "../../lib/compass/compass_qmc5883.h"

#include <math.h>

namespace {

/** A yaw-only turn: X and Y sweep a circle, Z barely moves. This is exactly
 *  what the web UI asks the operator to do - turn the robot in a full circle. */
void feedYawTurn(CompassQMC5883 &c, float radius, float z_noise, int n = 400) {
    for (int i = 0; i < n; ++i) {
        const float a = (float)i / n * 2.0f * (float)M_PI;
        c.addCalibrationSample((int16_t)(radius * cosf(a)),
                               (int16_t)(radius * sinf(a)),
                               (int16_t)(z_noise * sinf(a * 7.0f)));
    }
}

}  // namespace

/**
 * The reference field must match what a level robot actually reads.
 *
 * With the old maths this was the average of all three spans, halved - and Z
 * was never swept, so it dragged the reference far below the truth. Measured on
 * the robot: reference 1057.7 against a real field of 1962, a ratio of 1.86
 * against a tolerance of 0.35, so every single reading was rejected.
 */
void test_yaw_only_calibration_gives_a_usable_reference(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);
    // finishCalibration() needs samples_, which only update() increments, so
    // assert on the maths through the stored calibration instead.
    c.finishCalibration();

    const CompassQMC5883::Calibration &cal = c.calibration();
    // A level robot spinning in a circle of radius 1600 reads a field of 1600,
    // so that is what the reference must come out as.
    //
    // The tolerance here is 10%, deliberately much tighter than the driver's
    // own +-35% disturbance window. Asserting merely "inside 35%" was useless:
    // the old maths produced 1080 against a true 1600 - 32% out, wrong enough
    // to be the bug, yet close enough to pass. A calibration that only just
    // scrapes inside the rejection threshold has no margin left for a real
    // magnetic disturbance, which is the thing the check exists to catch.
    TEST_ASSERT_FLOAT_WITHIN(160.0f, 1600.0f, cal.field_norm);
}

/** An axis that was never swept must not be scaled by its own noise. */
void test_unswept_z_is_left_alone(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);
    c.finishCalibration();
    // Old maths: scale_z = avg/span_z, which for a noise-sized span is huge and
    // inflates the measured field, compounding the reference being too small.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, c.calibration().scale_z);
}

/** A genuine 3D sweep should still scale Z. */
void test_a_real_three_axis_sweep_still_scales_z(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 400; ++i) {
        const float a = (float)i / 400 * 2.0f * (float)M_PI;
        c.addCalibrationSample((int16_t)(1600.0f * cosf(a)),
                               (int16_t)(1600.0f * sinf(a)),
                               (int16_t)(800.0f * cosf(a * 3.0f)));
    }
    c.finishCalibration();
    // Z span is half the horizontal one, so it is corrected, not ignored.
    TEST_ASSERT_TRUE(c.calibration().scale_z > 1.5f);
}

/**
 * Sample count is not progress, and the driver must not pretend it is.
 *
 * The failure this guards against is the one people actually hit: press start,
 * nudge the robot, press finish. Thousands of samples accumulate from a sensor
 * that is running the whole time, so any count-based progress bar fills up
 * while the robot has barely moved.
 */
void test_standing_still_makes_no_progress(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 2000; ++i) c.addCalibrationSample(1600, 20, 5);
    TEST_ASSERT_EQUAL_UINT16(2000, c.calibrationSamples());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.calibrationCoverage());
}

/** A full turn visits every sector. */
void test_a_full_turn_covers_the_whole_circle(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, c.calibrationCoverage());
}

/**
 * Half a turn must be refused, even though it passes every span check.
 *
 * This is the gap the span test alone left open. Sweeping 180 degrees produces
 * the FULL span in one axis - the diameter of the circle - so span_x and span_y
 * both look healthy. What is wrong is the centre: it lands on the midpoint of a
 * half circle rather than of a whole one, so the hard-iron offset the
 * calibration exists to measure comes out displaced by roughly the field
 * strength itself. Every later heading is then wrong, and nothing says so.
 */
void test_half_a_turn_is_refused(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 400; ++i) {
        const float a = (float)i / 400 * (float)M_PI;      // 0..180 degrees only
        c.addCalibrationSample((int16_t)(1600.0f * cosf(a)),
                               (int16_t)(1600.0f * sinf(a)), 0);
    }
    // The old check would have been happy: both horizontal spans are wide.
    TEST_ASSERT_TRUE(c.calibrationCoverage() < 0.75f);
    TEST_ASSERT_FALSE(c.finishCalibration());
    TEST_ASSERT_FALSE(c.calibration().valid);
}

/**
 * Clearing must really go back to raw.
 *
 * A stored calibration switches on the magnetic-disturbance check, and a bad
 * one then rejects every reading - a working compass goes completely dead. This
 * is the way out, and it is now reachable from the web rather than only over a
 * USB cable, so it had better work.
 */
void test_clear_calibration_returns_to_raw(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);
    TEST_ASSERT_TRUE(c.finishCalibration());
    TEST_ASSERT_TRUE(c.calibrated());

    c.clearCalibration();
    TEST_ASSERT_FALSE(c.calibrated());
    TEST_ASSERT_FALSE(c.calibration().valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.calibrationCoverage());
}

/**
 * A short nudge with sensor noise on it must not read as a turn.
 *
 * This is the case that defeats a bounding-box test on its own. Sweep 15
 * degrees of a large circle along a diagonal and the box comes out almost
 * square, so the aspect check is happy; add ordinary sensor noise and the
 * points scatter around that little box through most of the sixteen sectors,
 * so the sector count is happy too. Before the radial-consistency factor this
 * combination scored 0.91 - a nudge reported as a finished calibration.
 *
 * What gives it away is the shape of the trail: on a real sweep every point
 * sits at nearly the same distance from the centre, and on a fat little segment
 * the middle points sit almost on top of it.
 */
void test_a_noisy_nudge_is_not_a_turn(void) {
    CompassQMC5883 c;
    c.startCalibration();
    uint32_t seed = 12345;
    for (int i = 0; i < 2000; ++i) {
        // 15 degrees of arc, placed on the diagonal so the bounding box is square.
        const float a = (39.0f + 15.0f * (float)i / 2000.0f) * 0.01745329f;
        seed = seed * 1103515245u + 12345u;
        const float nx = (float)((seed >> 16) % 81) - 40.0f;
        seed = seed * 1103515245u + 12345u;
        const float ny = (float)((seed >> 16) % 81) - 40.0f;
        c.addCalibrationSample((int16_t)(2400.0f * cosf(a) + nx),
                               (int16_t)(2400.0f * sinf(a) + ny), 0);
    }
    TEST_ASSERT_TRUE(c.calibrationCoverage() < 0.75f);
    TEST_ASSERT_FALSE(c.finishCalibration());
}

/** Starting a second sweep forgets the first one's coverage. */
void test_restarting_forgets_the_previous_sweep(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, c.calibrationCoverage());

    c.startCalibration();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.calibrationCoverage());
    TEST_ASSERT_EQUAL_UINT16(0, c.calibrationSamples());
}

/**
 * A full field sweep with the robot barely turning must be refused.
 *
 * This is the case only the gyro can catch. Wave a magnet past a stationary
 * compass, or set the robot on a turntable that slips, and the magnetometer
 * sees a perfectly clean circle - every sector visited, a round trail, a square
 * bounding box. Judged on its own data the sweep looks ideal, because the data
 * IS the thing being judged.
 *
 * The gyro is independent of the magnetic field entirely, so it reports the
 * truth: nothing turned.
 */
void test_a_perfect_field_sweep_without_turning_is_refused(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 400; ++i) {
        const float a = (float)i / 400 * 2.0f * (float)M_PI;
        c.addCalibrationSample((int16_t)(1600.0f * cosf(a)),
                               (int16_t)(1600.0f * sinf(a)), 0);
        c.addTurnYaw(0.0f);                  // the IMU says: heading never changed
    }
    TEST_ASSERT_TRUE(c.turnKnown());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.calibrationCoverage());
    TEST_ASSERT_FALSE(c.finishCalibration());
}

/** With the gyro agreeing, a real turn still passes. */
void test_a_real_turn_with_the_gyro_agreeing_passes(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 400; ++i) {
        const float a = (float)i / 400 * 2.0f * (float)M_PI;
        c.addCalibrationSample((int16_t)(1600.0f * cosf(a)),
                               (int16_t)(1600.0f * sinf(a)),
                               (int16_t)(40.0f * sinf(a * 7.0f)));
        c.addTurnYaw((float)i / 400.0f * 360.0f);   // a full turn, in absolute heading
    }
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 360.0f, c.turnedDeg());
    TEST_ASSERT_TRUE(c.calibrationCoverage() >= 0.75f);
    TEST_ASSERT_TRUE(c.finishCalibration());
}

/**
 * Turning one way then back is not a sweep, however far the wheels went.
 *
 * The rotation is accumulated SIGNED for exactly this reason. Rocking the robot
 * left and right racks up hundreds of degrees of movement while the heading
 * ends where it started, and an absolute total would call that a full circle.
 */
void test_rocking_back_and_forth_does_not_count(void) {
    CompassQMC5883 c;
    c.startCalibration();
    for (int i = 0; i < 400; ++i) {
        const float a = 0.9f * sinf((float)i / 400 * 12.0f * (float)M_PI);
        c.addCalibrationSample((int16_t)(1600.0f * cosf(a)),
                               (int16_t)(1600.0f * sinf(a)), 0);
        c.addTurnYaw((i % 2) ? 20.0f : -20.0f);   // rocking between two headings
    }
    TEST_ASSERT_TRUE(c.turnedDeg() < 60.0f && c.turnedDeg() > -60.0f);
    TEST_ASSERT_TRUE(c.calibrationCoverage() < 0.75f);
    TEST_ASSERT_FALSE(c.finishCalibration());
}

/** With no IMU at all the old field-only behaviour is unchanged. */
void test_without_an_imu_the_field_measure_still_works(void) {
    CompassQMC5883 c;
    c.startCalibration();
    feedYawTurn(c, 1600.0f, 40.0f);          // never feeds a heading
    TEST_ASSERT_FALSE(c.turnKnown());
    TEST_ASSERT_TRUE(c.calibrationCoverage() >= 0.75f);
    TEST_ASSERT_TRUE(c.finishCalibration());
}

int run_compass_tests(void) {
    RUN_TEST(test_a_perfect_field_sweep_without_turning_is_refused);
    RUN_TEST(test_a_real_turn_with_the_gyro_agreeing_passes);
    RUN_TEST(test_rocking_back_and_forth_does_not_count);
    RUN_TEST(test_without_an_imu_the_field_measure_still_works);
    RUN_TEST(test_yaw_only_calibration_gives_a_usable_reference);
    RUN_TEST(test_unswept_z_is_left_alone);
    RUN_TEST(test_a_real_three_axis_sweep_still_scales_z);
    RUN_TEST(test_standing_still_makes_no_progress);
    RUN_TEST(test_a_full_turn_covers_the_whole_circle);
    RUN_TEST(test_half_a_turn_is_refused);
    RUN_TEST(test_a_noisy_nudge_is_not_a_turn);
    RUN_TEST(test_clear_calibration_returns_to_raw);
    RUN_TEST(test_restarting_forgets_the_previous_sweep);
    return 0;
}
