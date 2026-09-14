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
 * The lever-arm correction: moving a reading from where a sensor is to where
 * the robot actually turns.
 *
 * These are pure geometry, so they are worth more than usual - the correction
 * can be fully verified here, and what is left for the real robot is only
 * MEASURING the offsets. That is the split the plan asks for: write and prove
 * the code now, confirm the numbers when the hardware allows.
 */
#include <unity.h>
#include "lever_arm.h"

static const float TOL = 0.001f;

void test_zero_offset_changes_nothing(void) {
    // A robot whose offsets have never been measured must behave exactly as it
    // did before this existed. Anything else would make the feature a
    // regression for every robot that has not been measured yet.
    LeverArm arm;                       // both zero
    float e, n;
    leverArmToCentre(12.5f, -3.25f, 137.0f, arm, &e, &n);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 12.5f, e);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -3.25f, n);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, leverArmTangentialSpeed(90.0f, arm));
}

void test_antenna_ahead_while_facing_north(void) {
    // Facing north, an antenna 0.30 m forward reads 0.30 m north of the
    // turning centre.
    LeverArm arm; arm.x = 0.30f; arm.y = 0.0f;
    float e, n;
    leverArmToCentre(0.0f, 0.30f, 0.0f, arm, &e, &n);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, n);
}

void test_antenna_ahead_while_facing_east(void) {
    // Turn the robot to face east and the same antenna is now 0.30 m EAST of
    // the centre. This is the whole point: the error rotates with the robot,
    // so it cannot be removed by a fixed offset.
    LeverArm arm; arm.x = 0.30f; arm.y = 0.0f;
    float e, n;
    leverArmToCentre(0.30f, 0.0f, 90.0f, arm, &e, &n);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, n);
}

void test_antenna_to_the_left_while_facing_north(void) {
    // y is LEFT, and facing north, left is west - so a negative easting.
    LeverArm arm; arm.x = 0.0f; arm.y = 0.20f;
    float e, n;
    leverArmToCentre(-0.20f, 0.0f, 0.0f, arm, &e, &n);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, n);
}

void test_spinning_on_the_spot_does_not_move_the_centre(void) {
    // The failure this exists to prevent. The robot turns a full circle without
    // going anywhere; the antenna sweeps a 0.30 m radius arc. Corrected, the
    // centre must stay put at every heading - uncorrected, it would trace the
    // whole circle and the filter would read it as a metre and a half of travel.
    LeverArm arm; arm.x = 0.30f; arm.y = 0.0f;
    for (int deg = 0; deg < 360; deg += 15) {
        const float h = deg * 0.01745329252f;
        // Where the antenna is, for a centre sitting exactly at the origin.
        const float ae = arm.x * sinf(h);
        const float an = arm.x * cosf(h);
        float e, n;
        leverArmToCentre(ae, an, (float)deg, arm, &e, &n);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, n);
    }
}

void test_offset_diagonal_round_trips_at_every_heading(void) {
    // Same argument with the offset on both axes, so a sign error in either
    // term is caught rather than cancelling.
    LeverArm arm; arm.x = 0.25f; arm.y = -0.10f;
    for (int deg = 0; deg < 360; deg += 23) {
        const float h = deg * 0.01745329252f;
        const float ae = arm.x * sinf(h) - arm.y * cosf(h);
        const float an = arm.x * cosf(h) + arm.y * sinf(h);
        float e, n;
        leverArmToCentre(ae, an, (float)deg, arm, &e, &n);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, e);
        TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, n);
    }
}

void test_tangential_speed_of_a_spinning_sensor(void) {
    // Spinning at 90 deg/s with the sensor 0.30 m out, the sensor is moving at
    // 0.471 m/s while the robot goes nowhere. Reported as speed, that is a
    // robot that appears to drive off every time it turns.
    LeverArm arm; arm.x = 0.30f; arm.y = 0.0f;
    const float v = leverArmTangentialSpeed(90.0f, arm);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.4712f, v);

    // Sign follows the rotation direction.
    TEST_ASSERT_FLOAT_WITHIN(0.005f, -0.4712f, leverArmTangentialSpeed(-90.0f, arm));

    // And a sensor at the centre has none, however fast the robot spins.
    LeverArm centred;
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, leverArmTangentialSpeed(720.0f, centred));
}

int run_lever_arm_tests(void) {
    RUN_TEST(test_zero_offset_changes_nothing);
    RUN_TEST(test_antenna_ahead_while_facing_north);
    RUN_TEST(test_antenna_ahead_while_facing_east);
    RUN_TEST(test_antenna_to_the_left_while_facing_north);
    RUN_TEST(test_spinning_on_the_spot_does_not_move_the_centre);
    RUN_TEST(test_offset_diagonal_round_trips_at_every_heading);
    RUN_TEST(test_tangential_speed_of_a_spinning_sensor);
    return 0;
}
