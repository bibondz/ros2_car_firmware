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
 * Mounting rotations, said as angles or as a quaternion.
 *
 * The point of having both forms is that nobody has to convert by hand, and a
 * hand conversion is exactly where a silent wrong number gets in. So the
 * conversion itself is worth testing properly: it is the only thing standing
 * between "the IMU is bolted on its side" and every heading being wrong.
 */

#include <unity.h>

#include "mount_rotation.h"

#include <math.h>

/** No rotation at all, entered either way, must be the identity. */
void test_zero_angles_are_the_identity(void) {
    const Quat q = quatFromRPY(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, q.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, q.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, q.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, q.z);
}

/** Angles survive a round trip through the quaternion form. */
void test_angles_round_trip(void) {
    const float cases[][3] = {
        {0, 0, 90}, {90, 0, 0}, {0, 45, 0}, {-30, 20, 170}, {10, -15, -95},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const Quat q = quatFromRPY(cases[i][0], cases[i][1], cases[i][2]);
        float r, p, y;
        rpyFromQuat(q, &r, &p, &y);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, cases[i][0], r);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, cases[i][1], p);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, cases[i][2], y);
    }
}

/** Yaw 90 turns "forward" into "left", which is the whole point of the frame. */
void test_yaw_ninety_turns_forward_into_left(void) {
    const Quat q = quatFromRPY(0.0f, 0.0f, 90.0f);
    float x, y, z;
    quatRotate(q, 1.0f, 0.0f, 0.0f, &x, &y, &z);   // +x is forward
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, y);      // +y is left
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, z);
}

/** An IMU bolted on its side: roll 90 sends "up" to the right. */
void test_roll_ninety_puts_up_on_its_side(void) {
    const Quat q = quatFromRPY(90.0f, 0.0f, 0.0f);
    float x, y, z;
    quatRotate(q, 0.0f, 0.0f, 1.0f, &x, &y, &z);   // +z is up
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, y);     // now pointing right
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, z);
}

/**
 * All-zero quaternion is what an empty form gives, and it is not a rotation.
 *
 * Left alone it would collapse every vector it touched to nothing - a mount
 * that silently deletes its own measurement. Identity is the honest reading of
 * "nobody filled this in".
 */
void test_an_empty_quaternion_is_treated_as_no_rotation(void) {
    Quat zero; zero.w = 0.0f; zero.x = 0.0f; zero.y = 0.0f; zero.z = 0.0f;
    const Quat q = quatNormalise(zero);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, q.w);

    float x, y, z;
    quatRotate(q, 1.0f, 2.0f, 3.0f, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, z);
}

/** A quaternion typed to four decimals is not quite unit length. It must still work. */
void test_a_slightly_off_unit_quaternion_is_repaired(void) {
    Quat rounded; rounded.w = 0.7071f; rounded.x = 0.0f;
    rounded.y = 0.0f; rounded.z = 0.7071f;          // yaw 90, rounded
    const Quat q = quatNormalise(rounded);
    float r, p, y;
    rpyFromQuat(q, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 90.0f, y);
    TEST_ASSERT_FALSE(isnan(p));                    // the clamp in rpyFromQuat
}

/** The mode selects one form and ignores the other - it never blends them. */
void test_the_mode_picks_one_form_and_ignores_the_other(void) {
    // Angles say yaw 90; the quaternion field says identity. In RPY mode the
    // angles win and the quaternion is not consulted at all.
    Quat q = mountRotation(ROT_RPY, 0.0f, 0.0f, 90.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    float r, p, y;
    rpyFromQuat(q, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, y);

    // Same inputs, quaternion mode: now the identity wins and yaw 90 is ignored.
    q = mountRotation(ROT_QUAT, 0.0f, 0.0f, 90.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    rpyFromQuat(q, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, y);
}

/** Pitch straight up is the gimbal-lock case; it must not produce NaN. */
void test_pitch_at_the_limit_does_not_produce_nan(void) {
    const Quat q = quatFromRPY(0.0f, 90.0f, 0.0f);
    float r, p, y;
    rpyFromQuat(q, &r, &p, &y);
    TEST_ASSERT_FALSE(isnan(r));
    TEST_ASSERT_FALSE(isnan(p));
    TEST_ASSERT_FALSE(isnan(y));
    // 0.1 rather than 0.01: at exactly 90 degrees the quaternion terms cancel
    // to within single-precision rounding, and asinf() of a value a few ULPs
    // below 1.0 comes back as 89.98. That is float arithmetic at the
    // singularity, not an error in the conversion - the test that matters here
    // is that it stays finite and close, which it does.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, p);
}

int run_mount_rotation_tests(void) {
    RUN_TEST(test_zero_angles_are_the_identity);
    RUN_TEST(test_angles_round_trip);
    RUN_TEST(test_yaw_ninety_turns_forward_into_left);
    RUN_TEST(test_roll_ninety_puts_up_on_its_side);
    RUN_TEST(test_an_empty_quaternion_is_treated_as_no_rotation);
    RUN_TEST(test_a_slightly_off_unit_quaternion_is_repaired);
    RUN_TEST(test_the_mode_picks_one_form_and_ignores_the_other);
    RUN_TEST(test_pitch_at_the_limit_does_not_produce_nan);
    return 0;
}
