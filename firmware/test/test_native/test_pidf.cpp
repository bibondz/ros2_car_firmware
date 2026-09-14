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
 * PIDF unit tests - the controller behind both the heading and the speed loop.
 */
#include <unity.h>
#include "Arduino.h"
#include "PIDF.h"

static void tick(unsigned long ms) { mock_advance_ms(ms); }

// Unity calls these from C, so they must not be name-mangled
extern "C" void setUp(void) { mock_reset_clock(); }
extern "C" void tearDown(void) {}

// ------------------------------------------------------------------- basics
void test_proportional_only(void) {
    PIDF pid(-100.0f, 100.0f, 2.0f, 0.0f, -100.0f, 100.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    float out = pid.compute(10.0f, 0.0f);      // error 10, Kp 2
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out);
}

void test_output_is_clamped_high(void) {
    PIDF pid(-50.0f, 50.0f, 100.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, pid.compute(10.0f, 0.0f));
}

void test_output_is_clamped_low(void) {
    PIDF pid(-50.0f, 50.0f, 100.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -50.0f, pid.compute(-10.0f, 0.0f));
}

void test_zero_error_zero_output(void) {
    PIDF pid(-100.0f, 100.0f, 5.0f, 1.0f, -100.0f, 100.0f, 0.5f, 0.0f, 0.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pid.compute(0.0f, 0.0f));
}

void test_sign_follows_error(void) {
    PIDF pid(-100.0f, 100.0f, 3.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    TEST_ASSERT_TRUE(pid.compute(5.0f, 0.0f) > 0.0f);
    tick(10);
    TEST_ASSERT_TRUE(pid.compute(-5.0f, 0.0f) < 0.0f);
}

// ---------------------------------------------------------------- deadband
void test_deadband_suppresses_small_error(void) {
    PIDF pid(-100.0f, 100.0f, 10.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, /*tol*/ 2.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.compute(1.5f, 0.0f));
}

void test_deadband_passes_large_error(void) {
    PIDF pid(-100.0f, 100.0f, 10.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    tick(10);
    TEST_ASSERT_TRUE(pid.compute(5.0f, 0.0f) > 1.0f);
}

void test_deadband_clears_integral(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 10.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 10; ++i) { tick(10); pid.compute(5.0f, 0.0f); }
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.compute(0.2f, 0.0f));   // inside deadband
}

// ---------------------------------------------------------------- integral
void test_integral_accumulates(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 1.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    float first = pid.compute(10.0f, 0.0f);
    tick(10);
    float second = pid.compute(10.0f, 0.0f);
    TEST_ASSERT_TRUE(second > first);
}

void test_integral_is_clamped(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 1.0f, -0.5f, 0.5f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 200; ++i) { tick(10); pid.compute(10.0f, 0.0f); }
    tick(10);
    float out = pid.compute(10.0f, 0.0f);
    TEST_ASSERT_TRUE(out <= 0.5f + 0.001f);
}

void test_integral_unwinds_with_opposite_error(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 5.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; ++i) { tick(10); pid.compute(10.0f, 0.0f); }
    tick(10);
    float positive = pid.compute(10.0f, 0.0f);
    for (int i = 0; i < 20; ++i) { tick(10); pid.compute(-10.0f, 0.0f); }
    tick(10);
    TEST_ASSERT_TRUE(pid.compute(-10.0f, 0.0f) < positive);
}

void test_reset_clears_state(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 5.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 20; ++i) { tick(10); pid.compute(10.0f, 0.0f); }
    pid.reset();
    tick(10);
    float after = pid.compute(10.0f, 0.0f);
    TEST_ASSERT_TRUE(after < 1.0f);      // integral started from zero again
}

// -------------------------------------------------------------- derivative
void test_derivative_reacts_to_change(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 0.0f, -10.0f, 10.0f, 1.0f, 0.0f, 0.0f);
    tick(10);
    pid.compute(0.0f, 0.0f);
    tick(10);
    float out = pid.compute(10.0f, 0.0f);     // error jumped -> D term positive
    TEST_ASSERT_TRUE(out > 0.0f);
}

void test_derivative_filter_smooths(void) {
    PIDF raw(-10000.0f, 10000.0f, 0.0f, 0.0f, -10.0f, 10.0f, 1.0f, 0.0f, 0.0f);
    PIDF filt(-10000.0f, 10000.0f, 0.0f, 0.0f, -10.0f, 10.0f, 1.0f, 0.0f, 0.0f);
    filt.setDFilterCutoffHz(2.0f);
    tick(10); raw.compute(0.0f, 0.0f); filt.compute(0.0f, 0.0f);
    tick(10);
    float raw_out = raw.compute(10.0f, 0.0f);
    float filt_out = filt.compute(10.0f, 0.0f);
    TEST_ASSERT_TRUE(filt_out < raw_out);
}

// ----------------------------------------------------------------- setters
void test_set_gains_takes_effect(void) {
    PIDF pid(-100.0f, 100.0f, 1.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    pid.setPIDF(4.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, pid.compute(10.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, pid.kp());
}

void test_set_output_limits(void) {
    PIDF pid(-100.0f, 100.0f, 10.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-5.0f, 5.0f);
    tick(10);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, pid.compute(10.0f, 0.0f));
}

void test_compute_with_error_matches_compute(void) {
    PIDF a(-100.0f, 100.0f, 2.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    PIDF b(-100.0f, 100.0f, 2.0f, 0.0f, -10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    float from_setpoint = a.compute(7.0f, 2.0f);
    float from_error = b.compute_with_error(5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, from_setpoint, from_error);
}

void test_long_stall_does_not_explode_integral(void) {
    PIDF pid(-1000.0f, 1000.0f, 0.0f, 1.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f);
    tick(10);
    pid.compute(1.0f, 0.0f);
    tick(5000);                                   // 5 s gap: dt is clamped to 0.2 s
    float out = pid.compute(1.0f, 0.0f);
    TEST_ASSERT_TRUE(out < 1.0f);
}

int run_pidf_tests(void) {
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_output_is_clamped_high);
    RUN_TEST(test_output_is_clamped_low);
    RUN_TEST(test_zero_error_zero_output);
    RUN_TEST(test_sign_follows_error);
    RUN_TEST(test_deadband_suppresses_small_error);
    RUN_TEST(test_deadband_passes_large_error);
    RUN_TEST(test_deadband_clears_integral);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_integral_is_clamped);
    RUN_TEST(test_integral_unwinds_with_opposite_error);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_derivative_reacts_to_change);
    RUN_TEST(test_derivative_filter_smooths);
    RUN_TEST(test_set_gains_takes_effect);
    RUN_TEST(test_set_output_limits);
    RUN_TEST(test_compute_with_error_matches_compute);
    RUN_TEST(test_long_stall_does_not_explode_integral);
    return 0;
}
