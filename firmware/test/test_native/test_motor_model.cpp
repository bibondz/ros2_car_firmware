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
 * MotorModel tests: back-EMF wheel speed from the INA226 volts and amps,
 * the series diode compensation (schematic draft_5) and the fault detection.
 */
#include <unity.h>
#include "Arduino.h"
#include "motor_model.h"

static MotorModel make(float ke = 0.12f, float r = 5.0f,
                       float vf = 0.45f, float rd = 0.0f) {
    MotorModel m;
    MotorModel::Config cfg;
    cfg.ke_v_per_rpm = ke;
    cfg.resistance_ohm = r;
    cfg.diode_vf_v = vf;
    cfg.diode_r_ohm = rd;
    cfg.wheel_diameter_m = 0.081f;
    cfg.min_duty = 0.12f;
    cfg.stall_current_a = 0.9f;
    cfg.open_current_a = 0.04f;
    cfg.filter_tau_s = 0.0f;         // no smoothing, so one call is enough
    m.begin(cfg);
    return m;
}

// ------------------------------------------------------------------ basics
void test_model_full_duty_no_load(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);      // no diode
    m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_TRUE(m.valid());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, m.rpm());   // 12 V / 0.12 = 100 rpm
}

void test_model_half_duty(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);
    m.update(0.01f, 12.0f, 0.0f, 0.5f, true);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 50.0f, m.rpm());
}

void test_model_current_drops_the_speed(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);
    m.update(0.01f, 12.0f, 0.5f, 1.0f, true);          // 0.5 A * 5 ohm = 2.5 V lost
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 79.2f, m.rpm());    // (12 - 2.5) / 0.12
}

void test_model_reverse_duty_gives_negative_rpm(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);
    m.update(0.01f, 12.0f, 0.0f, -1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -100.0f, m.rpm());
}

void test_model_mps_conversion(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);
    m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    // 100 rpm on a 81 mm wheel = 100/60 * pi * 0.081 = 0.424 m/s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.424f, m.mps());
}

// --------------------------------------------------- series diode (draft_5)
void test_diode_drop_is_subtracted(void) {
    MotorModel m = make(0.12f, 5.0f, 0.45f, 0.0f);
    m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 96.25f, m.rpm());   // (12 - 0.45) / 0.12
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.45f, m.diodeDrop());
}

void test_diode_slope_resistance_adds_with_current(void) {
    MotorModel m = make(0.12f, 5.0f, 0.45f, 0.10f);
    m.update(0.01f, 12.0f, 1.0f, 1.0f, true);          // drop = 0.45 + 1.0 * 0.10
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.55f, m.diodeDrop());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 53.75f, m.rpm());   // (12 - 0.55 - 5.0) / 0.12
}

void test_bigger_diode_means_lower_estimate(void) {
    MotorModel schottky = make(0.12f, 5.0f, 0.45f, 0.0f);
    MotorModel standard = make(0.12f, 5.0f, 0.90f, 0.0f);
    schottky.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    standard.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_TRUE(standard.rpm() < schottky.rpm());
}

void test_two_motors_can_have_different_diodes(void) {
    MotorModel left  = make(0.12f, 5.0f, 0.40f, 0.0f);
    MotorModel right = make(0.12f, 5.0f, 0.85f, 0.0f);
    left.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    right.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 96.7f, left.rpm());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 92.9f, right.rpm());
}

void test_diode_bigger_than_supply_clamps_to_zero(void) {
    MotorModel m = make(0.12f, 5.0f, 15.0f, 0.0f);     // silly value on purpose
    m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.rpm());
}

// ---------------------------------------------------------------- gating
void test_low_duty_is_not_published(void) {
    MotorModel m = make();
    m.update(0.01f, 12.0f, 0.0f, 0.05f, true);         // below min_duty
    TEST_ASSERT_FALSE(m.valid());
}

void test_missing_sensor_is_not_published(void) {
    MotorModel m = make();
    m.update(0.01f, 12.0f, 0.0f, 1.0f, false);
    TEST_ASSERT_FALSE(m.valid());
}

void test_back_emf_never_goes_negative(void) {
    MotorModel m = make(0.12f, 5.0f, 0.0f, 0.0f);
    m.update(0.01f, 12.0f, 5.0f, 0.5f, true);          // huge current, tiny duty
    TEST_ASSERT_TRUE(m.rpm() >= 0.0f);
}

// ---------------------------------------------------------------- faults
void test_stall_detected_after_the_hold_time(void) {
    mock_reset_clock();
    MotorModel m = make();
    // A stall means the wheel is NOT turning, so the back-EMF is ~0 and the
    // current settles at v_applied / R = (12 - 0.45) / 5 = 2.31 A. The old
    // 2.0 A left 1.55 V of back-EMF, i.e. 12.9 rpm, which the model correctly
    // refused to call a stall - the test data was wrong, not the model.
    m.update(0.01f, 12.0f, 2.3f, 1.0f, true);          // stalled: no back-EMF
    TEST_ASSERT_FALSE(m.stalled());                    // start-up surge is ignored
    for (int i = 0; i < 60; ++i) { mock_advance_ms(10); m.update(0.01f, 12.0f, 2.3f, 1.0f, true); }
    TEST_ASSERT_TRUE(m.stalled());
}

void test_stall_clears_when_the_wheel_turns(void) {
    mock_reset_clock();
    MotorModel m = make();
    // 2.3 A is the real stall current here; see the note above.
    for (int i = 0; i < 60; ++i) { mock_advance_ms(10); m.update(0.01f, 12.0f, 2.3f, 1.0f, true); }
    TEST_ASSERT_TRUE(m.stalled());
    m.update(0.01f, 12.0f, 0.2f, 1.0f, true);
    TEST_ASSERT_FALSE(m.stalled());
}

void test_open_circuit_detected(void) {
    // Commanded, drawing nothing, and it STAYS that way - that is a real open
    // circuit and must be reported.
    mock_reset_clock();
    MotorModel m = make();
    for (int i = 0; i < 60; ++i) {                     // 600 ms, past the hold
        m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
        mock_advance_ms(10);
    }
    TEST_ASSERT_TRUE(m.openCircuit());
}

/**
 * A motor that has only just been commanded is not an open circuit.
 *
 * Duty crosses min_duty at the very start of the PWM ramp while the current is
 * still climbing from zero, and the INA226s are read at just 5 Hz - so the
 * current can still be the pre-command zero for up to 200 ms after the duty is
 * up. Judged instantly that reads as "this motor is drawing nothing" on a motor
 * about to draw plenty, and the dashboard flashed a false open-circuit on every
 * single start. Reported from the robot.
 */
void test_open_circuit_not_flagged_during_the_current_ramp(void) {
    mock_reset_clock();
    MotorModel m = make();
    for (int i = 0; i < 15; ++i) {                     // 150 ms of stale zero
        m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
        mock_advance_ms(10);
    }
    TEST_ASSERT_FALSE(m.openCircuit());

    // and once the current does arrive it must never latch
    for (int i = 0; i < 60; ++i) {
        m.update(0.01f, 12.0f, 0.30f, 1.0f, true);
        mock_advance_ms(10);
    }
    TEST_ASSERT_FALSE(m.openCircuit());
}

void test_open_circuit_clear_with_normal_current(void) {
    MotorModel m = make();
    m.update(0.01f, 12.0f, 0.3f, 1.0f, true);
    TEST_ASSERT_FALSE(m.openCircuit());
}

void test_motor_model_reset_clears_state(void) {
    mock_reset_clock();
    MotorModel m = make();
    for (int i = 0; i < 60; ++i) { mock_advance_ms(10); m.update(0.01f, 12.0f, 2.0f, 1.0f, true); }
    m.reset();
    TEST_ASSERT_FALSE(m.valid());
    TEST_ASSERT_FALSE(m.stalled());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.rpm());
}

void test_config_can_be_changed_at_runtime(void) {
    MotorModel m = make(0.12f, 5.0f, 0.45f, 0.0f);
    m.config().diode_vf_v = 0.90f;                     // web UI pushes a new value
    m.update(0.01f, 12.0f, 0.0f, 1.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.90f, m.diodeDrop());
}

int run_motor_model_tests(void) {
    RUN_TEST(test_model_full_duty_no_load);
    RUN_TEST(test_model_half_duty);
    RUN_TEST(test_model_current_drops_the_speed);
    RUN_TEST(test_model_reverse_duty_gives_negative_rpm);
    RUN_TEST(test_model_mps_conversion);
    RUN_TEST(test_diode_drop_is_subtracted);
    RUN_TEST(test_diode_slope_resistance_adds_with_current);
    RUN_TEST(test_bigger_diode_means_lower_estimate);
    RUN_TEST(test_two_motors_can_have_different_diodes);
    RUN_TEST(test_diode_bigger_than_supply_clamps_to_zero);
    RUN_TEST(test_low_duty_is_not_published);
    RUN_TEST(test_missing_sensor_is_not_published);
    RUN_TEST(test_back_emf_never_goes_negative);
    RUN_TEST(test_stall_detected_after_the_hold_time);
    RUN_TEST(test_stall_clears_when_the_wheel_turns);
    RUN_TEST(test_open_circuit_detected);
    RUN_TEST(test_open_circuit_not_flagged_during_the_current_ramp);
    RUN_TEST(test_open_circuit_clear_with_normal_current);
    RUN_TEST(test_motor_model_reset_clears_state);
    RUN_TEST(test_config_can_be_changed_at_runtime);
    return 0;
}
