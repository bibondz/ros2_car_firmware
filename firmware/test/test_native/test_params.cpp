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
 * The parameter registry: defaults, overrides, bounds, and the moving guard.
 *
 * Worth testing hard, because this is the path by which a person changes what
 * the robot believes about itself. A bad value accepted here does not fail
 * loudly - it makes the back-EMF model, or the wheel diameter, quietly wrong,
 * and every speed the robot reports afterwards is wrong with it.
 */
#include <unity.h>
#include "param_registry.h"

// A small table of its own, so these tests do not move whenever the real one
// gains a parameter. The behaviours under test are the registry's, not the
// table's.
static const ParamDef TEST_TABLE[] = {
  {"a.live",    "V",   "a", 1.50f, 0.0f, 3.0f,  PARAM_LIVE,    "live value"},
  {"a.stopped", "m",   "a", 0.08f, 0.02f, 0.5f, PARAM_STOPPED, "needs a stop"},
  {"b.wide",    "ohm", "b", 5.00f, 0.1f, 50.0f, PARAM_LIVE,    "wide range"},
};
static const uint8_t TEST_COUNT = 3;

static ParamRegistry<8> make() {
  ParamRegistry<8> reg;
  reg.begin(TEST_TABLE, TEST_COUNT, nullptr);   // nullptr: no NVS in host tests
  return reg;
}

void test_defaults_come_from_the_table(void) {
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL_UINT8(3, reg.count());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, reg.get("a.stopped"));
    TEST_ASSERT_FALSE(reg.overridden(0));
}

void test_an_unknown_key_is_refused_not_created(void) {
    // Silently accepting an unknown name would let a typo look like a working
    // setting - the UI would show it saved and nothing would ever use it.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_UNKNOWN, reg.set("a.tpyo", 1.0f, false));
    TEST_ASSERT_EQUAL_INT16(-1, reg.indexOf("nothing.here"));
}

void test_a_value_in_range_is_kept_and_marked_overridden(void) {
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_OK, reg.set("a.live", 1.81f, false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.81f, reg.get("a.live"));
    TEST_ASSERT_TRUE(reg.overridden(0));
}

void test_out_of_range_is_refused_and_changes_nothing(void) {
    // Refusing but keeping the bad value would be the worst of both.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_OUT_OF_RANGE, reg.set("a.live", 99.0f, false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));
    TEST_ASSERT_EQUAL(PARAM_OUT_OF_RANGE, reg.set("a.live", -1.0f, false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));
    TEST_ASSERT_FALSE(reg.overridden(0));
}

void test_the_bounds_themselves_are_allowed(void) {
    // An exclusive bound would make the documented range a lie.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_OK, reg.set("a.live", 0.0f, false));
    TEST_ASSERT_EQUAL(PARAM_OK, reg.set("a.live", 3.0f, false));
}

void test_nan_is_refused_before_the_range_check(void) {
    // NaN compares false against every bound, so a range check alone lets it
    // through - and then every calculation downstream produces NaN with no
    // indication of where it started.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_NOT_FINITE, reg.set("a.live", NAN, false));
    TEST_ASSERT_EQUAL(PARAM_NOT_FINITE, reg.set("a.live", INFINITY, false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));
}

void test_stopped_only_is_refused_while_moving(void) {
    // T70. Changing the wheel diameter mid-drive silently redefines what every
    // speed means, and the robot does something surprising for a second or two.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_NEEDS_STOP, reg.set("a.stopped", 0.10f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, reg.get("a.stopped"));

    // Stopped, the same change is fine.
    TEST_ASSERT_EQUAL(PARAM_OK, reg.set("a.stopped", 0.10f, false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.10f, reg.get("a.stopped"));
}

void test_live_parameters_apply_while_moving(void) {
    // The other half of the guard: a LIVE value must NOT be blocked, or tuning
    // a gain while driving - the entire point of tuning - becomes impossible.
    ParamRegistry<8> reg = make();
    TEST_ASSERT_EQUAL(PARAM_OK, reg.set("a.live", 2.0f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, reg.get("a.live"));
}

void test_reset_restores_the_default_and_clears_the_override(void) {
    ParamRegistry<8> reg = make();
    reg.set("a.live", 2.5f, false);
    TEST_ASSERT_TRUE(reg.overridden(0));
    TEST_ASSERT_EQUAL(PARAM_OK, reg.reset("a.live", false));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));
    TEST_ASSERT_FALSE(reg.overridden(0));
}

void test_reset_of_a_stopped_parameter_also_needs_a_stop(void) {
    // Reset is a change like any other - it moves the value the loops are using.
    ParamRegistry<8> reg = make();
    reg.set("a.stopped", 0.10f, false);
    TEST_ASSERT_EQUAL(PARAM_NEEDS_STOP, reg.reset("a.stopped", true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.10f, reg.get("a.stopped"));
}

void test_reset_all_leaves_stopped_parameters_alone_while_moving(void) {
    ParamRegistry<8> reg = make();
    reg.set("a.live", 2.0f, false);
    reg.set("a.stopped", 0.10f, false);
    reg.resetAll(true);                        // moving
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.50f, reg.get("a.live"));      // reset
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.10f, reg.get("a.stopped"));   // kept
}

void test_storage_keys_are_short_enough_and_distinct(void) {
    // NVS keys are capped at 15 characters and readable names are longer, so
    // the stored key is a hash. A collision would silently make two parameters
    // share one stored value - which would look like one of them not saving.
    char seen[64][16];
    uint8_t n = 0;
    const char *names[] = {
        "motor.a.diode_vf", "motor.b.diode_vf", "motor.a.diode_r",
        "motor.b.diode_r", "motor.resistance", "motor.ke", "motor.stall_a",
        "robot.wheel_diameter", "robot.track", "robot.max_speed",
        "geometry.gps_x", "geometry.gps_y", "geometry.imu_x", "geometry.imu_y",
        "geometry.axle_x", "battery.warn_v", "battery.soft_stop_v",
        "battery.cutoff_v", "compass.declination", "estimator.gps_tau",
        "estimator.model_tau", "estimator.align_min",
    };
    const uint8_t count = sizeof(names) / sizeof(names[0]);
    for (uint8_t i = 0; i < count; ++i) {
        char key[16];
        paramStorageKey(names[i], key);
        TEST_ASSERT_TRUE(strlen(key) < 16);
        for (uint8_t j = 0; j < n; ++j) {
            TEST_ASSERT_TRUE_MESSAGE(strcmp(seen[j], key) != 0,
                                     "two parameters hash to the same NVS key");
        }
        strcpy(seen[n++], key);
    }
}

int run_param_tests(void) {
    RUN_TEST(test_defaults_come_from_the_table);
    RUN_TEST(test_an_unknown_key_is_refused_not_created);
    RUN_TEST(test_a_value_in_range_is_kept_and_marked_overridden);
    RUN_TEST(test_out_of_range_is_refused_and_changes_nothing);
    RUN_TEST(test_the_bounds_themselves_are_allowed);
    RUN_TEST(test_nan_is_refused_before_the_range_check);
    RUN_TEST(test_stopped_only_is_refused_while_moving);
    RUN_TEST(test_live_parameters_apply_while_moving);
    RUN_TEST(test_reset_restores_the_default_and_clears_the_override);
    RUN_TEST(test_reset_of_a_stopped_parameter_also_needs_a_stop);
    RUN_TEST(test_reset_all_leaves_stopped_parameters_alone_while_moving);
    RUN_TEST(test_storage_keys_are_short_enough_and_distinct);
    return 0;
}
