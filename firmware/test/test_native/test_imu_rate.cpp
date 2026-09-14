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
 * Report rates must describe recent traffic and remain sane after millis()
 * wraps. Lifetime counters with a wrapped elapsed time once reported millions
 * of Hz, while lifetime averages hid changes to the congested I2C bus.
 */
#include <unity.h>
#include "../../lib/imu/imu_bno085.h"

namespace {

void beginAt(ImuBno085 &imu, uint32_t now) {
    mock_reset_clock();
    mock_millis_value = now;
    Adafruit_BNO08x::mockReports(SH2_ROTATION_VECTOR, 0);
    TEST_ASSERT_TRUE(imu.begin(&Wire, 0x4A, ImuBno085::MODE_BOTH));
}

void advance(uint32_t ms) {
    // unsigned long is 64-bit on the host, but the ESP32 clock wraps at 32 bits.
    mock_millis_value = (uint32_t)(mock_millis_value + ms);
}

void reports(ImuBno085 &imu, uint8_t sensor_id, uint8_t count) {
    Adafruit_BNO08x::mockReports(sensor_id, count);
    imu.update(count);
}

void traffic(ImuBno085 &imu, uint8_t rv, uint8_t game, uint8_t gyro, uint8_t acc) {
    reports(imu, SH2_ROTATION_VECTOR, rv);
    reports(imu, SH2_GAME_ROTATION_VECTOR, game);
    reports(imu, SH2_GYROSCOPE_CALIBRATED, gyro);
    reports(imu, SH2_LINEAR_ACCELERATION, acc);
}

void assertRates(ImuBno085 &imu, float rv, float game, float gyro, float acc) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, rv, imu.rvHz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, game, imu.gameHz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, gyro, imu.gyroHz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, acc, imu.accHz());
}

}  // namespace

void test_imu_rates_survive_millis_rollover(void) {
    ImuBno085 imu;
    beginAt(imu, UINT32_MAX - 499u);
    for (int tick = 0; tick < 10; ++tick) {
        advance(100);
        traffic(imu, 1, 10, 10, 5);
    }
    TEST_ASSERT_EQUAL_UINT32(1000, imu.rateWindowMs());
    assertRates(imu, 10, 100, 100, 50);
}

void test_imu_window_restarts_across_rollover_and_forgets_old_traffic(void) {
    ImuBno085 imu;
    beginAt(imu, UINT32_MAX - 4999u);
    for (int tick = 0; tick < 99; ++tick) {
        advance(100);
        traffic(imu, 1, 10, 10, 5);
    }
    assertRates(imu, 10, 100, 100, 50);
    advance(100);
    imu.update();
    TEST_ASSERT_EQUAL_UINT32(0, imu.rateWindowMs());
    TEST_ASSERT_EQUAL_UINT32(0, imu.rvReports());
    TEST_ASSERT_EQUAL_UINT32(0, imu.gameReports());
    TEST_ASSERT_EQUAL_UINT32(0, imu.gyroReports());
    TEST_ASSERT_EQUAL_UINT32(0, imu.accReports());

    // The bus now delivers a different rate for every report. Old healthy
    // traffic must not dilute this diagnostic into a lifetime average.
    advance(1000);
    traffic(imu, 2, 20, 30, 4);
    assertRates(imu, 2, 20, 30, 4);
}

void test_imu_window_expires_without_events_or_diagnostic_reads(void) {
    ImuBno085 imu;
    beginAt(imu, 1234);
    advance(1000);
    traffic(imu, 10, 100, 100, 50);
    advance(10000);
    // An expired measurement must not be published before polling resumes.
    assertRates(imu, 0, 0, 0, 0);
    imu.update();
    TEST_ASSERT_EQUAL_UINT32(0, imu.rateWindowMs());
    advance(1000);
    imu.update();
    assertRates(imu, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_UINT32(0, imu.gyroReports());
}

void test_imu_manual_reset_restarts_counts_and_warmup(void) {
    ImuBno085 imu;
    beginAt(imu, 9000);
    advance(1000);
    traffic(imu, 10, 100, 100, 50);
    imu.resetReportCounts();
    reports(imu, SH2_ROTATION_VECTOR, 1);
    advance(99);
    TEST_ASSERT_EQUAL_FLOAT(0, imu.rvHz());
    advance(1);
    assertRates(imu, 10, 0, 0, 0);
}

int run_imu_rate_tests(void) {
    RUN_TEST(test_imu_rates_survive_millis_rollover);
    RUN_TEST(test_imu_window_restarts_across_rollover_and_forgets_old_traffic);
    RUN_TEST(test_imu_window_expires_without_events_or_diagnostic_reads);
    RUN_TEST(test_imu_manual_reset_restarts_counts_and_warmup);
    return 0;
}
