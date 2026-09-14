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
 * Native test runner:  pio test -e native
 *
 * Covers the logic that decides how the robot behaves - controller, estimator,
 * safety chain and the NMEA parser. Anything that needs real I2C/UART/PWM
 * silicon is not tested here; that is what the bench checklist in
 * docs/acceptance.md is for.
 */
#include <unity.h>

int run_pidf_tests(void);
int run_estimator_tests(void);
int run_drive_tests(void);
int run_motor_model_tests(void);
int run_safety_gps_tests(void);
int run_fusion_tests(void);
int run_pose_ekf_tests(void);
int run_lever_arm_tests(void);
int run_param_tests(void);
int run_wifi_store_tests(void);
int run_console_tests(void);
int run_compass_tests(void);
int run_mount_rotation_tests(void);
int run_imu_rate_tests(void);

int main(int, char **) {
    UNITY_BEGIN();
    run_pidf_tests();
    run_estimator_tests();
    run_drive_tests();
    run_motor_model_tests();
    run_safety_gps_tests();
    run_fusion_tests();
    run_pose_ekf_tests();
    run_lever_arm_tests();
    run_param_tests();
    run_wifi_store_tests();
    run_console_tests();
    run_compass_tests();
    run_mount_rotation_tests();
    run_imu_rate_tests();
    return UNITY_END();
}
