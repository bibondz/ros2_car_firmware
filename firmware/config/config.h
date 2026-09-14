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
 * @file config.h
 * @brief Configuration index - which file do I edit?
 *
 *  ┌────────────────┬──────────────────────────────────────────────────────┐
 *  │ pins.h         │ GPIO map. Only touch it if you rewire the board.     │
 *  │ robot.h        │ Wheel size, track width, top speed.                  │
 *  │ motor.h        │ Motor driver, direction, PWM, electrical model.      │
 *  │ imu.h          │ BNO085: address, mounting orientation.               │
 *  │ compass.h      │ QMC5883L in the GPS module: declination, mounting.   │
 *  │ gps.h          │ GPS module: baud rate, timeouts.                     │
 *  │ power.h        │ 3x INA226 addresses and shunts, battery cut-off.     │
 *  │ control.h      │ Loop rates, PID gains, ramps, estimator tuning.      │
 *  │ safety.h       │ Watchdog timeouts, stop reasons, LED patterns.       │
 *  │ network.h      │ Wi-Fi, micro-ROS agent hostname/IP, ROS domain.      │
 *  │ version.h      │ Firmware version, published so the host can compare. │
 *  └────────────────┴──────────────────────────────────────────────────────┘
 *
 * Most values here also exist in the YAML files on the ROS side, where the
 * customer can change them from the web UI without reflashing (docs/topics.md,
 * "config/pid" selectors). The values in these headers are the power-on
 * defaults the board falls back to.
 */
#ifndef CONFIG_H
#define CONFIG_H

    #include "pins.h"       // GPIO map
    #include "robot.h"      // physical robot
    #include "motor.h"      // drivers, PWM, motor model
    #include "imu.h"        // BNO085
    #include "compass.h"    // QMC5883L
    #include "gps.h"        // GEP-M10-DQ
    #include "power.h"      // INA226 + battery
    #include "control.h"    // rates, PID, estimator
    #include "safety.h"     // watchdogs, stop flags
    #include "network.h"    // Wi-Fi + micro-ROS agent
    #include "version.h"    // what firmware this is

#endif // CONFIG_H
