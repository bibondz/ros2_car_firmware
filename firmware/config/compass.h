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
 * @file compass.h
 * @brief QMC5883L magnetometer built into the GEP-M10-DQ GPS module.
 *
 * This is what tells the robot which way it points BEFORE it starts moving
 * (the GPS course only exists while driving). Mounted on the antenna mast it
 * sits far from the motors, which makes it the better of the two
 * magnetometers on the robot.
 *
 * Calibrate it once from the web UI (Dashboard -> ปรับเข็มทิศ). The result is
 * stored in the ESP32 flash and survives a reboot. Procedure: docs/tuning.md 2b.
 */
#ifndef COMPASS_CONFIG_H
#define COMPASS_CONFIG_H

#define USE_QMC5883L            1         // 0 = ignore it, fall back to the BNO085 + GPS course
#define QMC5883L_ADDR           0x0D

// Magnetic declination for your area. Look it up at magnetic-declination.com
// (Thailand is roughly +0.5 to +1 degree east).
#define COMPASS_DECLINATION_DEG 0.6f

#define COMPASS_MOUNT_OFFSET_DEG 0.0f     // added last: squares the module to the chassis
#define COMPASS_INVERT          0         // 1 if the heading counts the wrong way round
#define COMPASS_FIELD_TOL       0.35f     // reject a sample when |B| differs from the
                                          // calibrated value by more than 35 % (interference)

#endif // COMPASS_CONFIG_H
