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
 * Scripted BNO085 reports for testing the driver's accounting against the fake
 * clock. This does not simulate I2C or prove that a real sensor can initialise;
 * it exercises the same update() and diagnostic getters used by the firmware.
 */
#ifndef MOCK_ADAFRUIT_BNO08X_H
#define MOCK_ADAFRUIT_BNO08X_H

#include <stdint.h>
#include <Wire.h>

enum {
    SH2_ROTATION_VECTOR = 1,
    SH2_GAME_ROTATION_VECTOR,
    SH2_GYROSCOPE_CALIBRATED,
    SH2_LINEAR_ACCELERATION,
    SH2_CAL_ACCEL = 1,
    SH2_CAL_GYRO = 2,
    SH2_CAL_MAG = 4,
    SH2_OK = 0
};

struct sh2_SensorValue_t {
    uint8_t sensorId = 0;
    uint8_t status = 0;
    struct {
        struct { float real = 1, i = 0, j = 0, k = 0, accuracy = 0; } rotationVector;
        struct { float real = 1, i = 0, j = 0, k = 0; } gameRotationVector;
        struct { float x = 0, y = 0, z = 0; } gyroscope, linearAcceleration;
    } un;
};

inline int sh2_setCalConfig(uint8_t) { return SH2_OK; }
inline int sh2_saveDcdNow() { return SH2_OK; }
inline int sh2_clearDcdAndReset() { return SH2_OK; }

class Adafruit_BNO08x {
public:
    explicit Adafruit_BNO08x(int = -1) {}
    bool begin_I2C(uint8_t, TwoWire *) { return true; }
    bool enableReport(uint8_t, uint32_t) { return true; }
    bool wasReset() { return false; }
    bool getSensorEvent(sh2_SensorValue_t *event) {
        if (!pending_) return false;
        --pending_;
        *event = sh2_SensorValue_t{};
        event->sensorId = sensor_id_;
        return true;
    }

    // Tests drain one report type at a time so queue timing stays explicit.
    static void mockReports(uint8_t sensor_id, uint8_t count) {
        sensor_id_ = sensor_id;
        pending_ = count;
    }

private:
    inline static uint8_t sensor_id_ = 0;
    inline static uint8_t pending_ = 0;
};

#endif  // MOCK_ADAFRUIT_BNO08X_H
