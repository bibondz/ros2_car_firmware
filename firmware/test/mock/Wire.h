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
 * Just enough TwoWire for the native tests to compile drivers that take one.
 *
 * The compass driver's CALIBRATION maths is the thing worth testing here, and
 * that never touches the bus - but the class needs a TwoWire* in its signature,
 * so without this stub the whole header is unreachable from the host tests.
 * Nothing here pretends to transfer data: any test that needs real I2C traffic
 * should say so loudly by failing, rather than quietly passing against a fake
 * that always agrees.
 */
#ifndef MOCK_WIRE_H
#define MOCK_WIRE_H

#include <stdint.h>
#include <stddef.h>

class TwoWire {
public:
    void begin() {}
    void begin(int, int, uint32_t) {}
    void setClock(uint32_t) {}
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 1; }   // 1 = no device
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    int read() { return -1; }
    size_t write(uint8_t) { return 0; }
};

extern TwoWire Wire;

#endif  // MOCK_WIRE_H
