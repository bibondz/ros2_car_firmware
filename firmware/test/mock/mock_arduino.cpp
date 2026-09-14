/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
#include "Arduino.h"

unsigned long mock_millis_value = 0;
unsigned long mock_micros_value = 0;

int mock_pin_state[64]   = {0};
int mock_pin_mode[64]    = {0};
int mock_pin_written[64] = {0};
int mock_ledc_duty[8]    = {0};

MockSerial Serial;

// The single TwoWire instance the drivers expect. See mock/Wire.h.
#include "Wire.h"
TwoWire Wire;
