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
 * Minimal Arduino shim so the pure-logic libraries can be compiled and tested
 * on a PC (`pio test -e native`). Only what the tested code touches is here -
 * anything that needs real hardware is not part of the native tests.
 */
#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ---- fake clock, driven by the tests -------------------------------------
extern unsigned long mock_millis_value;
extern unsigned long mock_micros_value;

inline unsigned long millis() { return mock_millis_value; }
inline unsigned long micros() { return mock_micros_value; }
inline void delay(unsigned long ms) { mock_millis_value += ms; mock_micros_value += ms * 1000UL; }

/** Advance the fake clock (both millis and micros stay consistent). */
inline void mock_advance_ms(unsigned long ms) {
  mock_millis_value += ms;
  mock_micros_value += ms * 1000UL;
}
inline void mock_reset_clock() { mock_millis_value = 0; mock_micros_value = 0; }

// ---- pin API (recorded, never touches hardware) --------------------------
#define INPUT          0x0
#define OUTPUT         0x1
#define INPUT_PULLUP   0x2
#define INPUT_PULLDOWN 0x3
#define LOW            0x0
#define HIGH           0x1

extern int mock_pin_state[64];
extern int mock_pin_mode[64];
extern int mock_pin_written[64];

inline void pinMode(int pin, int mode) { if (pin >= 0 && pin < 64) mock_pin_mode[pin] = mode; }
inline void digitalWrite(int pin, int value) { if (pin >= 0 && pin < 64) mock_pin_written[pin] = value; }
inline int  digitalRead(int pin) { return (pin >= 0 && pin < 64) ? mock_pin_state[pin] : LOW; }
inline void mock_set_pin(int pin, int value) { if (pin >= 0 && pin < 64) mock_pin_state[pin] = value; }

// ---- ledc (ESP32 PWM) ----------------------------------------------------
extern int mock_ledc_duty[8];
inline void ledcSetup(int, uint32_t, uint8_t) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int channel, int duty) { if (channel >= 0 && channel < 8) mock_ledc_duty[channel] = duty; }

// ---- misc ----------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T> T constrain(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}
template <typename T> T abs_(T v) { return v < 0 ? -v : v; }
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
#ifndef sq
#define sq(x) ((x) * (x))
#endif

// ---- IPAddress -----------------------------------------------------------
// config/network.h declares a few of these at namespace scope. The host tests
// never send anything over the network, so this only has to be constructible
// and printable enough to satisfy the compiler.
class IPAddress {
public:
  IPAddress() : octets_{0, 0, 0, 0} {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : octets_{a, b, c, d} {}
  uint8_t operator[](int i) const { return octets_[i & 3]; }
  bool operator==(const IPAddress &o) const {
    return memcmp(octets_, o.octets_, 4) == 0;
  }

private:
  uint8_t octets_[4];
};

struct MockSerial {
  void begin(unsigned long) {}
  void println(const char *) {}
  void print(const char *) {}
  int  printf(const char *, ...) { return 0; }
};
extern MockSerial Serial;

#endif // ARDUINO_MOCK_H
