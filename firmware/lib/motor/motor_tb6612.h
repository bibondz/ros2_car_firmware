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
 * @file motor_tb6612.h
 * @brief TB6612FNG channel driver (PWM + 2 direction pins) using the ESP32 LEDC peripheral.
 *
 * On this board both half-bridges of each TB6612 are paralleled, so one board
 * = one motor and we only need 3 GPIOs: EN (PWMA+PWMB), IN1, IN2.
 *
 * Header-only, no dynamic allocation. LEDC is used directly instead of
 * analogWrite() so each motor keeps its own channel and the 20 kHz / 10-bit
 * setting cannot be changed behind our back by another library.
 *
 * NOTE: arduino-esp32 core 2.x API (platform espressif32 @ ^6.5.0).
 */
#ifndef MOTOR_TB6612_H
#define MOTOR_TB6612_H

#include <Arduino.h>

class MotorTB6612 {
public:
  MotorTB6612() {}

  void begin(int pin_en, int pin_in1, int pin_in2, int ledc_channel,
             uint32_t pwm_freq, uint8_t pwm_bits,
             bool invert = false, bool brake_on_stop = true) {
    pin_en_   = pin_en;
    pin_in1_  = invert ? pin_in2 : pin_in1;
    pin_in2_  = invert ? pin_in1 : pin_in2;
    channel_  = ledc_channel;
    pwm_max_  = (1 << pwm_bits) - 1;
    brake_    = brake_on_stop;

    pinMode(pin_in1_, OUTPUT);
    pinMode(pin_in2_, OUTPUT);
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);

    ledcSetup(channel_, pwm_freq, pwm_bits);
    ledcAttachPin(pin_en_, channel_);
    ledcWrite(channel_, 0);
    last_pwm_ = 0;
  }

  /** @param pwm signed duty, -pwm_max .. +pwm_max (positive = forward) */
  void spin(int pwm) {
    if (pwm > pwm_max_)  pwm =  pwm_max_;
    if (pwm < -pwm_max_) pwm = -pwm_max_;
    last_pwm_ = pwm;

    if (pwm == 0) { stop(); return; }

    if (pwm > 0) {                  // CW: IN1 = H, IN2 = L
      digitalWrite(pin_in1_, HIGH);
      digitalWrite(pin_in2_, LOW);
    } else {                        // CCW: IN1 = L, IN2 = H
      digitalWrite(pin_in1_, LOW);
      digitalWrite(pin_in2_, HIGH);
    }
    ledcWrite(channel_, abs(pwm));
  }

  /** Short brake (both IN high) or coast (both IN low), PWM forced to 0. */
  void stop() {
    last_pwm_ = 0;
    digitalWrite(pin_in1_, brake_ ? HIGH : LOW);
    digitalWrite(pin_in2_, brake_ ? HIGH : LOW);
    ledcWrite(channel_, brake_ ? pwm_max_ : 0);
  }

  /** Hard cut used by the safety layer: coast, no braking current. */
  void disable() {
    last_pwm_ = 0;
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
    ledcWrite(channel_, 0);
  }

  int  lastPwm() const { return last_pwm_; }
  int  maxPwm()  const { return pwm_max_; }

private:
  int  pin_en_  = -1;
  int  pin_in1_ = -1;
  int  pin_in2_ = -1;
  int  channel_ = 0;
  int  pwm_max_ = 1023;
  bool brake_   = true;
  int  last_pwm_ = 0;
};

#endif // MOTOR_TB6612_H
