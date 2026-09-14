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
 * @file motor.h
 * @brief Motor drivers: PWM, direction, braking and the electrical model.
 *
 * IF A WHEEL TURNS THE WRONG WAY -> flip MOTOR_A_INVERT / MOTOR_B_INVERT here.
 * No rewiring needed.
 *
 * The acceleration ramp lives in control.h (it is part of the control loop),
 * the pins live in pins.h.
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

//---------------------------- PWM / LEDC ---------------------------------//
#define PWM_FREQUENCY           20000     // 20 kHz, above hearing
#define PWM_BITS                10        // 0..1023
#define PWM_MAX                 1023
#define PWM_MIN_MOVE            180       // below this the gearmotor buzzes instead of turning.
                                          // Raise it if a wheel does not always start.
#define LEDC_CH_MOTOR_A         0
#define LEDC_CH_MOTOR_B         1
#define LEDC_CH_FAN             2

//-------------------------- direction / braking --------------------------//
#define MOTOR_A_INVERT          false     // LEFT wheel
#define MOTOR_B_INVERT          true      // RIGHT wheel - wired reversed on the robot;
                                          // see the note in pins.h for how this was found
#define MOTOR_BRAKE_ON_STOP     true      // true = short brake, false = coast

//--------------------- series diode (draft_5) ----------------------------//
// In draft_5 each motor rail has a diode between the INA226 and the TB6612 VM
// pin (it used to sit before the sensor). The sensor therefore reads the rail
// BEFORE the diode, and the driver actually gets
//      V_driver = V_sensor - (VF + I * dynamic resistance)
// Subtracting it matters for the back-EMF wheel-speed model below: at 0.45 V
// on an 11.1 V pack it is a 4 % error, and much more at low duty.
//
// >>> SET THESE TO THE PARTS YOU ACTUALLY FITTED <<<
// Each motor has its OWN diode (MOTOR_A_DIODE / MOTOR_B_DIODE in the drawing),
// so they get their own numbers - fit two different parts and the model still
// works. Typical forward drops:
//   Schottky  (SS34, SR540, 1N5822)  ~0.35 - 0.50 V
//   Standard  (1N4007, 1N5408)       ~0.80 - 1.00 V
// VF is the drop at working current, R_OHM is the slope (0 = flat drop model).
// Measure yours: docs/tuning.md 4d. Both are also editable from the web UI
// (Settings -> มอเตอร์) without reflashing.
// Measured on the bench, 2026-08-27, at working current. These are much
// larger than a small-signal Schottky: at ~12 V they are 15% of the rail, so
// getting them wrong makes the back-EMF wheel speed read high whenever the
// motor model is the fallback. Editable from the web UI (T69).
#define MOTOR_A_DIODE_VF_V      0.181f    // measured, left
#define MOTOR_A_DIODE_R_OHM     0.03f
#define MOTOR_B_DIODE_VF_V      0.168f    // measured, right
#define MOTOR_B_DIODE_R_OHM     0.03f
// The fan buck has a diode too (FAN_DIODE), but it sits after the fan sensor
// and the firmware neither controls nor measures the fans, so it needs no
// entry here - it only costs the fans ~0.4 V.

//------------------------- electrical model ------------------------------//
// Wheel speed worked out from the INA226 voltage and current (back-EMF), so
// the robot has a per-wheel speed without an encoder and can spot a blocked
// wheel or a broken motor wire. Measuring procedure: docs/tuning.md 4c.
//    rpm = ((V_sensor - V_diode) * duty - I * R) / Ke
#define MOTOR_KE_V_PER_RPM      0.12f     // 12 V / 100 rpm output shaft (datasheet)
#define MOTOR_RESISTANCE_OHM    5.0f      // winding + TB6612 + wiring, >>> MEASURE <<<
#define MOTOR_MIN_DUTY_FOR_EST  0.12f     // below this duty the estimate is not published
#define MOTOR_STALL_CURRENT_A   2.5f      // that much current with no back-EMF = blocked wheel.
                                          // MEASURED, not guessed. Free-running on blocks these
                                          // motors draw 0.17-0.33 A; at full speed they peak at
                                          // 1.12 A; and an instant forward/backward reversal at
                                          // full speed - the harshest thing the robot can be
                                          // asked to do - peaks at 2.28 A on rail A and 2.18 A
                                          // on B, because the back-EMF adds to the applied
                                          // voltage instead of opposing it.
                                          //
                                          // So anything at or below ~2.3 A is normal operation.
                                          // It sat at 0.9 A and then 1.0 A, both BELOW the
                                          // legitimate reversal peak - a real stall was
                                          // indistinguishable from a hard direction change, and
                                          // only the 400 ms hold below stopped it firing. 2.5 A
                                          // clears the measured peak with margin and still sits
                                          // well inside the INA226's 3.0 A calibration range.
                                          // Tunable live as motor.stall_a.
#define MOTOR_OPEN_CURRENT_A    0.04f     // commanded but no current = broken wire / dead channel

#endif // MOTOR_CONFIG_H
