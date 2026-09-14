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
 * @file control.h
 * @brief Control loop rates, PID gains and state estimator tuning.
 *
 * Every gain here can also be changed at runtime from the web UI
 * (topic /gps_localize/config/pid) - these are only the power-on defaults.
 */
#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

//===================== Loop / publish rates =================//
#define CTRL_PERIOD_MS          10        // 100 Hz control loop
// Publish rates are a POWER budget as much as a bandwidth one. Every publish is
// a Wi-Fi transmission, and the radio's transmit burst is the largest current
// draw on the board. Measured here: with the control loop stuck at 7.5 Hz the
// board ran for 20 minutes without a blink; the moment the loop was fixed and
// reached 96 Hz, the radio duty went up with it and the ESP32 began resetting
// with rst:0x1 (POWERON_RESET) roughly every 25 seconds - a supply collapse,
// not a crash. 50 Hz IMU over Wi-Fi buys nothing a host-side filter cannot get
// from 20 Hz, so it is not worth a reset.
#define PUB_ODOM_DIV            10        // 100/10 = 10 Hz odometry
#define PUB_IMU_DIV             5         // 100/5  = 20 Hz imu
#define PUB_TELEMETRY_DIV       10        // 100/10 = 10 Hz telemetry
#define INA_READ_DIV            20        // 100/20 =  5 Hz power sensors

//====================== Heading PID =========================//
// Input : heading error [deg], -180..180
// Output: differential PWM added to the left wheel and removed from the right
#define HEADING_KP              9.0f
#define HEADING_KI              0.35f
#define HEADING_KD              1.2f
#define HEADING_KF              0.0f
#define HEADING_I_MAX           250.0f
#define HEADING_I_MIN           (-HEADING_I_MAX)
#define HEADING_ERROR_TOL_DEG   1.5f      // deadband
#define HEADING_D_FILTER_HZ     8.0f
#define HEADING_OUT_MAX         600.0f    // max differential PWM
#define HEADING_SPIN_ERR_DEG    45.0f     // error above this -> turn on the spot before driving
#define HEADING_SPIN_EXIT_DEG   12.0f     // stop spinning once the error drops below this
#define HEADING_SPIN_PWM        420       // PWM used while turning on the spot

// ---------------------------------------------------------------- manual hold
//
// Manual driving used to be entirely open loop: the jog buttons produced a
// speed and a turn rate, those became two PWM numbers, and nothing ever looked
// at where the robot actually pointed. Two motors are never identical - this
// robot's own diode drops differ by 0.09 V and its drivers are not matched - so
// "forward" curved, and the operator had to keep tapping a turn button to
// fight it.
//
// With hold on, commanding straight latches the heading at that instant and the
// heading PID corrects the drift. Pressing a turn button releases the hold and
// re-latches it on release, so steering still feels direct.
//
// It is deliberately weak. This is a trim, not a controller: it should take out
// a slow drift without ever fighting the operator, and the clamp below is what
// guarantees that. A hold that can out-pull the jog command is worse than no
// hold at all.
#define MANUAL_HOLD_HEADING     1         // 0 disables, back to pure open loop
#define MANUAL_HOLD_MAX_PWM     90        // ceiling on the correction, of PWM_MAX
#define MANUAL_HOLD_MIN_SPEED   0.02f     // below this it is not really driving

// Before latching a heading to hold, wait until the robot has really stopped
// turning AND the heading estimate has caught up with it.
//
// Latching the instant the turn command stopped produced a small kick back the
// other way: the fused heading lags the robot - the compass is eased in over
// seconds and the IMU's own fusion has its own delay - so the value latched was
// where the robot had been a moment ago, not where it was pointing. The hold
// then did exactly what it was told and steered back to that stale heading.
// Reported from the robot as "when I stop turning it turns back a little, then
// goes forward".
//
// A rate threshold alone is not enough, because the estimate is still catching
// up for a moment after the rotation itself has stopped. The settle time is
// what covers that.
#define MANUAL_HOLD_SETTLE_DPS  12.0f     // yaw rate below this counts as stopped
#define MANUAL_HOLD_SETTLE_S    0.35f     // and it must stay there this long

// Close the speed loop in manual too, not just the heading one.
//
// The jog buttons produced a PWM straight from the requested speed and stopped
// there, so the number on the screen was a request rather than a result: the
// same 0.18 m/s meant one speed on a fresh battery, another on a flat one, and
// another again up a slope or on carpet. The web page said "the robot drives at
// exactly the speed set above", and that was not true.
//
// ONE THING MAKES THIS SUBTLE. The fused speed is only worth feeding back when
// it is measured independently of the motors. With no GPS and no IMU the
// estimator falls back to the motor model, which is computed FROM the PWM - so
// closing a loop on it compares the output with itself, learns nothing, and
// winds the integral up chasing an error that cannot move. The guard in
// updateManual() is not defensive tidiness; without it the robot accelerates on
// carpet because the model says it is going slower than it is.
#define MANUAL_SPEED_PID        1         // 0 disables, back to feed-forward only
#define MANUAL_SPEED_MAX_PWM    260       // ceiling on the speed correction
#define TURN_IN_PLACE_TOL_DEG   3.0f      // "turn to 90 deg" finishes inside this
#define TURN_IN_PLACE_SETTLE_MS 250       // and must stay inside it this long
// How long a turn-on-the-spot may take before it gives up.
//
// A turn is a ONE-SHOT command: it is meant to outlive the message that asked
// for it, so the command timeout deliberately does not apply to it. That leaves
// nothing bounding it, and a turn that can never reach its tolerance - a frozen
// heading, a wheel against a kerb - would spin both motors for ever. This is
// the bound. Reaching it stops the turn and says so; it does not latch a fault,
// because failing to face a direction is not a reason to end a mission.
#define TURN_IN_PLACE_MAX_MS    20000

//======================= Speed PI ===========================//
// Input : speed error [m/s] (target - fused estimate)
// Output: PWM correction added on top of the feed-forward term
#define SPEED_KP                420.0f
#define SPEED_KI                160.0f
#define SPEED_KD                0.0f
#define SPEED_KF                0.0f
#define SPEED_I_MAX             400.0f
#define SPEED_I_MIN             (-SPEED_I_MAX)
#define SPEED_ERROR_TOL_MPS     0.01f
#define SPEED_D_FILTER_HZ       5.0f
#define SPEED_FF_GAIN           1.0f      // 1.0 = full open-loop feed-forward (pwm = v/vmax * PWM_MAX)
#define SPEED_SLEW_MPS_S        0.6f      // ramp on the speed setpoint [m/s per second]

//==================== Motor acceleration ramp ===============//
// Applied to the PWM that actually reaches the TB6612, on top of the setpoint
// slew above. It limits inrush current, protects the gearbox and stops the
// robot from lurching when a command arrives as a step.
//   MOTOR_ACCEL_RAMP_S = seconds from stopped to full PWM
//   per 100 Hz control tick that is  PWM_MAX * dt / ramp = 1023 * 0.01 / 0.6 = 17 counts
//   as a fraction of full scale       0.017 per tick at 100 Hz  (= 0.17 at 10 Hz)
// An emergency stop bypasses the ramp completely (motor.disable()).
#define MOTOR_ACCEL_RAMP_S      0.6f      // soft start
#define MOTOR_DECEL_RAMP_S      0.3f      // stopping is allowed to be twice as quick

//=================== Heading estimator ======================//
// 0 = BNO085 rotation vector only (its own magnetometer, absolute but it sits
//     right next to the motors)
// 1 = BNO085 gyro, aligned to the GPS course while moving (relative until the
//     robot has driven a few metres)
// 2 = GPS course over ground only (only usable while moving)
// 3 = DEFAULT. BNO085 gyro, seeded by the QMC5883L compass in the GPS module
//     while standing still, refined by the GPS course while moving.
//     This is the one that knows which way the robot points BEFORE it starts.
#define HEADING_SOURCE          3
#define HEADING_ALIGN_MIN_MPS   0.25f     // only trust GPS course above this speed
#define HEADING_ALIGN_TAU_S     6.0f      // time constant of the IMU->GPS alignment [s]
#define HEADING_ALIGN_MAX_DPS   15.0f     // clamp on alignment correction rate [deg/s]
#define HEADING_MAG_TAU_S       4.0f      // time constant of the IMU->compass alignment [s]
#define HEADING_MAG_STILL_MPS   0.10f     // below this the robot counts as standing still
#define MAG_READ_DIV            5         // 100/5 = 20 Hz compass reads

//==================== Speed estimator =======================//
// Complementary filter: IMU longitudinal acceleration (fast) + GPS ground
// speed (slow but drift free). Falls back to the motor model when GPS is lost.
#define SPEED_EST_GPS_TAU_S     1.5f      // GPS pull-in time constant [s]
#define SPEED_EST_MODEL_TAU_S   2.5f      // motor-model pull-in when GPS is unavailable [s]
#define SPEED_EST_ACC_DEADBAND  0.08f     // ignore |accel| below this [m/s^2] (sensor noise)
#define SPEED_EST_MAX_MPS       1.5f      // hard clamp, protects against integrator runaway

//======================= Odometry ===========================//
#define ODOM_FRAME_ID           "odom"
#define ODOM_CHILD_FRAME_ID     "base_link"
#define IMU_FRAME_ID            "imu_link"
#define GPS_FRAME_ID            "gps_link"

#endif // CONTROL_CONFIG_H
