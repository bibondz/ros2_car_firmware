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
 * @file imu.h
 * @brief BNO085 address and mounting orientation.
 *
 * HEADING CONVENTION FOR THE WHOLE PROJECT: compass degrees.
 *   0 = north, 90 = east, positive = clockwise - the same numbers as the GPS
 *   course and as the bearing to a waypoint. The BNO085 reports yaw
 *   counter-clockwise, hence the -1 below. ROS odometry is converted back to
 *   ENU when it is published.
 *
 * How to check the signs: docs/tuning.md 2.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#define BNO085_I2C_ADDR         0x4A      // 0x4B if the ADR pad is bridged
// -1 means the sensor's RESET line goes nowhere, and that costs more than it
// looks like it should.
//
// WHAT GOES WRONG. Pulsing the ESP32's EN does not cut the BNO085's power - the
// 3V3 regulator keeps running - so a SOFTWARE reset never resets the sensor. It
// carries on streaming the reports the previous session asked for, and
// sh2_open()'s handshake then runs against a stream already in progress.
// Sometimes it syncs, often it does not. Measured on this robot: pulling the
// battery brings the IMU up, and a USB flash or an over-the-air update leaves
// it dead - while the sensor plainly answers its address AND returns a real
// SHTP packet, so nothing looks broken.
//
// BEWARE THE RESET REASON. esp_reset_reason() reports ESP_RST_POWERON for an EN
// pulse as well as for real power removal, so a board reset by esptool looks
// identical in the log to one that was unplugged - and only the second actually
// resets the sensor. Do not use the reset reason to decide whether the IMU
// should have come up; it cannot tell the two apart.
//
// Every software workaround was tried and none of them fixes it: retrying
// begin_I2C on a timer, sending the SHTP soft reset by hand (which is worse -
// the Adafruit HAL already sends the identical packet and waits 300 ms), and
// draining the sensor's backlog before the handshake. The sensor needs a real
// reset, and only two things can give it one: its RESET pin, or removing power.
//
// WHAT ACTUALLY CLEARS IT, PROVED ON HARDWARE: removing power from the whole
// robot for about ten seconds - the battery AND the USB cable, because the
// ESP32 and this sensor share the 3V3 rail and USB alone keeps them both
// alive. After that it comes up first time, every time.
//
// So the recovery is an operator action, not a code path, and the firmware's
// job is to SAY SO rather than to keep trying: the calibration endpoint reports
// -100 with exactly that instruction, and the web repeats it. A retry button is
// provided for after the power-down, so nobody has to reflash to clear a
// counter.
#define BNO085_RESET_PIN        -1        // not wired - see above, use 33 when it is
#define IMU_STALE_MS            500       // no event for this long -> imu_ok = false

// Which body axis points to the front of the robot.
//
// THE SENSOR IS MOUNTED BACKWARDS ON THIS ROBOT. Its printed X arrow points at
// the rear, so +X is the reverse direction and the sign here is negative.
//
// This was wrong for a long time and it was not a cosmetic error. With the sign
// at +1 the estimator was fed forward acceleration NEGATED, so it believed the
// robot accelerated backwards whenever it drove forwards. Measured on the
// floor: a steady 0.18 m/s forward command produced a fused speed reading
// anywhere from -0.36 to +0.34 m/s, negative while the robot was plainly moving
// forwards. That was blamed on integration drift at the time. It was not drift;
// it was this.
#define IMU_ACC_FORWARD_AXIS    0         // 0 = X, 1 = Y, 2 = Z
#define IMU_ACC_FORWARD_SIGN    (-1.0f)   // X arrow points at the REAR

// WHICH REPORTS TO ASK THE BNO085 FOR.  0 = game rotation vector only,
// 1 = magnetometer-referenced rotation vector only, 2 = both.
//
// This used to be derived from HEADING_SOURCE, which was wrong twice over. That
// value tells the ESTIMATOR which absolute references it may use; it has
// nothing to say about which reports the sensor should publish, and welding the
// two together meant you could not choose one without changing the other.
//
// It is 0 - one fusion - because the BNO085 does not run two. Asking for both
// was measured from opposite directions and it always starves one of them: with
// the magnetometer calibration running, the game vector got 50-70 Hz and the
// magnetometer-referenced one 0.0-0.5 Hz of the 10 it asked for; saving the
// calibration flipped it, giving the mag-referenced one 9.6 Hz and starving the
// game vector to 2-7. Neither is acceptable, because BOTH of the things this
// robot reads from the sensor come from the rotation vector:
//
//   - the heading backbone, which is the sensor's OWN integrated orientation,
//     not a gyro rate integrated in our control loop. The sensor corrects its
//     own gyro drift against gravity at its own rate; redoing that here would
//     be strictly worse, because every late cycle is rotation that silently
//     never happened.
//   - roll and pitch, which tilt-compensate the QMC5883L. A stale attitude
//     makes the compass wrong on any slope - so a starved rotation vector
//     quietly degrades the other heading source too.
//
// The GAME vector is the one to keep. It is the sensor's integrated
// orientation, immune to magnets because it uses no magnetometer at all, and
// north is supplied separately by the compass and the GPS course, both of which
// are cross-checked against it. The magnetometer-referenced vector would give
// absolute heading directly, but from a magnetometer sitting next to the
// motors - the sensor this robot trusts least, made the backbone of everything.
#define IMU_REPORT_MODE         0

// Sensor frame -> robot heading convention
#define IMU_YAW_TO_COMPASS_SIGN (-1.0f)   // CCW sensor yaw -> CW compass sense
#define IMU_GYRO_Z_SIGN         (+1.0f)   // -1.0f only if the board is mounted upside down
// 180 degrees, for the same reason as the acceleration sign above: the sensor
// faces the back of the robot, so its idea of "forward" is the robot's "back".
// Both of these describe ONE physical fact and must be changed together - a
// heading squared up while the acceleration still points the wrong way is worse
// than both being wrong, because it looks right.
#define IMU_YAW_OFFSET_DEG      180.0f    // squares the sensor to the chassis

#endif // IMU_CONFIG_H
