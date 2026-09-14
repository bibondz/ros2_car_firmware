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
 * The tunables, declared once.
 *
 * WHAT BELONGS HERE
 *
 * Anything that is a property of THIS robot rather than of the design: a
 * measured diode drop, a wheel that wore down, where the antenna ended up. Add
 * a line and the web settings page grows a field, because the page is built
 * from whatever the board reports.
 *
 * WHAT DOES NOT
 *
 * Pin numbers. A GPIO describes how the board is soldered, not a setting, and
 * making one changeable at runtime would turn a wiring mistake into a puzzle
 * with no error message. pins.h stays compiled in, and the UI shows it
 * read-only.
 *
 * LIVE vs STOPPED
 *
 * LIVE is safe to change while driving. STOPPED is refused while the robot is
 * moving - not because the write is dangerous, but because the value describes
 * the machine the control loops are steering. Changing the wheel diameter or
 * the track width mid-drive silently redefines what every speed and turn rate
 * means, and the robot will do something surprising for a second or two.
 */
#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H

#include <config.h>
#include "param_registry.h"

// Keep in step with the array below. The registry is statically sized, so this
// is what stops it silently truncating when a parameter is added.
#define PARAM_COUNT 90

static const ParamDef PARAM_TABLE[PARAM_COUNT] = {
  // ---- motor electrical model -------------------------------------------
  // These feed the back-EMF wheel-speed estimate, which is what the robot
  // falls back to whenever GPS is unavailable. The diode drops shipped at
  // 0.45 V from a datasheet and measure 1.81 V and 1.90 V - at 12 V that is
  // 15% of the rail, and the derived speed reads high by the same margin.
  {"motor.a.diode_vf", "V", "motor", MOTOR_A_DIODE_VF_V, 0.0f, 3.0f, PARAM_LIVE,
   "Measured forward drop of motor A's series diode. Measure across it with the motor running."},
  {"motor.b.diode_vf", "V", "motor", MOTOR_B_DIODE_VF_V, 0.0f, 3.0f, PARAM_LIVE,
   "Measured forward drop of motor B's series diode."},
  {"motor.a.diode_r", "ohm", "motor", MOTOR_A_DIODE_R_OHM, 0.0f, 1.0f, PARAM_LIVE,
   "Dynamic resistance of motor A's diode - the slope of its V/I curve."},
  {"motor.b.diode_r", "ohm", "motor", MOTOR_B_DIODE_R_OHM, 0.0f, 1.0f, PARAM_LIVE,
   "Dynamic resistance of motor B's diode."},
  {"motor.resistance", "ohm", "motor", MOTOR_RESISTANCE_OHM, 0.1f, 50.0f, PARAM_LIVE,
   "Winding + driver + wiring resistance. Measure across a stalled motor at a known duty."},
  {"motor.ke", "V/rpm", "motor", MOTOR_KE_V_PER_RPM, 0.0001f, 1.0f, PARAM_LIVE,
   "Back-EMF constant. Spin the motor at a known rpm and measure the open-circuit volts."},
  {"motor.stall_a", "A", "motor", MOTOR_STALL_CURRENT_A, 0.1f, 10.0f, PARAM_LIVE,
   "Current that means the motor is stalled rather than merely loaded."},

  // ---- physical robot ----------------------------------------------------
  // STOPPED: these define what a metre and a degree mean to every control loop.
  {"robot.wheel_diameter", "m", "robot", WHEEL_DIAMETER_M, 0.02f, 0.5f, PARAM_STOPPED,
   "Wheel diameter. Measure loaded, not off the shelf - a soft tyre reads smaller."},
  {"robot.track", "m", "robot", WHEEL_TRACK_M, 0.05f, 2.0f, PARAM_STOPPED,
   "Centre of the left drive wheel to centre of the right."},
  {"robot.max_speed", "m/s", "robot", ROBOT_MAX_SPEED_MPS, 0.05f, 5.0f, PARAM_STOPPED,
   "Fastest the robot may be commanded to drive."},

  // ---- where the sensors sit --------------------------------------------
  // A sensor offset from the drive axle sweeps an arc when the robot turns,
  // and the filter reads that arc as travel. Measured from the midpoint
  // between the drive wheels: x forward, y left.
  // ---- where everything is bolted, and which way it faces ----------------
  // All measured from the MIDDLE OF THE ROBOT, x forward, y left, z up. See
  // firmware/config/robot.h for the frame drawing and for which of these
  // rotations actually change the maths - two of them are recorded only, and
  // say so in their own help text rather than being quietly ignored.
  //
  // The drive axle midpoint is NOT here: it is halfway between the two drive
  // wheels, so it is derived and cannot disagree with them.
  {"geometry.wheel.l.x", "m", "geometry", WHEEL_L_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "left drive wheel: forward of the middle of the robot (+)"},
  {"geometry.wheel.l.y", "m", "geometry", WHEEL_L_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "left drive wheel: left of the middle of the robot (+)"},
  {"geometry.wheel.l.z", "m", "geometry", WHEEL_L_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "left drive wheel: above the middle of the robot (+)"},
  {"geometry.wheel.l.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "left drive wheel: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.wheel.l.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "left drive wheel roll, about the forward axis. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.l.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "left drive wheel pitch, nose up positive. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.l.yaw", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "left drive wheel yaw, turned left positive. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.l.qw", "", "geometry", WHEEL_L_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "left drive wheel mounting rotation qw (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.l.qx", "", "geometry", WHEEL_L_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "left drive wheel mounting rotation qx (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.l.qy", "", "geometry", WHEEL_L_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "left drive wheel mounting rotation qy (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.l.qz", "", "geometry", WHEEL_L_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "left drive wheel mounting rotation qz (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.r.x", "m", "geometry", WHEEL_R_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "right drive wheel: forward of the middle of the robot (+)"},
  {"geometry.wheel.r.y", "m", "geometry", WHEEL_R_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "right drive wheel: left of the middle of the robot (+)"},
  {"geometry.wheel.r.z", "m", "geometry", WHEEL_R_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "right drive wheel: above the middle of the robot (+)"},
  {"geometry.wheel.r.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "right drive wheel: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.wheel.r.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "right drive wheel roll, about the forward axis. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.r.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "right drive wheel pitch, nose up positive. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.r.yaw", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "right drive wheel yaw, turned left positive. Used when rot_mode is 0. Used by the maths."},
  {"geometry.wheel.r.qw", "", "geometry", WHEEL_R_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "right drive wheel mounting rotation qw (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.r.qx", "", "geometry", WHEEL_R_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "right drive wheel mounting rotation qx (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.r.qy", "", "geometry", WHEEL_R_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "right drive wheel mounting rotation qy (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.wheel.r.qz", "", "geometry", WHEEL_R_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "right drive wheel mounting rotation qz (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.caster.fl.x", "m", "geometry", CASTER_FL_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-left caster: forward of the middle of the robot (+)"},
  {"geometry.caster.fl.y", "m", "geometry", CASTER_FL_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-left caster: left of the middle of the robot (+)"},
  {"geometry.caster.fl.z", "m", "geometry", CASTER_FL_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-left caster: above the middle of the robot (+)"},
  {"geometry.caster.fl.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "front-left caster: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.caster.fl.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "front-left caster roll, about the forward axis. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fl.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "front-left caster pitch, nose up positive. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fl.yaw", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "front-left caster yaw, turned left positive. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fl.qw", "", "geometry", CASTER_FL_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "front-left caster mounting rotation qw (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fl.qx", "", "geometry", CASTER_FL_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "front-left caster mounting rotation qx (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fl.qy", "", "geometry", CASTER_FL_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "front-left caster mounting rotation qy (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fl.qz", "", "geometry", CASTER_FL_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "front-left caster mounting rotation qz (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fr.x", "m", "geometry", CASTER_FR_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-right caster: forward of the middle of the robot (+)"},
  {"geometry.caster.fr.y", "m", "geometry", CASTER_FR_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-right caster: left of the middle of the robot (+)"},
  {"geometry.caster.fr.z", "m", "geometry", CASTER_FR_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "front-right caster: above the middle of the robot (+)"},
  {"geometry.caster.fr.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "front-right caster: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.caster.fr.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "front-right caster roll, about the forward axis. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fr.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "front-right caster pitch, nose up positive. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fr.yaw", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "front-right caster yaw, turned left positive. Used when rot_mode is 0. Recorded only - a caster swivels, so its resting facing means nothing."},
  {"geometry.caster.fr.qw", "", "geometry", CASTER_FR_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "front-right caster mounting rotation qw (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fr.qx", "", "geometry", CASTER_FR_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "front-right caster mounting rotation qx (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fr.qy", "", "geometry", CASTER_FR_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "front-right caster mounting rotation qy (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.caster.fr.qz", "", "geometry", CASTER_FR_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "front-right caster mounting rotation qz (identity is 1,0,0,0). Recorded only. A caster swivels, so its resting facing means nothing to the maths."},
  {"geometry.gps.x", "m", "geometry", GPS_OFFSET_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "GPS antenna: forward of the middle of the robot (+)"},
  {"geometry.gps.y", "m", "geometry", GPS_OFFSET_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "GPS antenna: left of the middle of the robot (+)"},
  {"geometry.gps.z", "m", "geometry", GPS_OFFSET_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "GPS antenna: above the middle of the robot (+)"},
  {"geometry.gps.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "GPS antenna: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.gps.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "GPS antenna roll, about the forward axis. Used when rot_mode is 0. Recorded only - a single antenna is a point and has no facing."},
  {"geometry.gps.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "GPS antenna pitch, nose up positive. Used when rot_mode is 0. Recorded only - a single antenna is a point and has no facing."},
  {"geometry.gps.yaw", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "GPS antenna yaw, turned left positive. Used when rot_mode is 0. Recorded only - a single antenna is a point and has no facing."},
  {"geometry.gps.qw", "", "geometry", GPS_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "GPS antenna mounting rotation qw (identity is 1,0,0,0). Recorded only. A single antenna is a point and has no facing; this would matter for a dual-antenna heading receiver."},
  {"geometry.gps.qx", "", "geometry", GPS_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "GPS antenna mounting rotation qx (identity is 1,0,0,0). Recorded only. A single antenna is a point and has no facing; this would matter for a dual-antenna heading receiver."},
  {"geometry.gps.qy", "", "geometry", GPS_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "GPS antenna mounting rotation qy (identity is 1,0,0,0). Recorded only. A single antenna is a point and has no facing; this would matter for a dual-antenna heading receiver."},
  {"geometry.gps.qz", "", "geometry", GPS_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "GPS antenna mounting rotation qz (identity is 1,0,0,0). Recorded only. A single antenna is a point and has no facing; this would matter for a dual-antenna heading receiver."},
  {"geometry.imu.x", "m", "geometry", IMU_OFFSET_X_M, -3.0f, 3.0f, PARAM_STOPPED,
   "IMU: forward of the middle of the robot (+)"},
  {"geometry.imu.y", "m", "geometry", IMU_OFFSET_Y_M, -3.0f, 3.0f, PARAM_STOPPED,
   "IMU: left of the middle of the robot (+)"},
  {"geometry.imu.z", "m", "geometry", IMU_OFFSET_Z_M, -3.0f, 3.0f, PARAM_STOPPED,
   "IMU: above the middle of the robot (+)"},
  {"geometry.imu.rot_mode", "", "geometry", 0.0f, 0.0f, 1.0f, PARAM_STOPPED,
   "IMU: how its rotation is entered. 0 = roll/pitch/yaw in degrees (what you measure with a protractor), 1 = quaternion (what a datasheet gives). The other form is ignored, not blended."},
  {"geometry.imu.roll", "deg", "geometry", 0.0f, -180.0f, 180.0f, PARAM_STOPPED,
   "IMU roll, about the forward axis. Used when rot_mode is 0. Used by the maths."},
  {"geometry.imu.pitch", "deg", "geometry", 0.0f, -90.0f, 90.0f, PARAM_STOPPED,
   "IMU pitch, nose up positive. Used when rot_mode is 0. Used by the maths."},
  {"geometry.imu.yaw", "deg", "geometry", IMU_YAW_OFFSET_DEG, -180.0f, 180.0f, PARAM_STOPPED,
   "IMU yaw, turned left positive. Used when rot_mode is 0. Used by the maths. Defaults to 180 here because the sensor's X arrow points at the REAR - the same physical fact as IMU_ACC_FORWARD_SIGN, and the two must never disagree."},
  {"geometry.imu.qw", "", "geometry", IMU_QW, -1.0f, 1.0f, PARAM_STOPPED,
   "IMU mounting rotation qw (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.imu.qx", "", "geometry", IMU_QX, -1.0f, 1.0f, PARAM_STOPPED,
   "IMU mounting rotation qx (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.imu.qy", "", "geometry", IMU_QY, -1.0f, 1.0f, PARAM_STOPPED,
   "IMU mounting rotation qy (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},
  {"geometry.imu.qz", "", "geometry", IMU_QZ, -1.0f, 1.0f, PARAM_STOPPED,
   "IMU mounting rotation qz (identity is 1,0,0,0). Used: a part that is not square to the chassis does not behave as if it were."},

  // ---- battery -----------------------------------------------------------
  // A different pack chemistry or cell count changes all of these together.
  {"gps.stale_ms", "ms", "gps", GPS_STALE_MS, 500.0f, 30000.0f, PARAM_LIVE,
   "How long a fix stays trusted after sentences stop. Raise it where the sky is blocked."},
  // The two gates that decide whether the receiver's VELOCITY is believed, and
  // the one that decides whether it has a position at all. They were plain
  // #defines, so the matching numbers on the Settings page changed only the
  // host's navigation gate and left the firmware on its compiled-in values -
  // the setting appeared to work and half of it did nothing.
  {"gps.min_sats_speed", "", "gps", GPS_MIN_SATS_FOR_SPEED, 3.0f, 20.0f, PARAM_LIVE,
   "Satellites needed before the receiver's ground SPEED is believed. Below this the fused speed ignores GPS and uses the IMU and motor model instead. Strict on purpose: standing still indoors on a 3-satellite fix this receiver reported 7.28 m/s."},
  {"gps.max_hdop_speed", "", "gps", GPS_MAX_HDOP_FOR_SPEED, 0.5f, 10.0f, PARAM_LIVE,
   "Worst HDOP whose ground SPEED is still believed. Raising it lets indoor and under-cover noise into the speed estimate, which is what makes the odometry wander across a room where nothing moved."},
  {"gps.min_sats_fix", "", "gps", GPS_MIN_SATS_FOR_FIX, 3.0f, 20.0f, PARAM_LIVE,
   "Satellites needed to call the GPS a working sensor at all. Four is a 3D fix. A separate, looser question from the speed gate: a fix too rough to take a speed from is still a fix, and reporting it as a dead sensor sends you hunting a fault that is not there."},
  {"battery.cells", "", "battery", BATT_CELLS, 2.0f, 6.0f, PARAM_LIVE,
   "Cells in series: 2S, 3S, 4S. Rescales the warn, soft-stop and cutoff volts."},
  {"battery.warn_v", "V", "battery", BATT_WARN_V, 6.0f, 30.0f, PARAM_LIVE,
   "Orange on the web UI. Keep driving."},
  {"battery.soft_stop_v", "V", "battery", BATT_SOFT_STOP_V, 6.0f, 30.0f, PARAM_LIVE,
   "Stop driving, stay powered. Recovers on its own."},
  {"battery.cutoff_v", "V", "battery", BATT_CUTOFF_V, 6.0f, 30.0f, PARAM_LIVE,
   "Hard cut. LATCHES until the robot is switched off and on again."},

  // ---- compass -----------------------------------------------------------
  {"compass.declination", "deg", "compass", COMPASS_DECLINATION_DEG, -45.0f, 45.0f, PARAM_LIVE,
   "Magnetic declination where the robot is. Look it up for your location."},

  // ---- estimator ---------------------------------------------------------
  {"estimator.gps_tau", "s", "estimator", SPEED_EST_GPS_TAU_S, 0.05f, 20.0f, PARAM_LIVE,
   "How fast the speed estimate is pulled toward GPS. Larger is smoother and slower."},
  {"estimator.model_tau", "s", "estimator", SPEED_EST_MODEL_TAU_S, 0.05f, 20.0f, PARAM_LIVE,
   "Same, for the motor model when GPS is unavailable."},
  {"estimator.align_min", "m/s", "estimator", HEADING_ALIGN_MIN_MPS, 0.01f, 2.0f, PARAM_LIVE,
   "Below this speed GPS course is noise, so heading stops trusting it."},
  {"control.spin_above_deg", "deg", "control", HEADING_SPIN_ERR_DEG, 5.0f, 180.0f, PARAM_LIVE,
   "Heading error above which the robot stops and turns on the spot instead of driving an arc. Below it, it already drives and steers at the same time. Set 180 to never stop turning - smoother in an open field; keep it low in a corridor, where turning first is tidier and covers less ground."},
  {"net.tx_power_dbm", "dBm", "network", WIFI_TX_POWER_DBM, 2.0f, 20.0f, PARAM_LIVE,
   "Wi-Fi transmit power. 20 is full range, 13 is about a fifth of the transmit current for roughly half the range. It was lowered to 13 because transmit bursts were browning out the BNO085 on USB power with no battery; with a battery holding the rail there is headroom to raise it again. Raise it when the robot works far from the access point, and watch imu_resets: if that starts climbing, the supply is sagging and this is why."},
};

#endif  // PARAM_TABLE_H
