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
 * @file robot.h
 * @brief Physical robot: wheels, track width, top speed.
 *
 * MEASURE THESE ON YOUR ROBOT. Getting the track width wrong makes the
 * reported wheel speeds and the manual turn rate wrong; getting the wheel
 * diameter wrong scales every speed.
 *
 * The same numbers live in the ROS side `robot.yaml` and in
 * `pid.yaml: estimator.track_m` - keep all three in step.
 */
#ifndef ROBOT_H
#define ROBOT_H

#define WHEEL_DIAMETER_M        0.081f    // 81 mm wheel
#define WHEEL_TRACK_M           0.300f    // >>> MEASURE <<< centre of left wheel to centre of right

// JGA25-370-100RPM gearmotor on 3S LiPo (11.1 V nominal, 12.6 V full)
#define MOTOR_MAX_RPM           100.0f    // output shaft rpm at 12 V (datasheet)
#define MOTOR_RPM_RATIO         0.85f     // usable fraction: battery sag + safety margin

// Top ground speed = (rpm/60) * pi * D  ->  100/60 * pi * 0.081 = 0.424 m/s
#define ROBOT_MAX_SPEED_MPS     (MOTOR_MAX_RPM * MOTOR_RPM_RATIO / 60.0f * 3.14159265f * WHEEL_DIAMETER_M)
#define ROBOT_DEFAULT_SPEED_MPS 0.25f     // used when nothing else sets a speed

//--------------------------- where things sit ----------------------------//
// Everything bolted to this robot has a place and a facing, and both are
// measured HERE rather than assumed.
//
// FRAME: x forward, y left, z up. Metres. Measured FROM THE MIDDLE OF THE
// ROBOT - the centre of the chassis, which is the one point you can find with
// a tape measure without first knowing anything else.
//
//        front
//          ^ +x
//          |
//   +y <---O---> -y      O = middle of the chassis, the origin for everything
//          |
//          v -x
//        rear
//
// WHY THE MIDDLE AND NOT THE DRIVE AXLE. The lever-arm correction needs offsets
// from the point the robot TURNS about, which is the midpoint of the drive
// axle. That is a fine origin for the maths and a poor one for a person: you
// cannot measure from it until you have already located it. So the numbers you
// enter are from the middle of the chassis, and the firmware works out the axle
// midpoint from where you said the drive wheels are. One origin, measured once.
//
// ORIENTATION: each mount also carries a quaternion, w x y z, rotating the
// part's own frame into the robot's. Identity is (1, 0, 0, 0) and means "lined
// up with the robot", which is what a part bolted square to the chassis has.
//
// It is worth being plain about where a rotation actually changes anything:
//   IMU      - yes. Mounted sideways or upside down, its axes are not the
//              robot's, and every heading and yaw rate is wrong until this says
//              so.
//   wheels   - yes, for toe and camber. A wheel that does not point straight
//              ahead does not travel the distance its rotation implies.
//   casters  - no. A caster swivels; its resting facing means nothing. The
//              field exists so the set is uniform and so a footprint can be
//              drawn, and it is documented as inert rather than silently
//              ignored.
//   GPS      - no, for a single antenna: a point has no facing. It would matter
//              for a dual-antenna heading receiver, which this is not.
//
// All of these default to zero position and identity rotation, which is exactly
// the behaviour of a robot that has never been measured - so nothing changes
// until you measure something.

#define WHEEL_L_X_M   0.00f   // >>> MEASURE <<< left drive wheel, forward of centre (+)
#define WHEEL_L_Y_M   0.15f   // >>> MEASURE <<< left drive wheel, left of centre (+)
#define WHEEL_L_Z_M   0.00f   // >>> MEASURE <<< left drive wheel contact patch, up (+)
#define WHEEL_R_X_M   0.00f   // >>> MEASURE <<< right drive wheel, forward of centre (+)
#define WHEEL_R_Y_M  -0.15f   // >>> MEASURE <<< right drive wheel, left of centre (+) so this is negative
#define WHEEL_R_Z_M   0.00f   // >>> MEASURE <<< right drive wheel contact patch, up (+)

// TWO casters, both at the FRONT, with the driven wheels at the REAR. They
// were named "front" and "rear" when the layout was assumed to be one of each,
// which is not this robot: a name that describes the wrong machine is worse
// than no name, because it is read as if it were true.
#define CASTER_FL_X_M  0.00f  // >>> MEASURE <<< front-LEFT caster, forward of centre (+)
#define CASTER_FL_Y_M  0.00f  // >>> MEASURE <<< front-left caster, left of centre (+)
#define CASTER_FL_Z_M  0.00f  // >>> MEASURE <<< front-left caster contact patch, up (+)
#define CASTER_FR_X_M  0.00f  // >>> MEASURE <<< front-RIGHT caster, forward of centre (+)
#define CASTER_FR_Y_M  0.00f  // >>> MEASURE <<< front-right caster, left of centre (+) so this is negative
#define CASTER_FR_Z_M  0.00f  // >>> MEASURE <<< front-right caster contact patch, up (+)

#define GPS_OFFSET_X_M  0.00f  // >>> MEASURE <<< antenna forward of centre (+)
#define GPS_OFFSET_Y_M  0.00f  // >>> MEASURE <<< antenna left of centre (+)
#define GPS_OFFSET_Z_M  0.00f  // >>> MEASURE <<< antenna above centre (+)
#define IMU_OFFSET_X_M  0.00f  // >>> MEASURE <<< IMU forward of centre (+)
#define IMU_OFFSET_Y_M  0.00f  // >>> MEASURE <<< IMU left of centre (+)
#define IMU_OFFSET_Z_M  0.00f  // >>> MEASURE <<< IMU above centre (+)

// Mounting rotations, w x y z. Identity = bolted square to the chassis.
#define IMU_QW  1.00f
#define IMU_QX  0.00f
#define IMU_QY  0.00f
#define IMU_QZ  0.00f
#define GPS_QW  1.00f
#define GPS_QX  0.00f
#define GPS_QY  0.00f
#define GPS_QZ  0.00f
#define WHEEL_L_QW 1.00f
#define WHEEL_L_QX 0.00f
#define WHEEL_L_QY 0.00f
#define WHEEL_L_QZ 0.00f
#define WHEEL_R_QW 1.00f
#define WHEEL_R_QX 0.00f
#define WHEEL_R_QY 0.00f
#define WHEEL_R_QZ 0.00f
#define CASTER_FL_QW 1.00f
#define CASTER_FL_QX 0.00f
#define CASTER_FL_QY 0.00f
#define CASTER_FL_QZ 0.00f
#define CASTER_FR_QW 1.00f
#define CASTER_FR_QX 0.00f
#define CASTER_FR_QY 0.00f
#define CASTER_FR_QZ 0.00f

// The drive axle midpoint, derived rather than entered: it is simply halfway
// between the two drive wheels. Nobody has to measure it, and it cannot
// disagree with the wheel positions it is made of.
#define DRIVE_AXLE_X_M  ((WHEEL_L_X_M + WHEEL_R_X_M) * 0.5f)
#define DRIVE_AXLE_Y_M  ((WHEEL_L_Y_M + WHEEL_R_Y_M) * 0.5f)

#endif // ROBOT_H
