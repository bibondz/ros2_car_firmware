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
 * Move a sensor reading from where the sensor is to where the robot turns.
 *
 * WHY THIS EXISTS
 *
 * This robot has two driven wheels and two casters, so it rotates about the
 * midpoint of the drive axle. Nothing else is at that point: the GPS antenna is
 * bolted wherever it gets sky, the IMU wherever it fits.
 *
 * A sensor offset from the turning centre travels through an arc when the robot
 * spins on the spot. An antenna 30 cm ahead of the axle sweeps a 30 cm radius
 * circle while the robot has not moved anywhere at all - and to a filter reading
 * only the antenna, that is a metre of travel, at whatever speed the robot
 * turned. Without this correction, every turn injects a position error the size
 * of the offset, and it does so in a direction that changes with heading, so it
 * cannot be tuned out.
 *
 * The correction is a rotation and a subtraction. Given the robot's heading and
 * a sensor mounted at (x, y) in the body frame, the sensor sits at
 *
 *     world_offset = R(heading) * (x, y)
 *
 * away from the turning centre, so the centre is at
 *
 *     centre = sensor_reading - world_offset
 *
 * With both offsets zero this is exactly the identity, so a robot whose offsets
 * have never been measured behaves precisely as it did before.
 *
 * FRAME
 *
 * Body frame: x forward, y left, metres, measured from the drive axle midpoint.
 * Heading is compass degrees - 0 = north, increasing clockwise - because that is
 * what the estimator carries. The conversion to the ENU maths below happens
 * here, in one place, rather than at each call site.
 */
#ifndef LEVER_ARM_H
#define LEVER_ARM_H

#include <math.h>

struct LeverArm {
  float x = 0.0f;      // forward from the drive axle midpoint, metres
  float y = 0.0f;      // left of centre, metres

  bool isZero() const { return fabsf(x) < 1e-6f && fabsf(y) < 1e-6f; }
};

/**
 * Where the turning centre is, given a reading taken at an offset sensor.
 *
 * @param east,north      the sensor's position, ENU metres
 * @param heading_deg     robot heading, compass degrees (0 = north, CW)
 * @param arm             where the sensor sits in the body frame
 * @param out_east,out_north  the turning centre, same frame
 */
inline void leverArmToCentre(float east, float north, float heading_deg,
                             const LeverArm &arm,
                             float *out_east, float *out_north) {
  if (arm.isZero()) {                 // nothing measured: change nothing
    *out_east = east;
    *out_north = north;
    return;
  }
  // Compass degrees to radians east-of-north. A body-frame vector (x forward,
  // y left) maps to ENU as:
  //     east  =  x * sin(h) - y * cos(h)
  //     north =  x * cos(h) + y * sin(h)
  // At h = 0 (facing north) that gives east = -y, north = x, which is what
  // "x forward, y left" means when forward is north.
  const float h = heading_deg * 0.01745329252f;
  const float s = sinf(h), c = cosf(h);
  const float off_east  = arm.x * s - arm.y * c;
  const float off_north = arm.x * c + arm.y * s;
  *out_east  = east - off_east;
  *out_north = north - off_north;
}

/**
 * The velocity an offset sensor sees that the turning centre does not.
 *
 * A sensor on a rotating body has a tangential velocity of omega x r even when
 * the centre is perfectly still. This returns that component so a speed taken
 * at the sensor can have it removed - otherwise spinning on the spot reads as
 * driving forwards.
 *
 * @param yaw_rate_dps  rotation rate, degrees per second, counter-clockwise
 *                      positive (the ROS convention the estimator uses)
 */
inline float leverArmTangentialSpeed(float yaw_rate_dps, const LeverArm &arm) {
  if (arm.isZero()) return 0.0f;
  const float omega = yaw_rate_dps * 0.01745329252f;
  const float radius = sqrtf(arm.x * arm.x + arm.y * arm.y);
  return omega * radius;
}

#endif  // LEVER_ARM_H
