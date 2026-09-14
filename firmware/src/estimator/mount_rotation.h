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
 * @file mount_rotation.h
 * @brief How a part is turned relative to the robot, said either way round.
 *
 * WHY TWO WAYS OF SAYING THE SAME THING
 *
 * A mounting rotation is one fact, but the two useful ways to state it suit
 * different people and different moments:
 *
 *   roll / pitch / yaw   is what you MEASURE. A protractor against the chassis,
 *                        or "the IMU is bolted on its side, so roll 90". It is
 *                        readable, and a wrong entry is obvious at a glance.
 *
 *   quaternion           is what a DATASHEET or a calibration tool gives you,
 *                        and what the maths wants. It has no gimbal lock and no
 *                        argument about rotation order.
 *
 * Forcing a quaternion on someone holding a protractor means they have to
 * convert by hand, and a hand conversion is a silent wrong number waiting to
 * happen. Forcing angles on someone holding a datasheet quaternion is the same
 * problem in reverse. So each mount says which form it is using, and the
 * firmware converts.
 *
 * CONVENTION: intrinsic Z-Y-X - yaw about z, then pitch about the new y, then
 * roll about the new x. This is the aerospace convention and the one the rest
 * of this project already uses for the robot's own heading, where yaw is the
 * angle that matters and the other two are usually zero.
 *
 * Angles are DEGREES here, because that is what people measure in. The maths
 * below is the only place they become radians.
 */
#ifndef MOUNT_ROTATION_H
#define MOUNT_ROTATION_H

#include <math.h>

#ifndef DEG_TO_RAD_F
#define DEG_TO_RAD_F 0.01745329252f
#endif

struct Quat {
  float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

/** How a mount's rotation was entered. */
enum RotationMode {
  ROT_RPY  = 0,   ///< roll, pitch, yaw in degrees - the default, and measurable
  ROT_QUAT = 1,   ///< a quaternion, as a datasheet or a calibration tool gives it
};

/** Roll, pitch, yaw in degrees -> quaternion. Intrinsic Z-Y-X. */
inline Quat quatFromRPY(float roll_deg, float pitch_deg, float yaw_deg) {
  const float cr = cosf(roll_deg  * 0.5f * DEG_TO_RAD_F);
  const float sr = sinf(roll_deg  * 0.5f * DEG_TO_RAD_F);
  const float cp = cosf(pitch_deg * 0.5f * DEG_TO_RAD_F);
  const float sp = sinf(pitch_deg * 0.5f * DEG_TO_RAD_F);
  const float cy = cosf(yaw_deg   * 0.5f * DEG_TO_RAD_F);
  const float sy = sinf(yaw_deg   * 0.5f * DEG_TO_RAD_F);

  Quat q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

/** Quaternion -> roll, pitch, yaw in degrees. The exact inverse of the above,
 *  so the settings page can show the angles for a rotation entered as a
 *  quaternion, and vice versa, without anyone converting by hand. */
inline void rpyFromQuat(const Quat &q, float *roll_deg, float *pitch_deg,
                        float *yaw_deg) {
  const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  *roll_deg = atan2f(sinr_cosp, cosr_cosp) / DEG_TO_RAD_F;

  // Clamped, because a quaternion that is a hair off unit length - which is
  // what typing four rounded decimals into a web form produces - can push this
  // just outside [-1, 1] and asinf() then returns NaN. One bad character in a
  // settings field would otherwise poison every angle downstream.
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  *pitch_deg = asinf(sinp) / DEG_TO_RAD_F;

  const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  *yaw_deg = atan2f(siny_cosp, cosy_cosp) / DEG_TO_RAD_F;
}

/**
 * Make a quaternion usable, whatever was typed into the settings page.
 *
 * A quaternion has to be unit length for the rotation maths to mean anything,
 * and nothing stops someone entering four numbers that are not. All zeros is
 * the case that matters most: it is what an empty form gives, it is not a
 * rotation at all, and left alone it would silently collapse every vector it
 * touched to nothing. Identity is the honest reading of "not set".
 */
inline Quat quatNormalise(const Quat &in) {
  const float n = sqrtf(in.w * in.w + in.x * in.x + in.y * in.y + in.z * in.z);
  if (n < 1e-6f) return Quat();                 // not a rotation: treat as none
  Quat q;
  q.w = in.w / n; q.x = in.x / n; q.y = in.y / n; q.z = in.z / n;
  return q;
}

/** The rotation a mount is actually using, from whichever form it was given in. */
inline Quat mountRotation(int mode, float roll_deg, float pitch_deg, float yaw_deg,
                          float qw, float qx, float qy, float qz) {
  if (mode == ROT_QUAT) {
    Quat q; q.w = qw; q.x = qx; q.y = qy; q.z = qz;
    return quatNormalise(q);
  }
  return quatFromRPY(roll_deg, pitch_deg, yaw_deg);
}

/** Rotate a vector by a quaternion: v' = q v q*. */
inline void quatRotate(const Quat &q, float vx, float vy, float vz,
                       float *ox, float *oy, float *oz) {
  // t = 2 * (qv x v);  v' = v + q.w * t + qv x t
  const float tx = 2.0f * (q.y * vz - q.z * vy);
  const float ty = 2.0f * (q.z * vx - q.x * vz);
  const float tz = 2.0f * (q.x * vy - q.y * vx);
  *ox = vx + q.w * tx + (q.y * tz - q.z * ty);
  *oy = vy + q.w * ty + (q.z * tx - q.x * tz);
  *oz = vz + q.w * tz + (q.x * ty - q.y * tx);
}

#endif  // MOUNT_ROTATION_H
