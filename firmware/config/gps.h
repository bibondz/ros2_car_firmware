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
 * @file gps.h
 * @brief GEP-M10-DQ (u-blox M10050) UART settings.
 *
 * The baud rate is detected automatically at boot, so normally you do not have
 * to touch anything here. For the best results configure the module once with
 * u-center: 5 Hz navigation rate, GGA + RMC + VTG on, GSV/GSA/GLL off.
 *
 * The compass inside the same module is configured in compass.h, the pins in pins.h.
 */
#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

#define GPS_SERIAL_NUM          2         // Serial2
#define GPS_BAUD_DEFAULT        115200    // module default
// Widened after the receiver was found transmitting but never parsing: a
// u-blox M10 ships from different vendors at different rates, and 230400 /
// 460800 are common on the flight-controller variants. 4800 is the old NMEA
// default and costs one extra second to rule out.
#define GPS_BAUD_CANDIDATES     {115200, 38400, 9600, 57600, 230400, 460800, 4800}
#define GPS_BAUD_PROBE_MS       900      // time spent on each candidate while detecting
// How long the last good fix stays trusted once sentences stop arriving.
//
// This is a NOISE setting as much as a timing one. Under trees, beside a
// building or in any multipath-heavy spot the receiver drops in and out, and a
// short window makes the robot discard a perfectly usable fix over a one-second
// gap - which then costs the heading its absolute reference and drops the speed
// estimate onto the motor model. A longer window rides those gaps out at the
// cost of trusting a slightly older position.
//
// 3 s suits open sky. Somewhere noisy, raise it: gps.stale_ms is a live
// parameter, so it can be tuned where the robot actually works rather than
// guessed at here.
#define GPS_STALE_MS            3000      // no valid sentence for this long -> fix invalid
#define GPS_STALE_MS_MIN        500
#define GPS_STALE_MS_MAX        30000

// Before the fused speed will believe the receiver at all.
//
// WHY THE OLD TEST WAS NOT ENOUGH. It asked for a fix quality above zero and at
// least one satellite, which sounds reasonable and is not: measured on this
// robot, standing still indoors on a 3-satellite fix with HDOP 1.3, the
// receiver reported ground speeds of 1.6, 3.3, 4.8 and 7.28 m/s. All of it
// passed. The estimator dutifully pulled the fused speed to its 0.36 m/s
// ceiling and the odometry wandered a hundred metres across the room.
//
// POSITION QUALITY AND SPEED QUALITY ARE DIFFERENT THINGS. A marginal fix can
// give a position that is merely inaccurate while its Doppler velocity is
// nonsense, so counting satellites alone will never be enough.
#define GPS_MIN_SATS_FOR_SPEED  6       // see below - four was still indoors noise
#define GPS_MAX_HDOP_FOR_SPEED  1.5f    // see below - 5.0 admitted every indoor fix

// THE OTHER QUESTION: is there a fix at all?
//
// Not the same test, and using the speed gate for both was a bug with a
// visible symptom - the web UI drew the GPS as a missing sensor on 7
// satellites at HDOP 1.8, because that fails the strict SPEED gate above. The
// robot was navigating on that fix at the time.
//
// Four satellites is the real bar: it is the minimum for a 3D solution, and it
// is what "the receiver has a position" means. HDOP is deliberately absent -
// poor geometry makes a position less precise, not missing, and the HDOP value
// travels in telemetry for anything that needs to weigh it. This feeds
// HEALTH_NO_GPS, which is a statement about the SENSOR, not about how much the
// velocity can be trusted.
#define GPS_MIN_SATS_FOR_FIX    4

// WHY SIX SATELLITES AND HDOP 1.5, NOT FOUR AND 5.0.
//
// The first attempt asked for four satellites and HDOP under 5, which sounds
// like a reasonable 3D fix and indoors is not: measured here, a robot that had
// not moved held four to six satellites at HDOP 1.3-2.3 while the receiver
// reported 0.29, 0.47, 0.86 and 1.25 m/s. The fused speed still reached
// 0.2 m/s and the odometry wandered ten metres in ninety seconds.
//
// No threshold on the SPEED can separate that from real motion, because a
// genuine 0.3 m/s and a noisy 0.3 m/s are the same number. The only thing that
// separates them is the quality of the fix that produced them, so the bar goes
// where an indoor fix cannot reach it. Outdoors with sky view this receiver
// sees eight to twelve satellites and HDOP near 1, so it engages normally -
// which is the only place its velocity was ever worth having.

// The check that does the real work: the robot cannot outrun itself.
//
// Whatever the receiver claims, this machine physically cannot exceed
// ROBOT_MAX_SPEED_MPS. A reading well above that is wrong BY DEFINITION, and no
// amount of fix quality makes it right - which is exactly why this catches what
// the quality tests miss. The margin allows for a genuine downhill roll and for
// the receiver being a little optimistic, without admitting 7 m/s on a robot
// whose ceiling is 0.36.
#define GPS_SPEED_SANITY_FACTOR 1.2f
// A receiver standing still reports a small wandering speed rather than zero -
// noise in the velocity solution, not motion. Measured here: 0.39 to 0.82 m/s
// on a good 6-satellite fix with HDOP 1.3, on a robot that had not moved. That
// is under the sanity ceiling, so it got through and dragged the fused estimate
// up to 0.27 m/s.
//
// Below this the receiver is saying "about stopped" and the number itself is
// noise, so it is treated as zero rather than as a measurement. Above it, GPS
// is believed. Chosen at a third of this robot's top speed: fast enough to be
// real motion, slow enough that a stationary receiver's wander never reaches it.
#define GPS_SPEED_NOISE_FLOOR_MPS 0.12f

#endif // GPS_CONFIG_H
