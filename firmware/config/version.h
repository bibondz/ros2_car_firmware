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
 * @file version.h
 * @brief What firmware is on the board, so the host can say when it is old.
 *
 * The board and the host are updated separately - the host pulls from git, the
 * board needs a flash - so they drift apart as a matter of course. That is
 * fine, and the robot must keep working across the gap: an older board still
 * connects, still drives, still reports. What it must not do is drift SILENTLY,
 * because "the web UI shows a field that is always zero" and "the firmware
 * predates that field" look identical from the outside.
 *
 * So the version is published, and the host says plainly when the board is
 * behind. It is a notice, never a block.
 *
 * BUMP FIRMWARE_VERSION WHEN THE CONTRACT CHANGES - a new telemetry field, a
 * new topic, a changed unit or meaning. Not for every edit: a bug fix that
 * leaves the interface alone does not need it, and a version that changes on
 * every commit tells nobody anything.
 */
#ifndef VERSION_H
#define VERSION_H

// Major.minor. Minor for additions the host can ignore, major when something
// the host relies on changes shape.
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  62
#define FIRMWARE_VERSION        ((FIRMWARE_VERSION_MAJOR) * 100 + (FIRMWARE_VERSION_MINOR))

// __DATE__ and __TIME__ come from the compiler, so this identifies the exact
// build rather than the version someone remembered to bump. Two boards on the
// same version with different build stamps are two different images.
#define FIRMWARE_BUILD_STAMP    (__DATE__ " " __TIME__)

#endif // VERSION_H
