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
 * @file safety.h
 * @brief Watchdog timeouts and stop-reason bit flags.
 *
 * SAFETY CHAIN (each layer works on its own, no layer depends on the next):
 *   1. Hardware  : mushroom switch SW3 -> relay cuts EMER VCC (motor power).
 *   2. Firmware  : E-EMER / ESP EMER switch inputs stop the PWM immediately.
 *   3. Link      : /gps_localize/heartbeat must arrive every HEARTBEAT_TIMEOUT_MS.
 *   4. Command   : cmd_move / cmd_manual must be refreshed every CMD_TIMEOUT_MS.
 *   5. Agent     : micro-ROS agent ping failure stops the motors.
 *   6. Web       : browser heartbeat feeds layer 3 through safety_watchdog.
 */
#ifndef SAFETY_CONFIG_H
#define SAFETY_CONFIG_H

#define HEARTBEAT_TIMEOUT_MS    600       // ROS heartbeat (published at 10 Hz) may miss ~6 messages
#define CMD_TIMEOUT_MS          500       // motion command must be refreshed this often
#define AGENT_PING_PERIOD_MS    700
#define ESTOP_RELEASE_HOLD_MS   400       // inputs must stay clear this long before motion resumes
#define STARTUP_LOCKOUT_MS      1500      // no motion during the first moments after boot

//====================== Stop reason bits =====================//
#define STOP_HW_EMERGENCY       (1u << 0) // E-EMER line (relay / mushroom switch)
#define STOP_ESP_BUTTON         (1u << 1) // SW4 on-board button
#define STOP_SW_ESTOP           (1u << 2) // software e-stop from the web UI
#define STOP_HEARTBEAT_LOST     (1u << 3) // no ROS heartbeat -> network cut
#define STOP_CMD_TIMEOUT        (1u << 4) // command stream stalled
#define STOP_AGENT_LOST         (1u << 5) // micro-ROS agent unreachable
#define STOP_BATTERY_LOW        (1u << 6) // pack below BATT_CUTOFF_V
#define STOP_STARTUP            (1u << 7) // boot lockout still active
#define STOP_BATTERY_SOFT       (1u << 8) // pack below BATT_SOFT_STOP_V - stop driving,
                                          // not latched, clears when the voltage recovers
#define STOP_CONTACTOR_STUCK    (1u << 9) // we opened the contactor and the motor rails
                                          // are STILL live - the only way to catch a
                                          // welded contact, and the one fault where
                                          // pressing emergency does not remove power

//=================== Contactor cross-check ===================//
// The emergency input tells us what the SWITCH says. The motor-rail INAs tell
// us what the RAILS are actually doing. Believing only the first means a welded
// contactor reads as a safe robot: the button is pressed, the firmware has
// dropped IO14, every flag says stopped - and the motors are still connected to
// the pack. Only the INA can see that, so it is worth checking.
//
// Generous, because it must never cry wolf: the contactor takes time to open,
// the rail capacitance takes longer to bleed down, and the INAs are read at
// 5 Hz. A real weld holds the rail up indefinitely; a normal opening is well
// under a second.
#define CONTACTOR_SETTLE_MS     1500      // grace after commanding open, before judging

//======================= LED patterns ========================//
// Red LED behaviour (green LED is hardwired to 5 V, firmware never touches it)
#define LED_BLINK_FAST_MS       120       // link lost / command timeout
#define LED_BLINK_SLOW_MS       500       // software e-stop, waiting for reset
#define LED_HEARTBEAT_MS        1500      // short blip while everything is healthy

//==================== Robot run states =======================//
enum RunState {
  RUN_IDLE       = 0,   // armed, no motion commanded
  RUN_AUTO       = 1,   // following heading + speed from waypoint_nav
  RUN_MANUAL     = 2,   // driven by hand from the web UI
  RUN_ESTOP      = 3,   // stopped by an emergency input or the web e-stop
  RUN_COMM_LOSS  = 4    // stopped because the link or command stream died
};

#endif // SAFETY_CONFIG_H
