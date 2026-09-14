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
 * GPS_Localize - ESP32 firmware (micro-ROS)
 * ---------------------------------------------------------------------------
 * Outdoor differential-drive robot, GPS waypoints, no wheel encoders.
 *
 *  - closes the heading (BNO085 + PID) and speed loops LOCALLY at 100 Hz, so a
 *    Wi-Fi hiccup cannot make the robot swerve
 *  - stops the motors on its own if the ROS heartbeat, the command stream or
 *    the micro-ROS agent disappears (see lib/safety/safety_manager.h)
 *  - publishes GPS fix, IMU, dead-reckoned odometry and one packed telemetry
 *    array that feeds the web UI
 *
 * MEMORY POLICY (the customer runs this for hours, it must not creep):
 *  - no String, no malloc/new anywhere after setup()
 *  - every ROS message uses a statically allocated buffer, sized at compile time
 *  - the parsers (NMEA) and drivers work on fixed buffers
 *  - free heap is published in the telemetry array so a leak is visible on the web UI
 */
#include <Arduino.h>
#if defined(MICROROS_TRANSPORT_WIFI)
#include <esp_wifi.h>   // esp_wifi_set_max_tx_power - quarter-dBm units
#endif
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <gps_localize_msgs/msg/telemetry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <nav_msgs/msg/odometry.h>

#include <Wire.h>
#include <Preferences.h>
#include <config.h>
#include <motor_tb6612.h>
#include <INA226.h>
#include <gps_nmea.h>
#include <compass_qmc5883.h>
#include <imu_bno085.h>
#include <motor_model.h>
#include <state_estimator.h>
#include <pose_ekf.h>
#include <lever_arm.h>
#include <param_table.h>
#include <drive_controller.h>
#include <safety_manager.h>
#if defined(MICROROS_TRANSPORT_WIFI)
#include <HTTPUpdate.h>
#include <WiFiUdp.h>
#include <ota_updater.h>
#include <settings_console.h>
#include <wifi_store.h>
#endif
#include <power_monitor.h>

//============================ helpers ======================================//
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t t0 = -1; \
  if (t0 == -1) t0 = uxr_millis(); \
  if (uxr_millis() - t0 > (MS)) { X; t0 = uxr_millis(); } } while (0)

//========================= micro-ROS entities ==============================//
static rclc_support_t   support;
static rcl_allocator_t  allocator;
static rcl_init_options_t init_options;
static rcl_node_t       node;
static rcl_timer_t      ctrl_timer;
static rclc_executor_t  executor;

static rcl_publisher_t pub_odom;
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_fix;
static rcl_publisher_t pub_telemetry;
// Parameter values, so the UI can show what the ROBOT actually holds rather
// than what it last sent. Without a readback, "did that save?" is
// unanswerable - and these live in the board's NVS precisely so they survive
// a different control PC, a reboot and a reflash.
static rcl_publisher_t pub_params;

static rcl_subscription_t sub_cmd_move;
static rcl_subscription_t sub_cmd_manual;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_heartbeat;
static rcl_subscription_t sub_estop;
static rcl_subscription_t sub_config;

static nav_msgs__msg__Odometry        msg_odom;
static sensor_msgs__msg__Imu          msg_imu;
static sensor_msgs__msg__NavSatFix    msg_fix;
static gps_localize_msgs__msg__Telemetry msg_telemetry;

static geometry_msgs__msg__Twist      in_cmd_move;
static geometry_msgs__msg__Twist      in_cmd_manual;
static geometry_msgs__msg__Twist      in_cmd_vel;
static std_msgs__msg__Int32           in_heartbeat;
static std_msgs__msg__Bool            in_estop;
static std_msgs__msg__Float32MultiArray in_config;
static std_msgs__msg__Float32MultiArray msg_params;
static float params_buf[PARAM_COUNT + 1];

//--- static payload buffers (never freed, never grown) ---
// TELEMETRY_LEN is gone with the packed array: the message carries named
// fields now, so there is no length for the two sides to disagree about.
#define CONFIG_MAX_LEN 12
static float config_buf[CONFIG_MAX_LEN];

static char frame_odom[]  = ODOM_FRAME_ID;
static char frame_base[]  = ODOM_CHILD_FRAME_ID;
static char frame_imu[]   = IMU_FRAME_ID;
static char frame_gps[]   = GPS_FRAME_ID;

enum Conn { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static Conn conn_state = WAITING_AGENT;

//============================== hardware ===================================//
static MotorTB6612   motor_left;    // MOTOR A
static MotorTB6612   motor_right;   // MOTOR B
static PowerMonitor  power;      // the 4x INA226, and what each one means
static GpsNmea       gps;
static CompassQMC5883 compass;
static ImuBno085     imu(BNO085_RESET_PIN);
static Preferences   prefs;               // NVS: keeps the compass calibration over a reboot
static Preferences   param_store;         // NVS: parameter overrides, separate namespace
#if defined(MICROROS_TRANSPORT_WIFI)
// Only the Wi-Fi build has anywhere to use these. On the serial transport the
// UART *is* the micro-ROS link, so there is no settings console to edit the
// list from and no network to join with it - and leaving them unguarded broke
// that build outright, since wifi_store.h is included under the same #if.
static Preferences   wifi_prefs;          // NVS: the Wi-Fi list, its own namespace
static WifiStore     wifi_store;
#endif
static ParamRegistry<PARAM_COUNT> params;
static StateEstimator estimator;
static PoseEkf       pose_ekf;   // GPS-corrected position; see pose_ekf.h
static DriveController drive;
static SafetyManager  safety;
static MotorModel     model_left, model_right;   // wheel speed from volts + amps

//============================= app state ===================================//
static volatile uint32_t last_heartbeat_ms = 0;
static volatile uint32_t last_cmd_ms       = 0;
static volatile bool     sw_estop          = false;
static volatile bool     have_heartbeat    = false;


static uint32_t loop_us = 0;
static uint16_t div_odom = 0, div_imu = 0, div_tel = 0, div_ina = 0, div_mag = 0;
static uint16_t div_params = 0;
static uint8_t  mag_calib_state = 0;   // 0 idle, 1 capturing, 2 finished ok, 3 failed
static uint8_t  imu_calib_state = 0;   // 0 idle, 1 learning, 2 saved, 3 refused
// What the sensor said to the last calibration command. SH2_OK is 0; anything
// else is a negative code from sh2_err.h, or -100 for "the IMU never started".
// Published because "refused" alone is not a diagnosis, and the whole point of
// putting calibration on the web was that it should not need a USB cable.
static int      imu_calib_rc = 0;
// The BNO085 has no reset line on this board, so begin() can fail after a
// software reset and used to stay failed forever. Retry, slowly.
// Our own code, matching IMU_CALIB_RESULTS in telemetry.py: the command was not
// sent to the sensor at all, because the robot was being driven.
static const int IMU_CALIB_MOVING = -103;
static uint32_t last_imu_retry_ms = 0;
// Every 30 s, and only ten times.
//
// A retry is not free: begin_I2C() is a full SHTP handshake that holds the bus
// for a few hundred milliseconds and, when it fails, leaves the Adafruit
// library printing "I2C address not found" as it goes. At one attempt every
// three seconds that was enough to stop the two motor-rail INA226s answering
// at all - so pack_valid went false, both motor currents froze at their last
// good value, and the robot looked as though neither motor was drawing any
// current while being commanded to full PWM. A diagnostic that breaks the
// measurements around it is worse than no diagnostic.
//
// The cap is there because we know how this ends: with BNO085_RESET_PIN at -1
// the Adafruit HAL's hardwareReset() is a no-op, so nothing the software can do
// actually resets the sensor. After five minutes of trying, patience is not the
// missing ingredient - power is, or a wire from its RESET to a spare GPIO.
static const uint32_t IMU_RETRY_MS = 30000;
static const uint16_t IMU_RETRY_MAX = 10;
static uint32_t last_ctrl_us = 0;
static uint32_t last_gps_pub_ms = 0;

static bool time_synced = false;
static IPAddress agent_ip = AGENT_IP;        // re-resolved by name on every reconnect

// WHERE THE AGENT IS, LEARNED RATHER THAN COMPILED IN.
//
// The address used to come only from the header, and the header goes stale the
// moment the PC changes network - which then needs a reflash to fix, over a
// cable, on a robot that by definition cannot be reached over the network. Two
// pieces of state fix that, both in NVS:
//
//   override  - an address a person set deliberately ("the agent is HERE"),
//               tried first and never guessed over.
//   last good - the address that actually carried a session last time. Costs
//               nothing to try, and on a network whose DHCP is stable it means
//               the robot reconnects instantly after a reboot even when mDNS
//               is blocked, which is the common case on guest and shared Wi-Fi.
static Preferences   agent_prefs;
static char          agent_override[40] = {0};   // "" = none, else host or IP
static IPAddress     agent_last_good((uint32_t)0);
// The hand-set address has been tried and tried and never answers, so stop
// obeying it and go looking instead.
//
// WHY THIS HAD TO EXIST: an override that is a literal IP used to short-circuit
// resolveAgentIp() unconditionally, so the "look the agent up again after a few
// failed pings" self-heal below re-resolved to the same dead address forever.
// Seen for real - the control PC rebooted onto a different network and the
// board, pinned by hand to the old address, could not be recovered by anything
// short of a USB cable. Honouring a person's setting is right up to the point
// where it is provably wrong; after that it is just a robot that cannot come
// home. Never persisted: a power cycle starts by trusting the setting again.
static bool          agent_override_failed = false;
// The address we have been trying that never answers, whatever put us onto it.
//
// Abandoning the hand-set address was not enough on its own. The discovery
// chain below ends with "the address that carried a session last time", and
// after a hand-set address HAS worked once, last-known-good is that same
// address - so giving up on the override fell straight through to an identical
// guess and the gateway step, the one that actually recovers a hotspot setup,
// was never reached. Skipping the known-dead address wherever it appears is
// what makes the fall-through mean anything.
static IPAddress     agent_dead((uint32_t)0);

#if defined(MICROROS_TRANSPORT_WIFI)
/* ------------------------- agent beacon listener -------------------------
 *
 * The host broadcasts "GPSLOC-AGENT <version> <ip> <port>" a few times a
 * second. A board with no session adopts whatever it hears, so changing the
 * Wi-Fi, moving the PC or handing out fresh DHCP leases fixes itself within a
 * couple of seconds with nothing to configure on either side.
 *
 * This exists because every other route has a failure mode we have actually
 * hit: a hand-set address goes stale the moment the PC moves, mDNS is
 * link-local and cannot cross a subnet, and the gateway guess is only right
 * when the PC happens to be the gateway. Twice in one day the robot ended up
 * needing a USB cable to be told an address it could have been given.
 *
 * Trust: the beacon is only listened to while there is NO session, it must
 * carry the magic, and it must arrive from the board's own subnet. Even then
 * all it does is choose where to try - the agent still has to accept the
 * session, so a bogus beacon costs one failed attempt.
 */
static WiFiUDP  beacon_udp;
static bool     beacon_open = false;
static IPAddress beacon_from((uint32_t)0);   // last address a beacon offered

static void beaconBegin() {
  if (beacon_open) return;
  beacon_open = beacon_udp.begin(AGENT_BEACON_PORT) == 1;
  if (beacon_open)
    Serial.printf("[beacon] listening on UDP %u for the agent's announcement\n",
                  (unsigned)AGENT_BEACON_PORT);
}

/** @return the announced agent address, or 0.0.0.0 when nothing usable arrived. */
static IPAddress beaconPoll() {
  if (!beacon_open) return IPAddress((uint32_t)0);
  IPAddress found((uint32_t)0);
  // Drain whatever is queued: only the newest announcement is interesting, and
  // leaving packets behind would make the board act on stale ones later.
  for (int guard = 0; guard < 8; ++guard) {
    const int len = beacon_udp.parsePacket();
    if (len <= 0) break;
    char buf[80];
    const int n = beacon_udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = '\0';
    if (strncmp(buf, AGENT_BEACON_MAGIC, strlen(AGENT_BEACON_MAGIC)) != 0) continue;

    // Believe the address the packet CAME FROM, not one written inside it.
    // A datagram cannot lie about its source without the reply going somewhere
    // else, and the body could say anything at all.
    const IPAddress src = beacon_udp.remoteIP();
    const IPAddress mask = WiFi.subnetMask();
    if (((uint32_t)src & (uint32_t)mask) != ((uint32_t)WiFi.localIP() & (uint32_t)mask))
      continue;                       // not on our subnet: not our agent
    found = src;
  }
  return found;
}
#endif  // MICROROS_TRANSPORT_WIFI
// Rounds of five failed pings while a hand-set address is in force.
static uint8_t       agent_override_rounds = 0;

// Addresses that have actually carried a session, newest first.
//
// WHY MORE THAN ONE. The board can always reach the PC - that direction works
// even across a phone hotspot's NAT, which is exactly how a phone browsing the
// web UI works and why the robot could link the moment it was told where to
// look. What it cannot do is DISCOVER the address: mDNS and DNS are
// link-local, and the gateway guess only lands when the PC is the gateway. So
// across a NAT the board is left with nothing but memory - and remembering a
// single address meant that the moment the PC moved between two networks it
// used regularly, the one it remembered was always the wrong one.
//
// Four is enough for a machine that alternates between a desk, a hotspot and a
// site router, and small enough to try them all inside one retry round.
#define AGENT_GOOD_MAX 4
static IPAddress     agent_good[AGENT_GOOD_MAX];
static uint8_t       agent_good_n = 0;

static void agentGoodSave() {
  for (uint8_t i = 0; i < AGENT_GOOD_MAX; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "good%u", (unsigned)i);
    agent_prefs.putUInt(key, i < agent_good_n ? (uint32_t)agent_good[i] : 0);
  }
}

static void agentStoreLoad() {
  if (!agent_prefs.begin("gpsagent", false)) return;
  String ov = agent_prefs.getString("override", "");
  strncpy(agent_override, ov.c_str(), sizeof(agent_override) - 1);
  agent_override[sizeof(agent_override) - 1] = '\0';
  agent_last_good = IPAddress(agent_prefs.getUInt("lastgood", 0));

  agent_good_n = 0;
  for (uint8_t i = 0; i < AGENT_GOOD_MAX; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "good%u", (unsigned)i);
    const uint32_t v = agent_prefs.getUInt(key, 0);
    if (v) agent_good[agent_good_n++] = IPAddress(v);
  }
  // A board upgraded from the single-address version still knows one place
  // that worked; carry it across rather than starting blank.
  if (agent_good_n == 0 && (uint32_t)agent_last_good != 0) {
    agent_good[agent_good_n++] = agent_last_good;
    agentGoodSave();
  }
}

static void agentSetOverride(const char *host) {
  strncpy(agent_override, host ? host : "", sizeof(agent_override) - 1);
  agent_override[sizeof(agent_override) - 1] = '\0';
  agent_prefs.putString("override", agent_override);
  agent_override_failed = false;      // a new setting deserves a fresh chance
  agent_override_rounds = 0;
  agent_dead = IPAddress((uint32_t)0);
}

/** Remember an address only once it has actually carried a session. */
static void agentRememberGood(const IPAddress &ip) {
  if ((uint32_t)ip == 0) return;

  // Move it to the front, whether or not it was already known: the list is in
  // most-recently-worked order, so the address in use now is tried first after
  // a reboot.
  uint8_t at = agent_good_n;
  for (uint8_t i = 0; i < agent_good_n; ++i)
    if (agent_good[i] == ip) { at = i; break; }
  if (at == 0 && agent_good_n > 0) {
    if (ip == agent_last_good) return;          // already first: nothing to write
  }
  if (at == agent_good_n && agent_good_n < AGENT_GOOD_MAX) agent_good_n++;
  const uint8_t last = (at < agent_good_n) ? at : (uint8_t)(agent_good_n - 1);
  for (uint8_t i = last; i > 0; --i) agent_good[i] = agent_good[i - 1];
  agent_good[0] = ip;

  agent_last_good = ip;
  agent_prefs.putUInt("lastgood", (uint32_t)ip);
  agentGoodSave();
  Serial.printf("[agent] remembering %s (%u address(es) known to work)\n",
                ip.toString().c_str(), (unsigned)agent_good_n);
}
// Consecutive failures to build a micro-ROS session before the board gives up
// and reboots itself. At the 1 Hz ping cadence this is about half a minute -
// long enough that a slow agent restart is ridden out, short enough that nobody
// is left waiting on a board that will never come back on its own.
#define AGENT_CREATE_FAIL_LIMIT 30
static uint8_t create_failures = 0;
static uint8_t   agent_retries = 0;

//============================ forward decls ================================//
/** How much the fused speed is trusted, as a variance [(m/s)^2]. The estimator
 *  does not publish one, so this is a conservative constant: it only sets how
 *  fast position uncertainty grows along the direction of travel. */
#define SPEED_TRUST_VAR 0.02f

/** Heading trust as a variance [deg^2], from which source the estimator is
 *  currently locked to. A gyro-only heading is far less trustworthy than one
 *  referenced to GPS course, and that difference belongs in the covariance. */
static float headingTrustVar() {
  switch (estimator.headingRef()) {
    case StateEstimator::REF_GPS:     return 9.0f;    // 3 deg
    case StateEstimator::REF_MAG:     return 25.0f;   // 5 deg
    case StateEstimator::REF_IMU_MAG: return 64.0f;   // 8 deg
    default:                          return 400.0f;  // relative only, 20 deg
  }
}

/** Local ENU from lat/lon, anchored at the first good fix.
 *
 *  The EKF works in metres, but the receiver reports degrees. An
 *  equirectangular projection about a nearby datum is accurate to well under a
 *  centimetre over the hundreds of metres this robot covers, and costs two
 *  multiplies - a full geodetic conversion would be precision nobody can use
 *  on a 2 m fix.
 */
static double datum_lat = 0.0, datum_lon = 0.0;
static bool   datum_set = false;
static float  fix_east = 0.0f, fix_north = 0.0f;

static void fixToLocal(double lat, double lon) {
  if (!datum_set) {
    datum_lat = lat;
    datum_lon = lon;
    datum_set = true;
  }
  const double clat = cos(datum_lat * 0.017453292519943295);
  const float antenna_east  = (float)((lon - datum_lon) * 111320.0 * clat);
  const float antenna_north = (float)((lat - datum_lat) * 110540.0);

  // The receiver reports where the ANTENNA is. The robot turns about the drive
  // axle, and on a two-caster machine the antenna is nowhere near it, so every
  // turn sweeps the antenna through an arc the filter would otherwise read as
  // real movement. Move the reading back to the turning centre before anything
  // else sees it. With the offsets left at zero this is the identity.
  //
  // The offsets are entered from the MIDDLE of the robot, because that is the
  // point a person can find with a tape measure. The lever arm needs them from
  // the drive axle midpoint, which is the point the robot actually turns about
  // - so subtract one from the other here rather than asking anyone to measure
  // from a place they would first have to derive. The axle midpoint is just
  // halfway between the two drive wheels, so it can never disagree with them.
  const float axle_x = 0.5f * (params.get("geometry.wheel.l.x", WHEEL_L_X_M) +
                               params.get("geometry.wheel.r.x", WHEEL_R_X_M));
  const float axle_y = 0.5f * (params.get("geometry.wheel.l.y", WHEEL_L_Y_M) +
                               params.get("geometry.wheel.r.y", WHEEL_R_Y_M));
  LeverArm gps_arm;
  gps_arm.x = params.get("geometry.gps.x", GPS_OFFSET_X_M) - axle_x;
  gps_arm.y = params.get("geometry.gps.y", GPS_OFFSET_Y_M) - axle_y;
  leverArmToCentre(antenna_east, antenna_north, estimator.headingDeg(),
                   gps_arm, &fix_east, &fix_north);
}

static bool createEntities();
static bool destroyEntities();
static void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

//====================== compass calibration storage ========================//
static void loadCompassCalibration() {
  CompassQMC5883::Calibration cal;
  prefs.begin("gpsloc", true);                 // read only
  cal.valid = prefs.getBool("mag_ok", false);
  if (cal.valid) {
    cal.off_x = prefs.getFloat("mag_ox", 0.0f);
    cal.off_y = prefs.getFloat("mag_oy", 0.0f);
    cal.off_z = prefs.getFloat("mag_oz", 0.0f);
    cal.scale_x = prefs.getFloat("mag_sx", 1.0f);
    cal.scale_y = prefs.getFloat("mag_sy", 1.0f);
    cal.scale_z = prefs.getFloat("mag_sz", 1.0f);
    cal.field_norm = prefs.getFloat("mag_fn", 0.0f);
    compass.setCalibration(cal);
    Serial.println("[COMPASS] calibration restored from flash");
  } else {
    Serial.println("[COMPASS] not calibrated yet - run the calibration from the web UI");
  }
  prefs.end();
}

/**
 * Forget the compass calibration and go back to raw readings.
 *
 * There must be a way back, and this one is not optional. While UNCALIBRATED
 * the driver skips its magnetic-disturbance check entirely - look at the guard
 * in compass_qmc5883.h: `if (cal_.valid && cal_.field_norm > 1.0f)`. So a bad
 * calibration does not merely make headings inaccurate, it switches on a test
 * that then rejects every single reading, and the compass goes from working to
 * completely dead. That is exactly what happened here: the compass was healthy
 * all day, a calibration was saved, and from that moment mag_status read
 * "disturbed" on every frame with the heading pinned at zero.
 *
 * Clearing is therefore the recovery path, and it restores the behaviour that
 * was working before.
 */
static void clearCompassCalibration() {
  compass.clearCalibration();              // back to raw, and forget the sweep
  prefs.begin("gpsloc", false);
  prefs.putBool("mag_ok", false);
  prefs.end();
  Serial.println("[COMPASS] calibration cleared - back to raw readings");
}

static void saveCompassCalibration() {
  const CompassQMC5883::Calibration &cal = compass.calibration();
  prefs.begin("gpsloc", false);
  prefs.putBool("mag_ok", cal.valid);
  prefs.putFloat("mag_ox", cal.off_x);
  prefs.putFloat("mag_oy", cal.off_y);
  prefs.putFloat("mag_oz", cal.off_z);
  prefs.putFloat("mag_sx", cal.scale_x);
  prefs.putFloat("mag_sy", cal.scale_y);
  prefs.putFloat("mag_sz", cal.scale_z);
  prefs.putFloat("mag_fn", cal.field_norm);
  prefs.end();
  Serial.println("[COMPASS] calibration saved to flash");
}

//=============================== time ======================================//
static void syncTime() {
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) time_synced = true;
}

static void fillStamp(builtin_interfaces__msg__Time &stamp) {
  if (time_synced) {
    int64_t ns = rmw_uros_epoch_nanos();
    stamp.sec     = (int32_t)(ns / 1000000000LL);
    stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    stamp.sec     = (int32_t)(ms / 1000UL);
    stamp.nanosec = (uint32_t)((ms % 1000UL) * 1000000UL);
  }
}

//=========================== message plumbing ==============================//
static void bindString(rosidl_runtime_c__String &s, char *buf, size_t cap) {
  s.data     = buf;
  s.size     = strlen(buf);
  s.capacity = cap;
}

static void initMessages() {
  memset(&msg_odom, 0, sizeof(msg_odom));
  bindString(msg_odom.header.frame_id, frame_odom, sizeof(frame_odom));
  bindString(msg_odom.child_frame_id,  frame_base, sizeof(frame_base));
  msg_odom.pose.pose.orientation.w = 1.0;
  // covariance: position from GPS (~2.5 m), heading from IMU (~5 deg)
  msg_odom.pose.covariance[0]  = 6.0;    // x
  msg_odom.pose.covariance[7]  = 6.0;    // y
  msg_odom.pose.covariance[35] = 0.01;   // yaw
  msg_odom.twist.covariance[0]  = 0.05;
  msg_odom.twist.covariance[35] = 0.05;

  memset(&msg_imu, 0, sizeof(msg_imu));
  bindString(msg_imu.header.frame_id, frame_imu, sizeof(frame_imu));
  msg_imu.orientation.w = 1.0;
  msg_imu.orientation_covariance[0] = 0.01;
  msg_imu.orientation_covariance[4] = 0.01;
  msg_imu.orientation_covariance[8] = 0.05;
  msg_imu.angular_velocity_covariance[0] = 0.001;
  msg_imu.angular_velocity_covariance[4] = 0.001;
  msg_imu.angular_velocity_covariance[8] = 0.001;
  msg_imu.linear_acceleration_covariance[0] = 0.05;
  msg_imu.linear_acceleration_covariance[4] = 0.05;
  msg_imu.linear_acceleration_covariance[8] = 0.05;

  memset(&msg_fix, 0, sizeof(msg_fix));
  bindString(msg_fix.header.frame_id, frame_gps, sizeof(frame_gps));
  msg_fix.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
  msg_fix.position_covariance_type =
      sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

  // A named message needs no backing buffer: every field is a plain float in
  // the struct, so there is nothing to size or capacity-check.
  memset(&msg_telemetry, 0, sizeof(msg_telemetry));

  memset(&in_config, 0, sizeof(in_config));
  in_config.data.data     = config_buf;
  in_config.data.size     = 0;
  in_config.data.capacity = CONFIG_MAX_LEN;

  memset(&msg_params, 0, sizeof(msg_params));
  msg_params.data.data     = params_buf;
  msg_params.data.size     = 0;
  msg_params.data.capacity = PARAM_COUNT + 1;

  memset(&in_cmd_move, 0, sizeof(in_cmd_move));
  memset(&in_cmd_manual, 0, sizeof(in_cmd_manual));
  memset(&in_cmd_vel, 0, sizeof(in_cmd_vel));
  memset(&in_heartbeat, 0, sizeof(in_heartbeat));
  memset(&in_estop, 0, sizeof(in_estop));
}

//============================== callbacks ==================================//
/**
 * Twist: linear.x = speed [m/s] (signed), angular.z = absolute heading [deg 0..360]
 *        linear.y = 1.0 -> rotate on the spot to that heading and stop
 *                          ("turn 90 degrees", "turn to north"; works indoors,
 *                           no GPS needed)
 */
// Defined further down, next to the rest of the update handling, but called
// from the config callback above it.
static void startPullUpdate();
static void otaOnBegin();

static void cmd_move_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

  if ((float)m->linear.y > 0.5f) {
    drive.setTurnTo((float)m->angular.z);
    last_cmd_ms = millis();
    return;
  }

  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  drive.setAuto(v, (float)m->angular.z);
  last_cmd_ms = millis();
}

/**
 * Standard ROS teleop interface: linear.x [m/s], angular.z [rad/s] counter-clockwise.
 * Lets `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, a joystick node or
 * any other stock tool drive the robot without knowing about this project.
 */
static void cmd_vel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  // ROS yaw rate is counter-clockwise positive, this project steers clockwise positive
  drive.setManual(v, (float)(-m->angular.z) * 57.29578f);
  last_cmd_ms = millis();
}

/** Twist: linear.x = speed [m/s], angular.z = yaw rate [deg/s] - web UI jog buttons */
static void cmd_manual_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  drive.setManual(v, (float)m->angular.z);
  last_cmd_ms = millis();
}

static void heartbeat_cb(const void *msgin) {
  (void)msgin;
  last_heartbeat_ms = millis();
  have_heartbeat = true;
}

static void estop_cb(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  sw_estop = m->data;
  if (sw_estop) drive.stop();
}

/**
 * Runtime configuration, data[0] selects the block:
 *   0, Kp, Ki, Kd, Kf, tol, i_min, i_max   -> heading PID
 *   1, Kp, Ki, Kd, Kf, tol, i_min, i_max   -> speed PID
 *   2, heading_source, align_min, align_tau, gps_tau, model_tau, track_m,
 *      mag_enable, mag_tau, mag_still, declination_deg, mount_offset_deg
 *   3                                       -> reset odometry + estimator
 *   4, 1|0                                  -> start / finish compass calibration
 *   5, accel_ramp_s, decel_ramp_s           -> motor acceleration ramp
 *   6, ke, resistance, stall_a, dA_vf, dA_r, dB_vf, dB_r -> motor model
 *   7, heading_deg                          -> set the heading by hand
 *                                              ("set start pose" from the web map)
 */
/**
 * Push the registry's values into the objects that actually use them.
 *
 * Called after every change, because a parameter that is stored but not applied
 * is the worst outcome available: the UI shows it saved, NVS holds it, and the
 * robot carries on using the old number. Cheap enough to do wholesale rather
 * than tracking which one moved.
 */
/**
 * Publish what the board actually holds: [count, v0, v1, ...].
 *
 * A float array rather than a named message, for the same reason the config
 * topic is one - names would drag string support into the firmware image. The
 * host pairs these with names using config/params.json, generated from the same
 * table and checked by qc.py.
 *
 * Sent on change and slowly on a timer, not at telemetry rate: these move when
 * a person moves them, and a value nobody is changing does not need republishing
 * ten times a second.
 */
static void publishParams() {
  if (conn_state != AGENT_CONNECTED) return;
  msg_params.data.size = 0;
  params_buf[msg_params.data.size++] = (float)params.count();
  for (uint8_t i = 0; i < params.count() && msg_params.data.size < PARAM_COUNT + 1; ++i) {
    params_buf[msg_params.data.size++] = params.value(i);
  }
  RCSOFTCHECK(rcl_publish(&pub_params, &msg_params, NULL));
}

/**
 * A battery tier: the value someone set by hand if they set one, otherwise the
 * per-cell figure times the cell count.
 *
 * The cell count is the knob that should normally be touched - set 2S, 3S or 4S
 * and every tier follows. The absolute volts stay available for a pack with an
 * unusual chemistry, and an explicit setting always wins over the derived one,
 * because silently overriding what a person typed is how you lose their trust
 * in a safety limit.
 */
static float batteryTier(const char *key, float per_cell, float cells) {
  const int16_t i = params.indexOf(key);
  if (i >= 0 && params.overridden((uint8_t)i)) return params.value((uint8_t)i);
  return cells * per_cell;
}

// Live, so it can be tuned where the robot actually drives.
//
// These are CACHED here rather than looked up in the control callback, and that
// is not premature tidiness. ParamRegistry::get() resolves a string key by
// walking the table with strcmp, so every call is up to ninety string compares.
// The callback runs at 100 Hz, so each key left in it costs ten thousand
// lookups a second for a value that changes when a person edits a settings page
// - perhaps twice in a working day. applyParams() already refreshes on every
// change, so the cached copy is never stale.
static uint32_t gps_stale_ms = GPS_STALE_MS;
// Wi-Fi signal at the robot. Sampled in loop(), never in the control callback -
// see publishTelemetry for why. 0 means not associated, which is a different
// thing from a weak signal and is shown as such.
//
// WIFI_RSSI_ENABLE exists to MEASURE what the reading costs, not as a feature
// flag. Build without it and compare the loop timing with tools/loop_probe.py:
//     PLATFORMIO_BUILD_FLAGS=-DWIFI_RSSI_ENABLE=0 pio run -e wifi
// With it off the field is sent as 0, which the web UI shows as "not connected".
#ifndef WIFI_RSSI_ENABLE
#define WIFI_RSSI_ENABLE 0
#endif
static float    wifi_rssi_dbm = 0.0f;
static float    max_speed_cached     = ROBOT_MAX_SPEED_MPS;
static int      min_sats_speed_cached = GPS_MIN_SATS_FOR_SPEED;
static float    max_hdop_speed_cached = GPS_MAX_HDOP_FOR_SPEED;
static int      min_sats_fix_cached   = GPS_MIN_SATS_FOR_FIX;

static void applyParams() {
  MotorModel::Config mc;
  mc.ke_v_per_rpm     = params.get("motor.ke", MOTOR_KE_V_PER_RPM);
  mc.resistance_ohm   = params.get("motor.resistance", MOTOR_RESISTANCE_OHM);
  mc.wheel_diameter_m = params.get("robot.wheel_diameter", WHEEL_DIAMETER_M);
  mc.min_duty         = MOTOR_MIN_DUTY_FOR_EST;
  mc.stall_current_a  = params.get("motor.stall_a", MOTOR_STALL_CURRENT_A);
  mc.open_current_a   = MOTOR_OPEN_CURRENT_A;

  mc.diode_vf_v  = params.get("motor.a.diode_vf", MOTOR_A_DIODE_VF_V);
  mc.diode_r_ohm = params.get("motor.a.diode_r", MOTOR_A_DIODE_R_OHM);
  model_left.begin(mc);

  mc.diode_vf_v  = params.get("motor.b.diode_vf", MOTOR_B_DIODE_VF_V);
  mc.diode_r_ohm = params.get("motor.b.diode_r", MOTOR_B_DIODE_R_OHM);
  model_right.begin(mc);

  // Battery tiers, scaled to whatever pack is fitted.
  gps_stale_ms = (uint32_t)params.get("gps.stale_ms", (float)GPS_STALE_MS);
  max_speed_cached      = params.get("robot.max_speed", ROBOT_MAX_SPEED_MPS);
  min_sats_speed_cached = (int)params.get("gps.min_sats_speed", (float)GPS_MIN_SATS_FOR_SPEED);
  max_hdop_speed_cached = params.get("gps.max_hdop_speed", GPS_MAX_HDOP_FOR_SPEED);
  min_sats_fix_cached   = (int)params.get("gps.min_sats_fix", (float)GPS_MIN_SATS_FOR_FIX);

  const float cells = params.get("battery.cells", (float)BATT_CELLS);
  safety.setBatteryThresholds(
      batteryTier("battery.warn_v",      BATT_WARN_V_PER_CELL,      cells),
      batteryTier("battery.soft_stop_v", BATT_SOFT_STOP_V_PER_CELL, cells),
      batteryTier("battery.cutoff_v",    BATT_CUTOFF_V_PER_CELL,    cells));

  drive.setSpinAboveDeg(params.get("control.spin_above_deg", HEADING_SPIN_ERR_DEG));

  StateEstimator::Config &ec = estimator.config();
  ec.gps_tau_s        = params.get("estimator.gps_tau", SPEED_EST_GPS_TAU_S);
  ec.model_tau_s      = params.get("estimator.model_tau", SPEED_EST_MODEL_TAU_S);
  ec.align_min_mps    = params.get("estimator.align_min", HEADING_ALIGN_MIN_MPS);
  ec.track_m          = params.get("robot.track", WHEEL_TRACK_M);
  ec.wheel_diameter_m = params.get("robot.wheel_diameter", WHEEL_DIAMETER_M);
  ec.max_speed_mps    = params.get("robot.max_speed", ROBOT_MAX_SPEED_MPS);

  compass.setDeclination(params.get("compass.declination", COMPASS_DECLINATION_DEG));
}

static void config_cb(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size < 1) return;
  const int sel = (int)m->data.data[0];
  const size_t n = m->data.size;

  if (sel == 0 || sel == 1) {
    if (n < 6) return;
    PIDF &pid = (sel == 0) ? drive.headingPid() : drive.speedPid();
    pid.setPIDF(m->data.data[1], m->data.data[2], m->data.data[3], m->data.data[4], m->data.data[5]);
    if (n >= 8) pid.setIClamp(m->data.data[6], m->data.data[7]);
    pid.reset();
  } else if (sel == 8) {
    // [8, index, value] - set one parameter. By INDEX, because this topic is a
    // float array and cannot carry a name; tools/gen_param_manifest.py keeps
    // the web UI's idea of the indices in step with the table, and qc.py fails
    // when they drift.
    //
    // The bounds and the moving guard are enforced HERE, on the board, not only
    // in the UI. A value arriving over any transport - a stale browser tab, a
    // script, a serial console - gets the same refusal.
    if (n < 3) return;
    const uint8_t idx = (uint8_t)m->data.data[1];
    const bool moving = fabsf(estimator.speed()) > 0.05f;
    const ParamResult r = params.setIndex(idx, m->data.data[2], moving);
    if (r == PARAM_OK) {
      applyParams();
      publishParams();
      Serial.printf("[param] %s = %.4f\n",
                    idx < params.count() ? params.def(idx).key : "?",
                    m->data.data[2]);
    } else {
      Serial.printf("[param] refused index %u: %s\n", idx, paramResultText(r));
    }
  } else if (sel == 9) {
    // [9] reset every parameter to its compiled-in default.
    params.resetAll(fabsf(estimator.speed()) > 0.05f);
    applyParams();
    publishParams();
    Serial.println("[param] all reset to defaults");
  } else if (sel == 2) {
    StateEstimator::Config &c = estimator.config();
    if (n >= 2) c.heading_source = (uint8_t)m->data.data[1];
    if (n >= 3) c.align_min_mps  = m->data.data[2];
    if (n >= 4) c.align_tau_s    = m->data.data[3];
    if (n >= 5) c.gps_tau_s      = m->data.data[4];
    if (n >= 6) c.model_tau_s    = m->data.data[5];
    if (n >= 7) c.track_m        = m->data.data[6];
    if (n >= 8) c.mag_enable     = m->data.data[7] > 0.5f;
    if (n >= 9) c.mag_tau_s      = m->data.data[8];
    if (n >= 10) c.mag_still_mps = m->data.data[9];
    if (n >= 11) compass.setDeclination(m->data.data[10]);
    if (n >= 12) compass.setMounting(m->data.data[11], COMPASS_INVERT);
  } else if (sel == 3) {
    estimator.reset();
    drive.reset();
  } else if (sel == 4) {
    // compass calibration:
    //   [4, 1] start capturing
    //   [4, 0] finish and save
    //   [4, 2] clear - forget the stored calibration and go back to raw
    //
    // Clear is here rather than only on the USB console because a bad
    // calibration kills the compass outright, and needing a cable to undo it
    // means the robot is unusable in the field until someone fetches one.
    const int act = (n >= 2) ? (int)m->data.data[1] : 0;
    if (act == 1) {
      compass.startCalibration();
      mag_calib_state = 1;
    } else if (act == 2) {
      clearCompassCalibration();
      mag_calib_state = 0;
    } else if (act == 3) {
      // Abandon without saving. Finish either stores a calibration or refuses,
      // and neither is what you want when the sweep was started by accident or
      // there is no room to turn.
      compass.cancelCalibration();
      mag_calib_state = 0;
      Serial.println("[COMPASS] calibration cancelled - nothing saved");
    } else if (compass.calibrating()) {
      const bool good = compass.finishCalibration();
      mag_calib_state = good ? 2 : 3;
      if (good) saveCompassCalibration();
    }
  } else if (sel == 11) {
    // BNO085 calibration:
    //   [11, 1] start learning again
    //   [11, 0] save what it has learned into the sensor's own flash
    //   [11, 2] throw the stored calibration away and restart the sensor
    //
    // The BNO085 calibrates itself from motion, but stops once it has a saved
    // record. If the robot changes around it - a battery fitted beside it, the
    // sensor remounted - that record is wrong and its reported accuracy never
    // recovers. accuracy() is published so the web can show when the
    // figure-of-eight has actually worked rather than asking people to guess.
    // Refused while the robot is moving, and this one is not a matter of taste.
    // These sh2 commands block until the sensor answers - opProcess() spins on
    // shtp_service() with no timeout of its own - so a slow or absent reply
    // stalls the control loop for as long as it takes. A stalled control loop
    // is a robot that is still driving and no longer checking anything.
    // COMMANDED motion, not estimated. The estimate is the wrong test here for
    // the obvious reason: when the IMU is the thing that has failed, the speed
    // estimate falls back to the motor model, whose idle noise sat above any
    // sensible threshold and refused the calibration on a robot that was
    // standing perfectly still - while reporting the reason as "ok", because
    // the sensor had never been asked anything.
    if (fabsf(drive.targetSpeed()) > 1e-3f || fabsf(drive.targetYawRate()) > 1e-3f) {
      imu_calib_state = 3;
      imu_calib_rc = IMU_CALIB_MOVING;
      Serial.println("[imu] calibration refused: the robot is moving");
    } else {
      const int act = (n >= 2) ? (int)m->data.data[1] : 0;
      if (act == 3) {
        // Try to bring a dead IMU up again, right now, without a reflash.
        //
        // The automatic retry gives up after ten attempts because a BNO085
        // whose reset line is not wired cannot be recovered by software - see
        // ImuBno085::retryBegin. But the operator CAN change the situation
        // between attempts: unplugging USB and the battery really does reset
        // the sensor. Without this the only way to try again after doing that
        // was to reflash, which is absurd.
        imu.resetRetries();
        const bool up = imu.retryBegin();
        imu_calib_state = up ? 0 : 3;
        imu_calib_rc = up ? 0 : imu.lastCalibrationResult();
        Serial.printf("[imu] manual retry -> %s\n", up ? "up" : "still not starting");
      } else if (act == 1) {
        imu_calib_state = imu.startCalibration() ? 1 : 3;
      } else if (act == 2) {
        imu_calib_state = imu.clearCalibration() ? 0 : 3;
      } else {
        imu_calib_state = imu.saveCalibration() ? 2 : 3;
      }
      imu_calib_rc = imu.lastCalibrationResult();
      Serial.printf("[imu] calibration action %d -> state %u, sh2 rc %d, accuracy %u/3\n",
                    act, (unsigned)imu_calib_state, imu_calib_rc,
                    (unsigned)imu.accuracy());
    }
  } else if (sel == 5) {
    const float accel = (n >= 2) ? m->data.data[1] : MOTOR_ACCEL_RAMP_S;
    const float decel = (n >= 3) ? m->data.data[2] : MOTOR_DECEL_RAMP_S;
    drive.setRamp(accel, decel);
  } else if (sel == 6) {
    // electrical motor model:
    // [6, ke, resistance, stall_a, diodeA_vf, diodeA_r, diodeB_vf, diodeB_r]
    if (n >= 2) { model_left.config().ke_v_per_rpm = m->data.data[1];
                  model_right.config().ke_v_per_rpm = m->data.data[1]; }
    if (n >= 3) { model_left.config().resistance_ohm = m->data.data[2];
                  model_right.config().resistance_ohm = m->data.data[2]; }
    if (n >= 4) { model_left.config().stall_current_a = m->data.data[3];
                  model_right.config().stall_current_a = m->data.data[3]; }
    if (n >= 5) model_left.config().diode_vf_v   = m->data.data[4];
    if (n >= 6) model_left.config().diode_r_ohm  = m->data.data[5];
    if (n >= 7) model_right.config().diode_vf_v  = m->data.data[6];
    if (n >= 8) model_right.config().diode_r_ohm = m->data.data[7];
  } else if (sel == 10) {
    // Pull an update from the host. Deliberately its own selector rather than a
    // flag on another: it reboots the robot, and that should never be a side
    // effect of setting something else.
#if defined(MICROROS_TRANSPORT_WIFI)
    startPullUpdate();
#else
    // The serial build has no network to pull over, and on this transport the
    // UART is the micro-ROS link itself. Say so rather than failing to link:
    // this branch used to reference a function that only exists in the Wi-Fi
    // build, which broke the serial firmware outright.
    Serial.println("[OTA] pull updates need the Wi-Fi build; flash over USB instead");
#endif
  } else if (sel == 7) {
    // operator set the pose by hand on the map: pin the heading, restart odometry
    if (n < 2) return;
    const float imu_yaw = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.yawDeg() + IMU_YAW_OFFSET_DEG);
    estimator.setHeading(m->data.data[1], imu_yaw);
    estimator.resetOdom();
    drive.reset();
  }
}

//============================== publishing =================================//
static void publishOdom() {
  fillStamp(msg_odom.header.stamp);
  // Prefer the GPS-corrected position; fall back to dead reckoning until the
  // first fix anchors the filter.
  msg_odom.pose.pose.position.x = (double)(pose_ekf.hasFix() ? pose_ekf.x() : estimator.x());
  msg_odom.pose.pose.position.y = (double)(pose_ekf.hasFix() ? pose_ekf.y() : estimator.y());
  msg_odom.pose.pose.position.z = 0.0;

  const double half = 0.5 * (double)estimator.headingEnuRad();
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;
  msg_odom.pose.pose.orientation.z = sin(half);
  msg_odom.pose.pose.orientation.w = cos(half);

  msg_odom.twist.twist.linear.x  = (double)estimator.speed();
  msg_odom.twist.twist.angular.z = (double)estimator.yawRate();

  // Say so if this never actually goes out.
  //
  // RCSOFTCHECK discards the return code, which is right for a publisher that
  // may occasionally miss a cycle and wrong for one that never works at all:
  // /odom was silent for the entire life of this robot and nothing anywhere
  // said why. The cause, found in T186: Odometry serialises to 724 bytes - two
  // 36-wide float64 covariance arrays - and a BEST_EFFORT stream cannot
  // fragment, so it must fit in one transport MTU. That MTU was 512, because
  // micro_ros_platformio's Wi-Fi and serial transports are CUSTOM transports
  // and colcon.meta had only raised UCLIENT_UDP_TRANSPORT_MTU, which this build
  // never uses. Every other message here is 324 bytes or less, which is why
  // /odom alone was silent. UCLIENT_CUSTOM_TRANSPORT_MTU is now 1024, and
  // tools/qc.py fails if it drops back below the largest message.
  //
  // Printed once per failure run, not per cycle: this publishes at 20 Hz and a
  // message every cycle would flood the console, which on this board blocks
  // Serial and starves the control loop.
  const rcl_ret_t odom_rc = rcl_publish(&pub_odom, &msg_odom, NULL);
  static rcl_ret_t last_odom_rc = RCL_RET_OK;
  if (odom_rc != last_odom_rc) {
    last_odom_rc = odom_rc;
    if (odom_rc != RCL_RET_OK) {
      Serial.printf("[odom] rcl_publish refused it, rc=%d - nothing is reaching "
                    "the graph on this topic\n", (int)odom_rc);
    } else {
      Serial.println("[odom] publishing again");
    }
  }
}

static void publishImu() {
  fillStamp(msg_imu.header.stamp);
  msg_imu.orientation.w = (double)imu.qw();
  msg_imu.orientation.x = (double)imu.qx();
  msg_imu.orientation.y = (double)imu.qy();
  msg_imu.orientation.z = (double)imu.qz();
  msg_imu.angular_velocity.x = (double)imu.gyroX();
  msg_imu.angular_velocity.y = (double)imu.gyroY();
  msg_imu.angular_velocity.z = (double)imu.gyroZ();
  msg_imu.linear_acceleration.x = (double)imu.accX();
  msg_imu.linear_acceleration.y = (double)imu.accY();
  msg_imu.linear_acceleration.z = (double)imu.accZ();
  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

static void publishFix() {
  fillStamp(msg_fix.header.stamp);
  const uint8_t q = gps.fixQuality();
  msg_fix.status.status = (q == 0) ? sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX
                        : (q >= 2) ? sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX
                                   : sensor_msgs__msg__NavSatStatus__STATUS_FIX;
  msg_fix.latitude  = gps.latitude();
  msg_fix.longitude = gps.longitude();
  msg_fix.altitude  = (double)gps.altitude();

  // rough covariance from HDOP (2.5 m base accuracy for a consumer M10)
  double sigma = 2.5 * (double)(gps.hdop() > 0.5f ? gps.hdop() : 1.0f);
  msg_fix.position_covariance[0] = sigma * sigma;
  msg_fix.position_covariance[4] = sigma * sigma;
  msg_fix.position_covariance[8] = (sigma * 2.0) * (sigma * 2.0);
  RCSOFTCHECK(rcl_publish(&pub_fix, &msg_fix, NULL));
}

static void publishTelemetry(RunState state) {
  // Named fields, not indices. Adding a signal is one line in
  // gps_localize_msgs/msg/Telemetry.msg plus one line here - there is no
  // position to keep in step across three files any more.
  gps_localize_msgs__msg__Telemetry &m = msg_telemetry;
  m.state = (float)state;
  m.heading_deg = estimator.headingDeg();
  m.target_heading_deg = drive.targetHeading();
  m.heading_error_deg = drive.headingError();
  m.speed_mps = estimator.speed();
  m.target_speed_mps = drive.targetSpeed();
  m.wheel_rpm_left = estimator.wheelLeftRpm();
  m.wheel_rpm_right = estimator.wheelRightRpm();
  m.wheel_mps_left = estimator.wheelLeftMps();
  m.wheel_mps_right = estimator.wheelRightMps();
  m.roll_rate_dps = imu.gyroX() * 57.29578f;
  m.pitch_rate_dps = imu.gyroY() * 57.29578f;
  m.yaw_rate_dps = estimator.yawRate() * 57.29578f;
  m.imu_roll_deg = imu.rollDeg();
  m.imu_pitch_deg = imu.pitchDeg();
  m.imu_yaw_deg = imu.yawDeg();
  m.pwm_left = (float)drive.pwmLeft();
  m.pwm_right = (float)drive.pwmRight();
  m.battery_v = power.packVoltage();                      // pack, from the motor rail
  m.battery_a = power.moduleCurrent();                    // module draw, battery side
  m.motor_a_v = power.motorAVoltage();
  m.motor_a_a = power.motorACurrent();
  m.motor_b_v = power.motorBVoltage();
  m.motor_b_a = power.motorBCurrent();
  m.gps_fix_quality = (float)gps.fixQuality();
  m.gps_satellites = (float)gps.satellites();
  m.gps_hdop = gps.hdop();
  m.gps_speed_mps = gps.speedMps();
  m.gps_course_deg = gps.courseDeg();
  m.stop_flags = (float)safety.flags();
  m.free_heap_kb = (float)(ESP.getFreeHeap() / 1024);
  m.loop_us = (float)loop_us;
  m.odom_x = pose_ekf.hasFix() ? pose_ekf.x() : estimator.x();
  m.odom_y = pose_ekf.hasFix() ? pose_ekf.y() : estimator.y();
  m.mag_heading_deg = compass.headingDeg();
  m.heading_ref = (float)estimator.headingRef();          // 0 relative, 1 compass, 2 GPS, 3 IMU mag
  m.mag_status = compass.ok() ? 1.0f : (compass.disturbed() ? 2.0f : 0.0f);
  m.mag_calib_state = (float)mag_calib_state;
  // How far round the circle the compass sweep has actually got, 0..1. Sample
  // count is not progress: standing still piles up thousands of samples and
  // calibrates nothing. Zero unless a sweep is running, so the web shows a bar
  // that only moves while the robot turns.
  m.mag_calib_progress = compass.calibrating() ? compass.calibrationCoverage() : 0.0f;
  // How far it has really turned, so the web can say "247 of 360" rather than
  // only showing a bar. A number the operator can act on beats a proportion.
  m.mag_turned_deg = compass.turnedDeg();
  // The BNO085's own confidence in its magnetometer, 0 unreliable .. 3 high.
  // The heading chain refuses to use this sensor below 2, and until now that
  // number existed only in the USB console's `info` output - so from a browser
  // there was no way to tell a magnetometer that needs moving from one that is
  // broken. Both look identical: imu_magnetometer_missing, forever.
  m.imu_accuracy = (float)imu.accuracy();
  // Linear acceleration, gravity already removed by the sensor. Shown because
  // "the IMU is alive" and "the IMU is producing sane numbers" are different
  // claims, and only the second one is useful when a speed estimate looks wrong.
  m.imu_acc_x = imu.accX();
  m.imu_acc_y = imu.accY();
  m.imu_acc_z = imu.accZ();
  m.imu_calib_state = (float)imu_calib_state;
  // Counts the sensor resetting ITSELF. A rising number while accuracy stays
  // high says the sensor is healthy but keeps restarting - which is a power or
  // bus problem, not a calibration one, and looks identical on the dashboard.
  m.imu_resets = (float)imu.resets();
  // The delivered rate of each report. On the USB console these answered the
  // question "is the magnetometer broken or is the bus full", but the robot is
  // driven over Wi-Fi and the console needs a cable - so the answer was out of
  // reach exactly when it was wanted. Compare each against what was asked for:
  // 10 Hz for rv, 100 for game and gyro, 50 for accel.
  m.imu_hz_rv    = imu.rvHz();
  m.imu_hz_game  = imu.gameHz();
  m.imu_hz_gyro  = imu.gyroHz();
  m.imu_hz_accel = imu.accHz();
  // The compass and the IMU checked against each other. Near zero means two
  // independent sources agree, which is worth more than either alone; large
  // means one of them is disturbed and the robot is refusing it rather than
  // steering to it. Without this on the screen there is no way to tell a
  // disturbed compass from a drifted gyro - they look identical from outside.
  m.heading_innov_deg = estimator.headingInnovDeg();
  m.heading_gated     = estimator.magRejecting() ? 1.0f : 0.0f;
  // Sampled once a second in loop(), not read here.
  //
  // WiFi.RSSI() is not a free memory read: it calls esp_wifi_sta_get_ap_info(),
  // which takes the Wi-Fi driver's own lock, so it can wait on the radio task
  // mid-transmission. This runs inside controlCallback, where the PID, the
  // safety chain and the estimator live, and blocking that loop on the radio is
  // the exact shape of bug that has cost this project the most time. A signal
  // strength that is up to a second old is worth nothing less on a dashboard.
  m.wifi_rssi_dbm     = wifi_rssi_dbm;
  // imu_calib_rc holds the result of the last calibration COMMAND, and it
  // starts at 0 - which decodes as "ok". On a board where the IMU never
  // started, and where no calibration had ever been asked for, the web
  // therefore reported the IMU's last operation as "ok" while the sensor had
  // not opened at all. Two screens disagreed: the standalone tool said the
  // handshake was failing on every attempt, and the dashboard said everything
  // was fine. The dashboard was reading a field that had never been written.
  //
  // So when the driver has not started, its own diagnosis outranks a command
  // result that was never issued.
  m.imu_calib_result = (float)(imu.begun() ? imu_calib_rc
                                           : imu.lastCalibrationResult());
  m.sensor_health = (float)estimator.health();            // which sensors are missing, bitfield
  m.speed_source = (float)estimator.speedSource();
  m.motor_rpm_left = model_left.valid()  ? model_left.rpm()  : 0.0f;// back-EMF wheel speed
  m.motor_rpm_right = model_right.valid() ? model_right.rpm() : 0.0f;
  // Motor fault bitfield. Built here rather than assigned from an outer
  // variable: the old packed-array version computed `f` inside its own scope
  // block, and lifting the assignment out of that block left `f` undeclared.
  {
    uint8_t faults = 0;
    if (model_left.stalled())      faults |= 1 << 0;
    if (model_right.stalled())     faults |= 1 << 1;
    if (model_left.openCircuit())  faults |= 1 << 2;
    if (model_right.openCircuit()) faults |= 1 << 3;
    m.motor_faults = (float)faults;
  }
  m.fan_v = power.fanVoltage();                           // draft_5: fan rail sensor
  m.fan_a = power.fanCurrent();
  m.esp32_rail_v = power.esp32RailVoltage();              // ESP32 +5 V rail health
  // So the host can say when the board is behind it. Four bytes, no new
  // publisher, no string handling, no extra type support in the image.
  m.firmware_version = (float)FIRMWARE_VERSION;
  // How the board last restarted, and how long it has been up. A brownout,
  // a panic and a person pressing reset are indistinguishable in the logs
  // without this - and a 3.3 V rail sagging under Wi-Fi bursts shows up as
  // ESP_RST_BROWNOUT here and nowhere else.
  // Die temperature, if this chip actually has a usable sensor.
  //
  // The original ESP32 has one that Espressif disabled after early silicon:
  // temperatureRead() still answers, but on most WROOM-32 boards it returns a
  // fixed value near 53 C however hot the chip gets. A constant is WORSE than
  // no reading, because it looks like data - so believe it only once it has
  // been seen to move, and report NaN until then.
  {
    static float  first_temp = NAN;
    static bool   temp_moves = false;
    const float t_now = temperatureRead();
    if (isnan(first_temp)) {
      first_temp = t_now;
    } else if (!temp_moves && fabsf(t_now - first_temp) > 0.4f) {
      temp_moves = true;                  // it changed, so it is measuring
    }
    m.esp32_temp_c = temp_moves ? t_now : NAN;
  }
  m.reset_reason = (float)esp_reset_reason();
  m.uptime_s     = (float)(millis() / 1000.0f);
  m.pack_valid = power.packValid() ? 1.0f : 0.0f;         // 0 = rail unpowered, pack unknown
  m.pos_sigma_m = pose_ekf.positionSigma();               // 1-sigma position uncertainty [m]
  m.pos_anchored = pose_ekf.hasFix() ? 1.0f : 0.0f;       // has the EKF ever been anchored
  RCSOFTCHECK(rcl_publish(&pub_telemetry, &msg_telemetry, NULL));
}

//============================ control loop =================================//
static float forwardAccel() {
#if IMU_ACC_FORWARD_AXIS == 0
  return IMU_ACC_FORWARD_SIGN * imu.accX();
#elif IMU_ACC_FORWARD_AXIS == 1
  return IMU_ACC_FORWARD_SIGN * imu.accY();
#else
  return IMU_ACC_FORWARD_SIGN * imu.accZ();
#endif
}

// ===================== TEMPORARY loop profiler (T211) =====================
// Answers "why is the loop slow" with numbers instead of opinions: it splits
// controlCallback into its sections and reports the average and the worst of
// each. Printed once every 2 s, which is far too slow to affect what it
// measures. REMOVE once the question is settled - this is a diagnostic, not a
// feature, and Serial output in this loop is exactly the kind of thing that
// starves it.
#define LOOP_PROFILE 1
#if LOOP_PROFILE
static const int PROF_N = 9;
static const char *PROF_NAME[PROF_N] =
    {"imu", "gps", "sens", "model", "est", "ekf", "safety", "drive", "pub"};
static uint32_t prof_sum[PROF_N];
static uint32_t prof_max[PROF_N];
static uint32_t prof_cycles = 0;
static uint32_t prof_mark = 0;
#define PROF(i) do { const uint32_t _n = micros(); const uint32_t _d = _n - prof_mark; \
                     prof_sum[(i)] += _d; if (_d > prof_max[(i)]) prof_max[(i)] = _d; \
                     prof_mark = _n; } while (0)
#else
#define PROF(i) do {} while (0)
#endif

static void controlCallback(rcl_timer_t *timer, int64_t) {
  if (!timer) return;
  const uint32_t t_start = micros();
#if LOOP_PROFILE
  prof_mark = t_start;
#endif
  const uint32_t now_ms  = millis();

  float dt = (last_ctrl_us == 0) ? (CTRL_PERIOD_MS / 1000.0f)
                                 : (t_start - last_ctrl_us) * 1e-6f;
  last_ctrl_us = t_start;
  if (dt <= 0.0f || dt > 0.5f) dt = CTRL_PERIOD_MS / 1000.0f;

  //------------------------------ sensors ------------------------------//
  // The BNO085's reset line is not wired on this board, so an ESP32 software
  // reset - which is what an over-the-air update is - leaves the sensor in
  // whatever I2C state it was in and begin() fails. Without this retry that was
  // permanent: every Wi-Fi flash came back with imu_missing and stayed there
  // until someone went and pulled the power, which is the one errand OTA exists
  // to save. One probe every few seconds, and only while it has never started.
  //
  // Only while stopped, and with the start-up timeout restored for the attempt:
  // the handshake needs it, and it is the one operation here that is allowed to
  // take longer than a control period - which is exactly why it must not happen
  // while the robot is moving.
  if (!imu.begun() && imu.beginRetries() < IMU_RETRY_MAX &&
      fabsf(drive.targetSpeed()) <= 1e-3f && fabsf(drive.targetYawRate()) <= 1e-3f &&
      (now_ms - last_imu_retry_ms) >= IMU_RETRY_MS) {
    last_imu_retry_ms = now_ms;
    Wire.setTimeOut(I2C_TIMEOUT_INIT_MS);
    const bool up = imu.retryBegin();
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    if (up)
      Serial.printf("[IMU] BNO085 came up on retry %u\n", (unsigned)imu.beginRetries());
  }
  imu.update();
  PROF(0);

  /* Keep a good calibration through the resets instead of only trying to
   * prevent them.
   *
   * The BNO085 resets itself when the supply dips - the Wi-Fi radio was doing
   * it hundreds of times, and turning the radio down made it rare rather than
   * impossible. A reset that loses the calibration costs far more than the gap
   * in the data: accuracy drops to 0, the magnetometer-referenced heading stops
   * being usable, and it only comes back if somebody walks over and waves the
   * robot around again. Seen exactly that way - a steady 3/3 falling to 0 and
   * staying there.
   *
   * sh2_saveDcdNow writes the calibration into the sensor's own flash, and the
   * sensor restores it on the next reset by itself. So the first time it
   * reaches full confidence, that gets written down. After that a brown-out
   * costs a fraction of a second of data rather than the calibration.
   *
   * Once per boot, and never while somebody is running the calibration from the
   * web: saving ends a calibration run, and ending theirs from underneath them
   * is how a working procedure turns into "I pressed Start and nothing
   * happened".
   */
  static bool imu_dcd_saved = false;
  // accuracy() now follows whichever rotation vector is running, so this is
  // reachable again. While the report mode was game-only it was not: the single
  // writer lived in the other report's handler, accuracy stayed 0 for ever, and
  // this save could never fire.
  if (!imu_dcd_saved && imu_calib_state == 0 && imu.begun() && imu.accuracy() >= 3) {
    imu_dcd_saved = imu.saveCalibration();
    Serial.printf("[IMU] accuracy reached 3/3 - calibration %s to the sensor's "
                  "flash, so a brown-out reset no longer loses it\n",
                  imu_dcd_saved ? "SAVED" : "could NOT be saved");
  }

  gps.update();
  PROF(1);

  // Feed the IMU's own FUSED heading to the compass calibration, not the raw
  // gyro rate. The BNO085 has already integrated gyro and accelerometer into a
  // rotation vector, at its own rate and with the zero drift corrected against
  // gravity - re-integrating rate samples in this loop would throw that work
  // away and redo it worse, losing turn on every late or dropped cycle.
  //
  // This is the measurement that says whether the robot ACTUALLY went round.
  // Everything else the calibration knows comes from the magnetometer itself,
  // which is the thing being calibrated - so a distorted field could both cause
  // a bad result and vouch for it. The IMU cannot be fooled that way.
  if (imu.ok()) compass.addTurnYaw(imu.yawDeg());

  if (++div_mag >= MAG_READ_DIV) {
    div_mag = 0;
    compass.update(imu.rollDeg(), imu.pitchDeg());   // tilt compensated with the IMU
  }

  if (++div_ina >= INA_READ_DIV) {
    div_ina = 0;
    power.update();
  }

  PROF(2);
  // wheel speed from volts + amps (back-EMF), one estimate per motor
  model_left.update(dt, power.motorAVoltage(), power.motorACurrent(),
                    (float)drive.pwmLeft() / (float)PWM_MAX, power.motorAOk());
  model_right.update(dt, power.motorBVoltage(), power.motorBCurrent(),
                     (float)drive.pwmRight() / (float)PWM_MAX, power.motorBOk());

  PROF(3);
  //----------------------------- estimation ----------------------------//
  const float imu_yaw   = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.yawDeg() + IMU_YAW_OFFSET_DEG);
  const float yaw_rate  = IMU_GYRO_Z_SIGN * imu.gyroZ();   // CCW positive, ROS convention
  // A receiver with NO FIX still streams NMEA continuously, and the speed and
  // course in those sentences are meaningless - often stale, sometimes garbage.
  // Freshness alone is therefore not enough to trust them: it says the receiver
  // is talking, not that it knows where it is.
  //
  // Measured on the bench with the antenna indoors: 0 satellites, HDOP 99.99,
  // fix quality 0 - and a reported ground speed of 2.53 m/s. The estimator
  // believed it, so a board sitting still on a desk reported drifting at up to
  // 1.5 m/s. With a motor driver attached the controller would have acted on
  // that, and OTA refused every update because it thought the robot was moving.
  //
  // The old test - a fix quality above zero and at least one satellite - was
  // far too weak, and this robot proved it: standing still indoors on a
  // 3-satellite fix with HDOP 1.3, the receiver reported ground speeds of 1.6,
  // 3.3, 4.8 and 7.28 m/s, every one of which passed. The fused speed sat
  // pinned at its 0.36 m/s ceiling and the odometry wandered a hundred metres
  // across a room where nothing moved.
  //
  // Three tests now, and the last one is the one that actually works: this
  // machine cannot exceed ROBOT_MAX_SPEED_MPS, so a reading far above that is
  // wrong by definition however good the fix claims to be.
  const float max_speed      = max_speed_cached;
  const int   min_sats_speed = min_sats_speed_cached;
  const float max_hdop_speed = max_hdop_speed_cached;
  const bool  gps_quality_ok = gps.fresh(gps_stale_ms)
                          && gps.fixQuality() > 0
                          && gps.satellites() >= min_sats_speed
                          && gps.hdop() > 0.0f
                          && gps.hdop() <= max_hdop_speed;
  const bool  gps_plausible = fabsf(gps.speedMps())
                              <= max_speed * GPS_SPEED_SANITY_FACTOR;
  const bool  gps_ok    = gps_quality_ok && gps_plausible;

  // Which way the robot is being driven. Nothing the receiver sends can say -
  // ground speed is a magnitude and course is the direction of travel - so the
  // commanded speed is the only thing here that knows, and it supplies the sign
  // for both of the corrections below.
  const bool  reversing = drive.rampedSpeed() < -0.02f;

  // ... and course over ground is the direction of TRAVEL. Reversing, that is
  // 180 degrees from where the nose points, and aligning the heading to it would
  // spin the robot's idea of itself right round. Rather than guess a 180 degree
  // correction from a sign we only infer, the course is simply not offered as a
  // heading reference while reversing; the IMU carries the heading through.
  const bool  course_ok = gps.courseValid() && !reversing
                          && gps.speedMps() > estimator.config().align_min_mps;

  // WHETHER THERE IS A FIX AT ALL IS A DIFFERENT QUESTION FROM WHETHER ITS
  // VELOCITY CAN BE TRUSTED, and answering both with `gps_ok` was wrong.
  //
  // `gps_ok` above is a SPEED gate. Its thresholds are named for it -
  // GPS_MIN_SATS_FOR_SPEED, GPS_MAX_HDOP_FOR_SPEED - and they are deliberately
  // strict, because a marginal fix gives a position that is merely inaccurate
  // while its Doppler velocity is nonsense. That is the right bar for feeding
  // the speed filter.
  //
  // It is the wrong bar for the health flag. HEALTH_NO_GPS is what the web UI
  // draws as the GPS sensor being absent, and it was being raised on a perfectly
  // usable fix: 7 satellites at HDOP 1.8 fails the 1.5 speed gate, so the robot
  // reported its GPS as missing while navigating on it. Someone reading that
  // panel is told a sensor is broken when nothing is.
  //
  // This is the position-grade test: a real fix, fresh, with enough satellites
  // for a 3D solution. HDOP is not in it at all - a poor geometry makes a
  // position less precise, not absent, and how much to trust it is already
  // carried by the HDOP field itself.
  const bool  gps_fix_ok = gps.fresh(gps_stale_ms)
                        && gps.fixQuality() > 0
                        && gps.satellites() >= min_sats_fix_cached;

  StateEstimator::Inputs est_in;
  est_in.dt              = dt;
  est_in.imu_ok          = imu.ok();
  est_in.imu_yaw         = imu_yaw;
  est_in.yaw_rate        = yaw_rate;
  est_in.acc_fwd         = forwardAccel();
  est_in.gps_ok          = gps_ok;
  est_in.gps_fix_ok      = gps_fix_ok;
  // A stationary receiver reports a wandering speed, not zero. Anything below
  // the noise floor is that wander, so hand the estimator a clean zero rather
  // than a small lie it will faithfully integrate into the odometry.
  // NMEA ground speed is a MAGNITUDE - it has no sign, because a receiver has no
  // idea which way the vehicle faces. Fed straight in, a reversing robot
  // reported positive forward speed: the manual speed loop then saw a large
  // negative error and drove the motors harder backwards, and the odometry
  // integrated the wrong way. The only thing here that knows the intended
  // direction is the commanded speed, so it supplies the sign.
  const float gps_speed_signed = reversing ? -gps.speedMps() : gps.speedMps();
  est_in.gps_speed       = (fabsf(gps_speed_signed) < GPS_SPEED_NOISE_FLOOR_MPS)
                              ? 0.0f : gps_speed_signed;
  est_in.gps_course      = gps.courseDeg();
  est_in.gps_course_ok   = course_ok;
  est_in.mag_ok          = compass.ok() && !compass.calibrating();
  est_in.mag_heading     = compass.headingDeg();
  est_in.imu_mag_ok      = imu.magYawOk();
  // Only a magnetometer we actually asked for can be "missing".
  est_in.imu_mag_used    = imu.magRequested();
  est_in.imu_mag_heading = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.magYawDeg() + IMU_YAW_OFFSET_DEG);
  // Fallback speed reference. The back-EMF estimate knows about load (slope,
  // grass, a dragging wheel); the open-loop duty model does not. Use it when
  // both motor sensors answered, otherwise fall back to the duty model.
  if (model_left.valid() && model_right.valid()) {
    est_in.model_mps = 0.5f * (model_left.mps() + model_right.mps());
  } else {
    est_in.model_mps = drive.modelSpeedMps();
  }
  // Whether the rails are MEASURED live, not whether we asked them to be.
  // relayClosed() is the commanded state: with no battery, or a coil with no
  // 5 V behind it, or a welded contactor, the firmware can believe the rails
  // are live while they are dead - and the back-EMF model would then be trusted
  // on exactly the noise it invents speed from. packValid() is true only when a
  // motor rail actually reads above PACK_RAIL_MIN_VALID_V.
  est_in.motors_powered = power.packValid();
  estimator.update(est_in);
  PROF(4);

  // Position: predict at the control rate from the fused heading and speed,
  // then correct whenever a fix arrives. Without this, x/y is open-loop dead
  // reckoning and a small heading error becomes metres of drift.
  pose_ekf.predict(dt, estimator.speed(), estimator.headingDeg(),
                   SPEED_TRUST_VAR, headingTrustVar());

  // Correct on each NEW fix. gps.fresh() alone would re-apply the same fix a
  // hundred times a second and make the filter far more confident than the
  // data justifies, so only a changed timestamp counts as a measurement.
  // gps_fix_ok, not gps_ok. gps_ok is the strict SPEED gate - six satellites and
  // HDOP under 1.5 - and gating the POSITION filter with it meant that on a
  // perfectly usable rough fix (measured here: 4 satellites at HDOP 2.7 indoors,
  // 7 at 1.8 outdoors) the EKF received no correction at all and ran as pure
  // dead reckoning while /odom kept being published. The filter already weighs a
  // fix by its HDOP and quality, so a rough one is absorbed weakly rather than
  // believed - which is strictly better than refusing it.
  if (gps_fix_ok) {
    static uint32_t last_fix_ms = 0;
    const uint32_t fix_ms = gps.lastFixMs();
    if (fix_ms != last_fix_ms) {
      last_fix_ms = fix_ms;
      fixToLocal(gps.latitude(), gps.longitude());
      pose_ekf.updateGps(fix_east, fix_north, gps.hdop(), gps.fixQuality());
    }
  }

  PROF(5);
  //------------------------------- safety ------------------------------//
  SafetyManager::Inputs si;
  si.sw_estop         = sw_estop;
  si.heartbeat_age_ms = have_heartbeat ? (now_ms - last_heartbeat_ms) : 0xFFFFFFFF;
  si.cmd_age_ms       = (last_cmd_ms == 0) ? 0xFFFFFFFF : (now_ms - last_cmd_ms);
  si.agent_ok         = (conn_state == AGENT_CONNECTED);
  si.battery_v        = power.packVoltage();
  si.battery_valid    = power.packValid();
  // Measured, so a welded contactor cannot hide behind the commanded state.
  si.rails_live       = power.packValid();
  si.motion_requested = fabsf(drive.targetSpeed()) > 1e-3f ||
                        fabsf(drive.targetYawRate()) > 1e-3f;   // pure rotation counts too
  safety.update(si);
  PROF(6);

  //------------------------------- drive -------------------------------//
  if (!safety.allowMotion()) {
    drive.stop();
    motor_left.disable();
    motor_right.disable();
  } else {
    drive.update(dt, estimator);
    motor_left.spin(drive.pwmLeft());
    motor_right.spin(drive.pwmRight());
  }

  PROF(7);
  //------------------------------ publish ------------------------------//
  // A turn on the spot is motion, whatever the two target numbers say: both are
  // zero during a turn-to-heading, so the state shown on the web page was IDLE
  // while the robot was rotating. The command timeout deliberately still does
  // not apply to a turn - it is a one-shot command meant to outlive its
  // message, and TURN_IN_PLACE_MAX_MS is what bounds it instead.
  const bool moving = si.motion_requested || drive.turningInPlace();
  const RunState state = safety.runState(!drive.manual() && moving, drive.manual() && moving);

  if (++div_odom >= PUB_ODOM_DIV) { div_odom = 0; publishOdom(); }
  if (++div_imu  >= PUB_IMU_DIV)  { div_imu  = 0; if (imu.ok()) publishImu(); }
  // Slowly: values move when a person moves them. Republishing keeps a UI
  // that connected late from showing nothing until someone edits something.
  if (++div_params >= 200) { div_params = 0; publishParams(); }
  if (++div_tel  >= PUB_TELEMETRY_DIV) { div_tel = 0; publishTelemetry(state); }

  // GPS is published when a new solution arrives, capped at 5 Hz
  if (gps.lastFixMs() != last_gps_pub_ms && now_ms - last_gps_pub_ms >= 200) {
    last_gps_pub_ms = gps.lastFixMs();
    publishFix();
  }

  PROF(8);
  loop_us = micros() - t_start;

#if LOOP_PROFILE
  if (++prof_cycles >= 200) {                    // once every 2 s at 100 Hz
    char line[220];
    int n = snprintf(line, sizeof(line), "[prof] over %u cycles, avg/max us:", (unsigned)prof_cycles);
    for (int i = 0; i < PROF_N && n > 0 && n < (int)sizeof(line); ++i) {
      n += snprintf(line + n, sizeof(line) - n, "  %s %u/%u",
                    PROF_NAME[i], (unsigned)(prof_sum[i] / prof_cycles), (unsigned)prof_max[i]);
    }
    Serial.println(line);
    for (int i = 0; i < PROF_N; ++i) { prof_sum[i] = 0; prof_max[i] = 0; }
    prof_cycles = 0;
  }
#endif
}

//========================= entities create/destroy =========================//
static bool createEntities() {
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;

  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

  // BEST EFFORT, and this is a control-loop property rather than a networking
  // preference. A reliable micro-ROS publisher blocks inside rcl_publish until
  // the agent acknowledges, and these four calls happen in controlCallback().
  // Measured on hardware with all of them reliable: the 100 Hz control loop ran
  // at 7.5 Hz, loop_us reached 171 ms, and the link carried roughly ten packets
  // for every message published - the remainder being retransmissions. The
  // safety timing, the PID and the EKF prediction step all live in that loop.
  //
  // These are streams: each frame replaces the last and every consumer already
  // checks staleness, so a dropped frame costs nothing that waiting for the next
  // one does not fix.
  //
  // The host subscribers in gps_localize/qos.py must stay best-effort to match.
  // A BEST_EFFORT publisher does NOT match a RELIABLE subscriber, and the
  // failure is silent - the topic still lists, the publisher still shows as
  // connected, and no message is ever delivered. tools/qc.py checks the pairing.
  RCCHECK(rclc_publisher_init_best_effort(&pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ROS_NS "/odom"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), ROS_NS "/imu/data"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_fix, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), ROS_NS "/gps/fix"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_telemetry, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(gps_localize_msgs, msg, Telemetry), ROS_NS "/telemetry"));

  // Stays RELIABLE. Parameter values are events, not a stream: a UI that misses
  // the frame carrying a changed value shows the old number until someone edits
  // something else, and it is published once every 200 loops, so it cannot be
  // what stalls the loop.
  // BEST EFFORT, not default. A RELIABLE micro-ROS publish blocks inside the
  // control callback until the agent acknowledges it, and this one goes out
  // every two seconds carrying the whole parameter table - which grew from 24
  // entries to 85. Measured: a 129 ms spike in a loop whose period is 10 ms.
  // Losing a sample costs nothing, because the next one is two seconds away.
  // The host subscriber moved to best-effort first; see qos.py for why that
  // order matters.
  RCCHECK(rclc_publisher_init_best_effort(&pub_params, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), ROS_NS "/params/values"));

  RCCHECK(rclc_subscription_init_default(&sub_cmd_move, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_move"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_manual, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_manual"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_heartbeat, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), ROS_NS "/heartbeat"));
  RCCHECK(rclc_subscription_init_default(&sub_estop, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ROS_NS "/estop"));
  RCCHECK(rclc_subscription_init_default(&sub_config, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), ROS_NS "/config/pid"));

  RCCHECK(rclc_timer_init_default(&ctrl_timer, &support, RCL_MS_TO_NS(CTRL_PERIOD_MS), controlCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_move,   &in_cmd_move,   &cmd_move_cb,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_manual, &in_cmd_manual, &cmd_manual_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel,    &in_cmd_vel,    &cmd_vel_cb,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_heartbeat,  &in_heartbeat,  &heartbeat_cb,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_estop,      &in_estop,      &estop_cb,      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_config,     &in_config,     &config_cb,     ON_NEW_DATA));

  syncTime();
  return true;
}

static bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_fix, &node);
  rcl_publisher_fini(&pub_telemetry, &node);

  rcl_subscription_fini(&sub_cmd_move, &node);
  rcl_subscription_fini(&sub_cmd_manual, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_subscription_fini(&sub_heartbeat, &node);
  rcl_subscription_fini(&sub_estop, &node);
  rcl_subscription_fini(&sub_config, &node);

  rcl_timer_fini(&ctrl_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  time_synced = false;
  return true;
}

//================================ setup ====================================//
#if defined(MICROROS_TRANSPORT_WIFI)
#include <ESPmDNS.h>

/**
 * Find the micro-ROS agent by NAME, so a new DHCP lease on the Ubuntu PC does
 * not brick the robot (the mor_luam problem).
 *
 *   1. mDNS   "<AGENT_HOSTNAME>.local"  (Ubuntu: sudo apt install avahi-daemon)
 *   2. DNS    plain lookup, works when the router registers DHCP names
 *   3. AGENT_IP from network.h
 *
 * Called again on every reconnect, so the robot follows the PC to a new address
 * while it is running.
 */
/**
 * Accept whatever the user pasted into AGENT_HOSTNAME and return the bare
 * hostname: "mannaja", "mannaja.local" and "mannaja@mannaja.local" (copied
 * straight from the Ubuntu prompt) all become "mannaja".
 */
static const char *agentHostname() {
  static char name[40];
  const char *src = AGENT_HOSTNAME;

  const char *at = strchr(src, '@');       // drop the "user@" part
  if (at) src = at + 1;

  size_t len = strlen(src);
  const char *suffix = ".local";           // strip it: mDNS wants the bare name
  const size_t suffix_len = 6;
  if (len > suffix_len) {
    bool match = true;
    for (size_t i = 0; i < suffix_len; ++i) {
      char a = src[len - suffix_len + i];
      if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');   // no strcasecmp: not
      if (a != suffix[i]) { match = false; break; }          // portable everywhere
    }
    if (match) len -= suffix_len;
  }
  if (len >= sizeof(name)) len = sizeof(name) - 1;

  memcpy(name, src, len);
  name[len] = '\0';
  return name;
}

/**
 * Is this address on the network the board is actually joined to?
 *
 * An address outside the board's own subnet cannot be reached directly, so
 * preferring it over discovery is not a judgement call - it is always wrong.
 * A hand-set address is remembered in NVS and survives every reboot, so one
 * typed on a network the robot has since left goes on being tried for ever.
 *
 * Seen exactly that way: the board printed "[agent] using 192.168.100.74 (set
 * by hand)" on every boot while it was sitting on 192.168.12.x, wasted the
 * start of each session on an address it could not reach, and only recovered
 * when the beacon announced the real one a moment later.
 *
 * A zero gateway or mask means the board has not finished joining, and nothing
 * can be judged yet - say yes rather than discarding a setting on the strength
 * of a network that does not exist.
 */
/**
 * Push the configured transmit power at the radio.
 *
 * A parameter rather than a constant because it is the one knob that trades
 * range against supply current, and both sides of that trade change with the
 * robot: far from the access point it needs range, and on USB with no battery
 * it cannot afford the current - transmit bursts at full power were browning
 * out the BNO085, which reset and lost its calibration every time.
 *
 * esp_wifi's units are quarter-dBm, so the dBm value is multiplied by four.
 * Applied at every join, so a value changed from the web takes effect on the
 * next reconnect without a reflash.
 */
static void applyTxPower() {
  const float dbm = params.get("net.tx_power_dbm", (float)WIFI_TX_POWER_DBM);
  esp_wifi_set_max_tx_power((int8_t)lroundf(dbm * 4.0f));
  Serial.printf("[WiFi] tx power %.0f dBm - range against supply current, see "
                "net.tx_power_dbm\n", dbm);
}

static bool onOurSubnet(const IPAddress &ip) {
  const IPAddress mask = WiFi.subnetMask();
  const IPAddress mine = WiFi.localIP();
  if ((uint32_t)mask == 0 || (uint32_t)mine == 0) return true;
  return ((uint32_t)ip & (uint32_t)mask) == ((uint32_t)mine & (uint32_t)mask);
}

static IPAddress resolveAgentIp() {
  // 1. What a person told us, if anything. Never guessed over: if someone has
  //    said where the agent is, silently going somewhere else is worse than
  //    failing where they can see it.
  if (agent_override[0] && !agent_override_failed) {
    IPAddress fixed;
    if (fixed.fromString(agent_override) && fixed != agent_dead &&
        onOurSubnet(fixed)) {
      // Say it ONCE, not on every resolve. This runs from the reconnect loop,
      // and printing every time flooded the console at 115200 - which is not
      // merely noisy: Serial.printf blocks once the TX buffer fills, so the
      // logging starved the control loop and the compass reads went stale, and
      // the dashboard reported compass_missing on a compass that was fine.
      static IPAddress said((uint32_t)0);
      if (fixed != said) {
        said = fixed;
        Serial.printf("[agent] using %s (set by hand)\n", agent_override);
      }
      return fixed;
    }
    IPAddress looked = MDNS.queryHost(agent_override, AGENT_RESOLVE_TIMEOUT_MS);
    if ((uint32_t)looked != 0) {
      Serial.printf("[agent] %s -> %s (set by hand, resolved)\n",
                    agent_override, looked.toString().c_str());
      return looked;
    }
    static uint32_t last_moan_ms = 0;
    if (millis() - last_moan_ms > 30000) {
      last_moan_ms = millis();
      Serial.printf("[agent] \"%s\" was set by hand but is not reachable from "
                    "this network - falling through to discovery\n",
                    agent_override);
    }
  }

#if USE_AGENT_HOSTNAME
  static bool mdns_started = false;
  const char *host = agentHostname();
  if (!mdns_started) {
    mdns_started = MDNS.begin(WIFI_HOSTNAME);
    if (!mdns_started) Serial.println("[mDNS] could not start responder");
  }

  if (mdns_started) {
    IPAddress found = MDNS.queryHost(host, AGENT_RESOLVE_TIMEOUT_MS);
    if (found != IPAddress((uint32_t)0)) {
      Serial.printf("[agent] %s.local -> %s (mDNS)\n", host, found.toString().c_str());
      return found;
    }
  }

  // plain DNS, first the bare name (router DHCP names), then name.local
  IPAddress dns_found;
  char dotted[48];
  snprintf(dotted, sizeof(dotted), "%s.local", host);
  const char *candidates[2] = { host, dotted };
  for (uint8_t i = 0; i < 2; ++i) {
    if (WiFi.hostByName(candidates[i], dns_found) == 1 && dns_found != IPAddress((uint32_t)0)) {
      Serial.printf("[agent] %s -> %s (DNS)\n", candidates[i], dns_found.toString().c_str());
      return dns_found;
    }
  }
  // Neither name resolved. Before any guessing, try the address that actually
  // carried a session last time - it costs one ping and is right far more often
  // than a guess, because DHCP leases are usually stable.
  // Every address that has ever carried a session, newest first, one per round.
  //
  // ROTATING MATTERS. Returning only the newest meant a board that had moved
  // between two networks kept offering whichever it saw last, which after a
  // change is precisely the wrong one - and since nothing else in this chain
  // can cross a NAT, that was the end of it. Handing back a different candidate
  // on each retry works through the whole list in a few rounds instead.
  if (agent_good_n > 0) {
    static uint8_t turn = 0;
    for (uint8_t tried = 0; tried < agent_good_n; ++tried) {
      const IPAddress cand = agent_good[(turn + tried) % agent_good_n];
      if (cand == agent_dead || (uint32_t)cand == 0) continue;
      turn = (uint8_t)((turn + tried + 1) % agent_good_n);
      static IPAddress said_good((uint32_t)0);
      if (cand != said_good) {
        said_good = cand;
        Serial.printf("[agent] no name resolved; trying %s, which has worked before\n",
                      cand.toString().c_str());
      }
      return cand;
    }
  }

  // Then the DHCP gateway.
  //
  // WHY THE GATEWAY IS A GOOD GUESS. When the PC shares its connection - a
  // Windows hotspot, ICS, a phone tethering the robot - the PC IS the gateway
  // for the network the board just joined, so this lands exactly on the machine
  // running the agent, on whatever subnet it happens to hand out today. That is
  // the portable case: no address to edit when the network changes.
  //
  // It is only preferred when AGENT_IP is on a DIFFERENT subnet from the board,
  // which is precisely the situation where the compile-time address is stale.
  // Seen here: the board joined a hotspot on 192.168.137.x while AGENT_IP still
  // said 192.168.100.16, and the link came up only intermittently.
  const IPAddress gw = WiFi.gatewayIP();
  const IPAddress mask = WiFi.subnetMask();
  // Same rule as the hand-set address: a compiled-in one is a guess made
  // before this robot had ever seen a network, and the moment it is off-subnet
  // it is not merely stale but unreachable. The gateway is the better guess in
  // that case, because whoever is handing out addresses on this network is
  // usually the machine running the agent.
  const bool agent_on_our_subnet = onOurSubnet(AGENT_IP);
  if (!agent_on_our_subnet && (uint32_t)gw != 0) {
    Serial.printf("[agent] '%s' not found; AGENT_IP %s is off-subnet, using the "
                  "gateway %s instead\n",
                  host, AGENT_IP.toString().c_str(), gw.toString().c_str());
    return gw;
  }
  Serial.printf("[agent] '%s' not found, using fallback %s\n",
                host, AGENT_IP.toString().c_str());
#endif
  return AGENT_IP;
}

/**
 * Connect as fast as the radio allows:
 *   - persistent(false)  : no flash write on every connect
 *   - setSleep(false)    : no power-save latency (matters for a 100 Hz link)
 *   - static IP (opt)    : skips DHCP, typically 1-2 s quicker
 *   - autoReconnect      : the stack re-joins on its own if the AP blinks
 * Target: associated in well under 30 s even on a busy 2.4 GHz band.
 */
// Upper bound on the WIFI_NETWORKS table, so the "already tried" flags live on
// the stack instead of the heap. Raise it if you ever list more than this.
#define WIFI_MAX_NETWORKS 8
// How long to wait on ONE network before moving to the next candidate. The
// overall budget is still the timeout_ms argument.
#define WIFI_ATTEMPT_TIMEOUT_MS 8000

// Which entry of WIFI_NETWORKS actually associated. set_microros_wifi_transports
// must be handed these, not the first row of the table.
// Point at the store, not the header: these are what micro-ROS is handed,
// and after the first boot the header is not what the robot is using.
static const char *wifi_ssid_active = "";
static const char *wifi_pass_active = "";

static bool wifiConnect(uint32_t timeout_ms = 30000) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(WIFI_HOSTNAME);

#if USE_STATIC_IP
  if (!WiFi.config(STATIC_IP, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS)) {
    Serial.println("[WiFi] static IP rejected, falling back to DHCP");
  }
#endif

  const uint32_t t0 = millis();

  // Scan once, then try the listed networks strongest first. Everything below
  // works on fixed buffers: the only allocation is inside the SDK's scan, and
  // that is released with scanDelete() before this function returns. Nothing
  // here runs again after setup().
  int16_t found = WiFi.scanNetworks(false /*async*/, false /*show hidden*/);
  if (found < 0) found = 0;
  Serial.printf("[WiFi] scan found %d network(s), %u configured\n",
                (int)found, (unsigned)WIFI_NETWORK_COUNT);
  // List what is actually in range. Without this, a board that cannot join
  // tells you nothing about what it COULD join, and setup becomes guesswork.
  for (int16_t s_i = 0; s_i < found && s_i < 20; ++s_i) {
    bool known = false;
    for (uint8_t k = 0; k < wifi_store.count(); ++k) {
      if (WiFi.SSID(s_i) == wifi_store.ssid(k)) { known = true; break; }
    }
    Serial.printf("       %-32s %4ld dBm  %s\n",
                  WiFi.SSID(s_i).c_str(), (long)WiFi.RSSI(s_i),
                  known ? "<- configured" : "");
  }

  bool tried[WIFI_MAX_NETWORKS] = { false };
  const size_t table_n = (WIFI_NETWORK_COUNT < WIFI_MAX_NETWORKS)
                       ? WIFI_NETWORK_COUNT : WIFI_MAX_NETWORKS;

  for (size_t attempt = 0; attempt < table_n; ++attempt) {
    // pick the strongest configured network that is on the air and untried
    int best_idx = -1;
    int32_t best_rssi = -127;
    for (int16_t s = 0; s < found; ++s) {
      for (size_t k = 0; k < table_n; ++k) {
        if (tried[k]) continue;
        if (WiFi.SSID(s) != wifi_store.ssid((uint8_t)k)) continue;
        if (WiFi.RSSI(s) > best_rssi) { best_rssi = WiFi.RSSI(s); best_idx = (int)k; }
      }
    }
    if (best_idx < 0) break;                 // nothing configured is in range
    tried[best_idx] = true;

    // From the STORE, not the header. After the first boot the header is not
    // what this robot is using, and reading it here would try a network the
    // person may have deliberately removed.
    const char *try_ssid = wifi_store.ssid((uint8_t)best_idx);
    const char *try_pass = wifi_store.pass((uint8_t)best_idx);
    Serial.printf("[WiFi] trying \"%s\" (%ld dBm)\n", try_ssid, (long)best_rssi);
    WiFi.begin(try_ssid, try_pass);

    /* Turn the radio down, because it is the best suspect for the BNO085
     * resetting itself.
     *
     * The evidence is a controlled comparison that fell out of debugging
     * something else. The standalone IMU tool - same board, same sensor, same
     * I2C bus, NO Wi-Fi - ran for minutes with ZERO resets and a clean 3/3
     * accuracy. Every Wi-Fi firmware on the same hardware resets the sensor
     * repeatedly: 34 times in five minutes at best, 34 times in 68 seconds at
     * worst. The variable that differs is the radio.
     *
     * An ESP32 at full power pulls a few hundred milliamps in transmit bursts.
     * On USB alone, with no battery to hold the rail up, that is a plausible
     * brown-out of a sensor sharing 3V3 - and the BNO085's response to a
     * brown-out is exactly what is seen: it resets and loses its calibration.
     *
     * 13 dBm rather than the default 20 is about a fifth of the transmit
     * current for roughly half the range, which this robot can afford: it works
     * within a house or a car park, and the link is checked continuously
     * anyway. If this proves to be the cause, the real fix is a battery or a
     * capacitor on the sensor's supply, and this can go back up.
     */
    applyTxPower();

    const uint32_t attempt_start = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - attempt_start < WIFI_ATTEMPT_TIMEOUT_MS &&
           millis() - t0 < timeout_ms) {
      delay(50);
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifi_ssid_active = try_ssid;           // micro-ROS transport needs these
      wifi_pass_active = try_pass;
      WiFi.scanDelete();
      Serial.printf("[WiFi] connected to \"%s\" in %lu ms, ip %s, rssi %d dBm\n",
                    try_ssid, (unsigned long)(millis() - t0),
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      Serial.printf("[WiFi] this robot answers as %s.local\n", WIFI_HOSTNAME);
      return true;
    }
    WiFi.disconnect(false, true);
    if (millis() - t0 >= timeout_ms) break;
  }

  WiFi.scanDelete();
  Serial.println("[WiFi] no configured network could be joined");
  return false;
}
#endif

#if defined(MICROROS_TRANSPORT_WIFI)
OtaUpdater ota;

// The USB settings console. Present whatever the radio is doing, because the
// radio is the thing you most often need to reconfigure - and a board that
// knows none of the networks around it cannot be reached over one.
static bool consoleRobotIsMoving() {
  return fabsf(drive.targetSpeed()) > 1e-3f || fabsf(drive.targetYawRate()) > 1e-3f;
}

static void consolePrintInfo(void *) {
  Serial.printf("  firmware      v%d.%d\n", FIRMWARE_VERSION_MAJOR,
                FIRMWARE_VERSION_MINOR);
  Serial.printf("  uptime        %lu s\n", (unsigned long)(millis() / 1000));
  Serial.printf("  free heap     %u B\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("  wifi          %s, ip %s, rssi %d dBm\n",
                WiFi.status() == WL_CONNECTED ? wifi_ssid_active : "not joined",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  Serial.printf("  agent         %s:%u  (%s)\n", agent_ip.toString().c_str(),
                (unsigned)AGENT_PORT,
                conn_state == AGENT_CONNECTED ? "connected" : "not connected");
  Serial.printf("  gps           %lu byte(s) seen, %s\n",
                (unsigned long)gps.bytesSeen(),
                gps.fixQuality() > 0 ? "has a fix" : "no fix");
  Serial.printf("  contactor     %s\n", safety.relayClosed() ? "closed" : "open");
  // The BNO085 magnetometer only counts as usable at accuracy >= 2, and
  // nothing reported the number - so "imu_magnetometer_missing" gave no clue
  // whether the sensor was stuck at 0, climbing, or simply not reporting.
  Serial.printf("  imu mag       accuracy %u/3 (needs >=2), %s, last report %lu ms ago\n",
                (unsigned)imu.accuracy(),
                imu.magYawOk() ? "USABLE" : "not usable",
                (unsigned long)imu.magYawAgeMs());
  // Four reports share one I2C bus, and if it is saturated the slowest one
  // starves first while the fast ones look perfectly healthy - which reads on
  // the dashboard as a broken magnetometer and is nothing of the kind. Asked
  // against measured, per report, so the difference is visible rather than
  // inferred. This is the number to read after changing the bus clock or a
  // report rate: change ONE of them, then compare these.
  Serial.printf("  imu reports   rv %.1f/%.1f Hz  game %.1f/%.1f Hz  gyro %.1f Hz  "
                "accel %.1f Hz   (over %lu ms)\n",
                imu.rvHz(), imu.rvWantHz(), imu.gameHz(), imu.gameHzWant(),
                imu.gyroHz(), imu.accHz(), (unsigned long)imu.rateWindowMs());
}

static void consoleShowAgent(void *) {
  Serial.printf("  agent target  %s\n", agent_ip.toString().c_str());
  Serial.printf("  set by hand   %s%s\n",
                agent_override[0] ? agent_override : "(no - discovering)",
                agent_override_failed ? "  [never answered - being ignored]" : "");
  Serial.printf("  worked last   %s\n",
                (uint32_t)agent_last_good ? agent_last_good.toString().c_str() : "(nothing yet)");
  Serial.printf("  hostname      %s  (looked up by mDNS then DNS)\n", AGENT_HOSTNAME);
  Serial.printf("  compiled-in   %s\n", AGENT_IP.toString().c_str());
  Serial.printf("  link          %s\n",
                conn_state == AGENT_CONNECTED ? "connected" : "not connected");
}

static bool consoleSetAgent(void *, const char *host) {
  agentSetOverride(host);
  agent_ip = resolveAgentIp();

  // Re-point the TRANSPORT, not just the variable. micro-ROS is handed the
  // address once at start-up, so changing agent_ip alone leaves it talking to
  // wherever it was originally told - the command would report success and
  // change nothing observable, which is worse than refusing.
  //
  // Then drop back to WAITING_AGENT so the session is rebuilt against the new
  // address immediately, rather than after the next ping timeout.
  if (conn_state == AGENT_CONNECTED) {
    motor_left.disable();
    motor_right.disable();
    drive.stop();
    destroyEntities();
  }
  set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active,
                               agent_ip, AGENT_PORT);
  conn_state = WAITING_AGENT;
  create_failures = 0;
  return true;
}

static void consoleShowCompass(void *) {
  Serial.printf("  heading       %.1f deg\n", compass.headingDeg());
  Serial.printf("  field         %.1f  (reference %.1f from the calibration)\n",
                compass.field(), compass.calibration().field_norm);
  Serial.printf("  calibrated    %s\n", compass.calibration().valid ? "yes" : "no");
  Serial.printf("  disturbed     %s\n", compass.disturbed() ? "YES - readings rejected" : "no");
  Serial.printf("  fresh         %s\n", compass.ok() ? "yes" : "no");
  Serial.println("  note: the disturbance check is only active once calibrated, so a bad");
  Serial.println("        calibration can reject every reading - 'compass clear' undoes it");
}

static void consoleClearCompass(void *) { clearCompassCalibration(); }

static SettingsConsole<HardwareSerial, ParamRegistry<PARAM_COUNT>, WifiStore> console;

/**
 * May an update start right now?
 *
 * The only thing that must be true beforehand is that the robot is STANDING
 * STILL. It does not have to be e-stopped first: the firmware owns the
 * contactor, so it drops the motor rails itself in otaOnBegin() below. An
 * earlier version demanded the contactor already be open, which meant every
 * wireless flash began with a walk over to the robot to press the mushroom -
 * defeating the point of flashing wirelessly at all.
 *
 * Motion is the line that is not crossed automatically. Cutting power to a
 * machine that is moving, remotely, because someone somewhere started an
 * upload, is not a decision to take quietly - it stops being an update and
 * becomes an unannounced e-stop. So a moving robot simply does not answer.
 */
static bool otaIsSafe() {
  // Translational speed alone is not "stopped". A differential-drive robot
  // rotating on the spot has a centre-of-body speed of about zero while both
  // wheels are driving, so an update could start in the middle of a turn - and
  // beginning one cuts motor power at once. Ask every question instead.
  // "Moving" has to mean what the robot is DOING, not what a noisy estimate
  // says about it. The fused speed wanders at rest - measured at 0.127 m/s with
  // the wheels physically still - so a bare 0.05 threshold on it refuses
  // updates on a stationary robot. It did exactly that here and blocked its own
  // flash, with "[OTA] not accepting updates while the robot is moving" on a
  // robot that had not moved in ten minutes.
  //
  // The commanded state is exact and cannot drift, so it goes first; the
  // measured speed stays as a backstop for a robot being pushed or rolling
  // downhill, at a threshold well clear of the estimator's own noise.
  if (drive.pwmLeft() != 0 || drive.pwmRight() != 0) return false;
  if (drive.turningInPlace())                        return false;
  if (fabsf(drive.targetSpeed()) > 1e-3f)            return false;
  if (fabsf(drive.targetYawRate()) > 1e-3f)          return false;
  if (fabsf(estimator.speed()) >= 0.20f)             return false;
  return true;
}

/**
 * Make the robot safe, then let the flash proceed.
 *
 * Ordered deliberately: commands first so nothing re-applies a duty cycle, then
 * the motors, then the contactor - which removes the power those motors would
 * have used even if everything above somehow failed.
 */
/**
 * Fetch new firmware from the host over HTTP, on command.
 *
 * WHY PULL AND NOT PUSH
 *
 * ArduinoOTA is a PUSH: the PC opens a connection to the board. That needs the
 * board to be directly addressable, and on this robot it often is not - the
 * board sits on a Windows hotspot whose ICS layer rewrites its address, so the
 * PC has nothing to connect to. Measured: the agent and the board exchange UDP
 * happily in both directions on the port the BOARD opened, while a fresh
 * inbound connection to the board is simply unroutable.
 *
 * So the board fetches instead. An outbound HTTP GET traverses NAT, ICS,
 * hotspots and guest Wi-Fi alike - the same path the micro-ROS link already
 * proves works. The only requirement is that the board can reach the host,
 * which it must be able to do anyway or there would be no telemetry.
 *
 * Same safety rules as the push path: refuse while moving, and drop the motor
 * rails before a single byte is written.
 */
static void startPullUpdate() {
  if (!otaIsSafe()) {
    Serial.println("[update] refused: the robot is moving. Stop it and try again.");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[update] refused: no network");
    return;
  }
  char url[96];
  snprintf(url, sizeof(url), "http://%s:%u/firmware.bin",
           agent_ip.toString().c_str(), (unsigned)PULL_UPDATE_PORT);
  Serial.printf("[update] fetching %s\n", url);

  otaOnBegin();                     // motors off, contactor open, before any write

  WiFiClient client;
  httpUpdate.rebootOnUpdate(true);
  // The board keeps its own version, so a server offering the same build is a
  // no-op rather than a pointless reflash.
  t_httpUpdate_return r = httpUpdate.update(client, url);
  switch (r) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[update] FAILED (%d) %s\n", httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[update] the server has nothing newer");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[update] done, restarting");
      break;
  }
}

static void otaOnBegin() {
  drive.stop();
  motor_left.stop();
  motor_right.stop();
  safety.beginUpdateLockout();      // rails dead for the rest of this boot
  Serial.println("[OTA] motor rails cut for the update");
}
#endif  // MICROROS_TRANSPORT_WIFI

void setup() {
  Serial.begin(115200);
  delay(50);

  // motors first: make sure nothing spins while the rest boots
  motor_left.begin(MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, LEDC_CH_MOTOR_A,
                   PWM_FREQUENCY, PWM_BITS, MOTOR_A_INVERT, MOTOR_BRAKE_ON_STOP);
  motor_right.begin(MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, LEDC_CH_MOTOR_B,
                    PWM_FREQUENCY, PWM_BITS, MOTOR_B_INVERT, MOTOR_BRAKE_ON_STOP);
  motor_left.disable();
  motor_right.disable();

  safety.begin();

  pinMode(INA226_ALERT_MOTOR_A, INPUT);
  pinMode(INA226_ALERT_MOTOR_B, INPUT);
  pinMode(INA226_ALERT_PROCESSOR, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  // Fail an I2C transaction FAST rather than blocking the control loop on it.
  //
  // Every sensor read happens inside controlCallback(), so a bus that stops
  // answering stalls the 100 Hz loop for as long as the driver is willing to
  // wait. Measured during instant forward/backward reversals at full speed -
  // the noisiest thing this robot does, with the rails swinging 2.3 A - the
  // loop stalled for 1.07 SECONDS and telemetry went with it. At 400 kHz a
  // real transaction takes a few hundred microseconds, so 20 ms is enormous
  // headroom for a healthy bus and a short wait for a disturbed one: the read
  // is abandoned, the sensor is marked stale by its own driver, and the loop
  // keeps time.
  // Start-up gets the generous timeout; the strict loop value is applied at the
  // very end of setup(), once every driver has had its chance to initialise.
  Wire.setTimeOut(I2C_TIMEOUT_INIT_MS);

  // Which devices are actually on the bus, by address. Worth printing at every
  // boot: "the sensor initialised" and "the sensor is answering" are different
  // claims, and only this distinguishes a wiring fault from a driver one.
  {
    Serial.print("[I2C] devices:");
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); ++found; }
    }
    if (!found) Serial.print(" NONE - check SDA/SCL and power");
    Serial.println();
  }

  // MODE_BOTH gives the game rotation vector (smooth, relative) *and* the
  // magnetometer-referenced one, so the fallback chain has a spare absolute source
  // From IMU_REPORT_MODE, not HEADING_SOURCE. The two answer different
  // questions - which reports the sensor publishes, and which absolute
  // references the estimator may use - and deriving one from the other meant
  // neither could be chosen without disturbing the other. See config/imu.h.
  const ImuBno085::Mode imu_mode = (IMU_REPORT_MODE == 1) ? ImuBno085::MODE_MAG
                                 : (IMU_REPORT_MODE == 2) ? ImuBno085::MODE_BOTH
                                                          : ImuBno085::MODE_GAME;
  if (!imu.begin(&Wire, BNO085_I2C_ADDR, imu_mode)) {
    // Say WHICH failure this is. The address ACKing and the sensor working are
    // different claims, and for weeks they were reported as one line that could
    // mean either. The raw SHTP header separates them: a sensor that is talking
    // answers with a real packet length and channel, one that is not answers
    // 0xFF or nothing at all.
    uint8_t hdr[4];
    const bool read_ok = imu.probeHeader(hdr);
    Serial.printf("[IMU] BNO085 not found - raw SHTP header %s %02X %02X %02X %02X\n",
                  read_ok ? "read:" : "UNREADABLE:", hdr[0], hdr[1], hdr[2], hdr[3]);
  }
  imu.setStaleMs(IMU_STALE_MS);

  // Parameters before the drivers, so every begin() below sees the values
  // this robot was configured with rather than the compiled-in defaults.
  // The Wi-Fi list, seeded from the header only when NVS is empty. After that
  // the board's own list is the authority: a network added in the field must
  // not be undone by whatever the firmware was last built with.
#if defined(MICROROS_TRANSPORT_WIFI)
  wifi_prefs.begin("gpswifi", false);
  {
    WifiEntry seed[WIFI_STORE_MAX];
    uint8_t n = 0;
    for (size_t i = 0; i < WIFI_NETWORK_COUNT && n < WIFI_STORE_MAX; ++i) {
      if (!WIFI_NETWORKS[i].ssid || !WIFI_NETWORKS[i].ssid[0]) continue;
      strncpy(seed[n].ssid, WIFI_NETWORKS[i].ssid, WIFI_SSID_MAX - 1);
      seed[n].ssid[WIFI_SSID_MAX - 1] = '\0';
      strncpy(seed[n].pass, WIFI_NETWORKS[i].pass ? WIFI_NETWORKS[i].pass : "",
              WIFI_PASS_MAX - 1);
      seed[n].pass[WIFI_PASS_MAX - 1] = '\0';
      ++n;
    }
    wifi_store.begin(&wifi_prefs, seed, n);
    Serial.printf("[WiFi] %u network(s) known%s\n", (unsigned)wifi_store.count(),
                  wifi_store.seededFromHeader() ? ", seeded from the header" : "");
  }
#endif

  agentStoreLoad();
  param_store.begin("gpsparams", false);
  params.begin(PARAM_TABLE, PARAM_COUNT, &param_store);

  if (!power.begin(&Wire))
    Serial.println("[INA226] one or more sensors missing - see the rail readings");

#if USE_QMC5883L
  if (compass.begin(&Wire, QMC5883L_ADDR)) {
    compass.setDeclination(COMPASS_DECLINATION_DEG);
    compass.setMounting(COMPASS_MOUNT_OFFSET_DEG, COMPASS_INVERT);
    compass.setFieldTolerance(COMPASS_FIELD_TOL);
    loadCompassCalibration();
    Serial.println("[COMPASS] QMC5883L ready");
  } else {
    Serial.println("[COMPASS] QMC5883L not found - heading stays relative until the robot moves");
  }
#endif

  static const uint32_t gps_bauds[] = GPS_BAUD_CANDIDATES;
  gps.begin(&Serial2, PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD_DEFAULT, true,
            gps_bauds, sizeof(gps_bauds) / sizeof(gps_bauds[0]), GPS_BAUD_PROBE_MS);
  // Say which of the two failures this is. No bytes at all means power or
  // wiring - the receiver is not transmitting, and no baud will help. Bytes
  // but no sentences means it IS talking and the framing is wrong.
  Serial.printf("[GPS] listening at %lu baud, %lu raw byte(s) seen during probe%s\n",
                (unsigned long)gps.baud(), (unsigned long)gps.bytesSeen(),
                gps.bytesSeen() == 0
                  ? "  <- NOTHING ON THE WIRE: check 3V3 to the receiver and"
                    " that GPS TX goes to IO16"
                  : "");
  if (gps.probeCount() && gps.bytesSeen() && !gps.fresh(5000)) {
    // The receiver is transmitting but nothing parsed. Show the byte count
    // per baud: framing garbage trickles in at every wrong rate, so the one
    // that stands out is the receiver's real speed.
    Serial.println("[GPS] no NMEA parsed - bytes per candidate baud:");
    for (uint8_t i = 0; i < gps.probeCount(); ++i) {
      Serial.printf("        %7lu baud : %5lu byte(s)%s\n",
                    (unsigned long)gps.probeBaud(i), (unsigned long)gps.probeBytes(i),
                    gps.probeBytes(i) > 200 ? "   <- looks like the real one" : "");
    }
  }

  StateEstimator::Config ecfg;
  ecfg.heading_source   = HEADING_SOURCE;
  ecfg.align_min_mps    = HEADING_ALIGN_MIN_MPS;
  ecfg.align_tau_s      = HEADING_ALIGN_TAU_S;
  ecfg.align_max_dps    = HEADING_ALIGN_MAX_DPS;
  ecfg.gps_tau_s        = SPEED_EST_GPS_TAU_S;
  ecfg.model_tau_s      = SPEED_EST_MODEL_TAU_S;
  ecfg.acc_deadband     = SPEED_EST_ACC_DEADBAND;
  ecfg.max_speed_mps    = SPEED_EST_MAX_MPS;
  ecfg.track_m          = WHEEL_TRACK_M;
  ecfg.wheel_diameter_m = WHEEL_DIAMETER_M;
  ecfg.mag_tau_s        = HEADING_MAG_TAU_S;
  ecfg.mag_still_mps    = HEADING_MAG_STILL_MPS;
  ecfg.mag_enable       = USE_QMC5883L && compass.present();
  estimator.begin(ecfg);
  drive.begin();
  drive.setRamp(MOTOR_ACCEL_RAMP_S, MOTOR_DECEL_RAMP_S);

  MotorModel::Config mcfg;
  mcfg.ke_v_per_rpm    = MOTOR_KE_V_PER_RPM;
  mcfg.resistance_ohm  = MOTOR_RESISTANCE_OHM;
  mcfg.wheel_diameter_m = WHEEL_DIAMETER_M;
  mcfg.min_duty        = MOTOR_MIN_DUTY_FOR_EST;
  mcfg.stall_current_a = MOTOR_STALL_CURRENT_A;
  mcfg.open_current_a  = MOTOR_OPEN_CURRENT_A;

  mcfg.diode_vf_v  = MOTOR_A_DIODE_VF_V;      // each motor has its own diode
  mcfg.diode_r_ohm = MOTOR_A_DIODE_R_OHM;
  model_left.begin(mcfg);

  mcfg.diode_vf_v  = MOTOR_B_DIODE_VF_V;
  mcfg.diode_r_ohm = MOTOR_B_DIODE_R_OHM;
  model_right.begin(mcfg);

  // Every driver above was configured from the compiled-in defaults. Now
  // overwrite them with whatever this robot was actually configured with -
  // otherwise a measured diode drop is stored, shown in the UI, and ignored.
  applyParams();

  initMessages();

#if defined(MICROROS_TRANSPORT_WIFI)
  if (!wifiConnect()) {
    // Do NOT reboot. Rebooting on Wi-Fi failure makes a board that knows no
    // network in range loop forever, and a looping board cannot be configured
    // - the serial console never stays up long enough to talk to it. Proven on
    // the bench: with only an out-of-range SSID configured, the board restarted
    // every few seconds and was unreachable by any means.
    //
    // Stay up instead. The console keeps working, the scan result above says
    // what IS in range, and the retry below picks the network up the moment it
    // appears. Nothing can move: with no agent the safety manager holds
    // STOP_AGENT_LOST, so the motors stay disabled the whole time.
    Serial.println("[WiFi] no network joined - staying up so this board can");
    Serial.println("       still be reached over USB. Retrying in the background.");
    Serial.println("       Add a network to firmware/config/network_secrets.h,");
    Serial.println("       or pick one from the scan list above.");
  }
  agent_ip = resolveAgentIp();
  beaconBegin();      // start listening before the first ping, not after
  set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active, agent_ip, AGENT_PORT);

  // Wireless flashing, so a board bolted into the robot never needs a cable
  // again. Only useful once there is a network, hence here rather than earlier.
  if (WiFi.status() == WL_CONNECTED) {
    ota.begin(WIFI_HOSTNAME, otaIsSafe, otaOnBegin);
  }

  {
    // Available with or without a network, and started last so `info` can
    // report on everything set up above it.
    ConsoleHooks hooks;
    hooks.isMoving = consoleRobotIsMoving;
    hooks.printInfo = consolePrintInfo;
    hooks.showAgent = consoleShowAgent;
    hooks.setAgent  = consoleSetAgent;
    hooks.showCompass = consoleShowCompass;
    hooks.clearCompass = consoleClearCompass;
    console.begin(&Serial, &params, &wifi_store, hooks);
    Serial.println("[console] ready - type help for the settings commands");
  }
#else
  set_microros_serial_transports(Serial);
#endif

  // Every driver has now had its turn, so hand the bus over to the control
  // loop's rules: fail fast, and let each driver mark itself stale rather than
  // stalling the loop waiting for a sensor that is not going to answer.
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  conn_state = WAITING_AGENT;
}

//================================= loop ====================================//
void loop() {
#if defined(MICROROS_TRANSPORT_WIFI)
  // Serviced first and every pass: an update must be able to land even when the
  // agent is unreachable, which is exactly when a board most needs reflashing.
#if WIFI_RSSI_ENABLE
  // Once a second, outside controlCallback. Not free even here: loop() also
  // spins the executor that fires the control timer, so a wait on the Wi-Fi
  // driver in this spot can still make the next control tick late - it just
  // does not show in loop_us, which only times the callback itself.
  // tools/loop_probe.py measures the lateness directly.
  EXECUTE_EVERY_N_MS(1000, {
    wifi_rssi_dbm = (WiFi.status() == WL_CONNECTED) ? (float)WiFi.RSSI() : 0.0f;
  });
#endif

  ota.handle();
  // Bounded internally, so a fast paste cannot starve the control loop.
  console.poll();
#endif

  // Keep trying to join a network if we have none. Backed off so a board with
  // nothing in range does not spend its whole life scanning.
#if defined(MICROROS_TRANSPORT_WIFI)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t next_wifi_try_ms = 0;
    const uint32_t now_wifi = millis();
    if (now_wifi >= next_wifi_try_ms) {
      next_wifi_try_ms = now_wifi + 30000;      // every 30 s, not continuously
      Serial.println("[WiFi] retrying...");
      if (wifiConnect(15000)) {
        // A different network means every previous conclusion about where the
        // agent lives is worthless. Start again with a clean slate rather than
        // carrying a grudge from the last one.
        agent_dead = IPAddress((uint32_t)0);
        agent_override_failed = false;
        agent_override_rounds = 0;
        beacon_open = false;
        beaconBegin();
        agent_ip = resolveAgentIp();
        set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active,
                                     agent_ip, AGENT_PORT);
        Serial.println("[WiFi] back on the air");
      }
    }
  }
#endif

#if defined(MICROROS_TRANSPORT_WIFI)
  // While there is no session, take the address the host is announcing. Checked
  // every pass and not on a timer: the whole point is that a network change is
  // recovered from in seconds, and parsePacket() on an empty socket is cheap.
  if (conn_state != AGENT_CONNECTED && WiFi.status() == WL_CONNECTED) {
    beaconBegin();
    const IPAddress heard = beaconPoll();
    if ((uint32_t)heard != 0 && heard != agent_ip) {
      Serial.printf("[beacon] agent announced itself at %s - switching to it\n",
                    heard.toString().c_str());
      agent_ip = heard;
      beacon_from = heard;
      agent_dead = IPAddress((uint32_t)0);   // a fresh address deserves a try
      set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active,
                                   agent_ip, AGENT_PORT);
    }
  }
#endif

  switch (conn_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, {
        conn_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;
        static uint32_t waiting_agent_sec = 0;
        if (conn_state == WAITING_AGENT) {
          if (++waiting_agent_sec >= 60) {
            Serial.println("[agent] 60 seconds without agent ping reply - restarting ESP32 to clear stale IP/session");
            Serial.flush();
            delay(50);
            ESP.restart();
          }
        } else {
          waiting_agent_sec = 0;
        }
#if defined(MICROROS_TRANSPORT_WIFI)
        // after a few failed pings, look the agent up by name again: the PC may
        // have come back on a different IP address
        if (conn_state == WAITING_AGENT && ++agent_retries >= 5) {
          agent_retries = 0;
          // Three rounds of five failed pings - most of a minute - is enough to
          // conclude that a hand-set address is not where the agent is. Give up
          // on it and let discovery run, loudly, so the reason is on the record.
          if (!agent_override_failed && agent_override[0] &&
              ++agent_override_rounds >= 3) {
            agent_override_failed = true;
            agent_dead = agent_ip;      // and do not let discovery pick it again
            Serial.printf("[agent] \"%s\" was set by hand and has never answered - "
                          "ignoring it and discovering instead\n", agent_override);
          }
          if (WiFi.status() == WL_CONNECTED) {
            IPAddress fresh = resolveAgentIp();
            if (fresh != agent_ip) {
              agent_ip = fresh;
              set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active, agent_ip, AGENT_PORT);
              Serial.printf("[agent] now trying %s\n", agent_ip.toString().c_str());
            }
          }
        }
#endif
      });
      break;

    case AGENT_AVAILABLE:
      conn_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      // Only a real session counts - a ping proves reachability, not that a
      // session can be built, and remembering an address that merely answers
      // pings would make the next boot slower rather than faster.
      if (conn_state == AGENT_CONNECTED) {
        agentRememberGood(agent_ip);
        agent_override_rounds = 0;      // it works; trust the setting again
        agent_dead = IPAddress((uint32_t)0);
      }
      if (conn_state == WAITING_AGENT) {
        destroyEntities();
        // SELF-HEAL. The board can otherwise sit here forever: the ping keeps
        // succeeding, so it advances to AGENT_AVAILABLE every second, and
        // createEntities() keeps failing, so it falls straight back. Seen on
        // hardware for over five minutes at a time - the agent answering every
        // ping while its log recorded not one new session - and the only way
        // out was a power cycle by hand.
        //
        // A reboot is safe here by construction: with no session there is no
        // agent, STOP_AGENT_LOST is therefore set, and the motors are already
        // stopped. A board that reboots itself and comes back is strictly
        // better than one that pings politely and never returns.
        if (++create_failures >= AGENT_CREATE_FAIL_LIMIT) {
          Serial.printf("[agent] %u consecutive session failures - restarting so the "
                        "board comes back on its own\n", (unsigned)create_failures);
          Serial.flush();
          delay(50);
          ESP.restart();
        }
      } else {
        create_failures = 0;
      }
      break;

    case AGENT_CONNECTED: {
      // Ping the agent only when it has gone QUIET.
      //
      // rmw_uros_ping_agent() blocks loop() until the pong arrives - up to 300 ms
      // per attempt, three attempts - and loop() is also what spins the executor
      // that fires the 100 Hz control timer. So every ping made the next control
      // tick late by one Wi-Fi round trip, and a slow round trip made it late by
      // tens of milliseconds, all without showing in loop_us, which only times
      // the callback itself. tools/loop_probe.py saw it as roughly 2% of IMU
      // publishes arriving late, p99 around 100 ms, with the signal reading
      // compiled out entirely.
      //
      // A heartbeat or a drive command arriving within the last period proves the
      // session is alive more convincingly than a ping could, because it is real
      // traffic that came through the agent. So the ping only runs on silence.
      // Detection of a lost agent is unchanged: silence for one period brings the
      // ping straight back, and the safety chain already stops the motors on a
      // stale heartbeat without waiting for it.
      const uint32_t now_rx = millis();
      const bool heard_recently =
          (last_heartbeat_ms != 0 && now_rx - last_heartbeat_ms < AGENT_PING_PERIOD_MS) ||
          (last_cmd_ms       != 0 && now_rx - last_cmd_ms       < AGENT_PING_PERIOD_MS);
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS, {
        if (!heard_recently)
          conn_state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      });
      if (conn_state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      break;
    }

    case AGENT_DISCONNECTED:
      motor_left.disable();
      motor_right.disable();
      drive.stop();
      have_heartbeat = false;
      destroyEntities();
      conn_state = WAITING_AGENT;
      break;
  }

  // Motors must also stop while the agent is gone (the control timer is not running then)
  if (conn_state != AGENT_CONNECTED) {
    SafetyManager::Inputs si;
    si.sw_estop         = sw_estop;
    si.heartbeat_age_ms = 0xFFFFFFFF;
    si.cmd_age_ms       = 0xFFFFFFFF;
    si.agent_ok         = false;
    si.battery_valid    = false;
    si.motion_requested = false;
    safety.update(si);
    motor_left.disable();
    motor_right.disable();
  }
}
