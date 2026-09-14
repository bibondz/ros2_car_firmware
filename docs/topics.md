# Interface reference — ROS topics and web API

All ROS topics live under `/gps_localize`. `ROS_DOMAIN_ID = 10` (firmware
`config/network.h`, launch argument `domain_id`).

## Published by the firmware

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/gps_localize/odom` | `nav_msgs/Odometry` | 10 Hz | IMU+GPS dead reckoning, ENU frame `odom` → `base_link`. Silent until T186: at 724 bytes it is the one firmware message larger than the old 512-byte transport MTU, and a best-effort stream cannot fragment |
| `/gps_localize/imu/data` | `sensor_msgs/Imu` | 20 Hz | BNO085 quaternion, gyro, gravity-free accel |
| `/gps_localize/gps/fix` | `sensor_msgs/NavSatFix` | ≤5 Hz | published when a new solution arrives |
| `/gps_localize/telemetry` | `gps_localize_msgs/Telemetry` | 10 Hz | 64 named fields, listed below |

## Split out by `telemetry_split` — subscribe to these, not to `/telemetry`

`/telemetry` is the board's wire format: one compact frame, all `float32`, no
timestamp. `telemetry_split` turns each frame into six properly typed topics, so
nothing has to know the layout of a 64-field message to read a battery voltage.

| Topic | Type | Carries |
|---|---|---|
| `/gps_localize/power/battery` | `sensor_msgs/BatteryState` | pack volts, amps, charge estimate |
| `/gps_localize/power/rails` | `gps_localize_msgs/RailStatus` | all four INA226 rails |
| `/gps_localize/motors` | `gps_localize_msgs/MotorStatus` | duty, wheel and back-EMF speeds, faults |
| `/gps_localize/estimator` | `gps_localize_msgs/EstimatorStatus` | fused heading and speed, position, and their sources |
| `/gps_localize/safety/status` | `gps_localize_msgs/SafetyStatus` | run state, stop flags, contactor, heap |
| `/gps_localize/gps/status` | `gps_localize_msgs/GpsStatus` | fix quality, satellites, HDOP, course |

## How to reach the web UI, from the command line

    ros2 topic echo --once --field qr_ansi /gps_localize/web/connect

`/gps_localize/web/connect` (`gps_localize_msgs/WebConnect`) is latched, so the
answer is there the moment you ask, and republished when the machine changes
address. `--field url` gives just the address.

Or, without ROS on the path, the same thing from the project directory:

    python3 tools/robot_qr.py

Two details in that command are load-bearing, and both produce a code that
still looks exactly like a QR when they are wrong, which is the dangerous part.

**Use `--field`.** Plain `ros2 topic echo` prints the QR as one escaped line,
truncated after the first hundred or so characters — a `"  █▀▀▀▀▀█ ▄ ...`
fragment rather than a code. Untruncated it is no better: it puts a blank line
between every row, which doubles the QR's row spacing.

**Use `qr_ansi`, not `qr_ascii`.** The two carry the same address and differ
only in how they survive the terminal:

| Field | Drawn as | Scans on |
|---|---|---|
| `qr_ansi` | block cells with the colours stated in the output | any terminal theme |
| `qr_ascii` | half-block characters in the terminal's own text colour | a **light** background only |

On a dark theme `qr_ascii` comes out inverted, and an inverted QR is refused
outright by zbar and by many phone cameras. `qr_ascii` is still published,
because colour cannot go everywhere — use it for a log, a file, or anything to
be pasted somewhere else.

Checked by scanning, not by looking: `test_qr.py` rasterises both forms the way
a terminal grid paints them, at four cell shapes and on both themes, and reads
them back with `zbarimg`.

If the code comes out wrapped, the window is too narrow — the code needs about
70 columns. `tools/robot_qr.py` measures the window and says so rather than
printing a QR that cannot work.

These carry real types and named constants, so `msg.fix_quality ==
GpsStatus.FIX_RTK_FIXED` replaces remembering that 4 means RTK. They are also
stamped with the host clock, which the board cannot do — it has no synchronised
time, so anything it stamped would be milliseconds since boot in disguise.

**Why the split runs on the host rather than the ESP32.** Measured, not assumed:
raising micro-ROS's publisher limit from 8 to 12 costs 976 bytes of RAM, so the
board could easily hold the extra publishers. The cost is on the air. Six
messages per cycle carry six lots of XRCE framing instead of one, roughly 60%
more bytes over a Wi-Fi link that already drops frames when the robot moves
behind something. `sensor_msgs/BatteryState` alone, with its arrays and strings,
is heavier than the entire packed frame. One message out, exploded where there
is no such constraint.

An open contactor is reported as **not measured** — `BatteryState.voltage` is
NaN and `present` is false — never as 0 V. The pack sits behind the relay, and
reporting an unmeasurable pack as a flat one is what used to latch the robot
into e-stop three seconds after boot.

## Subscribed by the firmware

| Topic | Type | Meaning |
|---|---|---|
| `/gps_localize/cmd_move` | `geometry_msgs/Twist` | `linear.x` = speed [m/s], `angular.z` = **absolute heading** [compass deg]. `linear.y = 1` → rotate on the spot to that heading and stop |
| `/gps_localize/cmd_manual` | `geometry_msgs/Twist` | `linear.x` = speed [m/s], `angular.z` = yaw rate [deg/s, clockwise +] |
| `/gps_localize/cmd_vel` | `geometry_msgs/Twist` | **standard ROS teleop**: `linear.x` [m/s], `angular.z` [rad/s, counter-clockwise +]. Works with `teleop_twist_keyboard`, joysticks, any stock tool |
| `/gps_localize/heartbeat` | `std_msgs/Int32` | must arrive every < 600 ms or the motors stop |
| `/gps_localize/estop` | `std_msgs/Bool` | latched software emergency stop |
| `/gps_localize/config/pid` | `std_msgs/Float32MultiArray` | live gains, selector in `data[0]` |

Either command topic refreshes the command watchdog. A command stream that
stops for more than 500 ms while motion was requested stops the robot.

### `/gps_localize/config/pid` layout

| `data[0]` | Rest of the array |
|---|---|
| 0 | `kp, ki, kd, kf, tol, i_min, i_max` → heading PID |
| 1 | `kp, ki, kd, kf, tol, i_min, i_max` → speed PID |
| 2 | `heading_source, align_min_mps, align_tau_s, gps_tau_s, model_tau_s, track_m, mag_enable, mag_tau_s, mag_still_mps, declination_deg, mount_offset_deg` |
| 3 | *(nothing)* → reset odometry and the estimator |
| 4 | `1` start compass calibration · `0` finish and save it to the ESP32 flash · `2` clear it and go back to raw readings |
| 5 | `accel_ramp_s, decel_ramp_s` → motor acceleration ramp |
| 6 | `ke_v_per_rpm, resistance_ohm, stall_current_a, diodeA_vf, diodeA_r, diodeB_vf, diodeB_r` → electrical motor model |
| 7 | `heading_deg` → pin the heading by hand ("set start pose" on the map) and restart odometry |
| 8 | `index, value` → set one parameter, addressed by index; bounds and the moving guard are enforced on the board |
| 9 | *(nothing)* → every parameter back to its compiled-in default |
| 10 | *(nothing)* → pull a firmware image from the host over Wi-Fi and flash it |
| 11 | `1` start BNO085 calibration · `0` save it into the sensor's own flash · `2` clear it and restart the sensor. The firmware also saves it **by itself** the first time accuracy reaches 3/3, so a brown-out reset no longer loses the calibration |

## Telemetry array layout

Mirror of `firmware/src/main.cpp::publishTelemetry` and
`gps_localize/telemetry.py`. **Change both files together.**

| # | Field | Unit |
|---|---|---|
| 0 | `state` | 0 IDLE · 1 AUTO · 2 MANUAL · 3 ESTOP · 4 COMM_LOSS |
| 1 | `heading_deg` | compass deg (0 = north, CW) |
| 2 | `target_heading_deg` | compass deg |
| 3 | `heading_error_deg` | deg, −180..180 |
| 4 | `speed_mps` | m/s, fused |
| 5 | `target_speed_mps` | m/s |
| 6 / 7 | `wheel_rpm_left` / `wheel_rpm_right` | rpm (derived, no encoder) |
| 8 / 9 | `wheel_mps_left` / `wheel_mps_right` | m/s |
| 10 | `yaw_rate_dps` | deg/s, counter-clockwise + |
| 11 / 12 | `pwm_left` / `pwm_right` | −1023..1023 |
| 13 / 14 | `battery_v` / `battery_a` | V / A (processor INA226) |
| 15 / 16 | `motor_a_v` / `motor_a_a` | V / A |
| 17 / 18 | `motor_b_v` / `motor_b_a` | V / A |
| 19 | `gps_fix_quality` | 0 none, 1 GPS, 2 DGPS/SBAS |
| 20 | `gps_satellites` | count |
| 21 | `gps_hdop` | — |
| 22 | `gps_speed_mps` | m/s from the module |
| 23 | `gps_course_deg` | compass deg |
| 24 | `stop_flags` | bitfield, below |
| 25 | `free_heap_kb` | kB — watch for leaks |
| 26 | `loop_us` | control loop duration |
| 27 / 28 | `odom_x` / `odom_y` | m, ENU |
| 29 | `mag_heading_deg` | compass deg from the QMC5883L (tilt compensated) |
| 30 | `heading_ref` | 0 relative · 1 compass · 2 GPS course · 3 IMU magnetometer |
| 31 | `mag_status` | 0 no compass · 1 ok · 2 magnetic disturbance |
| 32 | `mag_calib_state` | 0 idle · 1 turning · 2 saved · 3 failed |
| 33 | `sensor_health` | bitfield of *missing* sensors, below |
| 34 | `speed_source` | 1 IMU+GPS · 2 GPS only · 3 IMU+model · 4 model only |
| 35 / 36 | `motor_rpm_left` / `motor_rpm_right` | wheel rpm from the INA226 volts+amps (back-EMF) |
| 37 | `motor_faults` | bit0 left stalled · bit1 right stalled · bit2 left open circuit · bit3 right open circuit |
| 38 / 39 | `fan_v` / `fan_a` | fan rail INA226 (schematic draft_5) |
| 40 | `esp32_rail_v` | ESP32 rail volts — health of the processor buck |
| 41 | `pack_valid` | 1 pack voltage is measurable · 0 the motor rail is unpowered, so the pack is *unknown*, not low |
| 42 | `pos_sigma_m` | 1-sigma position uncertainty from the EKF, m |
| 43 | `pos_anchored` | 1 the EKF has had at least one fix · 0 still open-loop dead reckoning |
| 44 | `firmware_version` | major×100 + minor; 0 from a board too old to send it |
| 45 | `reset_reason` | `esp_reset_reason()`: 9 brownout, 4 panic |
| 46 | `uptime_s` | seconds since boot |
| 47 | `esp32_temp_c` | die temperature, NaN when the chip has no usable sensor |
| 48 | `mag_calib_progress` | how far round the circle the compass sweep has got, 0..1; zero unless a sweep is running |
| 49 | `imu_accuracy` | the BNO085's own confidence in its magnetometer, 0 unreliable .. 3 high; below 2 the heading chain refuses it |
| 50 | `imu_calib_state` | 0 idle · 1 learning · 2 saved to the sensor's flash · 3 refused |
| 51 | `imu_calib_result` | what the sensor said: 0 SH2_OK, else a negative `sh2_err.h` code (−100 never started, −101 nothing at that address, −102 answers but is not running, −103 refused because the robot is moving, −104 opened but refused to turn any report on). Reports the DRIVER's own diagnosis whenever the driver never started — it used to report the last calibration *command*, which defaults to 0 and therefore read "ok" on a board whose IMU had never opened |
| 53 / 54 / 55 | `imu_acc_x` / `_y` / `_z` | linear acceleration from the BNO085, m/s². **Gravity already removed** — standing still reads ~0 on all three, not 9.81 on one. This is what the speed estimator consumes |
| 52 | `mag_turned_deg` | degrees actually turned during the compass sweep, from the **gyro**. Signed, so rocking back and forth cancels. Independent of the magnetometer being calibrated, which is the point |
| 56 | `imu_resets` | times the BNO085 has reset **itself** since boot. A rising count with a healthy accuracy is a power or bus problem, not a calibration one, and the two look identical on a dashboard |
| 61 | `heading_innov_deg` | how far the absolute heading reference sits from the IMU's own fused yaw, signed degrees. The compass and the BNO085 magnetometer both claim to know north and both are fooled by the same things; the gyroscope is fooled by none of them. Near zero means two independent sources agree, which is worth more than either alone |
| 62 | `heading_gated` | 1 while a reference is being **refused** for disagreeing with the IMU. Not a fault — the cross-check working. It clears when the disturbance passes, and a disagreement that outlasts `agree_recover_s` is accepted anyway, because a gate that can never give up would lock out a correct sensor for ever if the gyro really had drifted |
| 63 | `wifi_rssi_dbm` | Wi-Fi signal **at the robot**, dBm, negative. `0` means the board is not associated at all — the firmware sends a literal zero rather than the plausible-looking negative number the ESP32 API returns while disconnected. Better than −60 is strong, −70 works, past −80 the link starts dropping frames and the micro-ROS session is rebuilt |
| 57 / 58 / 59 / 60 | `imu_hz_rv` / `_game` / `_gyro` / `_accel` | the DELIVERED rate of each BNO085 report, Hz, against the 10 / 100 / 100 / 50 asked for. Four reports share one I²C bus and the slowest starves first, so a low `rv` beside healthy fast rates is a full bus rather than a bad magnetometer — they are indistinguishable without this, and one was mistaken for the other for hours |

### `sensor_health` bits

| Bit | Name | Meaning |
|---|---|---|
| 0 | `imu_missing` | no fresh BNO085 data — heading falls back to GPS course / compass |
| 1 | `gps_missing` | no fix — speed falls back to the motor model, navigation blocks |
| 2 | `compass_missing` | QMC5883L absent or disturbed — standing-still heading comes from the IMU magnetometer |
| 3 | `imu_magnetometer_missing` | BNO085 rotation vector unusable — accuracy below medium, **or** the report stale for more than ten of its own periods (it runs at 10 Hz, so one second). Those are different faults: a sensor reading 3/3 whose report is not arriving is a bus or driver problem, not a magnetometer that needs waving about. `imu_hz_rv` tells them apart |
| 4 | `heading_relative_only` | no absolute reference at all — heading is gyro-only, drive with care |

### `stop_flags` bits

| Bit | Name | Cause |
|---|---|---|
| 0 | `hardware_emergency` | E-EMER line (IO13, mushroom switch) |
| 1 | `esp_button` | on-board button SW4 |
| 2 | `software_estop` | web UI e-stop |
| 3 | `heartbeat_lost` | no ROS heartbeat (network) |
| 4 | `command_timeout` | command stream stalled |
| 5 | `agent_lost` | micro-ROS agent unreachable |
| 6 | `battery_low` | pack under 3.733 V/cell for 3 s — 11.199 V on 3S — latched |
| 7 | `startup_lockout` | first 1.5 s after boot |
| 8 | `battery_soft` | pack under 11.4 V - stop driving, not latched |
| 9 | `contactor_stuck` | commanded open, motor rails still live - a welded contact. The emergency stop cannot remove power in this state |

## ROS-side topics

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/gps_localize/odom/gps` | `nav_msgs/Odometry` | gps_odom → web_server |
| `/gps_localize/gps/datum` | `std_msgs/String` (JSON, latched) | gps_odom |
| `/gps_localize/pose/estimate` | `std_msgs/String` (JSON, latched) | gps_odom → web_server, waypoint_nav |
| `/gps_localize/nav/status` | `std_msgs/String` (JSON, latched) | waypoint_nav → web_server |
| `/gps_localize/nav/cmd` | `std_msgs/String` (JSON) | web_server, safety_watchdog → waypoint_nav |
| `/gps_localize/safety` | `std_msgs/String` (JSON, latched) | safety_watchdog → web_server |
| `/gps_localize/estop_request` | `std_msgs/Bool` | web_server → safety_watchdog |
| `/gps_localize/web/heartbeat` | `std_msgs/Int32` | web_server → safety_watchdog |
| `/gps_localize/config/updated` | `std_msgs/String` (JSON) | web_server → all nodes |
| `/gps_localize/log` | `std_msgs/String` (JSON) | any node → web_server |

`nav/cmd` actions: `start` (optional `index`), `pause`, `resume`, `stop`,
`next`, `goto` + `index`, **`set_pose` + `lat`/`lon`/`heading`**, **`goto_point` + `lat`/`lon`** (one-off destination
picked on the map, never stored in the waypoint list — the robot drives there,
stops and reports `FINISHED`), `reload`, `set_datum` (+ optional `lat`/`lon`).

## Web API (served by `web_server`, default port 8080)

| Method | Path | Body | Purpose |
|---|---|---|---|
| GET | `/api/stream` | — | Server-Sent Events: `telemetry` and `log` frames |
| GET | `/api/state` | — | one snapshot (same shape as an SSE telemetry frame) |
| GET | `/api/logs` | — | the ring buffer of recent events |
| POST | `/api/heartbeat` | `{}` | browser liveness — **the robot stops without it** |
| POST | `/api/manual` | `{forward, turn}` −1..1, or `{action:"stop"}` | manual driving |
| POST | `/api/nav` | `{action, …}` | mission control, same actions as `nav/cmd` |
| POST | `/api/nav` | `{action:"goto_point", lat, lon}` | drive to a point picked on the map (validated, not added to the list) |
| POST | `/api/estop` | `{stop: true|false}` | latch / release the emergency stop |
| GET | `/api/config` | `?group=nav` | one group, or all groups when omitted |
| POST | `/api/config` | `{group, data}` | save and apply |
| POST | `/api/config/reset` | `{group}` | back to shipped defaults |
| GET | `/api/waypoints` | — | current list |
| POST | `/api/waypoints` | `{waypoints:[{name,lat,lon}]}` | replace the list |
| POST | `/api/waypoints/add_here` | `{name?}` | append the robot's current position |
| POST | `/api/mag_calib` | `{start: true|false}` | compass calibration: start turning / finish and save |
| POST | `/api/turn` | `{deg: 90}` · `{heading: 270}` · `{compass: "N"}` | rotate on the spot and stop — works indoors, no GPS |
| POST | `/api/manual_limits` | `{speed_delta: 0.02}` · `{turn_delta: 5}` · `{speed_mps, turn_dps}` · `{save: true}` | the + / − buttons for manual speed and turn rate |
| POST | `/api/set_pose` | `{lat, lon, heading_deg}` | start pose dragged on the map (RViz 2D Pose Estimate style) |

Config groups (one YAML file each): `robot`, `nav`, `pid`, `motor`,
`estimator`, `safety`, `web`, `waypoints`.

## Handy commands

```bash
ros2 topic echo /gps_localize/telemetry --once
ros2 topic hz /gps_localize/imu/data                 # ~20 Hz
ros2 topic pub -1 /gps_localize/cmd_move geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 90.0}}'           # 0.2 m/s heading east
ros2 topic pub -1 /gps_localize/cmd_move geometry_msgs/msg/Twist \
  '{linear: {y: 1.0}, angular: {z: 180.0}}'          # turn on the spot to south
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/gps_localize/cmd_vel       # standard teleop
ros2 topic pub -1 /gps_localize/estop std_msgs/msg/Bool '{data: true}'
ros2 run gps_localize heading_check --seconds 30     # jitter test, < 2 deg
curl -s localhost:8080/api/state | python3 -m json.tool | head -40
curl -s -X POST localhost:8080/api/turn -d '{"compass":"N"}'
```

## `/gps_localize/pose/estimate`

Where the robot believes it is, and how much that is worth:

```json
{"lat": 13.7367, "lon": 100.5231, "source": "gps", "trusted": true, "heading_deg": 137.0}
{"lat": 13.7368, "lon": 100.5232, "source": "dead_reckoning", "trusted": false,
 "origin": {"lat": 13.7367, "lon": 100.5231}, "drift_m": 4.2, "age_s": 38.0, "heading_deg": 137.0}
{"lat": null, "lon": null, "source": "none", "trusted": false, "heading_deg": 0.0}
```

`dead_reckoning` appears only after an operator has set a start pose on the map
and while GPS is unavailable: it is that origin plus the firmware's IMU
odometry. It is deliberately **not** published as a `NavSatFix` — nothing in
the system should be able to mistake it for a satellite fix. The map draws the
robot hollow-orange in this state, and `waypoint_nav` ignores it unless
`nav.allow_dead_reckoning` is switched on for testing.
