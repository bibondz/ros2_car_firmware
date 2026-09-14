# Acceptance checklist

Work top to bottom. Each line has the exact command and what "pass" looks like.
Wheels stay off the ground until step 7.

## Build and host tests

```bash
python3 tools/qc.py                      # ☐ cross-file contracts hold (no build needed)
python3 -m pyflakes gps_localize_ws/src/gps_localize/gps_localize/*.py webapp/viewer_server.py

source /opt/ros/humble/setup.bash        # micro_ros_platformio needs $ROS_DISTRO
cd firmware
pio run -e wifi                          # ☐ compiles with no errors and no warnings
pio test -e native                       # ☐ 261 host tests pass
```

`tools/qc.py` is the cheap one to run first — it checks the things a compiler
cannot see: telemetry slot count firmware vs Python, config selectors, topic
names, that every endpoint the UI calls exists on both servers, that the CSS
hide utility still wins, that `config.h` includes exactly the headers on disk,
and that the Windows exe is newer than the web files it bundles.

`pio test -e native` runs on the PC, no board attached: PIDF (18 cases), state
estimator and sensor fallbacks (30), drive controller, turn-in-place and the
motor ramp (27), motor model and diode compensation (19), safety chain and the
NMEA parser (26). It needs a host compiler (`sudo apt install build-essential`).

## Link and sensors

| ☐ | Check | How | Pass |
|---|---|---|---|
| ☐ | Wi-Fi connect | `pio device monitor -b 115200` after reset | `[WiFi] connected in NNNN ms` under 30 000 ms. Set `USE_STATIC_IP 1` in `config/network.h` to shave 1–2 s |
| ☐ | Agent link | `ros2 node list` | `/gps_localize_firmware` appears within a few seconds |
| ☐ | GPS cold start | outdoors, clear sky, `ros2 topic echo /gps_localize/telemetry` | `gps_satellites` ≥ 8 within 60 s (named telemetry field) |
| ☐ | IMU rate | `ros2 topic hz /gps_localize/imu/data` | ≈20 Hz |
| ☐ | Telemetry rate | `ros2 topic hz /gps_localize/telemetry` | ≈10 Hz |
| ☐ | Power sensors | web dashboard | four INA226 rows (battery, motor A, motor B, fans) show plausible volts/amps |
| ☐ | Compass | web dashboard → Sensors | chip green after calibration (docs/tuning.md §2b) |

## Motion (wheels off the ground)

```bash
# standard teleop interface, m/s and rad/s
ros2 topic pub -r 5 /gps_localize/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
ros2 topic pub -r 5 /gps_localize/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.2}}'
ros2 topic pub -r 5 /gps_localize/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.8}}'
```

| ☐ | Check | Pass |
|---|---|---|
| ☐ | forward | both wheels turn forward, PWM ramps up over ~0.6 s, no jump |
| ☐ | backward | both wheels reverse |
| ☐ | turn | wheels counter-rotate, robot would turn left for `angular.z > 0` |
| ☐ | command timeout | stop publishing → wheels stop within ~0.5 s |

## Indoor manual mode (no GPS needed)

Open the web UI, Dashboard → **ขับเอง / Manual drive**.

| ☐ | Check | How | Pass |
|---|---|---|---|
| ☐ | keyboard | W A S D or arrow keys | robot drives while held, stops on release |
| ☐ | space bar | press Space | everything stops immediately, mission pauses |
| ☐ | turn by angle | press `90° ↻` | rotates ~90° clockwise on the spot and stops by itself |
| ☐ | turn to compass | press `N` / `E` / `S` / `W` | rotates until it faces that direction and stops |
| ☐ | held turn, 5 s | hold ◀ or ▶ for 5 s | motors keep running the whole time, no drop-out, no reset |
| ☐ | **drives straight** | hold ▲ for 5 m on a flat floor | it tracks straight, within roughly half a robot width. It used to curve: manual drive was open loop end to end, and two motors are never identical — this robot's diode drops differ by 0.09 V |
| ☐ | steering still feels direct | hold ▲, tap ◀, release | the turn happens immediately with no fight, and the new heading is held afterwards rather than the old one being pulled back |

Indoors there is no GPS: heading comes from the compass. Steel desks and
reinforced floors disturb it — the sensor chip turns orange and the robot falls
back to the IMU. That is expected inside a building.

## Heading quality

```bash
ros2 run gps_localize heading_check --seconds 30
```

| ☐ | Check | Pass |
|---|---|---|
| ☐ | jitter standing still | standard deviation < 2° over 30 s |
| ☐ | absolute reference | tool reports `compass` or `gps_course`, not `relative` |
| ☐ | turn by hand | heading follows, increases clockwise |

## Outdoor drive

| ☐ | Check | Pass |
|---|---|---|
| ☐ | waypoint run | 3 waypoints 10–20 m apart, robot reaches each within `arrive_radius_m` (2.5 m default, floored at 2.0 m — GPS accuracy limited) and holds still for `arrive_hold_s` before advancing |
| ☐ | straight line | between waypoints it holds the line, no S-weaving (tune with docs/tuning.md §5) |
| ☐ | compass lost fallback | cover/disconnect the compass, start a mission: log shows *"driving straight to learn it from the GPS course"*, then normal navigation |
| ☐ | network cut | walk out of Wi-Fi range while driving | robot stops within ~1 s, red LED blinks fast |
| ☐ | reconnect | robot stays stopped until START/RESUME is pressed |
| ☐ | mushroom switch | motors cut instantly, red LED solid, motor INA226 volts drop to ~0 |

## Endurance

| ☐ | Check | How | Pass |
|---|---|---|---|
| ☐ | battery under load | drive at PWM ≈130 for 2 minutes | pack ≥ 10.5 V on the dashboard afterwards |
| ☐ | thermals | touch the drivers after the run | TB6612 and buck warm, not too hot to touch |
| ☐ | no memory leak | leave the stack running 1 h with the UI open | `ESP RAM ว่าง` in the footer flat (±2 kB), web page still responsive |
| ☐ | loop time | telemetry field 26 | `loop_us` stays under ~4000 µs |

## Docker and Windows acceptance

Use [Docker verification](docker_verification.md) for local evidence and
[Windows handoff](windows_handoff.md) for destination-machine tests. Validate
HTTP and the ROS graph without hardware first; then separately verify USB
attachment, serial firmware, fresh telemetry, reconnect, and restart recovery.
Physical motion tests require an attending operator. A green HTTP healthcheck
does not establish that an ESP32 is connected.
