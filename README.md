# GPS_Localize

Start with the [illustrated step-by-step guide](docs/step_by_step.md) for screenshots of each UI tab and the [Wi-Fi setup walkthrough](docs/wifi_setup.md).

หุ่นยนต์วิ่งกลางแจ้งตาม GPS waypoint · ESP32 (micro-ROS) + ROS 2 Humble + เว็บควบคุมในเครื่อง

Outdoor GPS-waypoint robot. The ESP32 closes the heading and speed loops
locally at 100 Hz; the server (native Ubuntu or a Linux Docker container) plans the route, watches the safety
chain and serves a local web UI. Wired to
schematic **draft_5** (`SCH_Schematic1_5-draft_5_2026-08-04.pdf`).

```
                 Wi-Fi (micro-ROS / UDP)
 ┌────────────────┐          ┌───────────────────────────────────────────┐
 │ ESP32 DevKitC  │◄────────►│ Ubuntu 22.04 + ROS 2 Humble               │
 │                │          │                                           │
 │ heading PID    │  odom    │  micro_ros_agent                          │
 │ speed PI       │  imu     │  waypoint_nav     ← waypoints.yaml        │
 │ GPS NMEA       │  fix     │  gps_odom                                 │
 │ BNO085 IMU     │ telemetry│  safety_watchdog  ← heartbeat             │
 │ 4× INA226      │          │  web_server ──► http://<ip>:8080          │
 │ 2× TB6612FNG   │◄─cmd─────│                        ▲                  │
 │ safety + LED   │◄─beat────│                        │ SSE + JSON       │
 └────────────────┘          └────────────────────────┼──────────────────┘
                                                      │
                                          เบราว์เซอร์ / browser (phone or laptop)
```

## Hardware (draft_5)

| Part | Detail |
|---|---|
| MCU | ESP32-DevKitC 38-pin |
| Motors | 2× JGA25-370-100RPM, wheels Ø81 mm, **no encoders** |
| Drivers | 2× TB6612FNG (half-bridges paralleled = 1 board per motor), series diode on each VM |
| IMU | BNO085 (I²C 0x4A) — heading PID reference |
| GPS | GEP-M10-DQ (u-blox M10050), UART2 @ IO16/IO17 |
| Power sense | 4× INA226 (processor, motor A, motor B, fan rail) |
| Battery | 3S LiPo 11.1 V → LM2596 buck → 5 V |
| Cooling | 4× 5 V fans on their own LM2596 buck + sensor (always on) |
| Indicators | RED LED on IO2 = e-stop / link lost · GREEN LED hardwired to 5 V = module on |
| Safety | mushroom switch → relay cuts motor power, plus on-board e-stop button |

Full pin table and the wiring changes still to be made on the PCB:
[docs/hardware.md](docs/hardware.md).

## Repository layout

```
firmware/                 ESP32 firmware (PlatformIO, micro-ROS)
  config/                 one header per topic: pins, robot, motor, imu, compass,
                          gps, power, control, safety, network (index: config.h)
  lib/                    motor, INA226, GPS NMEA, IMU, compass, estimator, PIDF, safety
  src/main.cpp            100 Hz control loop + micro-ROS entities
  test/                   261 host tests (pio test -e native)
gps_localize_ws/src/gps_localize/
  gps_localize/           ROS 2 nodes (waypoint_nav, gps_odom, safety_watchdog, web_server)
  config/                 one YAML per topic: robot, nav, pid, motor, estimator,
                          safety, web, waypoints (copied to ~/.gps_localize on first run)
  web/                    the web UI (3 files, no frameworks, works offline)
  launch/                 bringup.launch.py, web_only.launch.py
setup/                    one-click check + install + run (Ubuntu) and a Windows launcher
webapp/                   offline demonstration viewer (no real robot connection)
  exe/GPS_Localize_Viewer.exe   optional local build, not included in Git
  run_windows.bat / run_ubuntu.sh
tools/qc.py               cross-file QC (telemetry layout, endpoints, topics, CSS, exe freshness)
docs/                     hardware, architecture, topics, install, web guide, tuning, safety
```

## Who runs what

| Machine | Runs | Can drive the robot? |
|---|---|---|
| Robot PC (Ubuntu 22.04 + Humble) | `ros2 launch gps_localize bringup.launch.py` → serves port 8080 to the whole Wi-Fi | yes — it is the server |
| Windows Docker server | Docker Desktop Wi-Fi, or WSL2 Docker Engine USB; see [Docker guide](docs/docker.md) | yes, subject to hardware acceptance |
| Phone / tablet / laptop on the same Wi-Fi | just a browser at `http://<robot-ip>:8080` | yes, full control, no install |
| Windows or Ubuntu PC with no robot | `webapp/run_windows.bat` / `run_ubuntu.sh` | **no real robot** — every button drives a simulated one, clearly labelled; good for training |

## Installation choices

**Without Docker:** follow [native Ubuntu / Windows WSL2 installation](docs/native.md)
for required software, Wi-Fi or USB firmware, manual startup, autostart, and updates.

For Docker on Linux or Windows, including **Windows USB into Docker**, follow
[docs/docker.md](docs/docker.md). It lists required software and exact build,
flash, run, stop, update, and troubleshooting commands.
[All documentation](docs/README.md) · [Verification evidence](docs/docker_verification.md).

## Quick start — native Ubuntu

```bash
cd setup && chmod +x setup_and_run.sh && ./setup_and_run.sh
```

It checks the machine, checks/installs ROS 2 Humble, installs the micro-ROS
agent and the other dependencies, builds the workspace, checks the environment
(domain id, ports, firewall) and then starts everything and prints the web
address. Run it again later and it skips straight to starting the robot.
`./install_shortcut.sh` puts a double-clickable icon on the Ubuntu desktop.
Windows: `setup\setup_and_run.bat` (opens a running robot's page, or the local
view-only app). Details: [setup/README.md](setup/README.md).

Manual route, if you prefer to do it yourself:

```bash
cd ~/GPS_Localize/gps_localize_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch gps_localize bringup.launch.py     # web UI on port 8080

cd ~/GPS_Localize/firmware
# Create config/network_secrets.h from its example first; see docs/install.md.
pio run -e wifi -t upload
```

Step-by-step install, autostart and troubleshooting: [docs/install.md](docs/install.md).

## What runs where

| Job | Where | Why |
|---|---|---|
| Heading PID (BNO085), speed control, motor PWM | ESP32, 100 Hz | Wi-Fi latency must never reach the wheels |
| GPS parsing, IMU fusion, dead reckoning | ESP32 | sensors are wired to it |
| Waypoint sequencing, bearing/distance | ROS 2 | easy to edit, easy to log |
| Safety heartbeat, e-stop latch | ROS 2 + ESP32 | two independent layers |
| Web UI, settings files, waypoint editor | ROS 2 (`web_server`) | one place the customer touches |

## Safety chain (five independent layers)

1. **Hardware** — mushroom switch → relay cuts motor power.
2. **Firmware inputs** — `E-EMER` (IO13, the mushroom switch) stops the PWM and
   opens the motor-rail contactor on IO14.
3. **Link** — no `/gps_localize/heartbeat` for 600 ms → firmware brakes (red LED blinks fast).
4. **Command** — no motion command for 500 ms → firmware brakes.
5. **Web** — the browser must POST `/api/heartbeat`; if it stops, `safety_watchdog`
   stops feeding layer 3 and the robot halts. This is the "cut it over the network"
   requirement.

Test procedure: [docs/safety.md](docs/safety.md).
Full bring-up checklist (build, host tests, indoor, outdoor, endurance):
[docs/acceptance.md](docs/acceptance.md).

## Sensor fusion and fallbacks

Heading uses whatever is healthy, best first — **GPS course** (moving) →
**QMC5883L compass** in the GPS module (standing still) → **BNO085
magnetometer** → gyro only. Speed: **IMU+GPS** → GPS → IMU+motor model →
model. Losing a sensor degrades the estimate, it never stops the robot, and
the web UI shows which source is in use. No compass at all? `waypoint_nav`
drives straight for two metres to learn the heading from the GPS course, then
turns onto the bearing.

```bash
python3 tools/qc.py                    # cross-file contract check, no build needed
cd firmware && pio test -e native      # 261 host tests: PID, fusion, ramp, motor model, safety, NMEA
```

## Web UI

Everything the customer needs, in Thai and English, on one page:
live speed / odometry / lat-lon / wheel speed, battery, motor and fan current from
the four INA226, GPS quality, a local map with the waypoint list, manual
driving buttons, the emergency stop, an editable settings page and an event
log. No internet, no app install: [docs/web_ui.md](docs/web_ui.md).

## Memory discipline

The robot is expected to run for hours unattended, so both sides are written
to use a constant amount of RAM:

* firmware: no `String`, no `malloc` after `setup()`, fixed-size ROS messages,
  bounded NMEA parsing per loop, free heap published in the telemetry array
* ROS/web: bounded log ring buffer, bounded SSE queues, capped client count,
  no per-request state, browser trail and log lists capped

Watch `ESP RAM ว่าง` in the web footer — a flat number means no leak.

## License

This project is licensed under the Apache License 2.0.

Original development and substantial implementation:
**Phuthiphong Wongchantib**

See `LICENSE`, `NOTICE`, and `ATTRIBUTION.md` for details.
