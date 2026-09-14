# Running everything on this PC

Everything on this machine already starts by itself. This page is the short
answer to "what do I actually type", in the order you would type it, plus what
each command proves — because a command that appears to work is not the same as
a robot that is working, and most of the time lost on this project has gone to
that gap.

`docs/install.md` is how to set a machine up from nothing. This is how to run
the one that is already set up.

## The short version

```bash
python3 tools/doctor.py        # is anything broken, and what do I do about it
```

Then open **<http://gps-robot-web.local>** — or `http://<this PC's IP>` — from
any browser on the same network. There is nothing else to start.

## What is already running

Five services come up at boot and restart themselves if they die. Nothing here
needs to be started by hand.

| Service | What it does | Why it is separate |
|---|---|---|
| `micro_ros_agent` | listens on **UDP 8888** for the ESP32 | The board reconnects to it, not the other way round, so it has to outlive any restart of the stack. The port is fixed: the VirtualBox NAT rule `microros-agent` forwards host UDP 8888 to this VM, and changing it breaks the board's side. |
| `gps_localize` | the ROS nodes and the web UI | The actual robot software. |
| `gps_localize_netwatch` | restarts the stack when the network changes | DDS binds its interfaces when a node starts. Switch Wi-Fi and the nodes keep advertising the old address and vanish from the graph, while every topic still *looks* present. |
| `gps_localize_mdns` | republishes `gps-robot-web.local` | So the URL still works after the address changes. |
| `gps_localize_beacon` | announces where the agent is, on **UDP 8889** | The robot connects to *us*, never the other way round, so it has to know our address. Everything else that tells it breaks when the network moves: a hand-set address goes stale, mDNS cannot cross a subnet, and the gateway guess is only right when the PC *is* the gateway. This says it outright, twice a second. An unlinked board adopts it within seconds; a linked one ignores it. |

```bash
systemctl status micro_ros_agent gps_localize gps_localize_beacon --no-pager
journalctl -u micro_ros_agent -f          # the board linking and unlinking
journalctl -u gps_localize -f             # the nodes and the web server
sudo systemctl restart gps_localize       # after a network change, if netwatch missed it
```

## Opening the web UI

| From | URL |
|---|---|
| this PC | <http://localhost> |
| a phone or laptop on the same Wi-Fi | `http://gps-robot-web.local` or `http://<IP>` |

```bash
hostname -I | awk '{print $1}'    # the address to hand out
```

Port 80 and port 8080 both serve the same page; 80 exists so the address can be
typed without a port number.

If the page loads on this PC and nowhere else, the machine is behind
VirtualBox NAT. `tools/doctor.py` says so explicitly rather than leaving it to
be guessed — look for the `reachable from other devices` line.

## Checking the robot is really there

The trap here is that every one of these can look right while the robot is
doing nothing, so it is worth knowing what each one actually proves.

```bash
export ROS_DOMAIN_ID=10                 # must match the board; a mismatch shows
source /opt/ros/humble/setup.bash       # an empty topic list and no error at all
source gps_localize_ws/install/setup.bash

ros2 node list                          # /gps_localize_firmware = a board is connected
ros2 topic list                         # topics alone prove nothing: a host node that
                                        # only subscribes creates the topic too
ros2 topic hz /gps_localize/telemetry   # ~10 Hz
ros2 topic echo --once /gps_localize/telemetry
```

**The node in the list is the proof.** The topic being present is not.

For everything at once, with the numbers rather than a verdict:

```bash
python3 tools/doctor.py                 # services, port, web, reachability, board,
                                        # loop timing, resets, rails, safety, sensors
python3 tools/doctor.py --quick         # skip the timed checks
python3 tools/doctor.py --json
```

Every check prints what it measured, and says `?` rather than guessing when it
cannot get the evidence. Exit status is 0 when nothing is broken.

## Before committing anything

All four must pass:

```bash
python3 tools/qc.py                        # cross-file contracts
python3 tools/port_qc.py --static-only .   # port gate
cd firmware && pio test -e native          # 261 host tests
pio run -e wifi                            # the firmware still builds
```

These all pass on a bench with no hardware, because with no board the callbacks
that consume telemetry never run. When the change touches anything the robot
feeds, run the probe too — it publishes synthetic firmware frames so those paths
actually execute here:

```bash
python3 tools/live_probe.py
```

## Flashing the board

With the robot stopped and motor power disconnected, update over Wi-Fi using
the firmware pull API below. The current UI has no separate Firmware section:

```bash
cd firmware && pio run -e wifi            # build; the host serves the result
curl -X POST http://localhost/api/firmware/pull
```

The board fetches the image from this PC and reboots into it. Watch the version
change rather than watching the console: **opening the serial port resets the
board**, which is a good way to sabotage the very test you are running.

```bash
watch -n2 "curl -s localhost/api/state | python3 -c \
  'import json,sys; print(json.load(sys.stdin)[\"state\"][\"telemetry\"][\"firmware_version\"])'"
```

Over USB, when the board is not on the network:

```bash
cd firmware && pio run -e wifi -t upload
pio device monitor -b 115200              # type `help` for the settings console
```

## Calibration

All of it is in the browser now, on the **ปรับเทียบ / Calibration** tab. Nothing
needs a cable or a terminal. `docs/calibration.md` has the detail; the short
version is that steps 1 and 2 are enough to drive:

1. **Compass** — press Start, turn the robot slowly through one full circle,
   press Finish. The progress bar only moves while the robot actually turns.
2. **IMU** — press Start, move the robot in slow figure-of-eights away from the
   motors and the battery until the accuracy reads 2, press Save.

## Driving

Wheels off the ground the first time, and prove the emergency stop before
anything else.

```bash
python3 tools/motor_id.py --dry-run       # what it would send
python3 tools/motor_id.py                 # identifies which rail is which motor
```

Otherwise drive from the web UI: the arrows on the dashboard, or waypoints on
the map.

## When something is wrong

| Symptom | Cause and fix |
|---|---|
| `ros2 topic list` is empty | `ROS_DOMAIN_ID` is not 10 in this shell |
| Topics present, node list empty, web data minutes old | The PC changed network and DDS is stranded. `sudo systemctl restart gps_localize` — netwatch normally does this for you |
| Web UI reachable here, not from a phone | VirtualBox NAT. Bridge the adapter, or forward the port |
| Board never appears | Open the web UI's **วิธีเชื่อมต่อ / Help** tab and read the *Find the robot* panel: it says whether the board is on this network at all, and whether we are announcing the agent. If it is on the network it adopts the beacon within seconds. If it is not, it is on a different Wi-Fi |
| `battery` reads "pack not measurable" | The contactor is open — the emergency is pressed. That is not a flat battery |
| `gps_missing` indoors | No sky view. Normal |
| `imu_magnetometer_missing` | The BNO085 has not reached accuracy 2 **or** its heading report is stale. Not a fault — calibrate it (step 2 above). If it flickers on and off while the accuracy reads a steady 3/3, the report is not arriving rather than the magnetometer being bad — the **IMU reports** row on the dashboard shows the delivered rate of each report against the rate asked for, and says which of the two it is. A `rv` near zero beside healthy `game`/`gyro`/`accel` rates is normal on this robot: the BNO085 runs one fusion, never two (see `docs/hardware.md`), and this is the third heading source behind the GPS course and the QMC5883L |
| The IMU keeps resetting, or its accuracy falls to 0 and stays there | Check the radio before the sensor. Wi-Fi transmit bursts were browning out the BNO085 on USB-only power — 34 resets in 68 seconds — and `WIFI_TX_POWER` is 13 dBm to stop it. The firmware also writes the calibration into the sensor's own flash the first time it reaches 3/3, so a reset no longer loses it. `imu_resets` on the dashboard is the number to watch |
| `right_open_circuit` / `left_open_circuit` | No current at all on that rail while it is being driven. Reads like a broken wire and on this robot it was not: the right rail drew 0.001 A against the left's 0.6 A for 24 consecutive moves, then worked perfectly after a full power-down. A wire does not repair itself — a driver latched into thermal or overcurrent protection does exactly this, and stays off until VM is removed. Same fix: unplug everything for 10 seconds |
| `imu_missing` right after a flash | The BNO085's RESET line is not wired, so an ESP32 software reset does not reset it and its handshake fails. **Unplug the battery AND the USB cable for 10 seconds.** Both share the 3V3 rail, so USB alone keeps the sensor alive and nothing resets it — not a reflash, not the emergency stop, not `ESP.restart()`. Proved: it clears every time on a full power-down and never on anything less. The reset reason cannot confirm it either — an EN pulse also reports `POWERON`. Afterwards press **Retry start** on the Calibration tab |
| Compass reads "disturbed" on every frame | A bad calibration is stored. Clear it on the Calibration tab |

## Everything else that is a command

```bash
python3 tools/robot_qr.py            # a QR of the web address, for a phone
python3 tools/soak_watch.py          # long-run watcher: resets, heap, loop timing
python3 tools/loop_probe.py 45       # is the control loop real time? loop_us plus late IMU stamps
python3 tools/self_update.py --check # is this checkout behind the repository
tools/publish_branches.sh            # regenerate the workspace/firmware/docker branches
```

## Docker and terminal manual commands

For container installation, Windows USB, firmware selection, and start/stop
commands, see [Docker operation](docker.md). Keep the real server's browser
open: the safety watchdog normally requires its heartbeat. Repeated commands
do not bypass the emergency stop or other interlocks.

For an attended forward test, with the area clear and e-stop available:

```bash
ros2 topic pub --rate 10 /gps_localize/cmd_manual geometry_msgs/msg/Twist '{linear: {x: 0.18, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

This requests 0.18 m/s continuously. Ctrl+C ends the publisher; send zero:

```bash
ros2 topic pub --once /gps_localize/cmd_manual geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

In Docker, open `docker compose exec gps_localize bash` first, using the same
overrides as startup. Keep YAML inside one quoted argument. For this firmware,
manual `angular.z` uses degrees/second, clockwise positive; see [topics](topics.md).
