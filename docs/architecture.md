# GPS_Localize — how the whole thing fits together

Everything in this document was read out of the code, not from memory. File and
line references point at where each fact lives, so you can check any of it.

---

## The short answer: no, it is not "just ROS"

Four different network protocols are in play, and they are easy to confuse
because three of them are UDP.

| # | Link | Protocol | Port | Who talks |
|---|---|---|---|---|
| 1 | ESP32 → host | **micro-ROS over UDP4** | **8888** | the robot to the agent |
| 2 | inside the host | **DDS (Fast DDS)** over UDP, unicast + multicast | ephemeral + 7400-range | ROS 2 nodes to each other |
| 3 | browser → host | **HTTP + SSE** over TCP | **8080** | your phone or laptop to the web UI |
| 4 | name lookup | **mDNS** over UDP | 5353 | finding `.local` names on the Wi-Fi |

ROS 2 itself never speaks to the ESP32. The **agent** is the translator: the
ESP32 speaks the compact micro-XRCE-DDS wire protocol over UDP 8888, and the
agent turns that into real DDS traffic that ordinary ROS 2 nodes can see.

That is why the agent is not optional, and why the robot vanishes from
`ros2 topic list` the moment the agent stops even though the ESP32 is still
transmitting.

Because it is not optional, it runs as its own systemd unit,
`micro_ros_agent.service`, rather than as a child of the bringup launch. The
launch is started with `start_agent:=false` so exactly one agent binds UDP 8888.
The split exists so that restarting or stopping the host stack - something done
often while working on it - does not take down the one process the board is
trying to reach.

---

## The picture

```
  ESP32  (firmware/src/main.cpp)
    |
    |  micro-ROS over UDP4 : 8888          Wi-Fi
    |  or micro-ROS over USB serial        cable
    v
  micro-ROS agent  (MicroXRCEAgent)
    |
    |  DDS / Fast DDS, ROS_DOMAIN_ID = 10
    v
  ROS 2 nodes            gps_odom · heading_check · safety_watchdog
    |                    waypoint_nav · web_server
    |  HTTP + SSE : 8080
    v
  browser                phone, laptop, anything on the Wi-Fi
```

Everything below the ESP32 runs on one machine — natively, or all inside one
Docker container. That matters: **DDS discovery never has to cross a machine
boundary**, which is what makes the Docker-on-Windows setup work with nothing
but two published ports.

---

## 1. ESP32 → host: micro-ROS over UDP 8888

`firmware/config/network.h:83`

```c
static const uint16_t AGENT_PORT = 8888;
```

`firmware/src/main.cpp:940`

```c
set_microros_wifi_transports(ssid, pass, agent_ip, AGENT_PORT);
```

There is a serial alternative on the same code path
(`set_microros_serial_transports(Serial)`, `main.cpp:942`), selected at build
time by the PlatformIO environment: `-e wifi` or `-e serial`.

**Finding the agent.** The firmware does not depend on a fixed IP, and since the
address can now be set and remembered on the board, it does not depend on being
reflashed either. In order it tries: an address set by hand (`agent <host|ip>` on
the USB console, kept in NVS), then mDNS, then plain DNS, then the address that
last carried a real session, then the compile-time fallback, then the DHCP
gateway. Only a genuine session updates the remembered address — a successful
ping proves the agent is reachable, not that a session can be built. It resolves
in order (`main.cpp:729`, `:734`):

1. mDNS — `<AGENT_HOSTNAME>.local`, UDP 5353
2. plain DNS on the bare name, if the router registers DHCP names
3. `AGENT_IP`, the hardcoded fallback

and re-resolves on every reconnect, so the host PC changing address does not
strand the robot. The robot also announces itself as `esp32-gps-localize.local`.

---

## 2. Inside the host: DDS

`ROS_DOMAIN_ID = 10` (`firmware/config/network.h:97`, and the same value in the
launch file and the systemd unit).

The domain id must match on **both** sides or the two halves cannot see each
other at all — no error, just an empty topic list. It is the single most common
cause of "everything looks fine but nothing appears".

DDS discovery uses multicast. On one machine, or inside one container, that is
entirely internal.

---

## 3. Browser → host: HTTP on 8080

`gps_localize_ws/src/gps_localize/config/web.yaml:5` sets `port: 8080`, served
by a `ThreadingHTTPServer` (`httpd.py:273`) bound to `0.0.0.0` so a phone on the
same Wi-Fi can reach it.

Two kinds of traffic:

- **request/response JSON** — `/api/state`, `/api/waypoints`, `/api/topics`, …
- **server-sent events** — `/api/stream`, `Content-Type: text/event-stream`
  (`httpd.py:251`), a single long-lived connection pushing telemetry and log
  lines. Not WebSockets: SSE is one-way and needs no handshake, which is all a
  dashboard requires.

The browser must also POST `/api/heartbeat` continuously. If it stops, the
firmware stops the motors — see safety below.

---

## The data contract

The ESP32 publishes and subscribes exactly these. `tools/qc.py` fails the build
if the two sides ever disagree.

**Robot → host**

| Topic | Type |
|---|---|
| `/gps_localize/odom` | `nav_msgs/Odometry` |
| `/gps_localize/imu/data` | `sensor_msgs/Imu` |
| `/gps_localize/gps/fix` | `sensor_msgs/NavSatFix` |
| `/gps_localize/telemetry` | `gps_localize_msgs/Telemetry` — **56 named fields** |

**Host → robot**

| Topic | Type |
|---|---|
| `/gps_localize/cmd_move` | `geometry_msgs/Twist` |
| `/gps_localize/cmd_manual` | `geometry_msgs/Twist` |
| `/gps_localize/cmd_vel` | `geometry_msgs/Twist` |
| `/gps_localize/heartbeat` | `std_msgs/Int32` |
| `/gps_localize/estop` | `std_msgs/Bool` |
| `/gps_localize/config/pid` | `std_msgs/Float32MultiArray` |

Telemetry is one message rather than forty topics because each DDS message has
real overhead on a microcontroller. It used to be a packed `Float32MultiArray`
where meaning came from **position**, which meant an index kept in step across
three files. It is now `gps_localize_msgs/Telemetry` with **named** fields:
adding a signal is one line in the `.msg` plus one assignment.

The `.msg` lives in `gps_localize_ws/src/gps_localize_msgs/` and is copied to
`firmware/extra_packages/` because micro-ROS compiles type support into the
image and does not follow symlinks. QC checks the two copies stay identical and
that every declared field is actually assigned — an unassigned field would
publish a silent zero rather than fail.

---

## The sensors, and what each one really measures

Four INA226 current/voltage sensors. **Two of them measure voltage and current
at different points**, which caused a real bug and is documented in
`firmware/config/power.h` and `firmware/src/power/power_monitor.h`.

| Sensor | Voltage measured at | Current measured at |
|---|---|---|
| processor `0x41` | the rail feeding the ESP32 — **3V3**, from its own buck, since the board was taken off the shared 5 V rail | the **battery** side, including the buck |
| fan `0x45` | the fan +5 V rail | source, through the fan buck |
| motor A `0x40` | **VCC EMER — this is the pack voltage** | source → driver → motor |
| motor B `0x44` | same node as A | source → driver → motor |

**Pack voltage comes from the motor rails, never from the processor sensor.**
Reading the +5 V rail as the pack is what previously latched the robot into
e-stop three seconds after every boot: 5 V is below any 3S cutoff, and clearing
the latch needed a voltage that rail can never reach.

When the mushroom switch or the relay is open the motor rails read near zero.
That means *cannot measure*, not *flat battery*, and the firmware marks the
reading invalid rather than tripping the cutoff.

Other sensors: **BNO085** IMU (yaw, yaw rate, forward acceleration),
**QMC5883L** compass inside the GPS module, and a **GEP-M10 GNSS** module parsed
from NMEA.

---

## How position and heading are worked out

There are no wheel encoders. Everything is derived.

**Heading** — `firmware/src/estimator/state_estimator.h`, `heading_source = 4`
(AUTO). Sources are ranked and health-checked, and losing one degrades the
heading rather than stopping the robot:

```
GPS course  >  QMC5883 compass  >  BNO085 magnetometer  >  gyro only

That is a priority order, and on its own it is not enough: it says which source
to prefer, not whether the preferred one is telling the truth. The compass and
the BNO085's magnetometer are both absolute and both fooled by the same things —
a motor drawing current, a steel bench, rebar in a floor. The IMU's fused yaw is
fooled by none of them, because it comes from the gyroscope; it drifts slowly,
which is why it needs an absolute reference at all, but over the seconds a
disturbance lasts it is the honest one.

So every absolute reference is **checked against the IMU before it is believed**.
A reference that disagrees by more than `agree_tol_deg` is refused, and
`heading_innov_deg` and `heading_gated` report the disagreement so it can be
seen rather than guessed at. Measured on this robot: the compass moved 9.5° in
0.3 s while the gyroscope reported about 1 °/s, and applied blind that step went
straight into the heading and the manual hold steered to it.

Two details stop the gate causing worse problems than it solves. The **first**
alignment is never gated — before it there is nothing to disagree with, and
gating it would mean the robot never finds north at all. And a disagreement that
**persists** past `agree_recover_s` is accepted: a gate that can never give up
locks out a correct sensor for ever if the gyro really has drifted.

Manual driving is no longer outside that chain. It used to be open loop end to
end — the jog buttons produced a speed and a turn rate, those became two PWM
numbers, and nothing ever compared where the robot pointed with where it had
been pointing. Two motors are never identical, so "forward" drove an arc.
Commanding straight now latches the heading and the heading PID trims the wheel
difference to keep it; commanding a turn releases the hold and re-latches it on
release, so steering stays direct. The correction is clamped (`MANUAL_HOLD_MAX_PWM`)
because it is a trim and must never out-pull the person holding the button.

The **speed** loop is closed in manual as well, for the same reason: the
feed-forward from PWM to speed is a guess, so the same request gave one speed on
a fresh battery and another on a flat one, up a slope or on carpet. It closes
only when the fused speed was measured **independently of the motors** — GPS, or
IMU acceleration. With neither, the estimator falls back to the motor model,
which is computed from the very PWM being set; a loop closed on that compares
the output with itself, learns nothing, and winds its integral up chasing an
error that cannot close. `DriveController::speedClosed()` reports which of the
two is happening.
```

**Speed** — a complementary filter: IMU acceleration for the fast component,
GPS ground speed pulling it to truth, and the motor model (back-EMF from the
INA226 volts and amps) as the fallback when GPS goes stale.

**Position** — a 3-state EKF (`firmware/src/estimator/pose_ekf.h`) over x, y and
heading. It used to be plain dead reckoning that GPS never corrected, so the
error grew without bound.

Speed is an *input* rather than a state: `StateEstimator` already fuses it from
three sources and knows which one it is using, and two filters disagreeing
about one quantity is worse than one.

Measurement variance comes from the receiver's own `hdop` and fix quality, so
the gain follows the data — and an RTK receiver tightens the result with no code
change. A Mahalanobis gate rejects multipath jumps, but gives in after five
consecutive rejections so the estimate cannot sit stale and confident if the
receiver has genuinely moved.

`pos_sigma_m` is published so the UI can show honest accuracy; it grows while
coasting without GPS and shrinks on a fix.

---

## Safety, and why the browser matters

`firmware/src/safety/safety_manager.h` collects every stop condition into one
bitfield. Motion is allowed only when all of them are clear:

| Flag | Fires when |
|---|---|
| `STOP_HW_EMERGENCY` | the mushroom switch / relay line opens |
| `STOP_ESP_BUTTON` | the on-board button |
| `STOP_SW_ESTOP` | the web UI e-stop |
| `STOP_HEARTBEAT_LOST` | the browser stopped sending heartbeats |
| `STOP_CMD_TIMEOUT` | the command stream stalled while moving |
| `STOP_AGENT_LOST` | the micro-ROS agent went away |
| `STOP_BATTERY_SOFT` | pack below 11.4 V — stop driving, stay powered |
| `STOP_BATTERY_LOW` | pack below 3.733 V/cell for 3 s — 11.199 V on 3S — latched |
| `STOP_STARTUP` | boot lockout |

3S LiPo tiers: warn 11.7 V, soft stop 11.4 V, hard cut 11.199 V (about 11.2 V).
The hard cutoff is latched and requires a power cycle; it has no voltage-only clear threshold. A cell is
damaged below about 3.0 V/cell (9.0 V pack); these sit well above that.

**The control loops run on the ESP32 at 100 Hz, not on the host.** A Wi-Fi
hiccup cannot make the robot swerve, and if the link dies entirely the robot
stops itself. The host decides *where to go*; the robot decides *how to get
there* and when to refuse.

---

## Repository layout

```
firmware/          ESP32, PlatformIO
  config/          tunables and the pin map
  lib/             reusable drivers: INA226, BNO085, TB6612, QMC5883, NMEA, PIDF
  src/             this robot: estimator, control, safety, power
  test/            261 host tests, run with pio test -e native
gps_localize_ws/   ROS 2 Humble workspace
  .../gps_localize/  the five nodes, web UI, launch files, config
webapp/            the no-ROS viewer, for a PC with no robot
tools/             QC gates, the branch publisher, plan.html, sync scripts
setup/             install.sh and the Windows setup script
docs/              this file and the rest
```

Branches: `main` (everything), `workspace` (no firmware), `firmware` (no ROS or
web), `docker` (workspace + firmware + Dockerfile). The three subsets are
generated from `main`, never edited by hand.

---

## Ports to open

| Port | Protocol | Needed for |
|---|---|---|
| 8888 | **UDP** | the robot reaching the agent — **the one people forget** |
| 8080 | TCP | the web UI |
| 5353 | UDP | mDNS `.local` names (optional but convenient) |

For Windows Wi-Fi, Docker publishes TCP 8080 and UDP 8888; mDNS is not published.
Windows Firewall can block the published ports:
`setup/setup_windows.ps1` opens both and warns if the network profile is Public,
where Private/Domain rules do not apply at all.

---

## When nothing appears, check in this order

1. **Domain id** — must be 10 on both sides. A mismatch shows an empty topic
   list with no error at all.
2. **Is the agent running?** No agent, no topics, however healthy the ESP32 is.
3. **UDP 8888 inbound** — firewall on Windows, `ufw` on Linux.
4. **Same network?** Check the Wi-Fi the ESP32 joined against the PC's subnet.
5. **`ros2 node list`** — `/gps_localize_firmware` present means the board is
   really connected. Topic presence alone does not prove it: a host node that
   merely *subscribes* to `/gps_localize/gps/fix` puts that topic in the graph
   with no board anywhere.

Without a terminal, the **Topics tab** in the web UI shows the whole graph —
every topic, its type, rate and live value. That is the reason it exists:
Windows has no `ros2` CLI, and inside Docker the graph is not reachable from the
host either.

---

## Why micro-ROS and not plain UDP

A fair question, since the ESP32 already speaks UDP and the telemetry is a
single packed array. Writing a small custom UDP protocol would remove the agent
entirely.

**The ESP32 cannot run real ROS 2.** A full DDS stack does not fit on it —
micro-ROS exists precisely because of that. So the real choice is micro-ROS
versus a hand-written protocol, not micro-ROS versus "proper" ROS 2.

| | micro-ROS (what this uses) | hand-written UDP |
|---|---|---|
| Host tooling | `ros2 topic echo`, `ros2 bag`, rviz, plotjuggler all work unchanged | none of it; you write every tool yourself |
| Message format | typed, versioned, generated | you own serialisation, endianness, versioning |
| Adding a signal | add a field, QC checks both sides agree | edit two parsers by hand and hope |
| Reconnect / discovery | handled, including the agent going away and coming back | you write it |
| Extra moving part | **the agent must be running** | none |
| Flash and RAM | 930 KB flash, 81 KB RAM as built. **Measured: the rcl/rmw/uxr/ucdr stack is 65 KB of 808 KB of symbols — 8%.** The bulk is Wi-Fi, the Arduino core and the sensor drivers, none of which a custom protocol removes | saves that 8%, not the 47% |
| Failure mode | agent dies, robot vanishes from the graph | fewer parts to fail |

**This project keeps micro-ROS**, for two concrete reasons.

The host side already depends on the ROS graph: five nodes, launch files, and a
web server that reads topics. Replacing the transport means rewriting all of it
and losing every off-the-shelf tool — for a robot whose whole point is
navigation, throwing away rviz and bag files is a bad trade.

And the efficiency argument mostly does not apply here, because the design
already sidesteps it. Telemetry is **one** message of 61 fields rather than 61
topics, so the per-message DDS overhead people worry about is paid once per
cycle, not fifty-six times.

It used to be a packed `Float32MultiArray` — genuinely a binary payload wearing
a ROS message as a wrapper, which is the raw-UDP tradeoff smuggled inside ROS:
the same byte efficiency, and the same manual index bookkeeping. Moving to a
named `Telemetry` message kept the single-message efficiency and dropped the
bookkeeping, which is the combination worth having.

The agent being a required extra process is the genuine cost. It is why the
robot disappears from `ros2 topic list` the moment the agent stops, and why the
Docker image builds one from source. That is a real downside, accepted
knowingly.

### Measured on the real board, 2026-08-27

Numbers from the ESP32 on the bench, not estimates:

| | |
|---|---|
| telemetry configured | 10 Hz (`PUB_TELEMETRY_DIV 10` at 100 Hz) |
| telemetry measured | **6.2 - 8.1 Hz**, varying with Wi-Fi signal |
| micro-ROS code size | **65 KB of 808 KB** of symbols — 8% |
| flash / RAM total | 47.3% / 24.8% |

The rate shortfall is the honest cost, and it is worth being precise about what
causes it: it moved between 8.1 Hz at -46 dBm and 6.2 Hz at -67 dBm, so it
tracks **radio conditions**, not CPU. micro-XRCE-DDS acknowledges reliable
messages, so a weaker link slows the publish path. A fire-and-forget UDP
protocol would not slow down - it would silently drop instead, which for
telemetry is arguably worse and for commands is much worse.

The size argument is weaker than it looks. Removing micro-ROS entirely would
save 8% of the symbol bytes; the other 92% is Wi-Fi, the Arduino core and the
sensor drivers, none of which a custom protocol removes.

**When plain UDP would win:** a sensor node that only ever ships one fixed
struct to one consumer, on hardware too small for micro-ROS, where nobody needs
bags or rviz. That is not this robot.

## Windows USB deployment

The USB deployment uses usbipd-win to attach the ESP32 to WSL2 Ubuntu and runs
Docker Engine in that same distribution. Compose maps the Linux serial device
into the container, and the serial-transport override launches the agent at
115200 baud. Firmware must be built with `-e serial`. USB flashing alone does
not change Wi-Fi firmware into serial firmware. See [Docker guide](docker.md).
