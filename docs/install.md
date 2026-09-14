# Install and run — Ubuntu 22.04 + ROS 2 Humble

Start with the [illustrated step-by-step guide](step_by_step.md) for screenshots of each UI tab and the [Wi-Fi setup walkthrough](wifi_setup.md).

For the complete **no-Docker** path, including Windows WSL2 USB, environment
setup, manual operation, and serial autostart, start with [native.md](native.md).
This page retains additional network and installation reference details.

> **Shortcut:** `setup/setup_and_run.sh` does sections 1 to 6 of this page by
> itself — it checks the machine, installs ROS 2 and the dependencies if they
> are missing, builds the workspace, opens the firewall and starts everything.
> The rest of this page is what it does, in case you want to do it by hand or
> something goes wrong. See [setup/README.md](../setup/README.md).

For **Docker on Linux or Windows, including USB through WSL2**, use the
[Docker guide](docker.md). This page covers a native Ubuntu server.
A fresh Git clone does not include the optional Windows viewer executable.

## 1. ROS 2 Humble and the micro-ROS agent

Use Ubuntu 22.04. Clone the full source and let the checked-in installer configure
ROS repositories, dependencies, the agent, the workspace, and service units:

```bash
git clone https://github.com/MechcodeRobotech/ROS_GPS_CAR.git ~/GPS_Localize
cd ~/GPS_Localize
bash setup/install.sh --check
bash setup/install.sh --native
```

Repository access is required. The installer may request sudo. Inspect
`bash setup/install.sh --native --dry-run` to see its actions first. Humble's
micro-ROS agent is built from source; do not assume an apt package named
`ros-humble-micro-ros-agent` exists. Runtime web assets are served locally;
installation and first firmware builds need internet.

## 2. Build this workspace

```bash
# copy the repository to the robot PC, then
cd ~/GPS_Localize/gps_localize_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: with it, editing `web/*.html`, `web/*.js` or the
YAML defaults takes effect on the next start without rebuilding.

## 3. Network

The ESP32 talks to the agent over Wi-Fi (UDP 8888).

```bash
hostname -I          # note this address
sudo ufw allow 8888/udp     # only if the firewall is enabled
sudo ufw allow 8080/tcp     # web UI
```

### Find the agent by name, not by IP

If the router hands out a different address after a reboot, a hard-coded IP
breaks the robot. The firmware can look the PC up by name instead:

```bash
sudo apt install -y avahi-daemon        # publishes <hostname>.local on the LAN
sudo hostnamectl set-hostname mannaja
systemctl restart avahi-daemon
avahi-resolve-host-name -4 mannaja.local   # sanity check, -4 forces IPv4
```

Then in `config/network.h` keep `USE_AGENT_HOSTNAME 1` and set
`AGENT_HOSTNAME = "mannaja"`. The ESP32 tries mDNS first, then plain DNS,
then the fixed `AGENT_IP` as a fallback — and it repeats the lookup while
waiting for the agent, so it follows the PC to a new address without a reflash.

Create the ignored first-boot network seed from the checked-in template:

```bash
cd ~/GPS_Localize
cp firmware/config/network_secrets.example.h firmware/config/network_secrets.h
```

Edit `WIFI_NETWORKS` and the OTA password in that local file. Never commit it.
The board's saved NVS network list takes precedence after initial setup; use
the Wi-Fi firmware's 115200-baud USB console `wifi add` command to
change an existing board. `agent <PC-LAN-IP>` saves the server address;
`agent auto` restores discovery. Check `agent` to see the current choice.
Defaults such as port and domain live in `firmware/config/network.h`.

`ROS_DOMAIN_ID` must be the same on both sides. The launch file sets it to 10.

## 4. Flash the firmware

```bash
sudo apt install -y python3-pip && pip install -U platformio
source /opt/ros/humble/setup.bash        # micro_ros_platformio reads $ROS_DISTRO
cd ~/GPS_Localize/firmware
pio run -e wifi -t upload                # Wi-Fi transport (normal)
pio device monitor -b 115200             # check the boot messages
```

Inspect boot logs for Wi-Fi, agent discovery, and sensor status. GPS baud is
automatically probed; a single fixed boot string is not a health check. Close
the monitor before running a serial agent or flashing again.

Serial transport fallback (no Wi-Fi):

```bash
pio run -e serial -t upload
ros2 launch gps_localize bringup.launch.py transport:=serial serial_device:=/dev/ttyUSB0
```

## 5. Run

```bash
source ~/GPS_Localize/gps_localize_ws/install/setup.bash
ros2 launch gps_localize bringup.launch.py
```

Then open `http://<robot-ip>:8080` from a phone or laptop on the same network.

First start creates `~/.gps_localize/` with a copy of every settings file.
That directory is the live configuration; the web UI writes there.

Check that the firmware really connected:

```bash
ros2 node list                      # /gps_localize_firmware must appear
ros2 topic hz /gps_localize/telemetry   # ~10 Hz
```

## 6. One server, many devices

The robot PC is the server. Everyone else only needs a browser — no app, no
install, no internet.

```bash
hostname -I | awk '{print $1}'     # e.g. 192.168.1.50 -> tell the operators this
sudo ufw allow 8080/tcp            # only if the firewall is on
```

`config/web.yaml` must keep `host: 0.0.0.0` (the default) so the page is
reachable from the network; `127.0.0.1` would make it visible only on the robot
PC itself. `max_clients` (default 8) limits how many devices can stream at once.

| Device | What to do |
|---|---|
| Phone / tablet | connect to the same Wi-Fi, open `http://192.168.1.50:8080` |
| Laptop | same URL in any browser |
| Windows PC with no ROS | `webapp/run_windows.bat` → view only, cannot drive |

All connected browsers see the same live data, and any of them can drive.
Keep that in mind on site: agree who is the operator, because the robot obeys
whoever presses a button. Every one of them can also hit the emergency stop.

**No router on site?** Turn the robot PC into a hotspot and let the phones join
it — the ESP32 must then join the same hotspot (put its SSID/password in
`config/network.h`):

```bash
nmcli device wifi hotspot ifname wlan0 ssid GPS_Localize password robot12345
nmcli connection show --active          # confirm, then use the PC address above
```

### Desktop viewer (no ROS)

`webapp/` contains the same interface running without ROS, for looking at the
UI and preparing settings/waypoints on any Windows or Ubuntu machine:

```bat
webapp\run_windows.bat                 :: Windows, uses exe\GPS_Localize_Viewer.exe
```
```bash
webapp/run_ubuntu.sh                   # Ubuntu, uses system python3
```

By default it shows a labelled simulation and controls a simulated robot.
Use `--no-demo` for read-only mode. The optional executable is not in Git;
install Python 3 and PyYAML or build it using `webapp/build_exe.bat`. Saved
settings live beside the executable or under `webapp/settings/` for Python; copy them to
`~/.gps_localize/` on the robot PC to use them for real. Details:
[../webapp/README.md](../webapp/README.md).

## 7. Autostart on boot

Two units. `setup/install.sh` installs both; by hand it is:

```bash
cd ~/GPS_Localize/gps_localize_ws/src/gps_localize/systemd
sudo cp micro_ros_agent.service gps_localize.service /etc/systemd/system/
sudo nano /etc/systemd/system/micro_ros_agent.service   # set User= and the paths
sudo nano /etc/systemd/system/gps_localize.service      # the same two
sudo systemctl daemon-reload
sudo systemctl enable --now micro_ros_agent
sudo systemctl enable --now gps_localize
```

Checking them:

```bash
systemctl status micro_ros_agent      # is the board's agent up?
systemctl status gps_localize         # is the rest of the stack up?
journalctl -u micro_ros_agent -f      # watch the agent, board sessions appear here
journalctl -u gps_localize -f         # watch the nodes
```

**Why the agent is its own unit.** It used to be started by
`bringup.launch.py`, which meant it shared that launch's fate: stopping the
stack to work on it, or a node failing to start, also stopped the thing the
board is trying to reach. The board then retried against a host that looked
completely healthy to whoever was standing at it. Now `gps_localize.service`
runs the launch with `start_agent:=false` and the agent has its own restart
policy, so restarting the stack does not drop the board's link.

Exactly one agent may bind the port. If you start the launch by hand while the
service is running, pass `start_agent:=false` or the second agent exits with
`Address already in use` — and which of the two lost the race is not visible
from the board's side.

The agent listens on **UDP 8888**, and that number is fixed: the firmware is
built to connect to it, and on a VirtualBox host the port-forward rule names it
too. Changing it means changing all three together.

## Troubleshooting

| Symptom | Check |
|---|---|
| board is on Wi-Fi but never links | it may be aiming at the wrong address. Plug in USB, open the serial console at 115200 and type `agent` — it prints where it is looking and why. `agent <ip-of-this-pc>` points it somewhere else and takes effect at once; `agent auto` returns to discovery. Both are stored on the board, so this survives a reboot and needs no reflash |
| `ros2 node list` shows no firmware node | `systemctl status micro_ros_agent` first - it is a separate unit and can be down while the rest of the stack is up. Then: same `ROS_DOMAIN_ID`? UDP 8888 open? serial monitor shows which address the ESP32 resolved |
| ESP32 prints `name lookup failed` | `avahi-daemon` not installed/running on the PC, or the Wi-Fi AP blocks multicast — use the `AGENT_IP` fallback or a DHCP reservation |
| Web page loads but every value is `—` | `ros2 topic hz /gps_localize/telemetry`; if empty the ESP32 is not connected |
| Red LED blinks fast all the time | heartbeat or command timeout — is `safety_watchdog` running, is the browser open? |
| Red LED solid | emergency stop latched: mushroom switch, on-board button, or web e-stop (press RESET) |
| Robot refuses to start a mission | GPS gate: needs `min_satellites` and `max_hdop` from `nav.yaml`; the reason is shown on the dashboard |
| GPS never gets a fix | outdoors with sky view, wait 1–2 min for the first fix; check the antenna connector |
| Heading drifts / robot circles | see [tuning.md](tuning.md) — IMU mounting signs first, then the heading PID |
| Web UI unreachable from a phone | `web.host` must be `0.0.0.0`, port 8080 open in the firewall, same subnet |
| `Address already in use` on start | another copy is running: `systemctl stop gps_localize` or change `web.port` |

## Reaching the web UI

    http://gps-robot-web.local:8080

`install.sh` enables `gps_localize_mdns.service`, which publishes that name and
follows the machine when its address changes. It needs avahi:

    sudo apt install avahi-daemon avahi-utils

Without avahi the UI is still served, just only by IP address.
