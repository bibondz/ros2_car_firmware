# Install and run without Docker

This guide installs the robot server directly in Ubuntu. It uses no containers.
On Windows, Ubuntu runs under WSL2; the ROS server and firmware tools run inside
that Ubuntu installation.

## Choose what to install

| Your machine | Install | ESP32 connection |
|---|---|---|
| Ubuntu 22.04 PC | ROS 2 Humble, micro-ROS agent, this workspace; PlatformIO for firmware | Wi-Fi or USB |
| Windows PC hosting the server | WSL2 + Ubuntu 22.04, then the same native packages; usbipd-win for USB | USB is the straightforward route; Wi-Fi requires WSL inbound networking |
| Windows PC/phone using another server | A browser only | Open the Ubuntu server URL |
| Offline demonstration | Python 3 + PyYAML, or build the optional viewer executable | Simulated robot only |

Use the `main` branch for the complete project, including firmware. The
`workspace` branch omits firmware. This project has not validated a full robot
server running directly in Windows PowerShell without WSL. The browser and
standalone viewer do run directly on Windows.

## 1. Prepare Ubuntu and clone the project

On Windows, complete [Windows preparation](#windows-preparation) first. All
commands below marked `bash` run in the Ubuntu terminal, including on Windows.

Use Ubuntu **22.04** with ROS **Humble**. Keep system packages updated before
installing ROS. Follow the official
[ROS 2 Humble Ubuntu installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
to set a UTF-8 locale, enable Universe, and configure the ROS apt repository.
Choose `ros-humble-ros-base` for this server; the larger desktop package is only
needed if you also want tools such as RViz. ROS is installed on this Ubuntu
system, not on the ESP32. Repository setup must finish before the following
apt commands can find ROS packages.

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base build-essential cmake git curl \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-yaml
source /opt/ros/humble/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
git clone --branch main https://github.com/MechcodeRobotech/ROS_GPS_CAR.git ~/GPS_Localize
```

Use a Git account with repository access. If the clone already exists, use it;
do not clone over an existing directory. On WSL, keep it under your Linux home,
not `/mnt/c`, because firmware build dependencies use Linux-specific filenames.

## 2. Build the micro-ROS agent

The agent connects the ESP32 to ROS. Do not assume a Humble apt package named
`ros-humble-micro-ros-agent` is available. Build it once using
[micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup/tree/humble):

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/uros_ws/src
git clone --branch humble --depth 1 https://github.com/micro-ROS/micro_ros_setup.git ~/uros_ws/src/micro_ros_setup
cd ~/uros_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source ~/uros_ws/install/local_setup.bash
ros2 pkg prefix micro_ros_agent
```

If that last command prints an installed package directory, the agent is
available in this shell. On a machine where this workspace already exists and
works, source it instead of repeating its clone/build steps. If any build fails,
resolve that error before continuing; an existing directory alone is not proof
of a successful installation.

## 3. Build the robot workspace

```bash
source /opt/ros/humble/setup.bash
source ~/uros_ws/install/local_setup.bash
cd ~/GPS_Localize/gps_localize_ws
rosdep install --from-paths src --ignore-src -y --skip-keys micro_ros_agent
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 pkg prefix gps_localize
ros2 interface show gps_localize_msgs/msg/Telemetry
```

In **every new Ubuntu terminal** used for ROS commands, run:

```bash
source /opt/ros/humble/setup.bash
source ~/uros_ws/install/local_setup.bash
source ~/GPS_Localize/gps_localize_ws/install/setup.bash
export ROS_DOMAIN_ID=10
```

The project includes an installer, `bash setup/install.sh --native`, that can
build and install services for you. It **enables and starts services**, so use
it as an alternative service-installation path, not alongside an already
running manual launch. Check its actions with
`bash setup/install.sh --native --dry-run`. The manual steps here expose each
dependency/build result and allow starting only when you are ready.

## 4. Prepare USB and firmware tools

Skip firmware installation if the board already runs the correct current image.
Check [hardware](hardware.md) before connecting power. Disconnect motor power
before flashing; opening a serial monitor can reset the ESP32.

```bash
sudo usermod -aG dialout "$USER"
python3 -m pip install --user platformio
export PATH="$HOME/.local/bin:$PATH"
```

Log out and back in to refresh group membership. On WSL, close/reopen Ubuntu;
if needed, restart WSL, then reattach USB. Check `id -nG` includes `dialout`.
Keep the PATH export in future terminals or invoke `~/.local/bin/pio` directly.

Attach USB to WSL first on Windows, using the section below. In Ubuntu:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
cd ~/GPS_Localize
test -e firmware/config/network_secrets.h || \
  cp firmware/config/network_secrets.example.h firmware/config/network_secrets.h
```

Create the secrets file only if it does not already exist. Edit its Wi-Fi list
and OTA password for your installation. It is required at compile time even for
serial firmware, and must remain outside Git. Existing NVS Wi-Fi settings on the
board take precedence over this initial seed.

Close serial monitors and stop any existing serial agent before uploading.
Use the device that actually appeared, replacing `/dev/ttyUSB0` when necessary.
Choose **one** firmware environment:

| Runtime connection | Build/upload in `~/GPS_Localize/firmware` |
|---|---|
| Wi-Fi | `pio run -e wifi -t upload --upload-port /dev/ttyUSB0` |
| USB serial | `pio run -e serial -t upload --upload-port /dev/ttyUSB0` |

Source `/opt/ros/humble/setup.bash` first so PlatformIO sees `ROS_DISTRO=humble`.
For build-only verification, omit `-t upload --upload-port /dev/ttyUSB0`.
The first build downloads substantial dependencies and may take several minutes.
A cable used to flash `wifi` firmware does not make its runtime transport serial.

## 5. Start manually

First choose manual operation **or** systemd services. If services were installed
previously, stop `gps_localize` and `micro_ros_agent` before starting a full manual
launch. Also stop `gps_localize_netwatch` if installed, because it can restart
the managed stack after network changes. Do this only with the robot stopped.
Do not run a second server on the same ports/domain.

After the four environment lines from section 3, choose one:

```bash
# Wi-Fi firmware: agent uses UDP 8888.
ros2 launch gps_localize bringup.launch.py
```

```bash
# Serial firmware: agent owns this USB port at 115200 baud.
ros2 launch gps_localize bringup.launch.py transport:=serial serial_device:=/dev/ttyUSB0
```

Keep that terminal open. **Ctrl+C stops the manual stack.** When a separately
managed agent is intentionally kept running, start only the host nodes with:

```bash
ros2 launch gps_localize bringup.launch.py start_agent:=false
```

In that case, changing `transport:=serial` on this launch will not reconfigure
the separate agent; change the agent itself or use the complete manual launch.

## 6. Open the UI and verify the connection

Open **http://localhost:8080** on the server PC. From another device on the same
reachable LAN, open `http://<server-LAN-IP>:8080`. Find Linux addresses with
`hostname -I`. Port 80 is an optional second listener; 8080 is the explicit
address that works without granting a privileged-port capability.

In another Ubuntu terminal, source the environment from section 3:

```bash
ros2 node list
ros2 topic echo /gps_localize/telemetry --once --qos-reliability best_effort
ros2 topic hz /gps_localize/telemetry
```

Without a board, the host nodes and web page can still start. A real connection
requires `/gps_localize_firmware`, recent telemetry, and changing uptime. Expect
approximately 10 Hz telemetry; Ctrl+C ends the rate measurement. Inspect the
web page's stop reasons, battery, and sensor health before operating. Keep its
browser heartbeat active. See [running](running.md) for manual commands and
[safety](safety.md) for interlocks and attended tests.

For Wi-Fi on native Ubuntu, the ESP32 must reach this PC on UDP 8888. If UFW is
enabled, allow `8888/udp` and `8080/tcp` on the trusted robot network. The optional
friendly address needs port 80 too. In Wi-Fi firmware's 115200-baud console,
`agent <server-LAN-IP>` sets the destination and `agent auto` restores discovery.
Do not select a container or unrelated interface address. USB serial needs no
Wi-Fi connection to the ESP32.

## 7. Optional autostart on Ubuntu

After successful manual verification, stop that launch. The simplest service
installation uses the checked-in installer from the project root:

```bash
cd ~/GPS_Localize
bash setup/install.sh --native --dry-run
bash setup/install.sh --native
systemctl status micro_ros_agent gps_localize
```

The installer substitutes your user/project path into unit templates, then
enables the separate UDP agent and host stack. It also installs available
network helper units. Do not copy template units unchanged: `__USER__` and
`__PROJECT__` are placeholders. Check the resulting units with
`systemctl cat micro_ros_agent gps_localize`.

For **serial autostart**, stop both services with the robot stopped, then edit
the agent override:

```bash
sudo systemctl stop gps_localize micro_ros_agent
sudo systemctl edit micro_ros_agent
```

Save this drop-in (replace the device if necessary):

```ini
[Service]
ExecStart=
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source "$HOME/uros_ws/install/local_setup.bash" && exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200'
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart micro_ros_agent gps_localize
journalctl -u micro_ros_agent -u gps_localize --since '2 minutes ago' --no-pager
```

The host service already passes `start_agent:=false`. The serial drop-in changes
only the separately managed agent. USB permissions must apply to the service
user. Prefer `/dev/serial/by-id/...` for a stable device path if available.

For everyday service operation:

```bash
sudo systemctl stop gps_localize micro_ros_agent
sudo systemctl start micro_ros_agent gps_localize
journalctl -u micro_ros_agent -u gps_localize -f
```

To return the agent to Wi-Fi, remove the `ExecStart` override you created using
`systemctl edit micro_ros_agent`, reload systemd, and restart both services.
Inspect the effective unit afterward; it must run `udp4 --port 8888`.
To disable autostart, disable the two services and any installed network helper
that restarts them. WSL services run only while WSL is running; enabling a unit
does not arrange Windows login startup or usbipd reattachment.

## Windows preparation

### Install Ubuntu in WSL2

In Administrator PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
```

Restart if requested, open Ubuntu, and create its Linux user. Check `wsl -l -v`
shows version 2. Then follow sections 1–6 **inside Ubuntu**, installing ROS and
the agent there. No Docker installation is needed. Use manual launch first.
For optional services, systemd must be enabled in WSL; see
[Microsoft's systemd instructions](https://learn.microsoft.com/en-us/windows/wsl/systemd).

### Attach the ESP32 USB

Install usbipd-win in Administrator PowerShell:

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
usbipd bind --busid 4-4
```

Replace `4-4` with your ESP32 adapter's BUSID. Keep Ubuntu open, close Windows
serial monitors, then attach from regular PowerShell:

```powershell
usbipd attach --wsl --busid 4-4
```

In Ubuntu, identify `/dev/ttyUSB*` or `/dev/ttyACM*`, apply section 4 permissions,
and use `serial` firmware with the serial launch. Windows loses access to the
adapter while attached. After unplugging or restarting WSL, attach it again.
Stop the serial agent before returning USB to Windows:

```powershell
usbipd detach --busid 4-4
```

Reference: [Microsoft USB attachment guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).
A Windows `COM3` name is not the Linux serial path used by this ROS agent.

### Browser and Wi-Fi networking

Open http://localhost:8080 in Windows while the Ubuntu server runs. If localhost
forwarding fails, obtain the Ubuntu address with
`wsl -d Ubuntu-22.04 hostname -I` and try that address from Windows.

For an ESP32 connecting by **Wi-Fi** to a WSL server, ordinary WSL NAT does not
automatically expose UDP 8888 to the LAN. On supported Windows 11 systems,
configure mirrored networking and the applicable Windows/Hyper-V firewall
rules following [Microsoft's WSL networking guide](https://learn.microsoft.com/en-us/windows/wsl/networking).
Then configure the firmware's agent destination and verify actual telemetry.
TCP-only port forwarding does not solve the UDP agent connection. The same
inbound-access check applies when a phone opens the WSL web server.

WSL USB avoids the ESP32's inbound UDP requirement. Actual Windows USB and Wi-Fi
acceptance remains pending on a Windows machine; these commands are documented
routes, not a claim that this Linux session tested Windows drivers/networking.

## Updates and backups

Stop the robot and the active launch/services first. Save runtime settings:

```bash
mkdir -p ~/gps_localize_backups
cp -a ~/.gps_localize "$HOME/gps_localize_backups/runtime-$(date +%Y%m%d-%H%M%S)"
cd ~/GPS_Localize
git pull --ff-only
source /opt/ros/humble/setup.bash
source ~/uros_ws/install/local_setup.bash
cd gps_localize_ws
colcon build --symlink-install
source install/setup.bash
```

Restart using the same manual or service mode. Resolve local Git changes if pull
refuses; do not discard them. Runtime settings live in `~/.gps_localize`, so a
source update does not replace saved YAML settings. Back up the ignored Wi-Fi
header privately too. For restoring settings, stop the server first and copy
the saved runtime directory back before restart. Firmware is a separate update;
build/flash it only when needed, with motor power disconnected.

## Windows browser or offline viewer only

To control an existing Ubuntu server, just open its LAN URL in Windows. No ROS,
WSL, or Docker is needed on the browser machine.

For a local simulation, install Python 3, then in PowerShell:

```powershell
python -m pip install pyyaml
cd GPS_Localize
.\webapp\run_windows.bat
```

Use `--no-demo` for read-only mode. The optional executable must be built using
`webapp/build_exe.bat`; it is not in Git. The viewer is not a real robot server.
See [viewer instructions](../webapp/README.md).

## Troubleshooting

| Failure | Check |
|---|---|
| `ros2: command not found` | Source `/opt/ros/humble/setup.bash` in this terminal |
| `micro_ros_agent` package missing | Complete agent build and source `~/uros_ws/install/local_setup.bash` |
| `gps_localize` or message type missing | Build the workspace and source its install setup |
| rosdep says not initialized | Run the guarded `rosdep init` step, then `rosdep update` as your user |
| Serial permission denied | `dialout` membership, new login, correct service user, actual device owner |
| No USB device in WSL | usbipd attachment, USB data cable, correct adapter BUSID |
| Upload fails or serial agent cannot open port | Stop other agent/monitor processes owning that device |
| `network_secrets.h` missing | Copy the example locally; retain it outside Git |
| No telemetry | Correct firmware transport, domain 10, agent path/port; inspect firmware node |
| Address already in use | Stop duplicate manual/service/viewer stack |
| Port 80 denied | Use 8080; systemd unit grants the optional friendly-port capability |
| `systemctl` unavailable in WSL | Use manual launch or configure WSL systemd first |
| Motor refuses command | Read stop reasons and check browser heartbeat/interlocks; see safety guide |

The four repository gates remain `python3 tools/qc.py`,
`python3 tools/port_qc.py --static-only .`, `pio test -e native`, and
`pio run -e wifi` (the last two from `firmware/`). Passing builds does not replace
attended hardware acceptance.
