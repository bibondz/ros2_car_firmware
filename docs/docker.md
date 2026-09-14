# Docker installation and operation

Start with the [illustrated step-by-step guide](step_by_step.md) for screenshots of each UI tab and the [Wi-Fi setup walkthrough](wifi_setup.md).

For installation **without Docker**, use the [native Ubuntu / Windows WSL2 guide](native.md).

## Choose an installation

| Machine and connection | Install on the host | Run |
|---|---|---|
| Ubuntu Linux, ESP32 Wi-Fi | Docker Engine and Compose plugin, Git | Base Compose file |
| Windows, ESP32 Wi-Fi | Docker Desktop with Linux containers, Git | Base + Windows override |
| Windows, ESP32 USB | WSL2 Ubuntu 22.04, usbipd-win, Docker Engine and Compose **inside that Ubuntu** | Base + Windows + USB + serial transport overrides |
| Ubuntu Linux, ESP32 USB | Docker Engine and Compose, Git | Base + USB + serial transport overrides |
| Browser connected to an existing server | A browser only | Open the server URL |
| Offline demonstration | Python 3 and PyYAML, or a locally built viewer executable | `webapp/run_windows.bat` or `webapp/run_ubuntu.sh` |

The container contains ROS 2 Humble, the micro-ROS agent, navigation, safety,
the web server, and PlatformIO. ROS is not needed on the Windows host. The
viewer simulates a robot; it is not the Docker robot server.

Use a currently supported Windows version meeting the
[Docker Desktop requirements](https://docs.docker.com/desktop/setup/install/windows-install/).
Enable virtualization in firmware if WSL cannot start. For Linux, install
Docker Engine and the Compose plugin using the
[official Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/).
Verify `docker version` shows both client and server and `docker compose version`
works. Use `sudo docker` on Linux if Docker group membership is not configured.

## Get the project

Use an account with repository access. Do not paste credentials into clone URLs.

```bash
git clone --branch docker https://github.com/MechcodeRobotech/ROS_GPS_CAR.git GPS_Localize
cd GPS_Localize
```

The generated `docker` branch includes the Compose files. `main` is the source
of truth. Developers using `main` can render those files into a staging copy with
`bash tools/publish_branches.sh --render-docker /path/to/staging-copy`; the copy
must also contain `gps_localize_ws/src` and `firmware` before building.

## Linux Wi-Fi

```bash
docker compose up -d --build
docker compose ps
docker compose logs --tail 100
```

Open http://localhost:8080 or `http://<Linux-PC-LAN-IP>:8080` from another device.
The base configuration uses host networking. Stop any previous server/agent
before moving the real robot to this deployment: two stacks on domain 10 and
UDP 8888 conflict. The firmware must use Wi-Fi transport and reach this PC on
UDP 8888. For network configuration, see [installation](install.md).

## Windows Wi-Fi with Docker Desktop

1. Install Git and Docker Desktop. Select Linux containers and the WSL2 backend;
   start Docker Desktop and wait until `docker version` shows a server.
2. Clone the `docker` branch as above, then run in PowerShell from its root:

```powershell
docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d --build
```

3. In Administrator PowerShell, from the same folder:

```powershell
powershell -ExecutionPolicy Bypass -File setup\setup_windows.ps1
```

This configures TCP 8080 and UDP 8888 for Private/Domain networks. Use a trusted
Private network and confirm the PC LAN address using `ipconfig`. Configure the
ESP32's agent address to that LAN address, not a Docker/WSL internal IP. With
Wi-Fi firmware, the USB console command `agent <PC-LAN-IP>` saves the address.
Close the serial monitor before any upload. The console can reset the board.

4. Open http://localhost:8080. From another device use the Windows PC LAN IP.
   Verify live telemetry using the checks below.

All DDS communication stays inside the container. Only browser TCP 8080 and
ESP32 UDP 8888 are published. Docker Desktop offers optional host networking in
recent versions, but this setup does not require it; see
[Docker host networking](https://docs.docker.com/engine/network/drivers/host/).
Do not expect a ROS CLI outside the container to discover its nodes.

## Windows USB through WSL2

This route deliberately runs the Docker daemon in the Ubuntu distribution that
owns the USB device. Enabling Docker Desktop's WSL integration alone does not
prove that its daemon can see Ubuntu's `/dev/ttyUSB0`. Do not mix Docker Desktop
and the local WSL daemon for these commands. Disable Desktop integration for
this Ubuntu distribution, or quit Desktop while using this route.

### 1. Install and start WSL

In Administrator PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
winget install --interactive --exact dorssel.usbipd-win
```

Restart Windows if requested. Open Ubuntu and finish creating its user. Confirm
`wsl -l -v` lists Ubuntu-22.04 as version 2. Keep its terminal open during USB
attachment. Run `wsl --set-version Ubuntu-22.04 2` if conversion is necessary.

### 2. Install Docker Engine inside Ubuntu

In Ubuntu, follow the official Docker Engine Ubuntu installation linked above,
including `docker-compose-plugin`. Do not install ROS on this host.

If `systemctl` reports that systemd is not running, add the following section to
`/etc/wsl.conf`, preserving existing sections:

```ini
[boot]
systemd=true
```

Then run `wsl --shutdown` in PowerShell and reopen Ubuntu. Start Docker:

```bash
sudo systemctl enable --now docker
sudo docker -H unix:///var/run/docker.sock info --format '{{.Name}} {{.OperatingSystem}}'
```

The server must be the local Ubuntu Engine, not Docker Desktop. For the rest of
this guide, define this Bash function in each new Ubuntu terminal:

```bash
dkr() { sudo docker -H unix:///var/run/docker.sock "$@"; }
```

Clone the project under `~/GPS_Localize` in Ubuntu, not `/mnt/c`. Linux storage
avoids Windows filename restrictions encountered by the firmware build tools.

### 3. Attach the USB adapter

Plug the ESP32 into Windows. Close Windows serial monitors and PlatformIO.
In Administrator PowerShell, find its BUSID and share it once (replace `4-4`):

```powershell
usbipd list
usbipd bind --busid 4-4
```

Then in regular PowerShell:

```powershell
usbipd attach --wsl --busid 4-4
usbipd list
```

In Ubuntu:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Use the device that actually appears. The examples below default to
`/dev/ttyUSB0`; `SERIAL_DEVICE=/dev/ttyACM0` selects another adapter. While
attached, the USB device is owned by WSL and unavailable to Windows programs.
After unplug/replug, Windows reboot, or WSL shutdown, repeat attachment and
recreate the container. See [Microsoft's USB guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

### 4. Build and flash serial firmware

Disconnect motor power before flashing. USB flashing does not select runtime
transport automatically: `wifi` and `serial` are separate firmware environments.
The serial agent and a flashing tool cannot own the port simultaneously.

From the clone in Ubuntu, define a Compose helper:

```bash
cd ~/GPS_Localize
export SERIAL_DEVICE=/dev/ttyUSB0
usbcompose() {
  dkr compose -f docker-compose.yml -f docker-compose.windows.yml \
    -f docker-compose.serial.yml -f docker-compose.serial-transport.yml "$@"
}
# sudo may remove environment variables: pass the selected device explicitly.
dkr() { sudo env SERIAL_DEVICE="$SERIAL_DEVICE" docker -H unix:///var/run/docker.sock "$@"; }
usbcompose build
```

Create the local secrets header, required at compile time even for serial:

```bash
cp firmware/config/network_secrets.example.h firmware/config/network_secrets.h
```

Edit it if preparing Wi-Fi credentials. This file is ignored by Git and excluded
from the image. Mount it only for firmware compilation. With any existing USB
server stopped:

```bash
usbcompose down
usbcompose run --rm --no-deps \
  -v "$PWD/firmware/config/network_secrets.h:/firmware/config/network_secrets.h:ro" \
  gps_localize pio run -d /firmware -e serial -t upload --upload-port /dev/ttyUSB0
```

The first firmware build downloads substantial toolchains and needs internet.
Caches persist in named volumes. If the adapter disconnects after flash, repeat
`usbipd attach`, confirm its Linux device path, then start the server:

```bash
usbcompose up -d
usbcompose ps
usbcompose logs --tail 100
usbcompose exec gps_localize /usr/local/bin/docker-entrypoint.sh ros2 node list
```

Expect `/gps_localize_firmware` and fresh telemetry. Open http://localhost:8080
in Windows. WSL localhost forwarding normally exposes the published browser
port; if it does not, run `hostname -I` inside Ubuntu and try its address from
Windows. Access from phones to a WSL server is a separate network step; WSL NAT
does not automatically forward LAN clients. Use WSL mirrored networking where
supported or an explicit TCP forwarding rule following
[Microsoft's WSL networking guide](https://learn.microsoft.com/en-us/windows/wsl/networking).
USB transport does not require ESP32 Wi-Fi or inbound UDP through Windows.

To stop and return USB to Windows:

```bash
usbcompose down
```

```powershell
usbipd detach --busid 4-4
```

Docker Desktop also documents a separate USB/IP procedure for specified
backends: [Docker USB/IP](https://docs.docker.com/desktop/features/usbip/).
That procedure is not substituted for the local WSL Engine route above.

## Linux USB

Use the same flashing procedure and secrets mount, with ordinary Docker Engine.
The Windows networking override is optional on Linux. Define:

```bash
export SERIAL_DEVICE=/dev/ttyUSB0
usbcompose() {
  docker compose -f docker-compose.yml -f docker-compose.serial.yml \
    -f docker-compose.serial-transport.yml "$@"
}
```

Then follow build, flash, and run steps above. To flash **Wi-Fi firmware over
USB**, omit the serial-transport override and use `-e wifi` in the upload command.
After that, the agent must run in its default UDP mode. For OTA, use `wifi_ota`
and the robot IP only after a first USB flash; see [running](running.md).

## Verify and operate

Use the same Compose file options for every command, or define a shell helper
as above. For Docker Desktop PowerShell, this helper avoids repeating options:

```powershell
function gpscompose { docker compose -f docker-compose.yml -f docker-compose.windows.yml @args }
gpscompose ps
gpscompose logs --tail 100
gpscompose exec gps_localize /usr/local/bin/docker-entrypoint.sh ros2 node list
gpscompose exec gps_localize /usr/local/bin/docker-entrypoint.sh ros2 topic echo /gps_localize/telemetry --once --qos-reliability best_effort
```

For Linux, use `docker compose` instead of `gpscompose`; for USB use
`usbcompose`. `docker exec` skips the entrypoint, so bare `ros2` in an exec command
is not sufficient. An interactive `bash` also sources ROS through `.bashrc`.

A healthy HTTP container without a board is expected. A real connection needs
`/gps_localize_firmware`, recent telemetry, and changing uptime. Confirm the UI
is the real server, inspect stop reasons and sensor health, and retain the
browser heartbeat. Follow [operating instructions](running.md) and
[safety checks](safety.md) before commanding motion.

Stop: `docker compose down`. Start again: `docker compose up -d`. Inspect failures:
`docker compose logs --tail 200`. Restart after changing networks:
`docker compose restart`. Substitute your override helper for these commands.

## Updates, persistence, and backup

Stop the robot before updating. Run `git pull --ff-only`, then your Compose
command with `up -d --build`. Do not discard local changes if Git refuses to pull.
The runtime directory `/root/.gps_localize` is a named volume; image rebuilds do
not replace saved settings with new defaults. Back up before changing settings:

```bash
mkdir -p backup
docker compose cp gps_localize:/root/.gps_localize backup/runtime
```

Use the same Compose project name and directory across updates to reuse volumes.
`docker compose down` preserves them; `down -v` deletes them. PlatformIO downloads
and firmware workspaces have separate cache volumes. Restore settings with the
server stopped, using a temporary service container or the Docker volume tools;
never overwrite active runtime files during motion.

Keep the server powered and awake while operating. Docker restart policies do
not start WSL or attach USB after Windows reboot. Start Ubuntu, attach USB, and
run the appropriate Compose command again. Do not treat lid/sleep configuration
on the development PC as configuration already applied to another machine.

## Troubleshooting

| Symptom | Check or fix |
|---|---|
| No compose file | Clone `docker`, or render generated files from a staged `main` copy |
| USB path missing | Attach with usbipd; verify the device on the daemon host; recreate container |
| COM3 rejected | Use the WSL `/dev/ttyUSB*` or `/dev/ttyACM*` path, not a Windows COM name |
| Serial agent alive but no firmware | Flash `serial`, close other port owners, verify 115200 baud |
| Upload cannot open device | Stop serial server and monitors; repeat USB attach if needed |
| Missing `network_secrets.h` | Create from the example and mount it into the one-off build container |
| UI opens, telemetry absent | Check firmware node, transport, domain 10, agent logs and fresh telemetry |
| Windows Wi-Fi robot cannot connect | PC LAN IP, Private firewall profile, UDP 8888, same reachable LAN |
| Port 8080 or 8888 occupied | Stop the older stack or select `WEB_PORT`/`AGENT_PORT` for published ports |
| Published agent port changed | Configure firmware to send to that host port too |
| Bare exec `ros2` fails | Prefix command with `/usr/local/bin/docker-entrypoint.sh` |
| Firmware download fails | Check internet, free disk space, and complete build logs |
| Robot stops despite manual publisher | Read safety stop reasons; browser heartbeat and hardware interlocks still apply |

## Verification status

See [Docker verification record](docker_verification.md). Linux container tests
cannot establish Windows USB driver compatibility, physical attachment, or
robot motion safety. Record those separately on the destination Windows PC.
