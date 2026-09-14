# Docker verification record

Checked on 2026-09-13 on Ubuntu 22.04, Docker Engine 29.8.0, Compose 5.5.1.
This is Linux test evidence, not a Windows hardware certification.

| Check | Result |
|---|---|
| Build generated Dockerfile from current source | PASS; `docker build -t gps_localize:latest .` |
| Base + Windows Compose configuration | PASS; `docker compose ... config -q` |
| Base + Windows + USB + serial runtime merge | PASS; selectable device, serial launch command |
| Isolated server startup | PASS; domain 91, only localhost TCP 18080 exposed, no real USB mapped |
| HTTP `/`, `/app.js`, `/style.css`, `/api/state` | PASS; HTTP 200, nonempty responses |
| Host ROS graph | PASS; gps_odom, safety_watchdog, telemetry_split, waypoint_nav, web_server |
| UDP agent | PASS; startup log confirms port 8888 inside isolated container |
| Typed telemetry to browser API | PASS; synthetic 12.34 V appears in `state.telemetry.battery_v` |
| Image credential exclusion | PASS; local `network_secrets.h` absent from image |
| Runtime volume persistence | PASS; marker survives Compose down/up |
| Serial agent without hardware | PASS; opens Linux pseudo-terminal at 115200 in network-isolated container |
| Serial firmware compilation | PASS; `pio run -e serial`, 114.739 s |
| Windows Desktop runtime | NOT TESTED on Windows |
| Windows usbipd/WSL device attachment | NOT TESTED; Windows PC and adapter required |
| USB hardware telemetry and reconnect | NOT TESTED; follow Windows acceptance checklist |
| Motion | NOT TESTED; attending operator required |

Evidence resides locally under `_work/docker_windows_release/` and
`_work/docker_release_evidence/`. The smoke service is named
`gps_localize_release_smoke`; it uses its own project/volumes and ROS domain 91.
The serial pseudo-terminal probe uses domain 92 and `--network none`. These
tests do not send commands to the real robot or demonstrate a physical serial
micro-ROS session.

Reproduce Compose validation from the generated Docker branch:

```bash
docker compose config -q
docker compose -f docker-compose.yml -f docker-compose.windows.yml config -q
docker compose -f docker-compose.yml -f docker-compose.windows.yml \
  -f docker-compose.serial.yml -f docker-compose.serial-transport.yml config -q
```

Reproduce runtime checks using the [installation guide](docker.md), preserving
its transport overrides. Check the firmware node and fresh telemetry separately
Windows destination and append versions, commands, durations, and exact failures.

The source tree had an existing battery-tier test and documentation using older
thresholds. They now match the existing firmware defaults: warning 11.7 V,
soft stop 11.4 V, cutoff 11.199 V on 3S. No threshold was changed by this task.
The corrected native suite passes all 261 cases. Static port checks pass 11/11;
firmware Wi-Fi build passes in 15.737 s. `python3 tools/qc.py` passes all
cross-file contracts. The final image is
`sha256:747eaa213a2ec00f42eef25a580a09d1084297a530cc6cf9f82ee8b860133f53`.

Git delivery: source snapshot `952257d` and its generated workspace, firmware,
and docker branches were pushed atomically to the existing project remote.
Documentation-only completion records follow that snapshot. Windows acceptance
is tracked as hardware-blocked task T212; T211 covers completed local release work.
The isolated smoke container has been stopped. No real robot upload or motion
was performed during this release task.
