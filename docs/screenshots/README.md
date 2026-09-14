# Screenshot coverage and reproduction

These images accompany [the step-by-step guide](../step_by_step.md) and
[Wi-Fi setup](../wifi_setup.md). Open any PNG at full size to read small fields.

## What was captured

| Coverage | Files | Provenance |
|---|---|---|
| All 10 top-level UI tabs | `01-dash.png` through `10-help.png` | Actual repository HTML/CSS/JS, isolated sample API |
| All 7 server Settings groups | `settings-*.png` | Same isolated application |
| All 9 board parameter groups | `robot-*.png` | Same application; values derived from source defaults where possible |
| Waypoint row editor and topic watch | `waypoints-add-row.png`, `topics-watch.png` | Local UI interactions; no real writes or motion |
| Native package discovery, launch options, installer preview | `setup-*.png` | Actual read-only command output on existing Ubuntu installation |
| USB Wi-Fi command help and first-boot header | `wifi-*.png` | Source reference rendered in a terminal; not a board session |

Total: **33 PNGs**. The yellow banner visibly distinguishes example UI data from
real hardware measurements. External network requests, including map tiles,
were blocked during browser capture. Only `/api/heartbeat` POST requests were
accepted; they reached the local fixture, not the real server. The browser
manifest records source hashes, capture names, exceptions and POST paths.
The terminal manifest records displayed text, commands and exit codes.

These are screen captures. Terminal references
are labelled because the source help text was displayed without connecting to a
board. No private `network_secrets.h` is read by the capture tools.

## Screens and acceptance still required

The following need the actual destination machine/operator. Do not replace them
with invented screenshots or claim their outcomes from this Linux session.

| Destination | Capture in order | Record with each image |
|---|---|---|
| Windows installation | Windows version; WSL install/restart; Ubuntu first-user setup; `wsl -l -v`; Docker Desktop WSL integration if using Docker | OS/app versions, command, actual completion/error |
| Windows network settings | Settings → Network & internet → Wi-Fi; chosen router; Mobile hotspot properties with 2.4 GHz if used | Exact click path; hide password and personal network details |
| Windows USB | usbipd install, list, bind, attach; Linux device appearing; Docker device visibility if used | Actual adapter/BUSID mapping, permissions, reconnect after unplug/reboot |
| Windows server | Compose or native launch; browser at localhost; firmware node and fresh telemetry | Chosen transport, firewall/network mode, ports, actual successful route |
| Ubuntu Wi-Fi OS screen | Settings → Wi-Fi with an installed adapter | Actual connection steps and version-specific labels |
| Fresh Ubuntu installation | ROS repository/package install, agent build, workspace build | Full command logs; the existing-package screenshot is not a clean install |
| Attended robot setup | Successful flash/boot, saved Wi-Fi, reconnect, sensor checks and calibration | Operator/power conditions, real acknowledgements and measured results |

Track Windows physical acceptance under T212 and remaining installation/hardware
captures under T214 in `tools/plan_state.json`. Keep instructions usable while
these images remain unavailable. Redact secrets before adding any real capture.

## Reproduce browser captures

Use a full `main` checkout, because the capture reads both firmware defaults and
web sources. Capture tools are optional documentation tools, not robot runtime
dependencies. Install them in a local environment:

```bash
python3 -m venv _work/guide-venv
_work/guide-venv/bin/pip install playwright pyyaml
_work/guide-venv/bin/python -m playwright install chromium
_work/guide-venv/bin/python tools/capture_guide.py --root . --out _work/guide-images
```

Inspect the output and `manifest.json`, then promote accepted images using the
project patch workflow. Re-run when the UI changes. Do not point this harness at
a running robot server; it deliberately serves its own isolated backend.

On the current development machine, dependencies already exist under
`_work/screenshot_guide_evidence/python` and `.../browsers`; see HANDOFF for the
command using those paths. Do not reinstall them just to resume this task.

## Reproduce terminal captures

On an Ubuntu desktop with GTK 3/VTE Python bindings already available:

```bash
python3 tools/capture_terminal_guide.py --root . --out _work/guide-terminal-images
```

This requires the existing ROS and agent installation for successful package
checks. It uses an offscreen VTE widget and does not take over the desktop or
open USB. The installer runs only with `--dry-run`; launch uses `--show-args`.
Inspect `terminal-manifest.json`: nonzero exit codes are failures to document,
not successful installation evidence. The current captures returned zero for
all three command checks. GTK emitted an offscreen drawable warning but saved
the inspected images successfully.
