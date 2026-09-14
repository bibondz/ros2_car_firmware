# Documentation map

Start with the [illustrated step-by-step guide](step_by_step.md) for screenshots of each UI tab and the [Wi-Fi setup walkthrough](wifi_setup.md).

Start with the installation route matching your machine and connection.

| Need | Guide |
|---|---|
| Install without Docker on Ubuntu or Windows WSL2, including USB | [Native guide](native.md) |
| Choose a Docker deployment | [Docker installation matrix](docker.md#choose-an-installation) |
| Install ROS 2 and run directly on Ubuntu | [Native installation](install.md), [setup launcher](../setup/README.md) |
| Docker installation, USB attachment, flashing, start/stop, backup | [Docker guide](docker.md) |
| Current Docker test evidence and untested hardware paths | [Verification record](docker_verification.md) |
| Everyday operation and command examples | [Running](running.md) |
| UI tabs, configuration, waypoint controls | [Web UI](web_ui.md) |
| Wiring, pin assignments, power, sensors | [Hardware](hardware.md) |
| Components and data flow | [Architecture](architecture.md) |
| Message types, units, topics, HTTP endpoints | [Topics and API](topics.md) |
| Interlocks and attended tests | [Safety](safety.md), [acceptance](acceptance.md) |
| Calibration and control parameters | [Calibration](calibration.md), [tuning](tuning.md) |
| Offline viewer and executable build | [Viewer README](../webapp/README.md) |

`main` contains editable sources. `workspace`, `firmware`, and `docker` are
mechanically generated subsets. An ignored local viewer executable, CAD export,
Wi-Fi secret, or build cache is not part of a fresh clone. Follow the build
instructions rather than assuming those development-machine files are present.
