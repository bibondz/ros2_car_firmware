#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# GPS_Localize - one file, installs everything.
#
#   ./setup/install.sh                  pick the best mode for this machine
#   ./setup/install.sh --docker         containers, no ROS on the host
#   ./setup/install.sh --native         ROS 2 Humble installed on the host
#   ./setup/install.sh --dry-run        print every command, change nothing
#   ./setup/install.sh --check          report what is installed, change nothing
#   ./setup/install.sh --uninstall      remove the service and the ROS build
#
# Or straight from GitHub:
#   THE REPOSITORY IS PRIVATE, so the bare one-liner below returns 404 for
#   anyone who is not authenticated - including the machine you are installing
#   onto. Verified in a clean container: curl (22) error 404.
#
#   With the GitHub CLI already logged in:
#       gh repo clone MechcodeRobotech/ROS_GPS_CAR -- -b workspace ~/GPS_Localize
#       bash ~/GPS_Localize/setup/install.sh
#
#   Or with a personal access token:
#       curl -fsSL -H "Authorization: token $GITHUB_TOKEN" \
#           https://raw.githubusercontent.com/MechcodeRobotech/ROS_GPS_CAR/workspace/setup/install.sh | bash
#
#   Or copy the checkout across by any means and run setup/install.sh from it -
#   the installer itself needs nothing from GitHub.
#
#   Once the repository is public this works as written:
#       curl -fsSL https://raw.githubusercontent.com/MechcodeRobotech/ROS_GPS_CAR/workspace/setup/install.sh | bash
#
# WHY --dry-run EXISTS
# Most of this needs root, and a script that can only be exercised by running
# it for real as root is a script nobody checks until it breaks on someone's
# machine. --dry-run walks the identical decision tree and prints exactly what
# would run, so the logic is testable without touching the system. --check
# does the same for the detection half.
#
# Safe to run repeatedly: every step is skipped when it is already done.
set -uo pipefail

REPO_URL="https://github.com/MechcodeRobotech/ROS_GPS_CAR.git"
ROS_DISTRO_WANT="humble"
UBUNTU_WANT="22.04"
MODE="auto"
DRY_RUN=0
CHECK_ONLY=0
UNINSTALL=0
ASSUME_YES=0

for arg in "$@"; do
    case "$arg" in
        --docker)    MODE="docker" ;;
        --native)    MODE="native" ;;
        --dry-run)   DRY_RUN=1 ;;
        --check)     CHECK_ONLY=1 ;;
        --uninstall) UNINSTALL=1 ;;
        -y|--yes)    ASSUME_YES=1 ;;
        -h|--help)   sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $arg  (try --help)" >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------- output ----
if [ -t 1 ]; then
    C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_BAD=$'\033[31m'
    C_DIM=$'\033[2m'; C_HEAD=$'\033[36m'; C_OFF=$'\033[0m'
else
    C_OK=; C_WARN=; C_BAD=; C_DIM=; C_HEAD=; C_OFF=
fi
say()  { printf '  %s\n' "$*"; }
ok()   { printf '  %sOK%s    %s\n'   "$C_OK"   "$C_OFF" "$*"; }
warn() { printf '  %swarn%s  %s\n'   "$C_WARN" "$C_OFF" "$*"; }
bad()  { printf '  %sFAIL%s  %s\n'   "$C_BAD"  "$C_OFF" "$*"; }
head() { printf '\n%s[%s]%s\n' "$C_HEAD" "$*" "$C_OFF"; }

FAILED=0
fail() { bad "$*"; FAILED=1; }

# Run a command, or print it under --dry-run. Everything that changes the
# system goes through here, which is what makes the dry run trustworthy.
run() {
    if [ "$DRY_RUN" = 1 ]; then
        printf '  %swould run:%s %s\n' "$C_DIM" "$C_OFF" "$*"
        return 0
    fi
    "$@"
}

need_root() {
    if [ "$(id -u)" = 0 ]; then echo ""; return; fi
    if command -v sudo >/dev/null 2>&1; then echo "sudo"; return; fi
    echo ""
}
SUDO="$(need_root)"

# --------------------------------------------------------------- detect -----
detect() {
    head "this machine"
    OS_ID=""; OS_VER=""
    if [ -r /etc/os-release ]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        OS_ID="${ID:-}"; OS_VER="${VERSION_ID:-}"
    fi
    say "os          : ${PRETTY_NAME:-unknown}"
    say "arch        : $(uname -m)"
    say "user        : $(id -un)"

    HAVE_ROS=0
    if [ -d "/opt/ros/$ROS_DISTRO_WANT" ]; then HAVE_ROS=1; ok "ROS 2 $ROS_DISTRO_WANT present"
    else say "ROS 2 $ROS_DISTRO_WANT : not installed"; fi

    HAVE_DOCKER=0
    if command -v docker >/dev/null 2>&1; then
        HAVE_DOCKER=1
        if docker info >/dev/null 2>&1; then ok "docker present and running"
        else warn "docker present but not running (or you are not in the docker group)"; fi
    else
        say "docker      : not installed"
    fi

    HAVE_PIO=0
    command -v pio >/dev/null 2>&1 && { HAVE_PIO=1; ok "platformio present"; } || \
        say "platformio  : not installed (only needed to build firmware)"

    CAN_ROOT=0
    if [ "$(id -u)" = 0 ]; then CAN_ROOT=1
    elif [ -n "$SUDO" ] && sudo -n true 2>/dev/null; then CAN_ROOT=1; ok "passwordless sudo"
    elif [ -n "$SUDO" ]; then CAN_ROOT=2; say "sudo        : available, will ask for your password"
    fi
    [ "$CAN_ROOT" = 0 ] && warn "no way to become root - only --check and --dry-run will work"

    if [ "$OS_ID" != "ubuntu" ]; then
        warn "this installer targets Ubuntu $UBUNTU_WANT; on $OS_ID use --docker"
    elif [ "$OS_VER" != "$UBUNTU_WANT" ]; then
        warn "Ubuntu $OS_VER, not $UBUNTU_WANT - ROS 2 $ROS_DISTRO_WANT expects $UBUNTU_WANT; --docker avoids that"
    fi
}

choose_mode() {
    [ "$MODE" != "auto" ] && return
    if [ "$HAVE_ROS" = 1 ]; then MODE="native"
    elif [ "$HAVE_DOCKER" = 1 ]; then MODE="docker"
    elif [ "$OS_ID" = "ubuntu" ] && [ "$OS_VER" = "$UBUNTU_WANT" ]; then MODE="native"
    else MODE="docker"; fi
    say "chosen mode : $MODE"
}

# ---------------------------------------------------------------- native ----
install_native() {
    head "ROS 2 $ROS_DISTRO_WANT"
    if [ "$HAVE_ROS" = 1 ]; then
        ok "already installed, nothing to do"
    else
        say "installing ros-$ROS_DISTRO_WANT-ros-base"
        run $SUDO apt-get update -qq
        run $SUDO apt-get install -y -qq software-properties-common curl gnupg lsb-release
        run $SUDO add-apt-repository -y universe
        run bash -c "curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | $SUDO gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg"
        run bash -c "echo \"deb [arch=\$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" \
            | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null"
        run $SUDO apt-get update -qq
        run $SUDO apt-get install -y ros-$ROS_DISTRO_WANT-ros-base \
            python3-colcon-common-extensions python3-rosdep python3-pip
    fi

    head "micro-ROS agent"
    if command -v MicroXRCEAgent >/dev/null 2>&1 || \
       [ -x "$HOME/.local/bin/MicroXRCEAgent" ] || \
       [ -d "$HOME/uros_ws/install/micro_ros_agent" ]; then
        ok "already available"
    else
        # No apt package exists for Humble. Building Micro-XRCE-DDS-Agent
        # directly does not work either: its superbuild pins a FastDDS tag
        # that no longer exists upstream, and without the superbuild it needs
        # microxrcedds_client, which ROS does not ship. micro_ros_setup is the
        # supported route and resolves those versions itself.
        say "building the agent with micro_ros_setup (no package exists for $ROS_DISTRO_WANT)"
        run $SUDO apt-get install -y -qq build-essential cmake git python3-rosdep
        run mkdir -p "$HOME/uros_ws/src"
        run git clone -b "$ROS_DISTRO_WANT" --depth 1 \
            https://github.com/micro-ROS/micro_ros_setup.git \
            "$HOME/uros_ws/src/micro_ros_setup"
        run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
            cd '$HOME/uros_ws' && \
            rosdep install --from-paths src --ignore-src -y && \
            colcon build && \
            source install/local_setup.bash && \
            ros2 run micro_ros_setup create_agent_ws.sh && \
            ros2 run micro_ros_setup build_agent.sh"
        ok "agent built in ~/uros_ws - add it to your shell with:"
        say "  echo 'source ~/uros_ws/install/local_setup.sh' >> ~/.bashrc"
    fi

    head "workspace"
    if [ ! -d "$WS/src" ]; then
        fail "no workspace at $WS - run this from inside the project, or clone it first"
        return
    fi
    # A checkout COPIED from another machine - over a network share, on a USB
    # stick, out of a backup - brings build/ and install/ with it, and those
    # hold absolute paths from the machine that made them. colcon then stops
    # with "CMakeCache.txt directory is different than", naming a path that does
    # not exist here, and the install dies at a step that had nothing to do with
    # the user. Found by installing a copied tree in a clean container.
    #
    # Only stale caches are removed, and only when they really do point
    # somewhere else, so a normal rebuild keeps its cache and stays fast.
    local cache="$WS/build/gps_localize_msgs/CMakeCache.txt"
    if [ -f "$cache" ] && ! grep -q "CMAKE_HOME_DIRECTORY:INTERNAL=$WS/src/gps_localize_msgs" "$cache" 2>/dev/null; then
        warn "this workspace was built on another machine - clearing build/ and install/"
        run rm -rf "$WS/build" "$WS/install" "$WS/log"
    fi

    run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
        cd '$WS' && rosdep install --from-paths src --ignore-src -y \
          --skip-keys micro_ros_agent 2>/dev/null || true"
    run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
        cd '$WS' && colcon build --symlink-install --parallel-workers \$(nproc)"
    ok "workspace built"

    head "service"
    # Two units, installed together and in this order. The agent is separate so
    # that stopping the stack to work on it does not also stop the thing the
    # board is trying to reach - the board would then retry against a host that
    # looks perfectly healthy to whoever is standing at it.
    local agent_unit="$WS/src/gps_localize/systemd/micro_ros_agent.service"
    if [ -f "$agent_unit" ]; then
        run bash -c "sed -e 's|__USER__|$(id -un)|g' -e 's|__PROJECT__|$ROOT|g' \
            '$agent_unit' | $SUDO tee /etc/systemd/system/micro_ros_agent.service >/dev/null"
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now micro_ros_agent
        ok "micro_ros_agent service enabled - UDP 8888, starts on boot"
    else
        warn "no micro-ROS agent unit found - the board will have nothing to connect to"
    fi

    local unit="$WS/src/gps_localize/systemd/gps_localize.service"
    if [ -f "$unit" ]; then
        # The unit in git is a template. Copying it verbatim would install one
        # machine's user and home path onto every other machine, which is wrong
        # everywhere but here.
        run bash -c "sed -e 's|__USER__|$(id -un)|g' -e 's|__PROJECT__|$ROOT|g' \
            '$unit' | $SUDO tee /etc/systemd/system/gps_localize.service >/dev/null"
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now gps_localize
        ok "gps_localize service enabled - it will start on boot"
    else
        warn "no systemd unit found, skipping autostart"
    fi

    # The web UI's name on the network. Without it the only way in is the PC's
    # IP address, which is a DHCP lease and moves - the same trap that left the
    # firmware dialling an agent that had changed address.
    local mdns="$WS/src/gps_localize/systemd/gps_localize_mdns.service"
    if [ -f "$mdns" ] && command -v avahi-publish-address >/dev/null 2>&1; then
        run bash -c "sed -e 's|__USER__|$(id -un)|g' -e 's|__PROJECT__|$ROOT|g' \
            '$mdns' | $SUDO tee /etc/systemd/system/gps_localize_mdns.service >/dev/null"
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now gps_localize_mdns
        ok "web UI reachable at http://gps-robot-web.local:8080"
    elif [ -f "$mdns" ]; then
        warn "avahi-utils is missing, so gps-robot-web.local will not resolve"
        warn "  sudo apt install avahi-utils avahi-daemon"
    fi

    # Announces where the agent is, so the ROBOT never has to be told. The
    # counterpart to netwatch below: netwatch fixes this machine after an
    # address change, the beacon fixes the board. Without it a board whose
    # stored agent address has gone stale has no way back except a USB cable,
    # which is not an option once the robot is out in a field.
    local beacon="$WS/src/gps_localize/systemd/gps_localize_beacon.service"
    if [ -f "$beacon" ]; then
        run bash -c "sed -e 's|__USER__|$(id -un)|g' -e 's|__PROJECT__|$ROOT|g' \
            '$beacon' | $SUDO tee /etc/systemd/system/gps_localize_beacon.service >/dev/null"
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now gps_localize_beacon
        ok "agent beacon announcing on UDP 8889 - the robot finds us by itself"
    fi

    # Restarts the stack when this machine's address changes. DDS binds its
    # interfaces at node start-up, so without this a change of network leaves
    # every node advertising an address that no longer reaches it - and nothing
    # reports an error.
    local netwatch="$WS/src/gps_localize/systemd/gps_localize_netwatch.service"
    if [ -f "$netwatch" ]; then
        run bash -c "sed -e 's|__USER__|$(id -un)|g' -e 's|__PROJECT__|$ROOT|g' \
            '$netwatch' | $SUDO tee /etc/systemd/system/gps_localize_netwatch.service >/dev/null"
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now gps_localize_netwatch
        ok "the stack will follow this machine onto other networks by itself"
    fi
}

# ---------------------------------------------------------------- docker ----
install_docker() {
    head "docker"
    if [ "$HAVE_DOCKER" = 1 ]; then
        ok "already installed"
    else
        say "installing docker"
        run bash -c "curl -fsSL https://get.docker.com | $SUDO sh"
        run $SUDO usermod -aG docker "$(id -un)"
        warn "you were added to the docker group - log out and back in for it to apply"
    fi

    head "stack"
    if [ ! -f "$ROOT/docker-compose.yml" ]; then
        fail "no docker-compose.yml here - this is not the docker branch"
        say  "clone it:  git clone --branch docker $REPO_URL"
        return
    fi
    local files=(-f "$ROOT/docker-compose.yml")
    if [ "$(uname -s)" != "Linux" ] && [ -f "$ROOT/docker-compose.windows.yml" ]; then
        files+=(-f "$ROOT/docker-compose.windows.yml")
        say "not Linux: adding the override, host networking does not exist here"
    fi
    run docker compose "${files[@]}" up -d --build
    ok "stack started"
}

# ------------------------------------------------------------- uninstall ----
do_uninstall() {
    head "removing"
    if systemctl list-unit-files 2>/dev/null | grep -q '^gps_localize'; then
        run $SUDO systemctl disable --now gps_localize
        run $SUDO systemctl disable --now gps_localize_mdns 2>/dev/null || true
        run $SUDO systemctl disable --now gps_localize_netwatch 2>/dev/null || true
        run $SUDO systemctl disable --now micro_ros_agent 2>/dev/null || true
        run $SUDO rm -f /etc/systemd/system/gps_localize.service \
                        /etc/systemd/system/gps_localize_mdns.service \
                        /etc/systemd/system/gps_localize_netwatch.service \
                        /etc/systemd/system/micro_ros_agent.service
        run $SUDO systemctl daemon-reload
        ok "service removed"
    else
        say "service not installed"
    fi
    if [ -f "$ROOT/docker-compose.yml" ] && command -v docker >/dev/null 2>&1; then
        run docker compose -f "$ROOT/docker-compose.yml" down
        ok "containers stopped"
    fi
    [ -d "$WS/build" ] && { run rm -rf "$WS/build" "$WS/install" "$WS/log"; ok "ROS build removed"; }
    say "ROS 2 and Docker themselves are left alone - remove them with apt if you want them gone"
}

# ------------------------------------------------------------------ main ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$ROOT/gps_localize_ws"

printf '\n%sGPS_Localize installer%s\n' "$C_HEAD" "$C_OFF"
printf '%s\n' "============================================================"
[ "$DRY_RUN" = 1 ] && warn "dry run: nothing will be changed"

detect

if [ "$CHECK_ONLY" = 1 ]; then
    head "result"
    say "nothing was changed (--check)"
    exit 0
fi

if [ "$UNINSTALL" = 1 ]; then
    do_uninstall
    exit $FAILED
fi

if [ "$CAN_ROOT" = 0 ] && [ "$DRY_RUN" = 0 ]; then
    head "cannot continue"
    fail "installing needs root, and neither root nor sudo is usable here"
    say  "run it on a normal terminal, or use --dry-run to see what it would do"
    exit 1
fi

choose_mode
case "$MODE" in
    native) install_native ;;
    docker) install_docker ;;
    *) fail "unknown mode: $MODE"; exit 2 ;;
esac

head "done"
if [ "$FAILED" = 0 ]; then
    IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    ok "install finished"
    say ""
    say "web UI : http://${IP:-localhost}:8080"
    [ "$MODE" = docker ] && say "topics : docker compose exec gps_localize ros2 topic list"
    [ "$MODE" = native ] && say "topics : ros2 topic list      (ROS_DOMAIN_ID=10)"
else
    bad "something failed above"
fi
exit $FAILED
