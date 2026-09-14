#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# =============================================================================
#  GPS_Localize - one click setup and run   (Ubuntu 22.04 + ROS 2 Humble)
# =============================================================================
#  Double-click it (or run ./setup_and_run.sh) and it will:
#
#    1. check the machine (Ubuntu version, tools, network)
#    2. check ROS 2 - install ros-humble-ros-base if it is missing
#    3. install the micro-ROS agent and the few packages this project needs
#    4. build the workspace (colcon build --symlink-install)
#    5. check the environment (ROS_DOMAIN_ID, ports, firewall)
#    6. start everything and print the address of the web UI
#
#  Run it a second time and it skips straight to step 6: everything is already
#  installed, so it just starts the robot software.
#
#  Options:
#    --check        only check, install nothing, do not start
#    --reinstall    redo every step even if it was done before
#    --yes          never ask, assume yes (for unattended installs)
#    --no-browser   do not open a browser window
#    --service      also install the systemd autostart service
#    --firmware     also install PlatformIO (needed to flash the ESP32)
#    --help
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/gps_localize_ws"
STATE_DIR="${HOME}/.gps_localize"
MARKER="${STATE_DIR}/.setup_ok"
LOG="${STATE_DIR}/setup.log"

ROS_DISTRO_WANTED="humble"
UBUNTU_WANTED="22.04"
WEB_PORT=8080
AGENT_PORT=8888
DOMAIN_ID=10

OPT_CHECK_ONLY=0
OPT_REINSTALL=0
OPT_YES=0
OPT_BROWSER=1
OPT_SERVICE=0
OPT_FIRMWARE=0

# ------------------------------------------------------------------ pretty --
if [ -t 1 ]; then
    C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'
    C_HEAD=$'\033[1;36m'; C_DIM=$'\033[2m'; C_OFF=$'\033[0m'
else
    C_OK=""; C_WARN=""; C_ERR=""; C_HEAD=""; C_DIM=""; C_OFF=""
fi

step()  { printf '\n%s==> %s%s\n' "$C_HEAD" "$*" "$C_OFF"; log "STEP $*"; }
ok()    { printf '    %s[ ok ]%s %s\n' "$C_OK" "$C_OFF" "$*"; log "OK $*"; }
warn()  { printf '    %s[warn]%s %s\n' "$C_WARN" "$C_OFF" "$*"; log "WARN $*"; }
info()  { printf '    %s%s%s\n' "$C_DIM" "$*" "$C_OFF"; log "INFO $*"; }
fail()  { printf '\n%s[FAIL]%s %s\n' "$C_ERR" "$C_OFF" "$*"; log "FAIL $*"; }

log() { mkdir -p "$STATE_DIR" 2>/dev/null; printf '%s %s\n' "$(date '+%F %T')" "$*" >>"$LOG" 2>/dev/null; }

die() {
    fail "$*"
    printf '\n    Full log: %s\n' "$LOG"
    printf '    Fix the problem above and run this script again.\n\n'
    hold
    exit 1
}

hold() {
    # keep the window open when the script was double-clicked from a file manager
    if [ -t 0 ] && [ "${GPS_LOCALIZE_NO_HOLD:-0}" != "1" ]; then
        read -r -p "Press Enter to close... " _ || true
    fi
}

ask() {
    # ask "question" -> 0 = yes
    [ "$OPT_YES" = "1" ] && return 0
    if [ ! -t 0 ]; then
        warn "no terminal to ask '$1' - use --yes for unattended installs"
        return 1
    fi
    local answer
    read -r -p "    $1 [Y/n] " answer
    case "${answer:-y}" in [yY]|[yY][eE][sS]|"") return 0 ;; *) return 1 ;; esac
}

have() { command -v "$1" >/dev/null 2>&1; }

# ------------------------------------------------------------------- args ---
for arg in "$@"; do
    case "$arg" in
        --check)      OPT_CHECK_ONLY=1 ;;
        --reinstall)  OPT_REINSTALL=1 ;;
        --yes|-y)     OPT_YES=1 ;;
        --no-browser) OPT_BROWSER=0 ;;
        --service)    OPT_SERVICE=1 ;;
        --firmware)   OPT_FIRMWARE=1 ;;
        --help|-h)    awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' \
                          "${BASH_SOURCE[0]}"; exit 0 ;;
        *)            echo "unknown option: $arg (try --help)"; exit 2 ;;
    esac
done

printf '%s\n' "$C_HEAD"
cat <<'BANNER'
  ###########################################################
  #                                                         #
  #     GPS_Localize   -   setup and run                    #
  #     ติดตั้งและเปิดระบบหุ่นยนต์ ด้วยคลิกเดียว                    #
  #                                                         #
  ###########################################################
BANNER
printf '%s' "$C_OFF"
log "===== run $(date) args=$* ====="

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    have sudo || die "sudo is not installed and you are not root"
    SUDO="sudo"
fi

# =============================================================================
# 0. fast path - already installed?
# =============================================================================
quick_ready() {
    [ -f "$MARKER" ] || return 1
    [ -f "/opt/ros/${ROS_DISTRO_WANTED}/setup.bash" ] || return 1
    [ -f "${WS_DIR}/install/setup.bash" ] || return 1
    python3 -c 'import yaml' >/dev/null 2>&1 || return 1
    return 0
}

NEED_INSTALL=1
if [ "$OPT_REINSTALL" = "0" ] && quick_ready; then
    NEED_INSTALL=0
    step "Everything is already installed"
    ok "ROS 2 ${ROS_DISTRO_WANTED}, workspace and dependencies found"
    info "(run with --reinstall to redo the installation)"
fi

# =============================================================================
# 1. machine check
# =============================================================================
if [ "$NEED_INSTALL" = "1" ] || [ "$OPT_CHECK_ONLY" = "1" ]; then
    step "1/6  Checking this machine"

    if [ -r /etc/os-release ]; then
        . /etc/os-release
        info "OS: ${PRETTY_NAME:-unknown}"
        if [ "${ID:-}" != "ubuntu" ]; then
            warn "this project is tested on Ubuntu ${UBUNTU_WANTED}; continuing anyway"
        elif [ "${VERSION_ID:-}" != "$UBUNTU_WANTED" ]; then
            warn "Ubuntu ${VERSION_ID} found, ${UBUNTU_WANTED} is the tested version"
        else
            ok "Ubuntu ${UBUNTU_WANTED}"
        fi
    else
        warn "cannot read /etc/os-release"
    fi

    info "architecture: $(uname -m),  kernel: $(uname -r)"
    have python3 || die "python3 is missing:  sudo apt install python3"
    ok "python3 $(python3 -c 'import platform;print(platform.python_version())')"

    if [ ! -d "$WS_DIR/src/gps_localize" ]; then
        die "cannot find ${WS_DIR}/src/gps_localize - run this script from inside the project folder"
    fi
    ok "project found at ${REPO_ROOT}"
fi

# =============================================================================
# 2. ROS 2
# =============================================================================
detect_ros() {
    if [ -f "/opt/ros/${ROS_DISTRO_WANTED}/setup.bash" ]; then
        FOUND_DISTRO="$ROS_DISTRO_WANTED"
        return 0
    fi
    local candidate
    for candidate in /opt/ros/*/setup.bash; do
        [ -e "$candidate" ] || continue
        FOUND_DISTRO="$(basename "$(dirname "$candidate")")"
        return 0
    done
    return 1
}

install_ros() {
    step "Installing ROS 2 ${ROS_DISTRO_WANTED} (this takes a while, ~1 GB)"
    ask "Install ROS 2 ${ROS_DISTRO_WANTED} now?" || die "ROS 2 is required"

    $SUDO apt-get update -qq || die "apt update failed"
    $SUDO apt-get install -y -qq software-properties-common curl gnupg lsb-release \
        || die "cannot install the basic tools"
    $SUDO add-apt-repository -y universe >/dev/null 2>&1 || true

    if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
        curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | $SUDO gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
            || die "cannot download the ROS signing key (no internet?)"
    fi
    local codename
    codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-jammy}")"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" \
        | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null

    $SUDO apt-get update -qq || die "apt update failed after adding the ROS repository"
    $SUDO apt-get install -y "ros-${ROS_DISTRO_WANTED}-ros-base" \
        || die "installing ros-${ROS_DISTRO_WANTED}-ros-base failed"
    ok "ROS 2 ${ROS_DISTRO_WANTED} installed"
}

FOUND_DISTRO=""
if [ "$NEED_INSTALL" = "1" ] || [ "$OPT_CHECK_ONLY" = "1" ]; then
    step "2/6  Checking ROS 2"
    if detect_ros; then
        if [ "$FOUND_DISTRO" = "$ROS_DISTRO_WANTED" ]; then
            ok "ROS 2 ${FOUND_DISTRO} found at /opt/ros/${FOUND_DISTRO}"
        else
            warn "found ROS 2 ${FOUND_DISTRO}, this project is built for ${ROS_DISTRO_WANTED}"
            if ask "Use ${FOUND_DISTRO} anyway?"; then
                ROS_DISTRO_WANTED="$FOUND_DISTRO"
            else
                [ "$OPT_CHECK_ONLY" = "1" ] || install_ros
            fi
        fi
    else
        warn "no ROS 2 installation found"
        if [ "$OPT_CHECK_ONLY" = "1" ]; then
            info "run without --check to install it"
        else
            install_ros
        fi
    fi
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO_WANTED}/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    if [ "$OPT_CHECK_ONLY" = "1" ]; then
        warn "ROS 2 ${ROS_DISTRO_WANTED} is not installed yet"
        printf '\n%sCheck finished: ROS 2 is missing. Run this script without --check to install it.%s\n\n' \
            "$C_HEAD" "$C_OFF"
        hold
        exit 1
    fi
    die "ROS 2 setup file not found: ${ROS_SETUP}"
fi
# shellcheck disable=SC1090
set +u; . "$ROS_SETUP"; set -u
ok "using ROS 2 ${ROS_DISTRO:-$ROS_DISTRO_WANTED}"

# =============================================================================
# 3. dependencies
# =============================================================================
apt_missing() {
    local package
    for package in "$@"; do
        dpkg -s "$package" >/dev/null 2>&1 || echo "$package"
    done
}

build_agent_from_source() {
    warn "package ros-${ROS_DISTRO_WANTED}-micro-ros-agent not available, building it from source"
    local uros_ws="${HOME}/uros_ws"
    mkdir -p "${uros_ws}/src"
    if [ ! -d "${uros_ws}/src/micro_ros_agent" ]; then
        git clone -b "${ROS_DISTRO_WANTED}" https://github.com/micro-ROS/micro-ROS-Agent.git \
            "${uros_ws}/src/micro_ros_agent" || die "cannot clone the micro-ROS agent"
    fi
    have rosdep || $SUDO apt-get install -y -qq python3-rosdep
    $SUDO rosdep init >/dev/null 2>&1 || true
    rosdep update >/dev/null 2>&1 || true
    ( set +u; . "$ROS_SETUP"; cd "$uros_ws" \
      && rosdep install --from-paths src --ignore-src -y \
      && colcon build ) || die "building the micro-ROS agent failed"
    ok "micro-ROS agent built in ${uros_ws}"
    echo "${uros_ws}/install/setup.bash" > "${STATE_DIR}/.agent_overlay"
}

if [ "$NEED_INSTALL" = "1" ] || [ "$OPT_CHECK_ONLY" = "1" ]; then
    step "3/6  Checking the packages this project needs"

    BASE_PKGS="python3-yaml python3-colcon-common-extensions python3-pip git"
    MISSING="$(apt_missing $BASE_PKGS)"
    if [ -n "$MISSING" ]; then
        info "missing: ${MISSING}"
        if [ "$OPT_CHECK_ONLY" = "1" ]; then
            warn "run without --check to install them"
        else
            ask "Install: ${MISSING}?" || die "these packages are required"
            $SUDO apt-get update -qq
            # shellcheck disable=SC2086
            $SUDO apt-get install -y $MISSING || die "apt install failed"
            ok "packages installed"
        fi
    else
        ok "python3-yaml, colcon, pip, git"
    fi

    AGENT_PKG="ros-${ROS_DISTRO_WANTED}-micro-ros-agent"
    if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
        ok "micro-ROS agent found"
    elif [ "$OPT_CHECK_ONLY" = "1" ]; then
        warn "micro-ROS agent missing (run without --check to install it)"
    elif apt-cache show "$AGENT_PKG" >/dev/null 2>&1; then
        ask "Install ${AGENT_PKG}?" || die "the agent is required to talk to the ESP32"
        $SUDO apt-get install -y "$AGENT_PKG" && ok "micro-ROS agent installed" \
            || build_agent_from_source
    else
        build_agent_from_source
    fi

    if [ "$OPT_FIRMWARE" = "1" ]; then
        if have pio; then
            ok "PlatformIO found ($(pio --version 2>/dev/null))"
        else
            info "installing PlatformIO (for flashing the ESP32)"
            python3 -m pip install --user -q -U platformio || warn "PlatformIO install failed - flash from another PC"
            $SUDO usermod -a -G dialout "$USER" 2>/dev/null || true
            info "log out and back in so the serial port permission takes effect"
        fi
    fi
fi

# =============================================================================
# 4. build the workspace
# =============================================================================
if [ "$OPT_CHECK_ONLY" = "1" ]; then
    step "4/6  Workspace"
    if [ -f "${WS_DIR}/install/setup.bash" ]; then ok "already built"; else warn "not built yet"; fi
elif [ "$NEED_INSTALL" = "1" ] || [ ! -f "${WS_DIR}/install/setup.bash" ]; then
    step "4/6  Building the workspace"
    ( set +u; . "$ROS_SETUP"; cd "$WS_DIR" && colcon build --symlink-install ) \
        || die "colcon build failed - see the messages above and ${LOG}"
    ok "workspace built"
else
    step "4/6  Workspace already built"
    info "rebuild any time with:  cd ${WS_DIR} && colcon build --symlink-install"
fi

[ -f "${WS_DIR}/install/setup.bash" ] || die "workspace build produced no install/setup.bash"
# shellcheck disable=SC1091
set +u
[ -f "${STATE_DIR}/.agent_overlay" ] && . "$(cat "${STATE_DIR}/.agent_overlay")" 2>/dev/null
. "${WS_DIR}/install/setup.bash"
set -u

# =============================================================================
# 5. environment check
# =============================================================================
step "5/6  Checking the environment"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DOMAIN_ID}"
if [ "$ROS_DOMAIN_ID" != "$DOMAIN_ID" ]; then
    warn "ROS_DOMAIN_ID is ${ROS_DOMAIN_ID}, the firmware uses ${DOMAIN_ID} (conf_network.h)"
else
    ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
fi

if ros2 pkg prefix gps_localize >/dev/null 2>&1; then
    ok "package gps_localize is on the path"
else
    die "package gps_localize not found after building"
fi

mkdir -p "$STATE_DIR"
ok "settings folder ${STATE_DIR}"

IP_ADDR="$(hostname -I 2>/dev/null | awk '{print $1}')"
[ -n "$IP_ADDR" ] || IP_ADDR="127.0.0.1"
info "this machine: ${IP_ADDR}"

if have ss; then
    if ss -ltn 2>/dev/null | grep -q ":${WEB_PORT} "; then
        warn "port ${WEB_PORT} is already in use - another copy may be running"
        info "stop it with:  systemctl stop gps_localize   (or close the other terminal)"
    else
        ok "port ${WEB_PORT} is free"
    fi
fi

if have ufw && $SUDO ufw status 2>/dev/null | grep -q "Status: active"; then
    if ! $SUDO ufw status | grep -q "${WEB_PORT}/tcp"; then
        if ask "Firewall is on. Open port ${WEB_PORT}/tcp (web UI) and ${AGENT_PORT}/udp (ESP32)?"; then
            $SUDO ufw allow ${WEB_PORT}/tcp >/dev/null && $SUDO ufw allow ${AGENT_PORT}/udp >/dev/null
            ok "firewall rules added"
        else
            warn "phones and tablets will not be able to open the web UI"
        fi
    else
        ok "firewall already allows ${WEB_PORT}/tcp"
    fi
fi

if [ "$OPT_SERVICE" = "1" ]; then
    SERVICE_SRC="${WS_DIR}/src/gps_localize/systemd/gps_localize.service"
    step "Installing the autostart service"
    sed -e "s|^User=.*|User=${USER}|" \
        -e "s|^WorkingDirectory=.*|WorkingDirectory=${HOME}|" \
        -e "s|Environment=ROS_DOMAIN_ID=.*|Environment=ROS_DOMAIN_ID=${DOMAIN_ID}|" \
        -e "s|ExecStart=.*|ExecStart=/bin/bash -lc 'source ${ROS_SETUP} \&\& source ${WS_DIR}/install/setup.bash \&\& ros2 launch gps_localize bringup.launch.py'|" \
        "$SERVICE_SRC" | $SUDO tee /etc/systemd/system/gps_localize.service >/dev/null
    $SUDO systemctl daemon-reload
    $SUDO systemctl enable gps_localize >/dev/null 2>&1
    ok "installed - it will start automatically on boot"
    info "control it with:  sudo systemctl start|stop|status gps_localize"
fi

{
    echo "setup_ok $(date '+%F %T')"
    echo "ros_distro ${ROS_DISTRO_WANTED}"
    echo "workspace ${WS_DIR}"
} > "$MARKER"

if [ "$OPT_CHECK_ONLY" = "1" ]; then
    printf '\n%sCheck finished. Nothing was installed or started.%s\n\n' "$C_HEAD" "$C_OFF"
    hold
    exit 0
fi

# =============================================================================
# 6. run
# =============================================================================
step "6/6  Starting GPS_Localize"

cat <<EOF

    ${C_OK}เปิดหน้าเว็บควบคุมที่ / open the control page at${C_OFF}

        ${C_HEAD}http://${IP_ADDR}:${WEB_PORT}${C_OFF}

    มือถือ แท็บเล็ต หรือคอมเครื่องอื่นใน Wi-Fi เดียวกัน เปิดที่อยู่นี้ได้เลย
    Any phone, tablet or laptop on the same Wi-Fi can open that address.

    หยุดโปรแกรม: กด Ctrl+C ในหน้าต่างนี้ / press Ctrl+C here to stop
    บันทึกการติดตั้ง / setup log: ${LOG}

EOF

if [ "$OPT_BROWSER" = "1" ] && have xdg-open && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    ( sleep 4; xdg-open "http://localhost:${WEB_PORT}" >/dev/null 2>&1 ) &
fi

trap 'printf "\n%sstopped%s\n" "$C_HEAD" "$C_OFF"; exit 0' INT TERM
exec ros2 launch gps_localize bringup.launch.py domain_id:="${ROS_DOMAIN_ID}"
