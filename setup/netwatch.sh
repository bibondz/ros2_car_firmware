#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# Restart the ROS stack when this machine's network address changes.
#
# WHY THIS HAS TO EXIST
#
# DDS binds its interfaces when a node starts. Change the machine's address
# afterwards - move to another Wi-Fi, join a hotspot, plug in a cable - and every
# running node keeps advertising the address it had at start-up. Nothing errors.
# `ros2 node list` comes back empty, subscriber counts drop to zero, and the
# topics are all still there, so everything that is easy to check says the system
# is fine.
#
# It has already happened here for real: the board was linked to the agent and
# publishing telemetry that `ros2 topic echo` could read, while the web UI sat on
# data seven minutes old. The fix is a restart, and there is no reason a person
# should have to know that.
#
# WHY `ip monitor` RATHER THAN POLLING
#
# Address changes arrive as events, so the stack is back within a second or two
# instead of up to a polling interval. It also needs no NetworkManager, so this
# works the same on a machine using systemd-networkd, dhcpcd or anything else.
#
# WHAT IT DELIBERATELY DOES NOT DO
#
# It restarts on a change of ADDRESS, not on every network event. Wi-Fi generates
# a lot of noise - renewals, IPv6 privacy addresses coming and going, link
# flaps - and restarting the stack on each one would be worse than the problem,
# since a restart drops the micro-ROS link and the robot briefly loses its
# heartbeat.
#
#   ./netwatch.sh                    # watch, restart gps_localize on change
#   ./netwatch.sh --dry-run          # report changes without restarting
set -uo pipefail

UNIT="gps_localize"
DEBOUNCE=3
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --debounce) DEBOUNCE="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) sed -n '2,34p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# The address other machines would reach us on. Not the interface list: a
# machine can hold several addresses at once and only one of them is the route
# out, which is the one DDS will have bound.
primary_ip() {
    ip -4 route get 1.1.1.1 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}'
}

restart_stack() {
    local from="$1" to="$2"
    echo "[netwatch] address changed: ${from:-none} -> ${to:-none}"
    if [ "$DRY_RUN" = 1 ]; then
        echo "[netwatch] --dry-run, not restarting $UNIT"
        return
    fi
    if ! systemctl is-enabled --quiet "$UNIT" 2>/dev/null \
       && ! systemctl is-active --quiet "$UNIT" 2>/dev/null; then
        echo "[netwatch] $UNIT is not installed here, nothing to restart"
        return
    fi
    echo "[netwatch] restarting $UNIT so DDS rebinds to $to"
    systemctl restart "$UNIT" \
        && echo "[netwatch] $UNIT restarted" \
        || echo "[netwatch] could not restart $UNIT" >&2
}

# The address DDS is bound to, which is what every comparison below is
# against - not whatever transient value an event happens to report.
bound="$(primary_ip)"
echo "[netwatch] watching for address changes; bound to ${bound:-none}"

# `ip monitor` streams for as long as it runs, so this loop only turns over when
# something actually happened on an interface.
ip -4 monitor address 2>/dev/null | while read -r _; do
    latest="$(primary_ip)"
    [ "$latest" = "$bound" ] && continue

    # Something moved. Wait for it to settle before acting: joining a network
    # produces a burst of events, and the first address handed out is often not
    # the final one.
    #
    # The comparison that matters is against the address DDS is actually BOUND
    # to, not against the intermediate value that triggered this pass. An
    # earlier version compared to the intermediate, so a change that went
    # 192.168.100.16 -> none -> 10.126.95.220 was dismissed as "still settling"
    # and waited for a next event that never came - leaving the stack orphaned,
    # which is the exact failure this script exists to prevent. Caught by
    # reading its own log after a real network switch.
    sleep "$DEBOUNCE"
    settled="$(primary_ip)"

    if [ -z "$settled" ]; then
        # No route out yet. Do nothing: the event that brings one back will
        # arrive, and restarting into no network would achieve nothing.
        continue
    fi
    if [ "$settled" = "$bound" ]; then
        # A blip that came back to the same address. DDS is still bound
        # correctly, so a restart would drop the robot's link for nothing.
        continue
    fi

    restart_stack "$bound" "$settled"
    bound="$settled"
done
