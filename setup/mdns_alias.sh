#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# Publish a fixed name for the web UI on the local network.
#
# Without this the only way to reach the robot is to know the PC's IP address,
# which is a DHCP lease and therefore changes - that is exactly how AGENT_IP in
# the firmware ended up pointing at a machine that had moved. A name does not
# move. Anyone on the same network opens http://gps-robot-web.local:8080 and
# gets the robot, whatever address it happens to hold today.
#
# HYPHENS, NOT UNDERSCORES
#
# gps_robot_web.local would be the obvious spelling and it does not work.
# RFC 1123 does not allow an underscore in a hostname; avahi will publish it
# quite happily, and then browsers and system resolvers refuse to look it up.
# The name would appear correct everywhere except where someone types it.
#
# The address is resolved at start and re-checked, because a laptop that moves
# between networks gets a new one and the published record has to follow.
#
#   ./mdns_alias.sh                 # publish, then hold the name until killed
#   ./mdns_alias.sh --name foo      # publish a different name
#
# Runs as gps_localize_mdns.service. avahi-publish holds the record only while
# it is running, which is the behaviour we want: if the machine is off, the name
# should stop resolving rather than point somewhere stale.
set -uo pipefail

NAME="gps-robot-web.local"
INTERVAL=30

while [ $# -gt 0 ]; do
    case "$1" in
        --name) NAME="$2"; shift 2 ;;
        --interval) INTERVAL="$2"; shift 2 ;;
        -h|--help) sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if ! command -v avahi-publish-address >/dev/null 2>&1; then
    echo "avahi-publish-address is missing - install avahi-utils:" >&2
    echo "    sudo apt install avahi-utils avahi-daemon" >&2
    exit 1
fi

# The address a machine on the LAN would reach us on. Deliberately not
# 127.0.0.1: publishing loopback would make the name resolve for everyone and
# work for nobody.
current_ip() {
    ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}'
}

publisher_pid=""
service_pid=""
published_ip=""

# A browser typing http://name.local ALWAYS uses port 80 - there is no way to
# make a bare hostname imply another port, SRV record or not. This service
# record is therefore not a substitute for binding 80; it is what makes the
# robot appear correctly in service discovery (avahi-browse, Bonjour browsers,
# phone apps), carrying whichever port the UI is really on.
WEB_PORT="${WEB_PORT:-8080}"

cleanup() {
    [ -n "$publisher_pid" ] && kill "$publisher_pid" 2>/dev/null
    [ -n "$service_pid" ] && kill "$service_pid" 2>/dev/null
    exit 0
}
trap cleanup INT TERM

while true; do
    ip_now="$(current_ip)"

    if [ -z "$ip_now" ]; then
        # No route out. Drop the record rather than keep advertising an address
        # that no longer reaches anything.
        if [ -n "$publisher_pid" ]; then
            echo "[mdns] no network - withdrawing $NAME"
            kill "$publisher_pid" 2>/dev/null
            publisher_pid=""
            published_ip=""
        fi
    elif [ "$ip_now" != "$published_ip" ] || ! kill -0 "$publisher_pid" 2>/dev/null; then
        [ -n "$publisher_pid" ] && kill "$publisher_pid" 2>/dev/null
        echo "[mdns] publishing $NAME -> $ip_now"
        avahi-publish-address -R "$NAME" "$ip_now" &
        publisher_pid=$!
        published_ip="$ip_now"

        # Advertise the UI as a discoverable HTTP service as well, so it can be
        # found without knowing the name at all.
        [ -n "$service_pid" ] && kill "$service_pid" 2>/dev/null
        if command -v avahi-publish-service >/dev/null 2>&1; then
            avahi-publish-service "GPS_Localize robot" _http._tcp "$WEB_PORT" \
                "path=/" >/dev/null 2>&1 &
            service_pid=$!
        fi
    fi

    sleep "$INTERVAL"
done
