#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Drive the running stack with synthetic firmware messages, and see what breaks.

Every node that subscribes to something the ESP32 publishes has a code path that
only runs when a robot is attached. On a bench with no hardware those callbacks
never fire, so they can be arbitrarily broken while every other check reports a
clean result: the package builds, the nodes start, the web UI serves, and the
first real telemetry frame then kills three of them.

That is not hypothetical. Moving telemetry to a named message left five nodes
reading `msg.data`, which a named message does not have. Each raised
AttributeError inside its callback and died; systemd restarted them and they
died again on the next frame. The two worst were `safety_watchdog` and
`waypoint_nav` - the watchdog that stops the robot and the node that drives it.

This publishes a plausible frame on every topic the firmware owns, then reads
the journal and the web API to see whether anything fell over. It needs no
ESP32, which is the point: it makes a hardware-only failure reproducible here.

    python3 tools/live_probe.py            # probe the running stack
    python3 tools/live_probe.py --seconds 8

Exit status is 0 when nothing died, 1 when something did, and 2 when the stack
was not running to begin with.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request

WEB = 'http://127.0.0.1:8080'
UNIT = 'gps_localize'

# What the firmware publishes. Kept beside the contract in main.cpp rather than
# discovered, so a topic that quietly disappears shows up as a probe that never
# exercised it instead of a probe that silently tests less than it used to.
FIRMWARE_TOPICS = (
    '/gps_localize/telemetry',
    '/gps_localize/gps/fix',
    '/gps_localize/imu/data',
    '/gps_localize/odom',
)


def stack_is_up() -> bool:
    try:
        urllib.request.urlopen(WEB + '/api/state', timeout=5).read()
        return True
    except (urllib.error.URLError, OSError):
        return False


def publish(seconds: float) -> None:
    """Publish a believable frame on every firmware topic."""
    import rclpy
    from gps_localize_msgs.msg import Telemetry
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, NavSatFix

    rclpy.init()
    node = rclpy.create_node('live_probe')

    pub_tel = node.create_publisher(Telemetry, '/gps_localize/telemetry', 10)
    pub_fix = node.create_publisher(NavSatFix, '/gps_localize/gps/fix', 10)
    pub_imu = node.create_publisher(Imu, '/gps_localize/imu/data', 10)
    pub_odom = node.create_publisher(Odometry, '/gps_localize/odom', 10)

    # Values a real robot could actually produce: a healthy 3S pack, a usable
    # fix, and motion. Zeros would leave range and divide-by-zero paths untried.
    tel = Telemetry()
    tel.state = 1.0
    tel.heading_deg = 90.0
    tel.speed_mps = 0.5
    tel.battery_v = 11.9
    tel.battery_a = 1.2
    tel.gps_fix_quality = 1.0
    tel.gps_satellites = 8.0
    tel.gps_hdop = 1.1
    tel.odom_x = 1.0
    tel.odom_y = 2.0

    fix = NavSatFix()
    fix.latitude, fix.longitude, fix.altitude = 13.7563, 100.5018, 5.0

    imu = Imu()
    imu.orientation.w = 1.0

    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0

    deadline = time.time() + seconds
    frames = 0
    while time.time() < deadline:
        pub_tel.publish(tel)
        pub_fix.publish(fix)
        pub_imu.publish(imu)
        pub_odom.publish(odom)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)
        frames += 1

    print('  published %d frames on %d topics' % (frames, len(FIRMWARE_TOPICS)))
    rclpy.shutdown()


def deaths_since(mark: str) -> list[str]:
    try:
        out = subprocess.run(
            ['journalctl', '-u', UNIT, '--since', mark, '--no-pager'],
            capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line for line in out.splitlines()
            if 'process has died' in line
            or any(e in line for e in ('AttributeError', 'KeyError',
                                       'IndexError', 'TypeError'))]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=5.0,
                    help='how long to publish for (default 5)')
    args = ap.parse_args()

    print('live probe - synthetic firmware messages against the running stack')
    if not stack_is_up():
        print('  the stack is not answering on %s' % WEB)
        print('  start it first:  sudo systemctl start %s' % UNIT)
        return 2

    mark = time.strftime('%Y-%m-%d %H:%M:%S')
    time.sleep(1.0)          # so the journal window cannot miss the first death
    publish(args.seconds)
    time.sleep(4.0)          # let a dying node actually be logged as dead

    bad = deaths_since(mark)
    if bad:
        print('  FAIL - something died while telemetry was flowing:')
        for line in bad[:12]:
            print('    ' + line.split(': ', 1)[-1][:150])
        return 1

    print('  no node died')

    # Surviving is necessary but not sufficient: a node can swallow the error
    # and simply stop updating, so check the data actually arrived.
    try:
        state = json.loads(
            urllib.request.urlopen(WEB + '/api/state', timeout=6).read())['state']
    except Exception as exc:
        print('  FAIL - the web API stopped answering: %s' % exc)
        return 1

    fields = len(state.get('telemetry') or {})
    age = state.get('telemetry_age_s')
    print('  web decoded %d telemetry fields, age %.1fs'
          % (fields, age if age is not None else -1))
    if not fields:
        print('  FAIL - nothing decoded the telemetry that was just published')
        return 1

    # The decode must be OF THIS RUN. Without that this reports PASS on whatever
    # a previous run left behind - which it did, passing on 40-second-old data
    # while the frames just published were not arriving at all. A probe that
    # cannot tell "it worked" from "it worked once, earlier" is worth nothing.
    stale_after = args.seconds + 15.0
    if age is None or age > stale_after:
        print('  FAIL - the newest telemetry is %s s old, so nothing published '
              'in this run arrived' % ('unknown' if age is None else '%.1f' % age))
        print('         the subscriber is reachable but the data is not '
              'crossing (check transport, not discovery)')
        return 1

    print('  PASS - every firmware-fed path ran without hardware')
    return 0


if __name__ == '__main__':
    sys.exit(main())
