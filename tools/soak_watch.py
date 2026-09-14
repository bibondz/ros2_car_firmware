#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Watch the robot over hours and record anything that would otherwise be missed.

WHY A SOAK RUN NEEDS ITS OWN TOOL

The failures worth catching here do not announce themselves. A board that
brownouts under Wi-Fi bursts reboots and comes back looking healthy - the only
trace is a reset reason and an uptime that starts over, both of which are gone
by the time anyone looks at a live value. A slow heap leak looks fine for six
hours and then does not. Neither shows up in a snapshot.

So this samples continuously and reports on CHANGE and on EXTREME, not on a
timer: a log that prints a line every ten seconds for seven hours is a log
nobody reads.

    python3 tools/soak_watch.py --hours 7 --out ~/soak.log

What it records:

  every reboot        with the reason - brownout, panic, watchdog, or a clean
                      power-on - and how long the board had been up
  heap extremes       the lowest free heap seen, which is what a leak looks
                      like before it becomes a crash
  temperature         the die reading when the chip has a usable sensor, and
                      the highest seen
  telemetry gaps      any silence longer than a few seconds, which is what a
                      Wi-Fi drop or a lock-up looks like from here
"""

from __future__ import annotations

import argparse
import os
import sys
import time

RESET_NAMES = {
    0: 'UNKNOWN', 1: 'POWERON', 2: 'EXT', 3: 'SOFTWARE', 4: 'PANIC',
    5: 'INT_WATCHDOG', 6: 'TASK_WATCHDOG', 7: 'WATCHDOG',
    8: 'DEEPSLEEP', 9: 'BROWNOUT', 10: 'SDIO',
}

# The ones that mean something went wrong, as opposed to someone switching it on.
BAD_RESETS = {4, 5, 6, 7, 9}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hours', type=float, default=7.0)
    ap.add_argument('--out', default=os.path.expanduser('~/soak.log'))
    ap.add_argument('--gap', type=float, default=8.0,
                    help='silence longer than this counts as a dropout (s)')
    args = ap.parse_args()

    import rclpy
    from gps_localize_msgs.msg import Telemetry

    out = open(args.out, 'a', buffering=1)      # line buffered: readable live

    def say(line):
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        text = '%s  %s' % (stamp, line)
        print(text, flush=True)
        out.write(text + '\n')

    rclpy.init()
    node = rclpy.create_node('soak_watch')

    state = {
        'frames': 0, 'last_rx': None, 'boots': 0, 'bad_boots': 0,
        'last_uptime': None, 'last_reason': None,
        'min_heap': None, 'max_temp': None, 'gaps': 0, 'longest_gap': 0.0,
        'started': time.time(),
    }

    def on_telemetry(msg):
        now = time.time()
        state['frames'] += 1

        # A gap in telemetry is a dropout: Wi-Fi, a reboot, or a lock-up. Which
        # one it was is answered by the reset reason on the other side of it.
        if state['last_rx'] is not None:
            gap = now - state['last_rx']
            if gap > args.gap:
                state['gaps'] += 1
                state['longest_gap'] = max(state['longest_gap'], gap)
                say('GAP  %.1f s of silence' % gap)
        state['last_rx'] = now

        uptime = float(msg.uptime_s)
        reason = int(msg.reset_reason)

        # Uptime going BACKWARDS is the reboot signal. The reset reason alone
        # would not do it: it stays the same across a whole run, so it says why
        # the board last restarted, not that it just did.
        if state['last_uptime'] is not None and uptime < state['last_uptime'] - 1.0:
            state['boots'] += 1
            name = RESET_NAMES.get(reason, str(reason))
            if reason in BAD_RESETS:
                state['bad_boots'] += 1
                say('REBOOT  reason=%s  (ran for %.0f s)  <-- NOT a clean restart'
                    % (name, state['last_uptime']))
            else:
                say('reboot  reason=%s  (ran for %.0f s)' % (name, state['last_uptime']))
        state['last_uptime'] = uptime

        if reason != state['last_reason']:
            state['last_reason'] = reason
            say('reset reason now %s' % RESET_NAMES.get(reason, reason))

        heap = float(msg.free_heap_kb)
        if heap > 0 and (state['min_heap'] is None or heap < state['min_heap']):
            # Only report a NEW low, and only a meaningful one - heap wobbles by
            # a kilobyte constantly and reporting each would bury the trend.
            if state['min_heap'] is None or heap < state['min_heap'] - 2.0:
                say('heap low  %.0f KB' % heap)
            state['min_heap'] = heap

        temp = float(msg.esp32_temp_c)
        if temp == temp:                        # not NaN
            if state['max_temp'] is None or temp > state['max_temp'] + 1.0:
                say('die temp  %.1f C' % temp)
            if state['max_temp'] is None or temp > state['max_temp']:
                state['max_temp'] = temp

    node.create_subscription(Telemetry, '/gps_localize/telemetry', on_telemetry, 20)

    say('=== soak start: watching for %.1f h ===' % args.hours)
    deadline = time.time() + args.hours * 3600.0
    last_beat = time.time()

    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.time()
            # A quiet hourly line, so a reader can tell "nothing happened" from
            # "the watcher died".
            if now - last_beat >= 3600.0:
                last_beat = now
                say('still watching: %d frames, %d reboots (%d unclean), '
                    'min heap %s KB, max temp %s'
                    % (state['frames'], state['boots'], state['bad_boots'],
                       '%.0f' % state['min_heap'] if state['min_heap'] else '?',
                       '%.1f C' % state['max_temp'] if state['max_temp'] else 'no sensor'))
            # No telemetry at all for a long time is itself the finding.
            if state['last_rx'] and now - state['last_rx'] > 120.0:
                say('NO TELEMETRY for %.0f s' % (now - state['last_rx']))
                state['last_rx'] = now
    except KeyboardInterrupt:
        pass

    hours = (time.time() - state['started']) / 3600.0
    say('=== soak end after %.2f h ===' % hours)
    say('frames        : %d' % state['frames'])
    say('reboots       : %d  (%d unclean)' % (state['boots'], state['bad_boots']))
    say('telemetry gaps: %d, longest %.1f s' % (state['gaps'], state['longest_gap']))
    say('min free heap : %s' % ('%.0f KB' % state['min_heap'] if state['min_heap'] else 'n/a'))
    say('max die temp  : %s' % ('%.1f C' % state['max_temp'] if state['max_temp']
                                else 'no usable sensor on this chip'))
    if state['bad_boots']:
        say('VERDICT: the board restarted uncleanly %d time(s) - see the REBOOT '
            'lines above for the reason' % state['bad_boots'])
    elif state['boots']:
        say('VERDICT: %d clean restart(s), no brownout or panic' % state['boots'])
    else:
        say('VERDICT: no reboots at all - the rail held up for the whole run')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    out.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
