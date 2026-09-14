#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Check the whole robot, end to end, and say what is wrong and how to fix it.

WHY THIS EXISTS

Every fault this project has hit was found by running half a dozen unrelated
commands and comparing their output by eye: is the agent up, is the board in the
graph, is telemetry arriving, are the rails sane, is the loop keeping time, is
the web reachable. Doing that by hand is slow, easy to get wrong, and - worse -
easy to get RIGHT in a misleading way. Two real examples from this robot:

  * A safety test was reported as passing because the motors had never started,
    so there was nothing to stop. Nothing in the output said so.
  * The reset counter was inflated twofold for hours because the ESP32 ROM
    prints two lines that both contain "rst:".

So every check here states what it measured, not merely pass or fail, and any
check that cannot get the evidence it needs says UNKNOWN rather than guessing.
A green line you cannot trust is worse than a red one.

    python3 tools/doctor.py             # everything
    python3 tools/doctor.py --quick     # skip the timed checks
    python3 tools/doctor.py --json      # machine readable

Exit status is 0 when nothing is broken, 1 otherwise. Nothing here commands the
motors: it is safe to run at any time, with or without a robot attached.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'gps_localize_ws', 'src', 'gps_localize'))

OK, WARN, BAD, UNKNOWN = 'ok', 'warn', 'bad', 'unknown'
MARK = {OK: ' ok  ', WARN: 'warn ', BAD: 'FAIL ', UNKNOWN: ' ?   '}

results = []


def report(status, title, detail='', fix=''):
    results.append({'status': status, 'title': title, 'detail': detail, 'fix': fix})


def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr)
    except (OSError, subprocess.SubprocessError) as exc:
        return 1, str(exc)


# --------------------------------------------------------------- host checks
def check_services():
    for unit, needed in (('micro_ros_agent', True), ('gps_localize', True),
                         # Not fatal on its own - the robot can still be told an
                         # address by hand - but without it a board that has
                         # lost the agent has no way to find it again, which is
                         # the failure this project kept hitting.
                         ('gps_localize_beacon', False)):
        rc, out = run(['systemctl', 'is-active', unit])
        state = out.strip()
        if state == 'active':
            report(OK, '%s service' % unit, state)
        else:
            report(BAD if needed else WARN, '%s service' % unit, state or 'not found',
                   'sudo systemctl restart %s' % unit)


def check_agent_port():
    # The agent binds UDP 8888. Anything else there means the board cannot land.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', 8888))
        s.close()
        report(BAD, 'UDP 8888 has a listener', 'nothing is bound - the board has nowhere to connect',
               'sudo systemctl restart micro_ros_agent')
    except OSError:
        report(OK, 'UDP 8888 has a listener', 'bound (the agent is holding it)')


def check_web():
    for url in ('http://127.0.0.1/', 'http://127.0.0.1:8080/'):
        try:
            code = urllib.request.urlopen(url, timeout=4).getcode()
            report(OK if code == 200 else WARN, 'web %s' % url, 'HTTP %s' % code)
        except (urllib.error.URLError, OSError) as exc:
            report(BAD, 'web %s' % url, str(exc), 'systemctl restart gps_localize')


def check_reachability():
    """Can anything OTHER than this machine reach the web UI?

    The single most common confusion on this setup: everything answers on
    127.0.0.1 and nothing answers from a phone, because the VM is behind NAT.
    """
    addrs = []
    rc, out = run(['ip', '-4', '-br', 'addr'])
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] != 'lo':
            addrs.append((parts[0], parts[2].split('/')[0]))
    if not addrs:
        report(UNKNOWN, 'reachable from other devices', 'could not read any address')
        return
    detail = ', '.join('%s %s' % (n, a) for n, a in addrs)
    natted = any(a.startswith('10.0.2.') for _, a in addrs)
    if natted:
        report(WARN, 'reachable from other devices',
               '%s - this is a VirtualBox NAT address, which no phone or PC on '
               'your Wi-Fi can route to' % detail,
               'either add a VirtualBox port-forward (host TCP 8080 -> guest 8080) '
               'and use http://<host-ip>:8080, or set the adapter to Bridged, which '
               'also fixes Wi-Fi OTA and the .local name')
    else:
        report(OK, 'reachable from other devices', detail)


# -------------------------------------------------------------- robot checks
def read_telemetry(seconds):
    """Collect telemetry frames. Returns [] when the board is not talking."""
    try:
        import rclpy
        from gps_localize import telemetry as T
        from gps_localize.qos import stream_qos
        from gps_localize_msgs.msg import Telemetry
    except ImportError as exc:
        report(UNKNOWN, 'telemetry', 'workspace not sourced: %s' % exc,
               'source gps_localize_ws/install/setup.bash')
        return []

    rows = []
    rclpy.init()
    try:
        node = rclpy.create_node('doctor_%d' % (time.time() % 100000))
        node.create_subscription(Telemetry, '/gps_localize/telemetry',
                                 lambda m: rows.append(T.unpack(m)), stream_qos(30))
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return rows


def check_board(rows, window):
    if not rows:
        report(BAD, 'board is publishing', 'no telemetry in %.0f s' % window,
               'check the board is powered and joined to Wi-Fi; if it pings but never '
               'links, it self-reboots after ~30 s (see the reconnect self-heal)')
        return False
    hz = len(rows) / window
    report(OK if hz > 5 else WARN, 'board is publishing',
           '%d frames, %.1f Hz' % (len(rows), hz),
           '' if hz > 5 else 'expected ~10 Hz; a slow rate means the control loop is stalling')
    return True


def check_loop(rows):
    lu = [r['loop_us'] for r in rows if 'loop_us' in r]
    if not lu:
        return
    mean = sum(lu) / len(lu)
    status = OK if mean < 12000 else BAD
    report(status, 'control loop keeps time',
           'mean %.0f us, max %.0f us (period is 10000 us)' % (mean, max(lu)),
           '' if status is OK else
           'the loop is running slower than its own period - check that the four '
           'streamed topics are still best-effort on BOTH sides')


def check_resets(rows, window):
    up = [r['uptime_s'] for r in rows if 'uptime_s' in r]
    if len(up) < 3:
        report(UNKNOWN, 'board stays up', 'not enough frames to tell')
        return
    resets = sum(1 for a, b in zip(up, up[1:]) if b < a - 1.0)
    report(OK if resets == 0 else BAD, 'board stays up',
           '%d reset(s) in %.0f s, uptime now %.0f s' % (resets, window, up[-1]),
           '' if resets == 0 else
           'a reset with reason POWERON means the supply collapsed, not a crash - '
           'check the rail feeding the ESP32 under load')


def check_rails(rows):
    if not rows:
        return
    r = rows[-1]
    def avg(k):
        v = [x[k] for x in rows if k in x]
        return sum(v) / len(v) if v else float('nan')

    esp = avg('esp32_rail_v')
    if 3.0 <= esp <= 3.6:
        report(OK, 'ESP32 rail', '%.3f V, %.3f A' % (esp, avg('battery_a')))
    elif 4.5 <= esp <= 5.5:
        report(WARN, 'ESP32 rail',
               '%.3f V - that is the 5 V rail, so the board is NOT on its own 3V3 buck' % esp,
               'expected 3.0-3.6 V since the ESP32 was moved off the shared 5 V rail')
    else:
        report(BAD, 'ESP32 rail', '%.3f V is outside 3.0-3.6 V' % esp,
               'check the 3V3 buck; above 3.6 V will damage the ESP32 and the sensors')

    fan = avg('fan_v')
    report(OK if 4.5 <= fan <= 5.5 else WARN, 'fan rail',
           '%.3f V, %.3f A' % (fan, avg('fan_a')))

    if r.get('pack_valid'):
        v = avg('battery_v')
        if v >= 11.4:      st, note = OK, 'healthy'
        elif v >= 11.0:    st, note = WARN, 'below the 11.4 V warn tier'
        elif v >= 10.7:    st, note = WARN, 'below the 11.0 V soft-stop tier'
        else:              st, note = BAD, 'below the 10.7 V cutoff'
        report(st, 'battery', '%.2f V (%s), motor rails %.2f / %.2f V'
               % (v, note, avg('motor_a_v'), avg('motor_b_v')))
    else:
        report(WARN, 'battery', 'pack not measurable - the contactor is open, '
               'so this is expected with the emergency pressed',
               'release the emergency to read the pack')


def check_safety(rows):
    if not rows:
        return
    r = rows[-1]
    stops = r.get('stop_reasons') or []
    if 'contactor_stuck' in stops:
        report(BAD, 'safety state', 'CONTACTOR STUCK CLOSED - the emergency stop cannot '
               'remove power in this state',
               'disconnect the battery by hand before going near the wheels')
    elif stops:
        report(WARN, 'safety state', '%s (%s)' % (r.get('state_name'), ', '.join(stops)))
    else:
        report(OK, 'safety state', r.get('state_name', '?'))

    faults = r.get('motor_fault_names') or []
    advice = r.get('motor_fault_advice') or []
    report(OK if not faults else WARN, 'motor faults',
           'none' if not faults else ', '.join(faults),
           '' if not faults else '; '.join(advice))


def check_sensors(rows):
    if not rows:
        return
    r = rows[-1]
    missing = r.get('missing_sensors') or []
    detail = 'heading from %s' % r.get('heading_ref_name', '?')
    if not missing:
        report(OK, 'sensors', detail)
        return
    # GPS with no fix indoors is normal and should not read as a fault.
    only_expected = set(missing) <= {'gps_missing', 'imu_magnetometer_missing'}
    report(WARN if only_expected else BAD, 'sensors',
           '%s; missing: %s' % (detail, ', '.join(missing)),
           'gps_missing indoors just means no sky view; imu_magnetometer_missing '
           'means the BNO085 reports its magnetometer accuracy below 2. Check the '
           'number on the Calibration tab before assuming it needs moving - the '
           'flag has been wrong before, when the driver read the heading '
           'uncertainty in radians instead of the 0-3 status and inverted the '
           'whole scale' if only_expected else '')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--quick', action='store_true', help='skip the timed telemetry checks')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--seconds', type=float, default=12.0)
    args = ap.parse_args()

    check_services()
    check_agent_port()
    check_web()
    check_reachability()

    window = 3.0 if args.quick else args.seconds
    rows = read_telemetry(window)
    if check_board(rows, window):
        check_loop(rows)
        check_resets(rows, window)
        check_rails(rows)
        check_safety(rows)
        check_sensors(rows)

    if args.json:
        print(json.dumps(results, indent=2))
    else:
        width = max(len(r['title']) for r in results)
        print()
        for r in results:
            print('  %s %-*s  %s' % (MARK[r['status']], width, r['title'], r['detail']))
            if r['fix'] and r['status'] in (BAD, WARN):
                for line in _wrap(r['fix'], 74):
                    print('        -> %s' % line)
        bad = sum(1 for r in results if r['status'] == BAD)
        warn = sum(1 for r in results if r['status'] == WARN)
        print()
        print('  %d problem(s), %d warning(s)' % (bad, warn) if (bad or warn)
              else '  everything checked is healthy')
        print()
    return 1 if any(r['status'] == BAD for r in results) else 0


def _wrap(text, width):
    words, line, out = text.split(), '', []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line); line = w
        else:
            line = (line + ' ' + w).strip()
    if line:
        out.append(line)
    return out


if __name__ == '__main__':
    sys.exit(main())
