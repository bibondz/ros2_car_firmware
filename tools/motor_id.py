#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Label the motor rails, and find out which way each wheel really turns.

WHEELS OFF THE GROUND. This drives the motors.

Two questions get answered in one run, because both need the same motion and
neither can be answered from the bench:

1. **Which INA226 is motor A and which is motor B.** The addresses in power.h
   were measured rather than read off the schematic, and two of them were
   already found the wrong way round. Which motor rail is which has never been
   proved. It matters because the back-EMF speed model is fed per-motor current,
   so a swap quietly feeds each wheel's model the other wheel's current - a
   wrong number that looks entirely plausible.

2. **Why the web arrows are rotated** - forward produces a right turn, left
   produces forward. The whole software chain checks out, so the fault is in the
   motor directions, and the fix is a sign in config/motor.h once we know which
   sign.

HOW IT TELLS THEM APART WITHOUT DRIVING ONE MOTOR ALONE

There is no single-motor command in the firmware, but there does not need to be.
A turn in place drives the two wheels in OPPOSITE directions, and the INA226
reports SIGNED current. So:

    turn right : left wheel forward, right wheel reverse
    turn left  : left wheel reverse, right wheel forward

The rail whose current sign follows the left wheel is motor A. The one that
follows the right wheel is motor B. A forward command is included as a control:
both currents should then share a sign.

If a rail's current never moves at all, that channel is dead, which is its own
answer and explains a robot that turns when told to go straight.

    python3 tools/motor_id.py              # the full sequence
    python3 tools/motor_id.py --dry-run    # print what it would send, send nothing
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'gps_localize_ws', 'src', 'gps_localize'))

import rclpy                                             # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from std_msgs.msg import Int32                           # noqa: E402

from gps_localize import telemetry as T                  # noqa: E402
from gps_localize.qos import stream_qos                  # noqa: E402
from gps_localize_msgs.msg import Telemetry              # noqa: E402

NS = '/gps_localize'
SETTLE_S = 1.5          # let the rails come back to rest between moves
DRIVE_S = 2.0           # how long each move runs
TURN_DPS = 45.0         # gentle - this is identification, not a performance test
FWD_MPS = 0.10


class Rig:
    def __init__(self, node):
        self.node = node
        self.pub_manual = node.create_publisher(Twist, NS + '/cmd_manual', 5)
        self.pub_beat = node.create_publisher(Int32, NS + '/heartbeat', 5)
        self.lock = threading.Lock()
        self.tel = {}
        node.create_subscription(Telemetry, NS + '/telemetry',
                                 self._on_tel, stream_qos())
        self._beat_n = 0

    def _on_tel(self, msg):
        with self.lock:
            self.tel = T.unpack(msg)

    def beat(self):
        """The safety manager stops the motors without a fresh heartbeat."""
        self._beat_n += 1
        self.pub_beat.publish(Int32(data=self._beat_n))

    def send(self, forward_mps, turn_dps):
        cmd = Twist()
        cmd.linear.x = float(forward_mps)
        cmd.angular.z = float(turn_dps)
        self.pub_manual.publish(cmd)

    def snapshot(self):
        with self.lock:
            return dict(self.tel)


def spin(node, seconds, rig, forward=None, turn=None):
    """Spin the node, holding a command and the heartbeat up, collecting samples."""
    out = []
    end = time.time() + seconds
    while time.time() < end:
        rig.beat()
        if forward is not None:
            rig.send(forward, turn)
        rclpy.spin_once(node, timeout_sec=0.05)
        s = rig.snapshot()
        if s:
            out.append(s)
        time.sleep(0.05)
    return out


def summarise(samples, keys):
    """Mean of each key over the samples, ignoring the first few (ramp-up)."""
    if not samples:
        return {k: float('nan') for k in keys}
    use = samples[len(samples) // 3:] or samples
    return {k: sum(s.get(k, 0.0) for s in use) / len(use) for k in keys}


# Numeric only - summarise() averages these, and state_name/stop_reasons are
# read straight off the latest sample instead.
KEYS = ('motor_a_a', 'motor_b_a', 'motor_a_v', 'motor_b_v',
        'pwm_left', 'pwm_right')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    moves = [('turn RIGHT', 0.0, TURN_DPS),
             ('turn LEFT', 0.0, -TURN_DPS),
             ('FORWARD', FWD_MPS, 0.0)]

    if args.dry_run:
        print('would send, each for %.1f s with a heartbeat:' % DRIVE_S)
        for name, f, t in moves:
            print('  %-12s linear.x=%.2f  angular.z=%+.0f' % (name, f, t))
        return 0

    print('\n  WHEELS OFF THE GROUND. Starting in 3 s - Ctrl-C to abort.\n')
    time.sleep(3)

    rclpy.init()
    node = rclpy.create_node('motor_id')
    rig = Rig(node)

    # Wait for telemetry, and refuse to drive anything if the robot is latched.
    spin(node, 3.0, rig)
    tel = rig.snapshot()
    if not tel:
        print('  no telemetry - is the board connected?')
        return 1
    if tel.get('stop_reasons'):
        print('  refusing to drive: robot is stopped for %s' % (tel['stop_reasons'],))
        print('  release the emergency and try again.')
        return 1
    print('  state %s, rails A %.2f V / B %.2f V\n'
          % (tel.get('state_name'), tel.get('motor_a_v', 0), tel.get('motor_b_v', 0)))

    results = {}
    try:
        for name, fwd, turn in moves:
            print('  --- %s ---' % name)
            spin(node, SETTLE_S, rig, 0.0, 0.0)            # rest, keep heartbeat
            rest = summarise(spin(node, 0.6, rig, 0.0, 0.0), KEYS)
            got = summarise(spin(node, DRIVE_S, rig, fwd, turn), KEYS)
            spin(node, 0.5, rig, 0.0, 0.0)                 # stop
            results[name] = (rest, got)
            print('      pwm  L %+7.1f   R %+7.1f' % (got['pwm_left'], got['pwm_right']))
            print('      A    %+7.3f A (rest %+7.3f)' % (got['motor_a_a'], rest['motor_a_a']))
            print('      B    %+7.3f A (rest %+7.3f)' % (got['motor_b_a'], rest['motor_b_a']))
    finally:
        for _ in range(10):
            rig.send(0.0, 0.0)
            rig.beat()
            rclpy.spin_once(node, timeout_sec=0.02)
        print('\n  motors commanded to stop.')

    # ---------------------------------------------------------------- verdict
    print('\n  ---------------- what this says ----------------')
    right = results.get('turn RIGHT', ({}, {}))[1]
    left = results.get('turn LEFT', ({}, {}))[1]
    fwd = results.get('FORWARD', ({}, {}))[1]

    def moved(v, rest, thresh=0.02):
        return abs(v - rest) > thresh

    ra, rb = results['turn RIGHT']
    la, lb = results['turn LEFT']
    a_moves = moved(right.get('motor_a_a', 0), ra.get('motor_a_a', 0))
    b_moves = moved(right.get('motor_b_a', 0), ra.get('motor_b_a', 0))

    if not a_moves and not b_moves:
        print('  NEITHER rail drew current. No motor ran - check the relay, the')
        print('  driver enable, and that motion was actually allowed.')
        return 1
    if not a_moves:
        print('  Rail A never moved: that channel looks DEAD. A robot that turns')
        print('  when told to go straight is exactly what one dead channel gives.')
    if not b_moves:
        print('  Rail B never moved: that channel looks DEAD. A robot that turns')
        print('  when told to go straight is exactly what one dead channel gives.')

    # Sign flip between the two turns identifies which wheel each rail feeds.
    a_flip = (right.get('motor_a_a', 0) - ra.get('motor_a_a', 0)) * \
             (left.get('motor_a_a', 0) - la.get('motor_a_a', 0)) < 0
    b_flip = (right.get('motor_b_a', 0) - rb.get('motor_b_a', 0)) * \
             (left.get('motor_b_a', 0) - lb.get('motor_b_a', 0)) < 0
    print('  rail A current reverses between the two turns: %s' % ('yes' if a_flip else 'NO'))
    print('  rail B current reverses between the two turns: %s' % ('yes' if b_flip else 'NO'))
    if a_flip and b_flip:
        print('  Both reverse, which is what a turn in place should do.')
        print('  Now say which WHEEL went forward on "turn RIGHT": that wheel is')
        print('  the LEFT one, and whichever rail was positive then is motor A.')
    print()
    print('  FORWARD: pwm L %+.1f R %+.1f, A %+.3f A, B %+.3f A'
          % (fwd.get('pwm_left', 0), fwd.get('pwm_right', 0),
             fwd.get('motor_a_a', 0), fwd.get('motor_b_a', 0)))
    print('  On a forward command both currents should share a sign. If they do')
    print('  not, one motor is wired backwards - set MOTOR_A_INVERT or')
    print('  MOTOR_B_INVERT in firmware/config/motor.h and reflash.')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
