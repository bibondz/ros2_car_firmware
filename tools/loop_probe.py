#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""How close to real time is the firmware's control loop, measured from the PC.

The firmware reports `loop_us`, but that number only covers the time spent
inside controlCallback. A stall anywhere else - loop() waiting on the Wi-Fi
driver, a blocking agent ping - delays the callback without making it longer,
so loop_us stays small while the robot is not running in real time at all.

This probe measures both halves:

  loop_us       every telemetry frame (10 Hz), not a 2 Hz poll of the web API,
                so a spike cannot fall between samples
  imu stamps    the firmware stamps each /imu/data message itself, at a fixed
                divider of the control rate. A gap between consecutive stamps
                wider than the nominal period is the control loop running late,
                measured on the robot's own clock, so Wi-Fi jitter on the way
                to the PC does not count against it

Usage:
    python3 tools/loop_probe.py [seconds]      # default 45

Needs a running stack and a connected board. Run it twice under the same
conditions - same power, same place - to compare two firmware builds. A single
run on its own tells you much less than the difference between two.
"""
import math
import os
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '10')

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    from gps_localize_msgs.msg import Telemetry
except ImportError:
    sys.exit('source gps_localize_ws/install/setup.bash first - the Telemetry message is not importable')

# Must match firmware/config/control.h. The imu divider sets the nominal gap.
CTRL_PERIOD_MS = 10
PUB_IMU_DIV = 5
NOMINAL_IMU_GAP_MS = CTRL_PERIOD_MS * PUB_IMU_DIV


def pct(values, p):
    if not values:
        return float('nan')
    values = sorted(values)
    return values[min(len(values) - 1, int(math.ceil(p / 100.0 * len(values))) - 1)]


class Probe(Node):
    def __init__(self):
        super().__init__('loop_probe')
        # BEST_EFFORT, to match the firmware's publishers. A reliable
        # subscriber here would silently receive nothing.
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.imu_stamps = []
        self.loop_us = []
        self.create_subscription(Imu, '/gps_localize/imu/data', self.on_imu, qos)
        self.create_subscription(Telemetry, '/gps_localize/telemetry', self.on_tel, qos)

    def on_imu(self, msg):
        self.imu_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_tel(self, msg):
        self.loop_us.append(float(msg.loop_us))


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    probe = Probe()
    start = time.time()
    while time.time() - start < seconds:
        rclpy.spin_once(probe, timeout_sec=0.1)

    loop = probe.loop_us
    stamps = sorted(probe.imu_stamps)
    gaps = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]) if b > a]
    late = [g for g in gaps if g > NOMINAL_IMU_GAP_MS * 1.5]

    print('loop_probe  %.0f s' % seconds)
    if loop:
        print('  loop_us     n=%d  median %.0f  p90 %.0f  p99 %.0f  max %.0f'
              % (len(loop), pct(loop, 50), pct(loop, 90), pct(loop, 99), max(loop)))
    else:
        print('  loop_us     no telemetry received - is the board connected?')
    if gaps:
        print('  imu gaps    n=%d  nominal %d ms  median %.1f  p99 %.1f  max %.1f ms'
              % (len(gaps), NOMINAL_IMU_GAP_MS, pct(gaps, 50), pct(gaps, 99), max(gaps)))
        print('  late        %d of %d publishes more than 1.5x the nominal gap (%.1f%%)'
              % (len(late), len(gaps), 100.0 * len(late) / len(gaps)))
    else:
        print('  imu gaps    no /imu/data received')
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
