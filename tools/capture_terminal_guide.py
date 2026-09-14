#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Capture read-only setup checks in an offscreen GTK terminal.

Requires existing GTK 3/VTE Python bindings and a graphical display.
No package installation, robot connection, service change, or USB access occurs.
The Wi-Fi picture is a source-derived command reference, not a board session.
"""
from __future__ import annotations
import argparse
import json
import re
import subprocess
import time
from pathlib import Path
import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
gi.require_version('Vte', '2.91')
from gi.repository import Gdk, Gtk, Pango, Vte


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument('--out', type=Path, default=Path('docs/screenshots'))
    args = ap.parse_args()
    root, out = args.root.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = ('source /opt/ros/humble/setup.bash\n'
              'source "$HOME/uros_ws/install/local_setup.bash"\n'
              f'source "{root}/gps_localize_ws/install/setup.bash"\n')
    records = []

    def draw(name, title, text, command=None, rc=None):
        window = Gtk.OffscreenWindow()
        terminal = Vte.Terminal()
        height = max(420, min(1400, (len(text.splitlines())+4)*24))
        terminal.set_size_request(1320, height)
        window.set_default_size(1320, height)
        terminal.set_font(Pango.FontDescription('Monospace 12'))
        bg, fg = Gdk.RGBA(), Gdk.RGBA()
        bg.parse('#15202b'); fg.parse('#eef4f9')
        terminal.set_colors(fg, bg, [])
        window.add(terminal); window.show_all()
        for _ in range(15):
            while Gtk.events_pending(): Gtk.main_iteration()
            time.sleep(.02)
        terminal.feed((title+'\n\n'+text).replace('\n','\r\n').encode())
        for _ in range(15):
            while Gtk.events_pending(): Gtk.main_iteration()
            time.sleep(.02)
        window.get_pixbuf().savev(str(out/(name+'.png')), 'png', [], [])
        window.destroy()
        records.append({'file': name+'.png', 'title': title, 'command': command,
                        'exit_code': rc, 'displayed_text': text})
        print(name+'.png')

    def run(name, title, cmd, excerpt=False):
        result = subprocess.run(['bash', '-c', cmd], cwd=root, text=True,
                                capture_output=True, timeout=60)
        output = result.stdout+result.stderr
        if excerpt:
            lines = output.splitlines()
            output = '\n'.join(lines[:14]+['... excerpt; intervening lines omitted ...']+lines[-12:])
        draw(name, title, '$ '+cmd+'\n\n'+output, cmd, result.returncode)

    run('setup-native-check', 'NATIVE CHECK - actual read-only output from this Ubuntu PC',
        source+'python3 --version\nprintf "ROS distribution: %s\\n" "$ROS_DISTRO"\n'
        'ros2 pkg prefix micro_ros_agent\nros2 pkg prefix gps_localize')
    run('setup-launch-options', 'LAUNCH OPTIONS - inspection only; no server started',
        source+'ros2 launch gps_localize bringup.launch.py --show-args')
    run('setup-install-preview', 'NATIVE INSTALLER DRY-RUN - no install/start performed (excerpt)',
        'bash setup/install.sh --native --dry-run', excerpt=True)
    header = (root/'firmware/src/net/settings_console.h').read_text()
    help_body = header.split('void cmdHelp() {', 1)[1].split('void cmdInfo()', 1)[0]
    commands = re.findall(r'reply\("(.*?)"\);', help_body)
    draw('wifi-console-reference', 'WIFI CONSOLE REFERENCE - extracted from source; NOT a board session',
         'Where: USB monitor, 115200 baud, board running wifi firmware.\n'
         'Commands below are entered in the monitor, not the Ubuntu shell.\n\n'+
         '\n'.join(commands)+'\n\nSSID is one token (no spaces); password is the rest of the line.\n'
         'Example: wifi add Workshop24G YOUR_WIFI_PASSWORD\n'
         'Example: agent 192.0.2.10  (replace with your server LAN address)\n'
         'Stop the robot before changing networks. Do not share a real password.')
    template = (root/'firmware/config/network_secrets.example.h').read_text()
    start = template.index('static const WifiNetwork')
    draw('wifi-seed-template', 'WIFI FIRST-BOOT SEED - checked-in EXAMPLE, not private credentials',
         'Where: firmware/config/network_secrets.example.h\n'
         'Copy to network_secrets.h locally, then edit before the first flash.\n'
         'Existing saved NVS networks take precedence over the seed.\n\n'+template[start:])
    (out/'terminal-manifest.json').write_text(json.dumps({'created': time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'captures': records, 'path_normalization': 'The manifest replaces the personal home path with ${USER_HOME}; PNGs retain displayed output.', 'note': 'Actual read-only output rendered by an offscreen VTE terminal; source references are labelled and were not sent to a board.'}, indent=2).replace(str(Path.home()), '${USER_HOME}')+'\n')


if __name__ == '__main__':
    main()
