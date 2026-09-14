#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Turn the firmware's parameter table into a manifest the host and web can read.

WHY GENERATE RATHER THAN DUPLICATE

The board addresses parameters by INDEX, because the config topic is a float
array and cannot carry names. The web UI needs names, units, ranges and help
text. So something has to map one to the other, and there are only three ways to
do it:

  keep a second copy on the host   guaranteed to drift, and the failure is
                                   silent - the UI writes index 7 believing it
                                   is the diode drop while the board applies it
                                   to the track width
  send the table at runtime        strings over micro-ROS, which is exactly the
                                   dependency Telemetry.msg was kept free of
  GENERATE it from the header      one source of truth, and a QC check that
                                   fails when they disagree

The third is the only one where being wrong is loud.

    python3 tools/gen_param_manifest.py            # write the manifest
    python3 tools/gen_param_manifest.py --check    # fail if it is out of date

The --check form is what runs in QC: it regenerates in memory and compares, so
a parameter added to the header without regenerating is caught before it can
send the UI writing to the wrong index.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HEADER = os.path.join(ROOT, 'firmware', 'src', 'params', 'param_table.h')
MANIFEST = os.path.join(ROOT, 'gps_localize_ws', 'src', 'gps_localize',
                        'config', 'params.json')

# One table row. Deliberately strict: a row this does not match is an error
# rather than a row silently skipped, because a skipped row shifts every index
# after it and the UI would then write to the wrong parameter.
ROW = re.compile(
    r'\{\s*"(?P<key>[^"]+)"\s*,\s*'
    r'"(?P<unit>[^"]*)"\s*,\s*'
    r'"(?P<group>[^"]+)"\s*,\s*'
    r'(?P<default>[A-Za-z0-9_.\-+()* /]+?)\s*,\s*'
    r'(?P<min>-?[0-9.]+)f?\s*,\s*'
    r'(?P<max>-?[0-9.]+)f?\s*,\s*'
    r'(?P<when>PARAM_LIVE|PARAM_STOPPED)\s*,\s*'
    r'"(?P<help>(?:[^"\\]|\\.)*)"\s*\}',
    re.DOTALL)


def parse(header_text: str) -> list[dict]:
    """Every row of PARAM_TABLE, in declaration order - which IS the index."""
    start = header_text.index('static const ParamDef PARAM_TABLE')
    body = header_text[start:]
    rows = []
    for index, m in enumerate(ROW.finditer(body)):
        rows.append({
            'index': index,
            'key': m.group('key'),
            'unit': m.group('unit'),
            'group': m.group('group'),
            # The default is a macro like MOTOR_A_DIODE_VF_V, not a number. It
            # is carried through as written: the live value comes from the board
            # anyway, and printing the macro name tells a reader where to look.
            'default_expr': m.group('default').strip(),
            'min': float(m.group('min')),
            'max': float(m.group('max')),
            'when': 'stopped' if m.group('when') == 'PARAM_STOPPED' else 'live',
            'help': m.group('help').replace('\\"', '"'),
        })
    return rows


def build() -> dict:
    with open(HEADER) as handle:
        text = handle.read()

    declared = re.search(r'#define\s+PARAM_COUNT\s+(\d+)', text)
    rows = parse(text)

    if declared and int(declared.group(1)) != len(rows):
        raise SystemExit(
            'PARAM_COUNT says %s but %d rows parsed.\n'
            'The registry is statically sized, so a mismatch means it silently '
            'truncates - the last parameters would simply not exist.'
            % (declared.group(1), len(rows)))

    seen = {}
    for row in rows:
        if row['key'] in seen:
            raise SystemExit('duplicate parameter key %r at indices %d and %d'
                             % (row['key'], seen[row['key']], row['index']))
        seen[row['key']] = row['index']
        if row['min'] > row['max']:
            raise SystemExit('%s has min above max' % row['key'])

    return {
        'generated_from': 'firmware/src/params/param_table.h',
        'note': ('Generated - do not edit. The board addresses parameters by '
                 'index, and these indices are the table order. Run '
                 'tools/gen_param_manifest.py after changing the header.'),
        'count': len(rows),
        'params': rows,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true',
                    help='fail if the manifest on disk is out of date')
    args = ap.parse_args()

    fresh = build()
    text = json.dumps(fresh, indent=2) + '\n'

    if args.check:
        try:
            with open(MANIFEST) as handle:
                on_disk = handle.read()
        except OSError:
            print('manifest missing: run tools/gen_param_manifest.py')
            return 1
        if on_disk != text:
            print('params.json is out of date - regenerate it with:')
            print('    python3 tools/gen_param_manifest.py')
            print('Until then the web UI and the board disagree about which '
                  'index is which parameter, and a setting written from the UI '
                  'lands on the wrong one.')
            return 1
        print('params.json matches the header (%d parameters)' % fresh['count'])
        return 0

    os.makedirs(os.path.dirname(MANIFEST), exist_ok=True)
    with open(MANIFEST, 'w') as handle:
        handle.write(text)
    print('wrote %s (%d parameters)' % (os.path.relpath(MANIFEST, ROOT), fresh['count']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
