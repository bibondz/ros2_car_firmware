#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Update the host from git, and put it back if the update does not work.

WHY ROLLBACK IS THE POINT

Pulling is easy. The failure worth designing for is the one where the pull
succeeds, the build fails, and the robot is now running nothing - on a machine
that may be in a field, connected to by phone, with no way to get a working
version back. An update that can leave the system worse than it found it is not
an update, it is a gamble.

So: record where we were, pull, build, check the stack actually answers, and on
any failure go back to the recorded commit and rebuild it. The check is not
"did the build exit zero" but "does the web UI respond" - a build can succeed
and still produce a stack that dies on the first frame, which has happened here.

    python3 tools/self_update.py --check     # what would change, no side effects
    python3 tools/self_update.py --apply     # pull, build, verify, roll back on failure

--check is safe to run from a web request. --apply is not automatic and never
will be: an unattended robot deciding to update itself mid-mission is a worse
failure than being out of date.

WHICH BRANCH

Whatever the checkout is on. A workspace-only machine tracks `workspace`, this
one tracks `main`, and neither should be dragged onto the other by an updater
that assumed.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = 'http://127.0.0.1:8080'


def run(args, cwd=ROOT, timeout=600):
    """Run a command, returning (ok, output). Never raises on a non-zero exit."""
    try:
        r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)


def git(*args, **kw):
    return run(['git'] + list(args), **kw)


def current_branch():
    ok, out = git('rev-parse', '--abbrev-ref', 'HEAD')
    return out if ok else None


def current_commit():
    ok, out = git('rev-parse', 'HEAD')
    return out if ok else None


def stack_answers(timeout=4.0):
    """Does the web UI actually respond? The only check that means anything."""
    for base in (WEB, 'http://127.0.0.1'):
        try:
            urllib.request.urlopen(base + '/api/state', timeout=timeout).read()
            return True
        except (urllib.error.URLError, OSError):
            continue
    return False


def check():
    """What an update would bring, without touching anything."""
    branch = current_branch()
    if not branch:
        return {'ok': False, 'error': 'not a git checkout'}

    dirty_ok, dirty = git('status', '--porcelain')
    ok, fetch_out = git('fetch', '--quiet', 'origin', branch, timeout=120)
    if not ok:
        # A private repo with no credentials on this machine lands here, and
        # saying so beats "update failed".
        return {'ok': False, 'branch': branch,
                'error': 'could not reach the remote: %s' % fetch_out[:200]}

    _, behind = git('rev-list', '--count', 'HEAD..origin/%s' % branch)
    _, ahead = git('rev-list', '--count', 'origin/%s..HEAD' % branch)
    _, log = git('log', '--oneline', '--no-decorate', '-20',
                 'HEAD..origin/%s' % branch)

    return {
        'ok': True,
        'branch': branch,
        'commit': (current_commit() or '')[:12],
        'behind': int(behind or 0),
        'ahead': int(ahead or 0),
        'local_changes': bool(dirty.strip()) if dirty_ok else None,
        'changelog': [line for line in log.splitlines() if line.strip()],
        'update_available': int(behind or 0) > 0,
    }


def build():
    ok, out = run(['bash', '-lc',
                   'source /opt/ros/humble/setup.bash && '
                   'cd gps_localize_ws && '
                   'colcon build --symlink-install --parallel-workers $(nproc)'],
                  timeout=1800)
    return ok, out


def restart():
    ok, _ = run(['systemctl', 'restart', 'gps_localize'], timeout=120)
    if not ok:                                    # not root, or no systemd
        ok, _ = run(['sudo', '-n', 'systemctl', 'restart', 'gps_localize'], timeout=120)
    return ok


def apply(settle=25.0):
    state = check()
    if not state.get('ok'):
        return state
    if not state['update_available']:
        state['applied'] = False
        state['message'] = 'already up to date'
        return state

    # Local edits would be silently destroyed by a reset. Refuse rather than
    # decide on someone's behalf which of the two they wanted.
    if state.get('local_changes'):
        return {'ok': False, 'branch': state['branch'],
                'error': 'this checkout has uncommitted changes - commit or '
                         'discard them first, so the update cannot throw them away'}

    was = current_commit()
    branch = state['branch']
    steps = []

    def rollback(why):
        steps.append('FAILED: %s' % why)
        steps.append('rolling back to %s' % was[:12])
        git('reset', '--hard', was)
        built, _ = build()
        steps.append('rebuild after rollback: %s' % ('ok' if built else 'FAILED'))
        restart()
        back = stack_answers()
        steps.append('stack after rollback: %s' % ('answering' if back else 'NOT ANSWERING'))
        return {'ok': False, 'branch': branch, 'rolled_back_to': was[:12],
                'error': why, 'steps': steps, 'recovered': back}

    ok, out = git('merge', '--ff-only', 'origin/%s' % branch, timeout=120)
    if not ok:
        return {'ok': False, 'branch': branch,
                'error': 'cannot fast-forward - the local branch has diverged: %s'
                         % out[:200]}
    steps.append('updated to %s' % (current_commit() or '')[:12])

    built, out = build()
    steps.append('build: %s' % ('ok' if built else 'FAILED'))
    if not built:
        return rollback('the build failed: %s' % out[-400:])

    if not restart():
        steps.append('restart: could not restart the service')
    time.sleep(settle)

    # The check that matters. A build can succeed and still produce a stack that
    # dies on its first frame - that has happened in this project, to five nodes
    # at once, with every other check green.
    if not stack_answers():
        return rollback('the stack did not come back up after the update')

    steps.append('stack: answering')
    return {'ok': True, 'branch': branch, 'applied': True,
            'from': was[:12], 'to': (current_commit() or '')[:12], 'steps': steps}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--apply', action='store_true')
    ap.add_argument('--json', action='store_true', help='machine-readable output')
    args = ap.parse_args()

    result = check() if args.check else apply()

    if args.json:
        print(json.dumps(result, indent=2))
        return 0 if result.get('ok') else 1

    if not result.get('ok'):
        print('  %s' % result.get('error', 'failed'))
        for line in result.get('steps', []):
            print('    %s' % line)
        return 1

    if args.check:
        print('  branch          : %s' % result['branch'])
        print('  at              : %s' % result['commit'])
        print('  behind / ahead  : %d / %d' % (result['behind'], result['ahead']))
        if result.get('local_changes'):
            print('  local changes   : yes - an update would refuse until they are dealt with')
        if result['update_available']:
            print('  update available:')
            for line in result['changelog']:
                print('      %s' % line)
        else:
            print('  up to date')
    else:
        for line in result.get('steps', []):
            print('  %s' % line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
