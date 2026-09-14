#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Wrap a change set into a revertible patch folder.

    python3 tools/make_patch.py start  <name> [files...]   snapshot "before"
    python3 tools/make_patch.py finish <name>              snapshot "after", write diff + rollback

Every patch lands in _patches/<name>_<timestamp>/ with:

    before/            the files exactly as they were
    after/             the files exactly as they are now
    changes.diff       unified diff between the two
    rollback.bat/.sh   copies before/ back over the working tree
    README.md          what changed and how to undo it

Default file set is the web UI (the part that changes most). Pass paths
relative to the project root to patch anything else.

This exists because the project is not in git: without it, a half-finished
edit session has no way back.
"""
import datetime
import difflib
import json
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATCHES = os.path.join(ROOT, "_patches")
DEFAULT_FILES = [
    "gps_localize_ws/src/gps_localize/web/index.html",
    "gps_localize_ws/src/gps_localize/web/style.css",
    "gps_localize_ws/src/gps_localize/web/app.js",
]
STATE = os.path.join(PATCHES, ".open_patch.json")


def newest_patch(name):
    candidates = sorted(d for d in os.listdir(PATCHES)
                        if d.startswith(name + "_") and os.path.isdir(os.path.join(PATCHES, d)))
    return os.path.join(PATCHES, candidates[-1]) if candidates else None


def copy_set(files, dest):
    os.makedirs(dest, exist_ok=True)
    for rel in files:
        src = os.path.join(ROOT, rel)
        if not os.path.exists(src):
            continue
        target = os.path.join(dest, rel.replace("/", "__"))
        shutil.copy2(src, target)


def start(name, files):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    folder = os.path.join(PATCHES, "%s_%s" % (name, stamp))
    copy_set(files, os.path.join(folder, "before"))
    with open(STATE, "w", encoding="utf-8") as fh:
        json.dump({"folder": folder, "files": files}, fh)
    print("patch opened: %s" % folder)
    print("  %d file(s) snapshotted" % len(files))


def finish(name):
    with open(STATE, "r", encoding="utf-8") as fh:
        state = json.load(fh)
    folder, files = state["folder"], state["files"]
    copy_set(files, os.path.join(folder, "after"))

    diff_lines, summary = [], []
    for rel in files:
        flat = rel.replace("/", "__")
        before = os.path.join(folder, "before", flat)
        after = os.path.join(folder, "after", flat)
        old = open(before, encoding="utf-8").read().splitlines(keepends=True) if os.path.exists(before) else []
        new = open(after, encoding="utf-8").read().splitlines(keepends=True) if os.path.exists(after) else []
        diff_lines += list(difflib.unified_diff(old, new, fromfile="a/" + rel, tofile="b/" + rel, n=3))
        added = sum(1 for line in difflib.ndiff(old, new) if line.startswith("+ "))
        removed = sum(1 for line in difflib.ndiff(old, new) if line.startswith("- "))
        if added or removed:
            summary.append("%s  +%d / -%d" % (rel, added, removed))

    with open(os.path.join(folder, "changes.diff"), "w", encoding="utf-8") as fh:
        fh.writelines(diff_lines)

    # rollback scripts, written with raw strings so no escape mangles a path
    bat = ["@echo off", "REM Undo this patch: puts the previous files back.",
           "setlocal", 'cd /d "%~dp0"', ""]
    sh = ["#!/usr/bin/env bash", "set -e", 'cd "$(dirname "$0")"', ""]
    for rel in files:
        flat = rel.replace("/", "__")
        win = rel.replace("/", "\\")
        bat.append('copy /Y "before\\%s" "..\\..\\%s"' % (flat, win))
        sh.append('cp "before/%s" "../../%s"' % (flat, rel))
    bat += ["", "echo.", "echo   Rolled back.",
            "echo   If web files changed, rebuild the viewer: webapp\\build_exe.bat",
            "echo.", "pause"]
    sh += ["", 'echo "Rolled back. Rebuild the viewer if web files changed."']
    with open(os.path.join(folder, "rollback.bat"), "w", encoding="utf-8", newline="\r\n") as fh:
        fh.write("\n".join(bat) + "\n")
    with open(os.path.join(folder, "rollback.sh"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(sh) + "\n")

    with open(os.path.join(folder, "README.md"), "w", encoding="utf-8") as fh:
        fh.write("# Patch: %s\n\n## Files changed\n\n%s\n\n## Roll back\n\n"
                 "    rollback.bat     (Windows)\n    ./rollback.sh    (Ubuntu)\n\n"
                 "Then rebuild the Windows viewer if any web file changed:\n"
                 "`webapp/build_exe.bat`\n" %
                 (os.path.basename(folder),
                  "\n".join("* " + line for line in summary) or "* (no changes)"))

    os.remove(STATE)
    print("patch closed: %s" % folder)
    for line in summary:
        print("  " + line)
    print("  rollback.bat / rollback.sh written")


if __name__ == "__main__":
    os.makedirs(PATCHES, exist_ok=True)
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    action, patch_name = sys.argv[1], sys.argv[2]
    file_list = sys.argv[3:] or DEFAULT_FILES
    if action == "start":
        start(patch_name, file_list)
    elif action == "finish":
        finish(patch_name)
    else:
        print(__doc__)
        sys.exit(2)
