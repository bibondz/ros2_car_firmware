#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# Move the project between this machine and the Windows share.
#
#   tools/sync_windows.sh push          here  -> the share (for testing on Windows)
#   tools/sync_windows.sh pull          share -> _work/ for review, NOT the real tree
#   tools/sync_windows.sh diff          what changed on the Windows side
#   tools/sync_windows.sh report        show the last Windows test report
#   tools/sync_windows.sh --dry-run …   print what would move, change nothing
#
# WHY PULL GOES TO _work/
# Anything edited on Windows is unreviewed and untested here. Pulling it
# straight onto the real tree would put changes into git without QC ever
# seeing them - which is exactly the accident this project already had once.
# So a pull lands in _work/, QC runs, and promotion is a separate decision.
#
# CAD, schematics and the git directory are never synced: they are large,
# already on both sides, and .git in particular does not survive a vboxsf
# round trip intact.
set -uo pipefail

SHARE="${GPS_LOCALIZE_SHARE:-/mnt/winshare/GPS_Localize}"
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
WORK="$ROOT/_work"

DRY=""
ACTION=""
for a in "$@"; do
    case "$a" in
        --dry-run) DRY="--dry-run" ;;
        push|pull|diff|report) ACTION="$a" ;;
        -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $a  (try --help)" >&2; exit 2 ;;
    esac
done
[ -z "$ACTION" ] && { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

say()  { printf '  %s\n' "$*"; }
ok()   { printf '  OK    %s\n' "$*"; }
bad()  { printf '  FAIL  %s\n' "$*"; }
head() { printf '\n[%s]\n' "$*"; }

# Only source and tooling move. Everything here is either large, generated,
# or machine-specific.
EXCLUDES=(
    --exclude '.git/'          --exclude '_work/'
    --exclude '_patches/'      --exclude '_backup/'
    --exclude 'model/'         --exclude 'SCH_Schematic*'
    --exclude 'PNG_Schematic*' --exclude '.pio/'
    --exclude 'build/'         --exclude 'install/'
    --exclude 'log/'           --exclude '__pycache__/'
    --exclude '*.pyc'          --exclude '.claude/'
    --exclude 'network_secrets.h'
    # mor_luam belongs to a different project and was removed from this repo.
    # It still lives on the share, and --delete would wipe it from there -
    # which only did not happen the first time because the share refused the
    # delete. Never rely on that.
    --exclude 'mor_luam/'
)

require_share() {
    if [ ! -d "$SHARE" ]; then
        bad "the share is not mounted at $SHARE"
        say "mount it, or set GPS_LOCALIZE_SHARE to where it is"
        exit 1
    fi
}

case "$ACTION" in

push)
    require_share
    head "here -> $SHARE"
    # Refuse to push something that does not pass here first. Sending a broken
    # tree to Windows only produces a confusing test report.
    if [ -z "$DRY" ]; then
        if ! python3 tools/qc.py >/dev/null 2>&1; then
            bad "project QC fails here - fix it before pushing to Windows"
            say "run: python3 tools/qc.py"
            exit 1
        fi
        ok "project QC passes, safe to push"
    fi
    rsync -a --delete $DRY "${EXCLUDES[@]}" \
        --exclude 'windows_test_report.*' \
        "$ROOT"/ "$SHARE"/
    ok "pushed"
    say ""
    say "on Windows now:"
    say "  1. powershell -ExecutionPolicy Bypass -File setup\\setup_windows.ps1   (once, as admin)"
    say "  2. test_windows.bat"
    say "  3. tell me it is done - I pull the report and the changes back"
    ;;

pull)
    require_share
    head "$SHARE -> _work/  (staging, NOT the real tree)"
    mkdir -p "$WORK"
    rsync -a --delete $DRY "${EXCLUDES[@]}" "$SHARE"/ "$WORK"/
    ok "pulled into _work/"

    if [ -n "$DRY" ]; then exit 0; fi

    # Windows writes CRLF. Left alone it makes every file look modified and
    # breaks shebangs, so normalise before anything is compared or promoted.
    local_changed=$(find "$WORK" -type f \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' \
        -o -name '*.yml' -o -name '*.js' -o -name '*.html' -o -name '*.css' \
        -o -name '*.xml' -o -name '*.md' -o -name '*.h' -o -name '*.cpp' \) \
        -not -path '*/.pio/*' -exec grep -lI $'\r' {} + 2>/dev/null | wc -l)
    if [ "$local_changed" -gt 0 ]; then
        find "$WORK" -type f \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' \
            -o -name '*.yml' -o -name '*.js' -o -name '*.html' -o -name '*.css' \
            -o -name '*.xml' -o -name '*.md' -o -name '*.h' -o -name '*.cpp' \) \
            -not -path '*/.pio/*' -print0 \
            | xargs -0 -P "$(nproc)" -n 16 sed -i 's/\r$//'
        ok "normalised CRLF in $local_changed file(s) that Windows rewrote"
    fi
    find "$WORK" -name '*.sh' -exec chmod +x {} + 2>/dev/null

    head "what came back"
    if diff -rq "$ROOT" "$WORK" \
         -x '.git' -x '_work' -x '_patches' -x '_backup' -x 'model' \
         -x '.pio' -x 'build' -x 'install' -x 'log' -x '__pycache__' \
         -x 'plan.html' -x 'windows_test_report.*' -x 'SCH_Schematic*' \
         -x 'PNG_Schematic*' >/tmp/sync_diff.txt 2>&1; then
        ok "identical to what is here - Windows changed nothing"
    else
        say "$(wc -l < /tmp/sync_diff.txt) difference(s):"
        head -20 /tmp/sync_diff.txt | sed 's/^/    /'
        [ "$(wc -l < /tmp/sync_diff.txt)" -gt 20 ] && say "    ... see /tmp/sync_diff.txt"
    fi

    head "QC on what came back"
    if python3 tools/port_qc.py --static-only "$WORK" >/tmp/sync_qc.txt 2>&1; then
        ok "port QC passes on _work/"
    else
        bad "port QC FAILS on _work/ - do not promote"
        grep -A4 'FAIL' /tmp/sync_qc.txt | head -12 | sed 's/^/    /'
    fi

    if [ -f "$SHARE/windows_test_report.txt" ]; then
        head "windows test report"
        sed -n '1,14p' "$SHARE/windows_test_report.txt" | sed 's/^/    /'
        say "    (full report: $SHARE/windows_test_report.txt)"
    fi

    head "next"
    say "review the differences, then promote deliberately - nothing was"
    say "written to the real tree by this command."
    ;;

diff)
    require_share
    head "here vs the share"
    diff -rq "$ROOT" "$SHARE" \
        -x '.git' -x '_work' -x '_patches' -x '_backup' -x 'model' \
        -x '.pio' -x 'build' -x 'install' -x 'log' -x '__pycache__' \
        -x 'plan.html' -x 'windows_test_report.*' -x 'SCH_Schematic*' \
        -x 'PNG_Schematic*' -x 'network_secrets.h' 2>&1 | head -40 | sed 's/^/  /' \
        || true
    ;;

report)
    require_share
    if [ -f "$SHARE/windows_test_report.txt" ]; then
        cat "$SHARE/windows_test_report.txt"
    else
        bad "no report at $SHARE/windows_test_report.txt"
        say "run test_windows.bat on the Windows side first"
        exit 1
    fi
    ;;
esac
