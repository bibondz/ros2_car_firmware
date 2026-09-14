#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Run everything that can be tested on a Windows machine, and write a report.

    python tools\\test_windows.py            (or double-click test_windows.bat)

Windows has no ROS 2, so the ROS nodes cannot be launched here. What CAN be
tested is most of what actually breaks:

  - every file is present and non-empty          (the failure that started this)
  - every Python module compiles under this Python
  - every config YAML parses
  - the no-ROS viewer really serves the UI       (the demo path)
  - the firmware builds with PlatformIO          (the real robot path)
  - the host unit tests pass
  - the Docker setup is valid, and the Windows override is used

Anything the machine cannot do is reported as SKIP with the reason, never as a
pass. The report is written next to the project so it can be read from the
other side of the share:

    windows_test_report.txt     human readable
    windows_test_report.json    machine readable

Nothing is modified. Safe to run repeatedly.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "gps_localize_ws" / "src" / "gps_localize"

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
results: list[dict] = []


def record(name: str, status: str, detail: str = "", extra: list[str] | None = None):
    results.append({"name": name, "status": status, "detail": detail,
                    "extra": extra or []})
    mark = {PASS: "PASS", FAIL: "FAIL", SKIP: "skip"}[status]
    print(f"  {mark}  {name}" + (f"  -  {detail}" if detail else ""))
    for line in (extra or [])[:6]:
        print(f"          {line}")


def section(title: str):
    print(f"\n[{title}]")


def run(cmd: list[str], cwd: Path | None = None, timeout: int = 900):
    """Run a command, never raise. Returns (rc, combined output)."""
    try:
        p = subprocess.run(cmd, cwd=str(cwd) if cwd else None, timeout=timeout,
                           capture_output=True, text=True, errors="replace")
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except FileNotFoundError:
        return 127, f"{cmd[0]} not found"
    except subprocess.TimeoutExpired:
        return 124, f"timed out after {timeout}s"


def tool(name: str) -> str | None:
    """Find an executable, including the .exe/.cmd forms Windows uses."""
    for candidate in (name, name + ".exe", name + ".cmd", name + ".bat"):
        found = shutil.which(candidate)
        if found:
            return found
    # PlatformIO's usual per-user location, which is often not on PATH
    if name == "pio":
        guess = Path.home() / ".platformio" / "penv" / "Scripts" / "platformio.exe"
        if guess.exists():
            return str(guess)
        guess = Path.home() / ".platformio" / "penv" / "bin" / "platformio"
        if guess.exists():
            return str(guess)
    return None


def free_port(preferred: int) -> int:
    """A port the viewer can bind. Falls back if something already holds it."""
    for port in (preferred, preferred + 1, preferred + 2, 0):
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", port))
                return s.getsockname()[1]
            except OSError:
                continue
    return preferred


# --------------------------------------------------------------------------- #

def check_files():
    section("files")
    tracked = []
    for pattern in ("gps_localize_ws/**/*", "webapp/**/*", "firmware/**/*",
                    "tools/*", "setup/*", "docs/*"):
        tracked += [p for p in ROOT.glob(pattern) if p.is_file()]
    skip_parts = {".pio", "build", "install", "log", "__pycache__", "exe"}
    tracked = [p for p in tracked if not (skip_parts & set(p.parts))]

    allowed_empty = {"__init__.py", "gps_localize"}
    empty = [p for p in tracked if p.stat().st_size == 0 and p.name not in allowed_empty]
    record("no zero-byte files", FAIL if empty else PASS,
           f"{len(empty)} empty of {len(tracked)}",
           [str(p.relative_to(ROOT)) for p in empty])

    secrets = ROOT / "firmware" / "config" / "network_secrets.h"
    example = ROOT / "firmware" / "config" / "network_secrets.example.h"
    if secrets.exists():
        record("firmware Wi-Fi credentials present", PASS, "network_secrets.h found")
    elif example.exists():
        record("firmware Wi-Fi credentials present", SKIP,
               "copy network_secrets.example.h to network_secrets.h before building")
    else:
        record("firmware Wi-Fi credentials present", FAIL, "neither file exists")


def check_python():
    section("python")
    record("python version", PASS, platform.python_version())
    mods = sorted(list(PKG.glob("gps_localize/*.py")) +
                  list((ROOT / "webapp").glob("*.py")) +
                  list((ROOT / "tools").glob("*.py")))
    bad = []
    for m in mods:
        # compile() in memory rather than py_compile: py_compile insists on
        # writing a .pyc somewhere, and refuses os.devnull on both platforms.
        try:
            compile(m.read_text(encoding="utf-8", errors="replace"), str(m), "exec")
        except SyntaxError as e:
            bad.append(f"{m.name}:{e.lineno}: {e.msg}")
    record("every python module compiles", FAIL if bad else PASS,
           f"{len(mods)} modules", bad)

    try:
        import yaml
    except ImportError:
        record("config YAML parses", SKIP, "pyyaml not installed (pip install pyyaml)")
        return
    bad = []
    ymls = sorted(PKG.glob("config/*.yaml")) + sorted((ROOT / "webapp").glob("settings/*.yaml"))
    for y in ymls:
        try:
            yaml.safe_load(y.read_text(encoding="utf-8"))
        except Exception as e:
            bad.append(f"{y.name}: {str(e).splitlines()[0]}")
    record("config YAML parses", FAIL if bad else PASS, f"{len(ymls)} files", bad)


def check_viewer():
    """The no-ROS viewer is the whole Windows demo path, so prove it serves."""
    section("web UI without ROS (the viewer)")
    viewer = ROOT / "webapp" / "viewer_server.py"
    if not viewer.exists() or viewer.stat().st_size == 0:
        record("viewer serves the UI", FAIL, "webapp/viewer_server.py missing or empty")
        return

    port = free_port(8099)
    proc = subprocess.Popen(
        [sys.executable, str(viewer), "--host", "127.0.0.1", "--port", str(port),
         "--no-browser"],
        cwd=str(ROOT / "webapp"), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, errors="replace")
    try:
        base = f"http://127.0.0.1:{port}"
        deadline = time.time() + 25
        ready = False
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            try:
                with urllib.request.urlopen(base + "/", timeout=2) as r:
                    if r.status == 200:
                        ready = True
                        break
            except Exception:
                time.sleep(0.5)

        if not ready:
            out = (proc.stdout.read() if proc.stdout else "") or ""
            record("viewer serves the UI", FAIL, "did not answer on " + base,
                   out.strip().splitlines()[-6:])
            return

        problems = []
        for path in ("/", "/app.js", "/style.css"):
            try:
                with urllib.request.urlopen(base + path, timeout=5) as r:
                    if r.status != 200 or len(r.read()) == 0:
                        problems.append(f"{path} -> HTTP {r.status}, empty")
            except Exception as e:
                problems.append(f"{path} -> {e}")
        record("viewer serves the UI", FAIL if problems else PASS,
               f"index, app.js and style.css on port {port}", problems)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def check_firmware(skip_build: bool):
    section("firmware")
    pio = tool("pio") or tool("platformio")
    if not pio:
        record("PlatformIO available", SKIP,
               "not installed - pip install platformio")
        return
    record("PlatformIO available", PASS, pio)

    if not (ROOT / "firmware" / "config" / "network_secrets.h").exists():
        record("firmware builds (env wifi)", SKIP,
               "network_secrets.h missing; copy the .example.h first")
        return
    if skip_build:
        record("firmware builds (env wifi)", SKIP, "--quick given")
        record("host unit tests", SKIP, "--quick given")
        return

    rc, out = run([pio, "run", "-e", "wifi"], cwd=ROOT / "firmware")
    errs = [l for l in out.splitlines() if "error:" in l.lower()][:5]
    record("firmware builds (env wifi)", PASS if rc == 0 else FAIL,
           "clean" if rc == 0 else f"exit {rc}", errs)

    rc, out = run([pio, "test", "-e", "native"], cwd=ROOT / "firmware")
    summary = next((l for l in out.splitlines() if "test cases:" in l), "")
    if "native compiler" in out.lower() or "toolchain" in out.lower() and rc != 0:
        record("host unit tests", SKIP,
               "no native C++ toolchain on this machine (install MinGW / Build Tools)")
    else:
        record("host unit tests", PASS if rc == 0 else FAIL,
               summary.strip() or f"exit {rc}",
               [l for l in out.splitlines() if "FAILED" in l][:6])


def check_windows_network():
    """The link fails silently far more often than it fails loudly.

    Docker publishes the ports, but Windows Firewall drops the inbound packets
    before Docker sees them, so the container is healthy, the UI works locally,
    and the robot simply never connects with nothing in any log to explain it.
    Check the two things that actually cause that.
    """
    if platform.system() != "Windows":
        return
    section("windows networking")

    ps = tool("powershell") or tool("pwsh")
    if not ps:
        record("firewall allows the robot in", SKIP, "powershell not found")
        return

    rc, out = run([ps, "-NoProfile", "-Command",
                   "(Get-NetFirewallRule -DisplayName 'GPS_Localize*' "
                   "-ErrorAction SilentlyContinue).DisplayName"], timeout=90)
    have = [l.strip() for l in out.splitlines() if l.strip()]
    if len(have) >= 2:
        record("firewall allows the robot in", PASS, ", ".join(have))
    else:
        record("firewall allows the robot in", FAIL,
               "inbound UDP 8888 / TCP 8080 are not allowed",
               ["run as Administrator:",
                "  powershell -ExecutionPolicy Bypass -File setup\\setup_windows.ps1",
                "without this the ESP32 cannot reach the agent and the link never"
                " comes up, with no error anywhere"])

    # A Public network profile ignores Private/Domain firewall rules entirely.
    rc, out = run([ps, "-NoProfile", "-Command",
                   "Get-NetConnectionProfile | Where-Object "
                   "{$_.IPv4Connectivity -ne 'Disconnected'} | "
                   "ForEach-Object {\"$($_.Name)=$($_.NetworkCategory)\"}"], timeout=90)
    profiles = [l.strip() for l in out.splitlines() if "=" in l]
    public = [p for p in profiles if p.lower().endswith("=public")]
    record("network is Private, not Public", FAIL if public else PASS,
           ", ".join(profiles) if profiles else "no active network",
           ["a Public profile ignores the firewall rules above:"] +
           [f"  Set-NetConnectionProfile -Name '{p.split('=')[0]}' "
            f"-NetworkCategory Private" for p in public])


def check_docker(bring_up: bool):
    section("docker")
    docker = tool("docker")
    if not docker:
        record("Docker available", SKIP, "Docker Desktop not installed or not running")
        return
    rc, out = run([docker, "info"], timeout=60)
    if rc != 0:
        # These are different problems with different fixes, and calling a
        # permission problem "not running" sends people to restart a daemon
        # that was fine all along.
        low = out.lower()
        if "permission denied" in low or "connect to the docker daemon socket" in low:
            record("Docker available", SKIP,
                   "running, but this user cannot reach the socket",
                   ["sudo usermod -aG docker $USER, then log out and back in",
                    "the group is only applied to new login sessions"])
        else:
            record("Docker available", SKIP,
                   "installed but the daemon is not responding",
                   ["Windows/macOS: start Docker Desktop and wait for the whale",
                    "Linux: sudo systemctl start docker"])
        return
    record("Docker available", PASS, out.split("Server Version:")[-1].strip().splitlines()[0]
           if "Server Version:" in out else "")

    compose = ROOT / "docker-compose.yml"
    win = ROOT / "docker-compose.windows.yml"
    if not compose.exists():
        record("compose files present", SKIP,
               "this checkout is not the docker branch - clone branch 'docker' to test it")
        return
    record("compose files present", PASS if win.exists() else FAIL,
           "docker-compose.yml" + (" + windows override" if win.exists() else
                                   " but docker-compose.windows.yml is MISSING"))
    if not win.exists():
        return

    # network_mode: host does not exist on Docker Desktop, so the override must
    # be the one that applies on Windows.
    args = [docker, "compose", "-f", str(compose), "-f", str(win), "config"]
    rc, out = run(args, cwd=ROOT, timeout=120)
    if rc != 0:
        record("compose config is valid", FAIL, f"exit {rc}", out.strip().splitlines()[:5])
        return
    uses_host = "network_mode: host" in out
    published = "8888" in out and "8080" in out
    record("compose config is valid", PASS)
    record("windows override wins over host networking",
           PASS if (published and not uses_host) else FAIL,
           "ports published, host networking off" if (published and not uses_host)
           else "the override did not take effect")

    if not bring_up:
        record("stack starts and answers", SKIP,
               "not started; pass --up to build and run it (first build is slow)")
        return

    rc, out = run([docker, "compose", "-f", str(compose), "-f", str(win),
                   "up", "-d", "--build"], cwd=ROOT, timeout=3600)
    if rc != 0:
        record("stack starts and answers", FAIL, f"compose up exit {rc}",
               out.strip().splitlines()[-6:])
        return
    ok = False
    deadline = time.time() + 120
    while time.time() < deadline:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8080/", timeout=3) as r:
                if r.status == 200:
                    ok = True
                    break
        except Exception:
            time.sleep(3)
    record("stack starts and answers", PASS if ok else FAIL,
           "web UI on http://localhost:8080" if ok else "no answer on 8080 within 120s")
    run([docker, "compose", "-f", str(compose), "-f", str(win), "down"],
        cwd=ROOT, timeout=300)


# --------------------------------------------------------------------------- #

def write_report(started: float):
    counts = {s: sum(1 for r in results if r["status"] == s) for s in (PASS, FAIL, SKIP)}
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "GPS_Localize - Windows test report",
        "=" * 64,
        f"when     : {stamp}",
        f"machine  : {platform.system()} {platform.release()} ({platform.machine()})",
        f"python   : {platform.python_version()}",
        f"project  : {ROOT}",
        f"duration : {time.time() - started:.1f}s",
        "",
        f"PASS {counts[PASS]}   FAIL {counts[FAIL]}   SKIP {counts[SKIP]}",
        "",
        "A SKIP means this machine could not run that check - it is never a pass.",
        "=" * 64,
        "",
    ]
    for r in results:
        lines.append(f"[{r['status']:<4}] {r['name']}")
        if r["detail"]:
            lines.append(f"         {r['detail']}")
        for e in r["extra"][:10]:
            lines.append(f"           - {e}")
    lines += ["", "=" * 64,
              "VERDICT: " + ("something failed, see FAIL above"
                             if counts[FAIL] else "nothing failed")]

    (ROOT / "windows_test_report.txt").write_text("\n".join(lines) + "\n",
                                                  encoding="utf-8")
    (ROOT / "windows_test_report.json").write_text(json.dumps(
        {"timestamp": stamp, "machine": platform.platform(),
         "python": platform.python_version(), "counts": counts,
         "results": results}, indent=2), encoding="utf-8")
    print("\n" + "\n".join(lines[-3:]))
    print(f"\nreport written to {ROOT / 'windows_test_report.txt'}")
    return counts[FAIL]


def main() -> int:
    ap = argparse.ArgumentParser(description="Test GPS_Localize on Windows")
    ap.add_argument("--quick", action="store_true",
                    help="skip the firmware build and unit tests")
    ap.add_argument("--up", action="store_true",
                    help="also build and start the Docker stack (slow the first time)")
    args = ap.parse_args()

    started = time.time()
    print(f"GPS_Localize - Windows test\n{ROOT}\n")
    check_files()
    check_python()
    check_viewer()
    check_firmware(args.quick)
    check_windows_network()
    check_docker(args.up)
    return 1 if write_report(started) else 0


if __name__ == "__main__":
    sys.exit(main())
