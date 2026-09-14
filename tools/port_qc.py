#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""
GPS_Localize QC gate.

This is the single command that decides whether a staged tree in _work/ is allowed to
be promoted onto the real folders. Nothing gets promoted without a green run.

    python3 tools/qc.py _work/                 # all checks
    python3 tools/qc.py --static-only _work/   # checks 1-11 only, no build, no hardware
    python3 tools/qc.py --json _work/          # machine-readable, for plan.html

Exit code 0 means promotion is allowed. Any other value means stop.

Checks 1-11 are static: they only read files, so they run concurrently and every
failure is reported in one pass rather than stopping at the first one.

Checks 12-14 need a build tree, a running micro-ROS agent and one physical robot.
Those are shared, single-instance resources, so they run strictly one at a time and
stop at the first failure.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

# Text file types we lint. Anything not listed is treated as binary and skipped.
TEXT_SUFFIXES = {".py", ".sh", ".yaml", ".yml", ".js", ".html", ".css", ".xml", ".cfg",
                 ".md", ".service", ".ini"}

# Directories that never take part in QC: build output, caches, vendored deps, and the
# patch/backup archives which intentionally hold old copies.
#
# _work is the staging tree. When the gate is pointed at the real root it must not
# also walk staging, or every staged file is reported twice and this script reports
# its own staged copy.
#
# mor_luam is a different robot (BNO055, wheel encoders) that happens to live in this
# repository. It is not part of the Ubuntu port, so it is out of the gate's scope.
SKIP_DIRS = {"__pycache__", ".pio", "build", "install", "log", ".git", "node_modules",
             "_patches", "_backup", "_work", "model", "mor_luam", ".vscode"}

# Generated artifacts that are not source and must not be linted as such.
SKIP_FILES = {"plan.html"}

# Files that are legitimately empty and must not trip the zero-byte check.
ALLOWED_EMPTY = {
    "resource/gps_localize",   # ament index marker: presence is the signal
    "__init__.py",             # a package marker with no exports is normal
}

# This file carries the patterns it hunts for, in string literals. Without this it
# reports itself for every Windows-ism and 3.11+ name it knows how to detect.
# Matched by name as well as by path, because a staged copy of this script under
# _work/ is a different path but carries exactly the same literals.
SELF = Path(__file__).resolve()
SELF_NAME = SELF.name


def is_self(path: Path) -> bool:
    return path.name == SELF_NAME or path.resolve() == SELF


@dataclass
class Result:
    number: int
    name: str
    ok: bool
    detail: str = ""
    failures: list[str] = field(default_factory=list)
    seconds: float = 0.0
    # A check that needs hardware which is not attached is neither a pass nor
    # a failure. Marking it skipped keeps the gate usable on a bench with no
    # robot, without ever letting it claim the link was verified.
    skipped: bool = False

    def to_dict(self) -> dict:
        return {
            "number": self.number,
            "name": self.name,
            "ok": self.ok,
            "detail": self.detail,
            "failures": self.failures[:50],
            "failure_count": len(self.failures),
            "seconds": round(self.seconds, 2),
            "skipped": self.skipped,
        }


def walk_files(root: Path) -> list[Path]:
    """Every file under root worth checking, with build and archive dirs pruned."""
    out: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for name in filenames:
            if name in SKIP_FILES:
                continue
            out.append(Path(dirpath) / name)
    return out


def text_files(files: list[Path]) -> list[Path]:
    return [f for f in files if f.suffix in TEXT_SUFFIXES]


def rel(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None


# --------------------------------------------------------------------------- #
# Static checks (1-11). Each takes (root, files) and returns a Result.
# --------------------------------------------------------------------------- #

def check_01_no_empty(root: Path, files: list[Path]) -> Result:
    """The exact failure that destroyed this project once already."""
    bad = []
    for f in files:
        if f.stat().st_size != 0:
            continue
        r = rel(f, root)
        if f.name in ALLOWED_EMPTY or r.endswith(tuple(ALLOWED_EMPTY)):
            continue
        bad.append(r)
    return Result(1, "no zero-byte files", not bad,
                  f"{len(bad)} empty file(s)" if bad else "all files have content", bad)


def check_02_line_endings(root: Path, files: list[Path]) -> Result:
    """A stray \\r on a shebang makes a script fail with a baffling 'not found'."""
    bad = []
    for f in text_files(files):
        try:
            if b"\r" in f.read_bytes():
                bad.append(rel(f, root))
        except OSError:
            continue
    return Result(2, "LF line endings", not bad,
                  f"{len(bad)} file(s) contain CR" if bad else "no CRLF found", bad)


def check_03_exec_bits(root: Path, files: list[Path]) -> Result:
    """Windows filesystems do not carry the executable bit."""
    bad = []
    for f in files:
        if f.suffix == ".sh" and not os.access(f, os.X_OK):
            bad.append(rel(f, root))
            continue
        # A python file with a shebang is meant to be run directly.
        if f.suffix == ".py":
            try:
                if f.read_bytes()[:2] == b"#!" and not os.access(f, os.X_OK):
                    bad.append(rel(f, root))
            except OSError:
                pass
    return Result(3, "executable bits", not bad,
                  f"{len(bad)} file(s) not executable" if bad else "all scripts executable",
                  bad)


def check_04_py_syntax(root: Path, files: list[Path]) -> Result:
    """Parse with the running interpreter, which is the 3.10 we deploy on."""
    bad = []
    for f in (p for p in files if p.suffix == ".py"):
        src = read_text(f)
        if src is None:
            continue
        try:
            ast.parse(src, filename=str(f))
        except SyntaxError as e:
            bad.append(f"{rel(f, root)}:{e.lineno}: {e.msg}")
    return Result(4, "python syntax", not bad,
                  f"{len(bad)} file(s) fail to parse" if bad else "all modules parse", bad)


PY311_PATTERNS = [
    (re.compile(r"^\s*import\s+tomllib", re.M), "tomllib is 3.11+"),
    (re.compile(r"\bExceptionGroup\b"), "ExceptionGroup is 3.11+"),
    (re.compile(r"^\s*except\s*\*", re.M), "except* is 3.11+"),
    (re.compile(r"\bfrom\s+typing\s+import\s+[^\n]*\b(Self|TypeVarTuple|Unpack|LiteralString)\b"),
     "typing.Self / TypeVarTuple / Unpack / LiteralString are 3.11+"),
    (re.compile(r"\bitertools\.batched\b"), "itertools.batched is 3.12+"),
    (re.compile(r"@override"), "typing.override is 3.12+"),
]


def check_05_py310_compat(root: Path, files: list[Path]) -> Result:
    """The Windows side ran 3.12; the robot runs 3.10.12."""
    bad = []
    for f in (p for p in files if p.suffix == ".py" and not is_self(p)):
        src = read_text(f)
        if src is None:
            continue
        for pattern, why in PY311_PATTERNS:
            if pattern.search(src):
                bad.append(f"{rel(f, root)}: {why}")
    return Result(5, "python 3.10 compatible", not bad,
                  f"{len(bad)} incompatibility(ies)" if bad else "no 3.11+ only syntax", bad)


def check_06_yaml_valid(root: Path, files: list[Path]) -> Result:
    try:
        import yaml
    except ImportError:
        return Result(6, "yaml valid", True, "SKIPPED - pyyaml not installed")
    bad = []
    found = 0
    for f in (p for p in files if p.suffix in {".yaml", ".yml"}):
        found += 1
        src = read_text(f)
        if src is None:
            continue
        try:
            yaml.safe_load(src)
        except yaml.YAMLError as e:
            bad.append(f"{rel(f, root)}: {str(e).splitlines()[0]}")
    return Result(6, "yaml valid", not bad,
                  f"{len(bad)} invalid" if bad else f"{found} yaml file(s) parse", bad)


WINDOWSISMS = [
    # A drive letter is a SINGLE character with nothing word-like before it.
    # Without the word boundary this also matched things like the docker
    # volume mapping "gps_localize_data:/root/..." , which is not a path.
    (re.compile(r"\b[A-Za-z]:[\\/](?!/)"), "drive letter path"),
    (re.compile(r"\bCOM\d\b"), "COM port - use /dev/ttyUSB* or /dev/serial/by-id/*"),
    (re.compile(r"\bcp(?:874|1252|437)\b"), "Windows codepage - use utf-8"),
    (re.compile(r"os\.startfile|winreg|pywin32|win32api"), "Windows-only API"),
]


def check_07_no_windows_paths(root: Path, files: list[Path]) -> Result:
    """Comment lines are exempt: documenting the Windows equivalent is fine."""
    bad = []
    for f in text_files(files):
        if is_self(f):
            continue
        # Markdown is prose. Documentation legitimately shows the Windows way of
        # doing something next to the Linux way, and that is not a port defect.
        if f.suffix == ".md":
            continue
        src = read_text(f)
        if src is None:
            continue
        for i, line in enumerate(src.splitlines(), 1):
            stripped = line.lstrip()
            if stripped.startswith(("#", "//", "*", "<!--", "rem ", "REM ")):
                continue
            # Explicit opt-out for genuinely cross-platform code: a list that
            # carries Windows and POSIX paths side by side and picks whichever
            # exists is correct, not a leftover.
            if "qc: allow-windows-path" in line:
                continue
            for pattern, why in WINDOWSISMS:
                if pattern.search(line):
                    bad.append(f"{rel(f, root)}:{i}: {why}")
                    break
    return Result(7, "no windows-isms", not bad,
                  f"{len(bad)} occurrence(s)" if bad else "clean", bad)


def find_pkg(root: Path) -> Path | None:
    """Locate the ament package under root.

    Skip names are matched against the path *relative to root*, never the absolute
    path. Pointing the gate at _work/ must still find the package inside it, even
    though "_work" is a skip name when scanning the real tree from above.

    The workspace holds two packages now: gps_localize (the nodes, ament_python)
    and gps_localize_msgs (the Telemetry message, ament_cmake). These checks are
    about the node package - entry points, setup.py, rclpy - so the one with a
    setup.py is the one to return. Taking whichever package.xml sorted first
    made them test the message package and report it had no setup.py.
    """
    found = []
    for p in sorted(root.rglob("package.xml")):
        try:
            parts = p.relative_to(root).parts
        except ValueError:
            parts = p.parts
        if not any(part in SKIP_DIRS for part in parts):
            found.append(p.parent)
    for d in found:
        if (d / "setup.py").exists():
            return d
    return found[0] if found else None


def check_08_package_manifest(root: Path, files: list[Path]) -> Result:
    import xml.etree.ElementTree as ET
    pkg = find_pkg(root)
    if pkg is None:
        return Result(8, "package.xml", False, "no package.xml found under the tree",
                      ["package.xml missing"])
    manifest = pkg / "package.xml"
    try:
        tree = ET.parse(manifest)
    except ET.ParseError as e:
        return Result(8, "package.xml", False, f"malformed: {e}", [str(e)])
    rootel = tree.getroot()
    bad = []
    if rootel.findtext("name") is None:
        bad.append("<name> missing")
    build_type = rootel.findtext("./export/build_type")
    if build_type != "ament_python":
        bad.append(f"export/build_type is {build_type!r}, expected 'ament_python'")
    declared = {e.text for e in rootel.iter() if e.tag.endswith("depend") and e.text}
    for need in ("rclpy", "std_msgs", "sensor_msgs", "geometry_msgs", "nav_msgs"):
        if need not in declared:
            bad.append(f"missing <depend>{need}</depend>")
    return Result(8, "package.xml", not bad,
                  f"{len(bad)} problem(s)" if bad else "manifest well-formed", bad)


def check_09_entry_points(root: Path, files: list[Path]) -> Result:
    """Every console_scripts target must resolve to a module that exists."""
    pkg = find_pkg(root)
    if pkg is None:
        return Result(9, "entry points", False, "no package found", ["package.xml missing"])
    setup_py = pkg / "setup.py"
    if not setup_py.exists() or setup_py.stat().st_size == 0:
        return Result(9, "entry points", False, "setup.py missing or empty",
                      ["setup.py missing or empty"])
    src = read_text(setup_py) or ""
    bad = []
    targets = re.findall(r"['\"]([\w_]+)\s*=\s*([\w_.]+):([\w_]+)['\"]", src)
    if not targets:
        bad.append("no console_scripts entry points declared")
    for script, module, func in targets:
        mod_path = pkg / (module.replace(".", "/") + ".py")
        if not mod_path.exists():
            bad.append(f"{script}: module {module} -> {rel(mod_path, root)} not found")
        elif mod_path.stat().st_size == 0:
            bad.append(f"{script}: module {module} is empty")
        else:
            body = read_text(mod_path) or ""
            if not re.search(rf"^\s*def\s+{re.escape(func)}\s*\(", body, re.M):
                bad.append(f"{script}: {module} has no def {func}()")
    return Result(9, "entry points", not bad,
                  f"{len(bad)} broken" if bad else f"{len(targets)} entry point(s) resolve",
                  bad)


def check_10_data_files(root: Path, files: list[Path]) -> Result:
    """Without these, the web UI 404s after install and launch files vanish."""
    pkg = find_pkg(root)
    if pkg is None:
        return Result(10, "data_files installed", False, "no package found",
                      ["package.xml missing"])
    src = read_text(pkg / "setup.py") or ""
    bad = []
    for need in ("config", "launch", "web"):
        if not (pkg / need).is_dir():
            continue
        if not re.search(rf"['\"][^'\"]*{need}[^'\"]*['\"]", src):
            bad.append(f"setup.py data_files does not install {need}/")
    return Result(10, "data_files installed", not bad,
                  f"{len(bad)} directory(ies) not installed" if bad
                  else "config/launch/web installed", bad)


# The firmware is the authoritative contract; parse it rather than hardcoding, so the
# check stays honest when the firmware changes.
def firmware_contract(root: Path) -> tuple[set[str], int]:
    main_cpp = None
    for candidate in (root / "firmware/src/main.cpp",
                      root.parent / "firmware/src/main.cpp"):
        if candidate.exists() and candidate.stat().st_size > 0:
            main_cpp = candidate
            break
    if main_cpp is None:
        return set(), 0
    src = read_text(main_cpp) or ""
    ns_match = re.search(r'#define\s+ROS_NS\s+"([^"]+)"', src)
    ns = ns_match.group(1) if ns_match else "/gps_localize"
    topics = {ns + t for t in re.findall(r'ROS_NS\s+"(/[^"]+)"', src)}
    len_match = re.search(r"#define\s+TELEMETRY_LEN\s+(\d+)", src)
    telem_len = int(len_match.group(1)) if len_match else 0

    # Telemetry used to be a packed float array, so its width was a #define in
    # main.cpp. It is a named message now, and that #define is gone - which made
    # this return 0 and silently switched OFF the entire telemetry half of check
    # 11 while the check still reported PASS. The field count now comes from the
    # message definition, which is the thing that actually decides the layout.
    if not telem_len:
        for msg in (root / "gps_localize_ws/src/gps_localize_msgs/msg/Telemetry.msg",
                    root / "firmware/extra_packages/gps_localize_msgs/msg/Telemetry.msg"):
            if msg.exists():
                body = read_text(msg) or ""
                telem_len = sum(
                    1 for line in body.splitlines()
                    if line.strip() and not line.lstrip().startswith("#")
                )
                if telem_len:
                    break
    return topics, telem_len


def telemetry_widths(files: list[Path]) -> dict[str, int]:
    """How wide each host node believes the telemetry array is.

    Parsed with ast rather than regex so a names tuple spanning fifty lines, with
    comments on every entry, is counted correctly. Returns {"file:NAME": width}.
    """
    widths: dict[str, int] = {}
    for f in (p for p in files if p.suffix == ".py" and not is_self(p)):
        src = read_text(f)
        if src is None or "telem" not in src.lower():
            continue
        try:
            tree = ast.parse(src)
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if not isinstance(target, ast.Name):
                    continue
                name = target.id
                value = node.value
                if name.endswith("_LEN") or name in {"TELEMETRY_LEN", "TELEM_LEN"}:
                    if isinstance(value, ast.Constant) and isinstance(value.value, int):
                        widths[f"{f.name}:{name}"] = value.value
                elif name in {"FIELDS", "TELEMETRY_FIELDS", "TELEM_FIELDS", "NAMES"}:
                    if isinstance(value, (ast.Tuple, ast.List)):
                        widths[f"{f.name}:{name}"] = len(value.elts)
    return widths


def check_11_contract_match(root: Path, files: list[Path]) -> Result:
    """Host topics and telemetry width must equal what the ESP32 actually publishes."""
    expected, telem_len = firmware_contract(root)
    if not expected:
        return Result(11, "firmware contract", True,
                      "SKIPPED - firmware/src/main.cpp not readable")
    py_src = "\n".join(read_text(f) or "" for f in files if f.suffix == ".py")
    if not py_src.strip():
        return Result(11, "firmware contract", False,
                      "no python source to check against the contract",
                      ["host nodes are empty or missing"])
    # A check that silently checks nothing is worse than one that fails: it
    # reports PASS. If the firmware is present, its telemetry width must be
    # discoverable, and not being able to find it is itself the failure.
    bad = []
    if not telem_len:
        bad.append("cannot determine the telemetry width from the firmware or "
                   "the message definition - this check would otherwise pass "
                   "while verifying nothing")
    for topic in sorted(expected):
        short = topic.rsplit("/", 1)[-1]
        # Host nodes write topics three ways, and all three are legitimate:
        # the full path, the last segment, or - most commonly - NS + the rest,
        # which is how every node in this workspace does it. Missing the third
        # made this report a subscribed topic as unreferenced.
        parts = topic.split('/')
        suffix = '/' + '/'.join(parts[2:]) if len(parts) > 2 else topic
        if (topic not in py_src
                and f'"{short}"' not in py_src and f"'{short}'" not in py_src
                and f"'{suffix}'" not in py_src and f'"{suffix}"' not in py_src):
            bad.append(f"topic {topic} is never referenced by any host node")
    # The decoder must cover every field the firmware fills. A host node may express
    # that three ways, in descending order of quality: a declared length constant, a
    # names sequence it enumerates, or literal indexing. Accept all three.
    if telem_len:
        widths = telemetry_widths(files)
        if widths:
            for where, width in sorted(widths.items()):
                if width != telem_len:
                    bad.append(f"{where} declares {width} telemetry fields, "
                               f"firmware publishes {telem_len}")
        else:
            idx = {int(m) for m in
                   re.findall(r"(?:telem|t|data|msg\.data)\s*\[\s*(\d+)\s*\]", py_src)}
            if not idx:
                bad.append(f"no telemetry decoder found; "
                           f"firmware publishes {telem_len} floats")
            else:
                missing = sorted(set(range(telem_len)) - idx)
                if missing:
                    bad.append(f"telemetry indices never decoded: {missing}")
                over = sorted(i for i in idx if i >= telem_len)
                if over:
                    bad.append(f"telemetry indices out of range "
                               f"(len={telem_len}): {over}")
    return Result(11, "firmware contract", not bad,
                  f"{len(bad)} mismatch(es)" if bad
                  else f"{len(expected)} topics + {telem_len} telemetry fields match", bad)


STATIC_CHECKS = [
    check_01_no_empty, check_02_line_endings, check_03_exec_bits, check_04_py_syntax,
    check_05_py310_compat, check_06_yaml_valid, check_07_no_windows_paths,
    check_08_package_manifest, check_09_entry_points, check_10_data_files,
    check_11_contract_match,
]


# --------------------------------------------------------------------------- #
# Live checks (12-14). Shared single-instance resources: strictly serial.
# --------------------------------------------------------------------------- #

def run(cmd: list[str], cwd: Path | None = None, timeout: int = 600):
    return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)


def check_12_colcon_build(root: Path) -> Result:
    ws = None
    for candidate in root.rglob("src"):
        if (candidate.parent / "src").is_dir() and find_pkg(candidate):
            ws = candidate.parent
            break
    if ws is None:
        return Result(12, "colcon build", False, "no colcon workspace found",
                      ["workspace with src/ + package.xml not found"])
    cmd = ["colcon", "build", "--symlink-install",
           "--parallel-workers", str(os.cpu_count() or 4)]
    try:
        proc = run(cmd, cwd=ws)
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return Result(12, "colcon build", False, f"could not run colcon: {e}", [str(e)])
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout).strip().splitlines()[-20:]
        return Result(12, "colcon build", False,
                      f"build failed (exit {proc.returncode})", tail)
    warnings = [l for l in (proc.stdout + proc.stderr).splitlines()
                if "warning" in l.lower()]
    return Result(12, "colcon build", True,
                  f"build clean ({len(warnings)} warning(s))", warnings[:20])


def check_13_node_smoke(root: Path) -> Result:
    """A node that dies in the first 5 s is broken regardless of what it prints."""
    pkg = find_pkg(root)
    if pkg is None:
        return Result(13, "node smoke test", False, "no package found", ["package missing"])
    src = read_text(pkg / "setup.py") or ""
    scripts = [m[0] for m in re.findall(r"['\"]([\w_]+)\s*=\s*([\w_.]+):([\w_]+)['\"]", src)]
    pkg_name = pkg.name
    bad = []
    conflicts = []
    for script in scripts:
        try:
            proc = subprocess.Popen(["ros2", "run", pkg_name, script],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True)
        except FileNotFoundError:
            return Result(13, "node smoke test", False, "ros2 not on PATH", ["ros2 missing"])
        time.sleep(5)
        if proc.poll() is not None:
            err_text = proc.stderr.read() or ""
            err = err_text.strip().splitlines()[-5:]
            # A node that cannot bind its port is not a broken node. Once the
            # systemd service is installed it owns 8080, so the smoke copy is
            # refused - which would otherwise fail this check on every machine
            # where the install actually succeeded.
            if "Address already in use" in err_text:
                conflicts.append(
                    f"{script}: port already held, most likely by the "
                    f"gps_localize service (systemctl stop gps_localize to smoke-test it)")
            else:
                bad.append(f"{script} exited {proc.returncode}: {' | '.join(err)}")
        else:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    if bad:
        detail = f"{len(bad)} node(s) crashed"
    elif conflicts:
        detail = (f"{len(scripts) - len(conflicts)} node(s) stayed up 5s, "
                  f"{len(conflicts)} could not bind a port already in use")
    else:
        detail = f"{len(scripts)} node(s) stayed up 5s"
    return Result(13, "node smoke test", not bad, detail, bad + conflicts)


def firmware_node_name(root: Path) -> str:
    """The node name the ESP32 announces, read from the firmware config."""
    for candidate in (root / "firmware/config/network.h",
                      root.parent / "firmware/config/network.h"):
        if candidate.exists():
            m = re.search(r'#define\s+ROS_NODE_NAME\s+"([^"]+)"',
                          read_text(candidate) or "")
            if m:
                return "/" + m.group(1)
    return "/gps_localize_firmware"


def check_14_live_link(root: Path) -> Result:
    """The ESP32 really is talking to the agent.

    This is the one check that needs the robot. With no board attached there is
    nothing to verify, so it reports skipped rather than failing - otherwise the
    gate could never pass on a bench, and a gate that can never pass gets
    ignored. It only fails when the firmware IS in the graph but its telemetry
    has stopped, which is a genuine fault.
    """
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "10"))
    try:
        proc = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                              text=True, timeout=30, env=env)
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return Result(14, "live link", True, f"SKIPPED - ros2 unavailable: {e}",
                      skipped=True)

    topics = [t for t in proc.stdout.splitlines() if "gps_localize" in t]
    expected, _ = firmware_contract(root)

    # Topic presence does NOT mean the board is connected: a host node that
    # merely SUBSCRIBES to /gps_localize/gps/fix puts that topic in the graph
    # with no ESP32 anywhere. The firmware NODE only appears when the board is
    # actually talking through the agent, so that is the signal.
    node_name = firmware_node_name(root)
    try:
        nodes = subprocess.run(["ros2", "node", "list"], capture_output=True,
                               text=True, timeout=20, env=env).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        nodes = ""
    if node_name not in nodes:
        return Result(14, "live link", True,
                      f"SKIPPED - {node_name} is not in the graph, no board attached",
                      ["connect the ESP32 and run again to verify the link",
                       f"{len(topics)} host-side gps_localize topics are up"],
                      skipped=True)

    bad = []
    missing = sorted(set(expected) - set(topics))
    if missing:
        bad.append(f"{len(topics)}/{len(expected)} topics present; missing: {missing}")

    # 'ros2 topic hz' NEVER exits on its own, so running it with a timeout
    # always raised TimeoutExpired and this check reported "telemetry has
    # stopped" even while the board was publishing at 8 Hz. It only ever looked
    # correct because it skipped when no board was attached.
    #
    # 'echo --once' terminates as soon as one message arrives, so it answers
    # the actual question: is anything being published right now?
    try:
        echo = subprocess.run(
            ["ros2", "topic", "echo", "--once", "/gps_localize/telemetry"],
            capture_output=True, text=True, timeout=20, env=env)
        if echo.returncode != 0 or "data:" not in echo.stdout:
            bad.append("firmware is in the graph but telemetry is not arriving")
    except subprocess.TimeoutExpired:
        bad.append("no telemetry message within 20 s, though the firmware node is up")

    return Result(14, "live link", not bad,
                  f"{len(bad)} problem(s)" if bad
                  else f"{len(topics)} topics live, telemetry flowing", bad)


LIVE_CHECKS = [check_12_colcon_build, check_13_node_smoke, check_14_live_link]


# --------------------------------------------------------------------------- #

def main() -> int:
    ap = argparse.ArgumentParser(description="GPS_Localize QC gate")
    ap.add_argument("root", nargs="?", default="_work",
                    help="tree to check (default: _work)")
    ap.add_argument("--static-only", action="store_true",
                    help="checks 1-11 only: no build, no hardware")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    if not root.is_dir():
        print(f"qc: {root} is not a directory", file=sys.stderr)
        return 2

    files = walk_files(root)
    results: list[Result] = []

    # Static checks are independent readers: run them all at once so a single pass
    # reports every failure instead of stopping at the first.
    with ThreadPoolExecutor(max_workers=min(len(STATIC_CHECKS), (os.cpu_count() or 4) * 2)) as pool:
        futures = []
        for fn in STATIC_CHECKS:
            start = time.monotonic()
            futures.append((fn, start, pool.submit(fn, root, files)))
        for fn, start, fut in futures:
            try:
                r = fut.result()
            except Exception as e:                      # a check must never take QC down
                r = Result(0, fn.__name__, False, f"check raised: {e}", [repr(e)])
            r.seconds = time.monotonic() - start
            results.append(r)

    results.sort(key=lambda r: r.number)

    # Live checks share a build tree, one UDP port and one robot. Serial, stop at first
    # failure so a broken build does not cascade into confusing hardware errors.
    if not args.static_only:
        for fn in LIVE_CHECKS:
            start = time.monotonic()
            try:
                r = fn(root)
            except Exception as e:
                r = Result(0, fn.__name__, False, f"check raised: {e}", [repr(e)])
            r.seconds = time.monotonic() - start
            results.append(r)
            if not r.ok:
                break

    passed = all(r.ok for r in results)

    if args.json:
        print(json.dumps({
            "root": str(root),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "static_only": args.static_only,
            "promotion_allowed": passed,
            "checks": [r.to_dict() for r in results],
        }, indent=2))
        return 0 if passed else 1

    width = max(len(r.name) for r in results) + 2
    print(f"\nQC  {root}   {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * (width + 34))
    for r in results:
        mark = "skip" if r.skipped else ("PASS" if r.ok else "FAIL")
        print(f"  {r.number:>2}  {mark}  {r.name:<{width}} {r.detail}  ({r.seconds:.2f}s)")
        if not r.ok:
            for line in r.failures[:15]:
                print(f"          - {line}")
            if len(r.failures) > 15:
                print(f"          ... and {len(r.failures) - 15} more")
    print("-" * (width + 34))
    if args.static_only and passed:
        print("STATIC CHECKS PASS - run without --static-only before promoting\n")
    elif passed:
        print("ALL CHECKS PASS - promotion allowed\n")
    else:
        print("QC FAILED - promotion blocked\n")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
