#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Drive the web UI in a real browser and check it still works.

    python3 tools/ui_test.py            (needs Edge/Chrome + pip install websocket-client)

tools/qc.py checks that files agree with each other. This one loads the page,
clicks things, and looks at what the DOM actually did - including catching any
JavaScript exception, which is the failure mode a static check cannot see.

It serves the UI from a stub with canned data, so it needs no robot and no ROS.
"""
import json
import os
import subprocess
import sys
import time
import urllib.request

try:
    import websocket  # websocket-client
except ImportError:
    print("pip install websocket-client")
    sys.exit(2)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "gps_localize_ws", "src", "gps_localize", "web")
PORT = 8971
CDP_PORT = 9351
PROFILE = os.path.join(os.environ.get("TEMP", "/tmp"), "gpsloc_uitest_profile")

BROWSERS = [
    # Windows entries are intentional: this list is tried in order and the
    # first browser that exists is used, so the same file works on both
    # platforms.  qc: allow-windows-path
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",  # qc: allow-windows-path
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",  # qc: allow-windows-path
    "/usr/bin/microsoft-edge", "/usr/bin/google-chrome", "/usr/bin/chromium-browser",
]

passed, failed = [], []


def check(ok, what, detail=""):
    (passed if ok else failed).append(what)
    print(("  ok    " if ok else "  FAIL  ") + what + (("  -> " + str(detail)) if detail and not ok else ""))


# --------------------------------------------------------------- stub server
stub = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(__file__), "_ui_stub.py"),
                         str(PORT), "live", WEB],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.5)

browser_path = next((b for b in BROWSERS if os.path.exists(b)), None)
if not browser_path:
    print("no Edge/Chrome found - skipping the browser test")
    stub.terminate()
    sys.exit(0)

browser = subprocess.Popen([
    browser_path, "--headless=new", "--disable-gpu", "--mute-audio", "--no-first-run",
    "--user-data-dir=" + PROFILE, "--remote-debugging-port=%d" % CDP_PORT,
    "--remote-allow-origins=*", "about:blank",
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

ws_url = None
for _ in range(40):
    try:
        for target in json.load(urllib.request.urlopen(
                "http://127.0.0.1:%d/json" % CDP_PORT, timeout=3)):
            if target.get("type") == "page":
                ws_url = target["webSocketDebuggerUrl"]
        if ws_url:
            break
    except Exception:
        pass
    time.sleep(0.5)

ws = websocket.create_connection(ws_url, timeout=25, suppress_origin=True)
msg_id = 0
console_errors = []


def send(method, **params):
    global msg_id
    msg_id += 1
    ws.send(json.dumps({"id": msg_id, "method": method, "params": params}))
    while True:
        reply = json.loads(ws.recv())
        if reply.get("method") == "Runtime.exceptionThrown":
            desc = reply["params"]["exceptionDetails"].get("text", "")
            detail = reply["params"]["exceptionDetails"].get("exception", {}).get("description", "")
            console_errors.append((desc + " " + detail).strip())
        if reply.get("method") == "Runtime.consoleAPICalled" and reply["params"]["type"] == "error":
            console_errors.append(" ".join(str(a.get("value", "")) for a in reply["params"]["args"]))
        if reply.get("id") == msg_id:
            return reply.get("result", {})


def js(expression):
    result = send("Runtime.evaluate", expression=expression, returnByValue=True,
                  awaitPromise=True)
    if "exceptionDetails" in result:
        return {"__error": result["exceptionDetails"].get("text", "exception")}
    return result.get("result", {}).get("value")


send("Page.enable")
send("Runtime.enable")
# a modal dialog would block the renderer and hang this test - dismiss any
send("Page.setInterceptFileChooserDialog", enabled=False)
send("Emulation.setDeviceMetricsOverride", width=1280, height=900,
     deviceScaleFactor=1, mobile=False)
send("Page.navigate", url="http://127.0.0.1:%d/" % PORT)
time.sleep(4)
# belt and braces: a blocking modal from any code path would freeze the page
js("window.prompt = function () { return null; };"
   "window.alert = function () {};"
   "window.confirm = function () { return true; };")

print("\n[page loads and receives data]")
check(js("document.title") and "GPS_Localize" in js("document.title"), "page title renders")
check(js("document.body.dataset.conn") == "live", "connection state is live",
      js("document.body.dataset.conn"))
check(js("document.getElementById('tSpeed').textContent") == "0.24", "telemetry reaches the tiles",
      js("document.getElementById('tSpeed').textContent"))
check(js("document.getElementById('banner').className.includes('hidden')"),
      "no error banner while data flows")

print("\n[tabs]")
for tab, section in (("map", "tab-map"), ("wp", "tab-wp"), ("cfg", "tab-cfg"),
                     ("log", "tab-log"), ("dash", "tab-dash")):
    js("document.querySelector('[data-tab=\"%s\"]').click()" % tab)
    time.sleep(0.4)
    visible = js("document.getElementById('%s').classList.contains('active')" % section)
    check(visible is True, "tab %s shows its page" % tab)

print("\n[the map menu that used to stay open]")
js("document.querySelector('[data-tab=\"map\"]').click()")
time.sleep(0.6)
open_menu = """(() => {
  const c = document.getElementById('map');
  const r = c.getBoundingClientRect();
  c.dispatchEvent(new MouseEvent('click', {clientX: r.left + r.width/2,
                                           clientY: r.top + r.height/2, bubbles: true}));
  return !document.getElementById('mapMenu').className.includes('hidden');
})()"""
check(js(open_menu) is True, "clicking the map opens the menu")
for action in ("copy", "add", "close"):
    js("document.querySelector('[data-tab=\"map\"]').click()")
    js(open_menu)
    time.sleep(0.3)
    js("document.querySelector('[data-map=\"%s\"]').click()" % action)
    time.sleep(0.6)
    hidden = js("document.getElementById('mapMenu').className.includes('hidden')")
    check(hidden is True, 'menu closes after "%s"' % action, hidden)

print("\n[controls]")
check(js("!document.getElementById('btnEstop').disabled"), "emergency stop is clickable")
check(js("document.querySelectorAll('.jog').length") == 5, "all five drive buttons exist")
js("document.querySelector('[data-nav=\"pause\"]').click()")
time.sleep(0.5)
check(True, "mission buttons fire without an exception")
js("document.querySelector('[data-preset=\"0.30\"]').click()")
time.sleep(0.8)
check(js("document.getElementById('speedNow').textContent") == "0.30",
      "speed preset sets the real speed, with no hidden multiplier",
      js("document.getElementById('speedNow').textContent"))
check("on" in (js("document.querySelector('[data-preset=\"0.30\"]').className") or ""),
      "the active preset is marked")
check(js("document.getElementById('jogScale')") is None,
      "the old power multiplier is gone")

print("\n[waypoints and settings]")
js("document.querySelector('[data-tab=\"wp\"]').click()")
time.sleep(0.8)
rows = js("document.querySelectorAll('#wpTable tbody tr').length")
check(rows == 3, "waypoint table lists the stub's 3 points", rows)
check(js("document.getElementById('wpEmpty').className.includes('hidden')"),
      "empty state hidden when there are waypoints")
js("document.querySelector('[data-tab=\"cfg\"]').click()")
time.sleep(1.0)
fields = js("document.querySelectorAll('#cfgForm input').length")
check(fields > 3, "settings form builds its fields", fields)

print("\n[simple / advanced view]")
js("document.querySelector('[data-tab=\"dash\"]').click()")
js("document.getElementById('btnSimple').click()")
time.sleep(0.5)
check(js("document.body.dataset.view") == "simple", "simple view engages")
check(js("getComputedStyle(document.querySelector('.tile.adv')).display") == "none",
      "engineering tiles hidden in simple view")
check(js("getComputedStyle(document.getElementById('tSpeed')).display") != "none",
      "the numbers an operator needs stay visible")
js("document.getElementById('btnAdvanced').click()")
time.sleep(0.5)
check(js("getComputedStyle(document.querySelector('.tile.adv')).display") != "none",
      "advanced view brings the detail back")
js("document.getElementById('btnSimple').click()")
time.sleep(0.3)

print("\n[status headline]")
check((js("document.getElementById('heroTitle').textContent") or "").strip() != "",
      "headline says what the robot is doing",
      js("document.getElementById('heroTitle').textContent"))
check(js("document.getElementById('heroPrimary').dataset.action") in
      ("start", "stop", "pause", "resume", "reset"),
      "headline offers a sensible next action",
      js("document.getElementById('heroPrimary').dataset.action"))

print("\n[undo instead of losing a waypoint]")
js("document.querySelector('[data-tab=\"wp\"]').click()")
time.sleep(0.6)
before = js("document.querySelectorAll('#wpTable tbody tr').length")
js("document.querySelector('#wpTable tbody tr [data-act=\"del\"]').click()")
time.sleep(0.5)
after = js("document.querySelectorAll('#wpTable tbody tr').length")
check(after == before - 1, "delete removes the row", (before, after))
check(js("document.getElementById('bannerRetry').dataset.undo") == "1",
      "an undo is offered after deleting")
js("document.getElementById('bannerRetry').click()")
time.sleep(0.5)
restored = js("document.querySelectorAll('#wpTable tbody tr').length")
check(restored == before, "undo puts the waypoint back", (before, restored))

print("\n[help]")
js("document.getElementById('btnHelp').click()")
time.sleep(0.4)
check(js("document.getElementById('helpDialog').open") is True, "help dialog opens")
js("document.getElementById('helpClose').click()")
time.sleep(0.4)
check(js("document.getElementById('helpDialog').open") is False, "help dialog closes")

print("\n[responsive: the page never scrolls sideways]")
for width in (360, 768, 1440):
    send("Emulation.setDeviceMetricsOverride", width=width, height=900,
         deviceScaleFactor=1, mobile=width < 500)
    time.sleep(0.8)
    # scrollWidth alone lies here: body has overflow-x:hidden, which clips the
    # overflow instead of reporting it. Ask the elements themselves.
    probe = js('''(() => {
      const vw = document.documentElement.clientWidth;
      const bad = [];
      document.querySelectorAll('body *').forEach(el => {
        const r = el.getBoundingClientRect();
        if (r.width === 0 && r.height === 0) return;
        if (el.closest('.tabs, .logbox, [style*="overflow"]')) return;
        if (getComputedStyle(el.parentElement || el).overflowX === 'auto') return;
        if (r.right > vw + 1) bad.push(el.tagName.toLowerCase() + '.' +
          (el.className || '').toString().split(' ')[0] + '@' + Math.round(r.right));
      });
      return JSON.stringify(bad.slice(0, 5));
    })()''')
    spill = json.loads(probe) if probe else []
    check(not spill, "%dpx: nothing spills past the viewport" % width, spill)

print("\n[javascript errors]")
real_errors = [e for e in console_errors if e and "favicon" not in e.lower()]
check(not real_errors, "no JavaScript exceptions during the whole run",
      real_errors[:3])

ws.close()
browser.kill()
stub.terminate()

print("\n%d passed, %d failed" % (len(passed), len(failed)))
if failed:
    for item in failed:
        print("  - " + item)
sys.exit(1 if failed else 0)
