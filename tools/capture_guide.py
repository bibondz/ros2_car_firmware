#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Capture the actual UI using an isolated, explicitly labelled example backend.

Install Playwright and its Chromium browser in a local development environment.
Run: python3 tools/capture_guide.py --root . --out docs/screenshots
This never connects to ROS, opens USB, or forwards requests to the real server.
"""
from __future__ import annotations
import argparse
import ast
import hashlib
import html
import importlib.util
import json
import mimetypes
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import yaml
from playwright.sync_api import sync_playwright


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=Path(__file__).resolve().parent.parent)
    ap.add_argument('--out', type=Path, default=Path('docs/screenshots'))
    args = ap.parse_args()
    root, out = args.root.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    pkg = root / 'gps_localize_ws/src/gps_localize'
    web = pkg / 'web'
    config = {p.stem: yaml.safe_load(p.read_text()) for p in (pkg/'config').glob('*.yaml')}
    # Reuse the repository's screenshot fixture definitions without executing
    # its standalone server or inheriting command-line arguments.
    tree = ast.parse((root/'tools/_ui_stub.py').read_text())
    wanted = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'snapshot'
              or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in
              ('TELEMETRY', 'WAYPOINTS') for t in n.targets)]
    fixture = {'time': time, 'MODE': 'ready', 'MANUAL_SPEED': .18, 'MANUAL_TURN': 45.0}
    exec(compile(ast.Module(body=wanted, type_ignores=[]), '<fixture>', 'exec'), fixture)
    fixture['TELEMETRY'].update(state=0.0, state_name='IDLE', speed_mps=0.0,
        target_speed_mps=0.0, pwm_left=0.0, pwm_right=0.0, battery_v=12.3,
        wifi_rssi_dbm=-55.0, firmware_version=162.0, imu_hz_game=50.0)
    macros = {}
    for header in (root/'firmware/config').glob('*.h'):
        if header.name == 'network_secrets.h':
            continue
        for key, expr in re.findall(r'^#define\s+(\w+)\s+([^\n]+)', header.read_text(), re.M):
            macros[key] = expr.split('//')[0].strip()

    def number(expr, seen=()):
        expr = re.sub(r'(?<=\d)[fFuUlL]+\b', '', expr)
        for key in set(re.findall(r'\b[A-Za-z_]\w*\b', expr)):
            if key in seen or key not in macros:
                raise ValueError(key)
            expr = re.sub(r'\b'+re.escape(key)+r'\b', str(number(macros[key], (*seen, key))), expr)
        if not re.fullmatch(r'[0-9.\s()+*/-]+', expr):
            raise ValueError(expr)
        return float(eval(expr, {'__builtins__': {}}, {}))

    params = json.loads((pkg/'config/params.json').read_text())['params']
    for param in params:
        try:
            param['value'] = number(param['default_expr'])
        except (ValueError, SyntaxError, ZeroDivisionError):
            param['value'] = None
    qr_spec = importlib.util.spec_from_file_location('guide_qr', pkg/'gps_localize/qr.py')
    qr = importlib.util.module_from_spec(qr_spec)
    qr_spec.loader.exec_module(qr)
    example_url = 'http://192.0.2.10:8080'
    api_calls = []

    def snapshot():
        data = fixture['snapshot']()
        data['nav'].update(state='IDLE', distance_m=0.0)
        data['server'].update(lan_url=example_url, uptime_s=742)
        return data

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def reply(self, data, status=200, content_type='application/json'):
            body = json.dumps(data).encode() if content_type == 'application/json' else data
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            path, query = parsed.path, parse_qs(parsed.query)
            if path == '/api/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                try:
                    for _ in range(900):
                        self.wfile.write(('event: telemetry\ndata: '+json.dumps(snapshot())+'\n\n').encode())
                        self.wfile.flush()
                        time.sleep(.5)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if path == '/api/state':
                return self.reply({'ok': True, 'state': snapshot()})
            if path == '/api/config':
                group = query.get('group', ['nav'])[0]
                return self.reply({'ok': True, 'data': config.get(group, {})})
            if path == '/api/params':
                return self.reply({'ok': True, 'params': params, 'moving': False, 'from_robot': True})
            if path == '/api/waypoints':
                return self.reply({'ok': True, 'waypoints': fixture['WAYPOINTS']})
            if path == '/api/logs':
                return self.reply({'ok': True, 'logs': [
                    {'t': '10:00:00', 'source': 'guide', 'message': 'Documentation example; no robot connected.'},
                    {'t': '10:00:01', 'source': 'guide', 'message': 'Illustration of an idle, connected dashboard.'}]})
            if path == '/api/connect':
                return self.reply({'ok': True, 'connect': {'local': 'http://localhost:8080',
                    'lan': example_url, 'mdns': 'http://gps-robot-web.local:8080', 'mdns_may_be_blocked': True}})
            if path == '/api/qr':
                return self.reply({'ok': True, 'url': example_url, 'svg': qr.to_svg(example_url)})
            if path == '/api/find_robot':
                return self.reply({'ok': True, 'linked': True, 'found_at': '192.0.2.20',
                    'robot_name': 'example robot', 'beacon': True, 'agent_ip': '192.0.2.10',
                    'hint': 'Documentation example addresses; replace with your own LAN addresses.'})
            if path == '/api/topics':
                return self.reply({'ok': True, 'topics': [
                    {'name': '/gps_localize/telemetry', 'types': ['gps_localize_msgs/msg/Telemetry'],
                     'publishers': 1, 'subscribers': 4, 'hz': 10},
                    {'name': '/gps_localize/imu/data', 'types': ['sensor_msgs/msg/Imu'],
                     'publishers': 1, 'subscribers': 1, 'hz': 20}]})
            if path == '/api/topic':
                return self.reply({'ok': True, 'type': 'gps_localize_msgs/msg/Telemetry',
                    'hz': 10, 'age_s': .1, 'value': {'battery_v': 12.3, 'speed_mps': 0.0}})
            if path == '/api/bugreport':
                return self.reply({'ok': True, 'reports': []})
            if path.startswith('/api/'):
                return self.reply({'ok': False, 'error': 'Not part of the documentation fixture'}, 404)
            target = (web / ('index.html' if path == '/' else path.lstrip('/'))).resolve()
            if not target.is_relative_to(web) or not target.is_file():
                return self.reply({'ok': False}, 404)
            return self.reply(target.read_bytes(), content_type=mimetypes.guess_type(target.name)[0] or 'application/octet-stream')

        def do_POST(self):
            raw = self.rfile.read(int(self.headers.get('Content-Length', 0)))
            api_calls.append(self.path)
            if self.path == '/api/heartbeat':
                return self.reply({'ok': True})
            # No settings, firmware, calibration, or motion action is executed.
            return self.reply({'ok': False, 'error': 'Documentation fixture: writes disabled'}, 403)

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    captures = []
    errors = []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True)
            page = browser.new_page(viewport={'width': 1440, 'height': 1100}, device_scale_factor=1)
            page.on('pageerror', lambda error: errors.append(str(error)))
            # Map tiles and any accidental external requests cannot leave the fixture.
            page.route('**/*', lambda route: route.continue_() if route.request.url.startswith(url+'/') else route.abort())
            page.goto(url, wait_until='domcontentloaded')
            page.wait_for_function("document.body.dataset.conn === 'live'")
            page.evaluate("""() => {
                const b = document.createElement('div'); b.id = 'documentation-label';
                b.textContent = 'DOCUMENTATION EXAMPLE • Actual UI / sample values • No robot connected';
                b.style.cssText = 'background:#fff1b8;color:#3b2b00;padding:12px;text-align:center;font:bold 16px sans-serif;border-bottom:2px solid #a16c00;';
                document.body.prepend(b);
            }""")
            page.wait_for_timeout(700)

            def capture(name, title, selector=None):
                page.evaluate('window.scrollTo(0, 0)')
                page.wait_for_timeout(250)
                image = out / (name+'.png')
                if selector:
                    page.locator(selector).screenshot(path=str(image))
                else:
                    page.screenshot(path=str(image), full_page=True)
                captures.append({'file': image.name, 'title': title,
                    'kind': 'actual UI with labelled example backend', 'selector': selector})
                print(image.name, flush=True)

            tabs = [('dash', 'Dashboard'), ('map', 'Map'), ('wp', 'Waypoints'),
                    ('cfg', 'Settings'), ('calib', 'Calibration'), ('geom', 'Geometry'),
                    ('board', 'On the robot'), ('topics', 'ROS Topics'), ('log', 'Log'), ('help', 'Help')]
            for index, (tab, title) in enumerate(tabs, 1):
                page.locator(f'[data-tab="{tab}"]').click()
                page.wait_for_timeout(500)
                capture(f'{index:02d}-{tab}', title)
            page.locator('[data-tab="cfg"]').click()
            for index, group in enumerate(['nav', 'pid', 'motor', 'estimator', 'robot', 'safety', 'web']):
                page.locator('#cfgGroups button').nth(index).click()
                page.wait_for_timeout(350)
                capture('settings-'+group, 'Settings / '+group)
            page.locator('[data-tab="board"]').click()
            for group in sorted({p['group'] for p in params}):
                page.locator('#paramGroups button').filter(has_text=re.compile('^'+re.escape(group)+r' \(')).click()
                capture('robot-'+group, 'On the robot / '+group)
            page.locator('[data-tab="wp"]').click()
            page.locator('#wpAddRow').click()
            capture('waypoints-add-row', 'Waypoints / add an editable row (not saved)')
            page.locator('[data-tab="topics"]').click()
            page.locator('#topicsBody button').first.click()
            page.wait_for_timeout(400)
            capture('topics-watch', 'Topics / inspect sample telemetry')
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    metadata = {'created': time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'source_commit': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in web.glob('*') if p.is_file()},
        'note': 'Actual source UI, example telemetry and compiled parameter defaults; no real robot, USB, or ROS access. External requests blocked.',
        'captures': captures, 'browser_errors': errors, 'post_paths': sorted(set(api_calls))}
    (out/'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    if errors:
        raise SystemExit('Browser errors: '+repr(errors))
    print(f'Captured {len(captures)} images; no browser exceptions.', flush=True)


if __name__ == '__main__':
    main()
