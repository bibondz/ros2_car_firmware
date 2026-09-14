# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Canned-data server used by tools/ui_test.py (and handy for screenshots).

The live server holds the SSE connection open forever, which stops Chromium's
--screenshot from ever firing. This stub sends ONE telemetry frame and closes
the stream, so the page renders a full "ready" dashboard and then goes idle.

    python stub_server.py <port> <mode>
        mode = ready   one good frame, waypoints present
               empty   one good frame, no waypoints, no GPS datum
               offline stream refused, /api/state fails -> error + retry state
"""
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB = (sys.argv[3] if len(sys.argv) > 3 else
       os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'gps_localize_ws', 'src', 'gps_localize', 'web'))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8192
MODE = sys.argv[2] if len(sys.argv) > 2 else 'ready'
MANUAL_SPEED = 0.18
MANUAL_TURN = 45.0

TELEMETRY = {
    'state': 1.0, 'state_name': 'AUTO', 'heading_deg': 137.4,
    'target_heading_deg': 142.0, 'heading_error_deg': -4.6,
    'speed_mps': 0.24, 'target_speed_mps': 0.25,
    'wheel_rpm_left': 54.8, 'wheel_rpm_right': 58.2,
    'wheel_mps_left': 0.23, 'wheel_mps_right': 0.25,
    'yaw_rate_dps': -3.2, 'pwm_left': 512.0, 'pwm_right': 548.0,
    'battery_v': 11.62, 'battery_a': 1.42,
    'motor_a_v': 11.5, 'motor_a_a': 0.62, 'motor_b_v': 11.5, 'motor_b_a': 0.68,
    'fan_v': 11.6, 'fan_a': 0.31,
    'gps_fix_quality': 1.0, 'gps_satellites': 11.0, 'gps_hdop': 0.9,
    'gps_speed_mps': 0.24, 'gps_course_deg': 137.0,
    'stop_flags': 0.0, 'stop_reasons': [], 'free_heap_kb': 168.0, 'loop_us': 2210.0,
    'odom_x': 12.4, 'odom_y': -3.8,
    'mag_heading_deg': 138.0, 'heading_ref': 2.0,
    'heading_ref_name': 'GPS course', 'mag_status': 1.0, 'mag_status_name': 'ok',
    'mag_calib_state': 0.0, 'mag_calib_name': 'idle',
    'sensor_health': 0.0, 'missing_sensors': [],
    'speed_source': 1.0, 'speed_source_name': 'IMU + GPS',
    'motor_rpm_left': 56.0, 'motor_rpm_right': 57.5,
    'motor_faults': 0.0, 'motor_fault_names': [],
}

WAYPOINTS = [
    {'name': 'ประตูหน้า', 'lat': 13.736717, 'lon': 100.523186},
    {'name': 'ลานจอด', 'lat': 13.736917, 'lon': 100.523486},
    {'name': 'โรงเก็บของ', 'lat': 13.737117, 'lon': 100.523086},
]


def snapshot():
    has_fix = MODE != 'empty'
    return {
        'time': time.time(),
        'telemetry': TELEMETRY,
        'telemetry_age_s': 0.2,
        'fix': ({'lat': 13.736817, 'lon': 100.523286, 'alt': 12.0,
                 'status': 1, 'age_s': 0.3} if has_fix else {}),
        'odom': {'x': 12.4, 'y': -3.8, 'speed': 0.24},
        'nav': ({'state': 'RUNNING', 'index': 1, 'total': 3,
                 'target': WAYPOINTS[1], 'distance_m': 18.4, 'bearing_deg': 142.0,
                 'heading_error_deg': -4.6, 'eta_s': 74.0, 'block_reason': '',
                 'estop': False, 'fix_ok': True, 'heading_ref': 2,
                 'waypoints': WAYPOINTS} if has_fix else
                {'state': 'IDLE', 'index': 0, 'total': 0, 'target': None,
                 'distance_m': 0, 'bearing_deg': 0, 'block_reason': '',
                 'waypoints': []}),
        'safety': {'estop': False, 'link_ok': True, 'esp_ok': True},
        'pose': {'lat': 13.736817, 'lon': 100.523286, 'source': 'gps', 'trusted': True},
        'server': {'mode': 'robot', 'clients': 1, 'uptime_s': 742,
                   'allow_config_edit': True, 'manual_speed_mps': MANUAL_SPEED,
                   'manual_turn_dps': MANUAL_TURN, 'max_speed_mps': 0.42},
    }


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        pass

    def _json(self, payload, status=200):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?')[0]

        if path == '/api/stream':
            if MODE == 'offline':
                self._json({'ok': False, 'error': 'no backend'}, 503)
                return
            if MODE == 'live':
                # keep streaming, so the page stays in the healthy state
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Transfer-Encoding', 'chunked')
                self.end_headers()
                try:
                    for _ in range(80):
                        frame = ('event: telemetry\ndata: %s\n\n'
                                 % json.dumps(snapshot())).encode()
                        self.wfile.write(('%x\r\n' % len(frame)).encode())
                        self.wfile.write(frame + b'\r\n')
                        self.wfile.flush()
                        time.sleep(0.4)
                except Exception:
                    pass
                return

            body = ('retry: 60000\n\nevent: telemetry\ndata: %s\n\n'
                    % json.dumps(snapshot())).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Content-Length', str(len(body)))   # then close
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/api/state':
            if MODE == 'offline':
                self._json({'ok': False, 'error': 'no backend'}, 503)
                return
            self._json({'ok': True, 'state': snapshot()})
            return

        if path == '/api/logs':
            return self._json({'ok': True, 'logs': [
                {'t': '10:41:02', 'source': 'nav', 'message': 'mission started at waypoint 1/3'},
                {'t': '10:41:39', 'source': 'nav', 'message': 'waypoint 1 (ประตูหน้า) reached (0.8 m)'},
                {'t': '10:42:10', 'source': 'web', 'message': 'settings saved: nav'},
            ]})

        if path == '/api/waypoints':
            return self._json({'ok': True,
                               'waypoints': [] if MODE == 'empty' else WAYPOINTS})

        if path == '/api/config':
            return self._json({'ok': True, 'data': {'nav': {
                'cruise_speed_mps': 0.25, 'approach_speed_mps': 0.12,
                'arrive_radius_m': 2.5, 'arrive_hold_s': 1.5,
                'arrive_sigma_k': 1.5, 'loop_waypoints': False,
                'require_fix': True, 'min_satellites': 6, 'max_hdop': 3.0}}})

        # static files
        name = 'index.html' if path in ('/', '') else path.lstrip('/')
        target = os.path.join(WEB, name)
        if not os.path.isfile(target):
            return self._json({'ok': False}, 404)
        ctype = ('text/html' if name.endswith('.html') else
                 'text/css' if name.endswith('.css') else
                 'application/javascript' if name.endswith('.js') else 'text/plain')
        with open(target, 'rb') as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header('Content-Type', ctype + '; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length)
        if MODE == 'offline':
            return self._json({'ok': False, 'error': 'no backend'}, 503)
        try:
            body = json.loads(raw.decode('utf-8')) if raw else {}
        except ValueError:
            body = {}
        path = self.path.split('?')[0]

        # mirror what the real servers answer, so the UI can be tested honestly
        if path == '/api/manual_limits':
            global MANUAL_SPEED, MANUAL_TURN
            if 'speed_mps' in body:
                MANUAL_SPEED = float(body['speed_mps'])
            if 'speed_delta' in body:
                MANUAL_SPEED += float(body['speed_delta'])
            if 'turn_dps' in body:
                MANUAL_TURN = float(body['turn_dps'])
            if 'turn_delta' in body:
                MANUAL_TURN += float(body['turn_delta'])
            MANUAL_SPEED = max(0.02, min(MANUAL_SPEED, 0.42))
            MANUAL_TURN = max(5.0, min(MANUAL_TURN, 180.0))
            return self._json({'ok': True, 'speed_mps': round(MANUAL_SPEED, 3),
                               'turn_dps': round(MANUAL_TURN, 1),
                               'saved': bool(body.get('save'))})

        if path == '/api/waypoints':
            return self._json({'ok': True, 'waypoints': body.get('waypoints', [])})

        self._json({'ok': True})


ThreadingHTTPServer.allow_reuse_address = True
server = ThreadingHTTPServer(('127.0.0.1', PORT), Handler)
print('stub %s on %d' % (MODE, PORT), flush=True)
server.serve_forever()
