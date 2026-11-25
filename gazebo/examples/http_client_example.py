"""대시보드 HTTP Bridge Client 연동 예제 (표준 라이브러리만 사용).

  python3 examples/http_client_example.py                     # health + telemetry 2 Hz 출력
  python3 examples/http_client_example.py --mission           # set_target 패드 상공 → request_landing
  python3 examples/http_client_example.py --base http://192.168.0.10:8080
"""

import argparse
import json
import time
import urllib.error
import urllib.request


class Bridge:
    def __init__(self, base):
        self.base = base.rstrip('/')

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=3) as r:
            return json.loads(r.read())

    def command(self, command, **parameters):
        """POST /api/command → {"request_id", "accepted", "command_id", "code", "message"}"""
        self.n = getattr(self, 'n', 0) + 1
        body = {'request_id': f'example-{self.n}', 'command': command, 'parameters': parameters}
        req = urllib.request.Request(self.base + '/api/command', data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:      # 400 (JSON 아님)
            return json.loads(e.read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='http://localhost:8080')
    ap.add_argument('--mission', action='store_true')
    a = ap.parse_args()
    b = Bridge(a.base)

    h = b.get('/api/health')
    print('health:', h['api_version'], h['status'], 'simulation_ready =', h['simulation_ready'])
    print('frames:', b.get('/api/frames')['local_frame'])

    phase = 'start' if a.mission else None
    while True:
        t = b.get('/api/telemetry')
        if t['telemetry_id']:
            print(f"[{t['flight_mode']:<9}] {t['timestamp']} xyz=({t['x']:6.2f},{t['y']:6.2f},{t['z']:5.2f}) "
                  f"alt={t['altitude_m']:5.2f} rpy=({t['roll']},{t['pitch']},{t['yaw']}) deg bat={t['battery']}% "
                  f"stream={t['camera_stream_url']}")
            st = t['flight_mode']
            if phase == 'start' and st == 'IDLE':
                print('  →', b.command('set_target', frame='GAZEBO_WORLD_ENU', x=0.0, y=0.0, z=4.0))
                phase = 'goto'
            elif phase == 'goto' and st == 'HOVER':
                print('  →', b.command('request_landing', telemetry_id=t['telemetry_id']))
                phase = 'land'
            elif phase == 'land' and st == 'IDLE':
                print('mission done')
                phase = None
        time.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
