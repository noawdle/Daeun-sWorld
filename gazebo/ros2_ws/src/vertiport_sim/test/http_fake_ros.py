"""HTTP Bridge 실시간 통합 테스트: 가짜 PX4/카메라 + 실제 노드 전부 + 실제 HTTP 요청.

대시보드 API v1 (gazebo_bridge_client.py 규격) 검증: /api/health, /api/telemetry 필드·자료형·순서,
/api/command (set_target / hover / request_landing / cancel_mission, request_id·command_id·code),
/api/frames, /api/perception, /api/scene, /api/camera, /camera MJPEG, snapshot, 뷰어, CORS
실행: python3 test/http_fake_ros.py   (약 30초)
"""
import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, 'fake_ros'), os.path.join(HERE, '..'), HERE]

import rclpy  # noqa: E402
from fake_px4 import FakeCamera, FakePx4, load_params  # noqa: E402

PORT = 18080
BASE = f'http://127.0.0.1:{PORT}'


def req(method, path, body=None, raw=False):
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    r = urllib.request.Request(BASE + path, data=data, method=method, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(r, timeout=5) as resp:
            b = resp.read()
            return resp.status, (b if raw else json.loads(b)), dict(resp.headers)
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read()), dict(e.headers)


def wait_for(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = pred()
        if v:
            return v
        time.sleep(0.2)
    raise AssertionError(f'timeout: {what}')


def main():
    rclpy.reset()
    rclpy.init()
    rclpy.VT[0] = time.monotonic()
    o = [-6.0, 0.0, 0.24]
    load_params(os.path.join(HERE, '..', 'config', 'vertiport.yaml'), {
        'vertiport_commander': {'origin_offset_enu': o},
        'vertiport_telemetry_bridge': {'origin_offset_enu': o},
        'vertiport_http_bridge': {'http_port': PORT, 'http_host': '127.0.0.1'},
        'vertiport_scenario': {'dry_run': True},
        'vertiport_mock_judge': {'auto_mission': False, 'act': False},
    })
    from vertiport_sim.commander_node import CommanderNode
    from vertiport_sim.http_bridge import HttpBridge
    from vertiport_sim.mock_judge_node import MockJudge
    from vertiport_sim.perception_stub import PerceptionStub
    from vertiport_sim.scenario_node import ScenarioNode
    from vertiport_sim.telemetry_bridge import TelemetryBridge
    keep = [FakePx4(), FakeCamera(), CommanderNode(), TelemetryBridge(), HttpBridge(), ScenarioNode(),
            PerceptionStub(), MockJudge()]

    stop = threading.Event()

    def pump():
        while not stop.is_set():
            rclpy.VT[0] = time.monotonic()
            rclpy.advance(0.0)
            time.sleep(0.005)
    threading.Thread(target=pump, daemon=True).start()
    try:
        run_checks()
    finally:
        stop.set()
        del keep


DASH_KEYS = ['telemetry_id', 'timestamp', 'frame', 'id', 'x', 'y', 'z', 'altitude_m', 'roll', 'pitch', 'yaw',
             'attitude_convention', 'speed', 'battery', 'voltage', 'flight_mode', 'armed', 'emergency',
             'latitude', 'longitude', 'gps', 'satellites', 'mavlink', 'camera_stream_url']


def cmd(command, **params):
    body = {'request_id': f'test-{time.time()}', 'command': command, 'parameters': params}
    st, r, _ = req('POST', '/api/command', body)
    assert st == 200 and r['request_id'] == body['request_id'] and r['command_id'].startswith('cmd-'), r
    assert set(r) >= {'request_id', 'accepted', 'command_id', 'code', 'message'}
    return r


def state():
    return req('GET', '/api/telemetry')[1]['flight_mode']


def run_checks():
    ok = lambda m: print('  OK ', m)  # noqa: E731
    wait_for(lambda: req('GET', '/api/health')[1]['status'] == 'ok', 10, 'health ok')
    st, h, hd = req('GET', '/api/health')
    assert st == 200 and hd.get('Access-Control-Allow-Origin') == '*'
    assert h['api_version'] == '1' and h['simulation_ready'] is True
    ok(f"/api/health → api_version {h['api_version']}, status {h['status']}, simulation_ready {h['simulation_ready']}")

    st, t, _ = req('GET', '/api/telemetry')
    assert list(t)[:len(DASH_KEYS)] == DASH_KEYS, list(t)
    assert t['frame'] == 'GAZEBO_WORLD_ENU' and t['attitude_convention'] == 'ENU_FLU_RZYX_DEG'
    assert abs(t['z'] - 0.24) < 0.02 and abs(t['altitude_m']) < 0.02 and abs(t['yaw'] - 90) < 0.1
    assert t['flight_mode'] == 'IDLE' and t['camera_stream_url'] == f'{BASE}/camera'
    assert isinstance(t['battery'], float) and isinstance(t['satellites'], int)
    assert t['gps'] == '3D_FIX' and t['mavlink'] == 'CONNECTED'
    t2 = req('GET', '/api/telemetry')[1]
    time.sleep(0.2)
    t3 = req('GET', '/api/telemetry')[1]
    assert t3['telemetry_id'] != t2['telemetry_id'] and t3['timestamp'] > t2['timestamp'], '샘플마다 새 ID/시각'
    ok(f"/api/telemetry → {json.dumps({k: t[k] for k in DASH_KEYS}, ensure_ascii=False)}")

    r = cmd('take_off')
    assert not r['accepted'] and r['code'] == 'UNKNOWN_COMMAND'
    st, r, _ = req('POST', '/api/command', b'{oops')
    assert st == 400 and r['code'] == 'INVALID_REQUEST'
    r = cmd('hover')
    assert not r['accepted'] and r['code'] == 'REJECTED_STATE'
    r = cmd('set_target', frame='GAZEBO_WORLD_ENU', x=500, y=0, z=4)
    assert r['code'] == 'OUT_OF_BOUNDS'
    r = cmd('set_target', frame='WGS84', x=1, y=0, z=4)
    assert r['code'] == 'UNSUPPORTED_FRAME'
    ok('거부 코드: UNKNOWN_COMMAND, INVALID_REQUEST(400), REJECTED_STATE, OUT_OF_BOUNDS, UNSUPPORTED_FRAME')

    r = cmd('set_target', frame='GAZEBO_WORLD_ENU', x=0, y=0, z=4.24)
    assert r['accepted'] and r['code'] == 'ACCEPTED', r
    ok(f'set_target → {r}')
    wait_for(lambda: state() == 'HOVER', 25, 'HOVER over pad')

    req('POST', '/api/scenario', {'op': 'apply', 'name': 'S3_fod'})
    wait_for(lambda: len(req('GET', '/api/perception')[1].get('fod', [])) == 1, 10, 'perceived fod')
    st, p, _ = req('GET', '/api/perception')
    assert p['landing_pad']['detected'] and p['fod'][0]['gt_error_m'] < 0.1 and p['frame'] == 'GAZEBO_WORLD_ENU'
    wait_for(lambda: (req('GET', '/api/scene')[1].get('decision') or {}).get('decision') == 'NO_LAND', 5, 'decision')
    st, sc, _ = req('GET', '/api/scene')
    assert sc['drone']['flight_mode'] == 'HOVER' and sc['scenario_objects'][0]['id'] == 'fod_1'
    assert sc['landing_pad']['x'] == 0 and sc['camera']['camera_stream_url'].endswith('/camera')
    ok(f"/api/perception, /api/scene → fod err {p['fod'][0]['gt_error_m']} m, decision {sc['decision']['decision']}")

    r = cmd('request_landing', telemetry_id='sample-does-not-exist')
    assert r['code'] == 'UNKNOWN_TELEMETRY'
    tid = req('GET', '/api/telemetry')[1]['telemetry_id']
    r = cmd('request_landing', telemetry_id=tid)
    assert r['accepted'], r
    time.sleep(1.0)
    r = cmd('hover')
    assert r['accepted'] and r['state'] == 'HOLD'
    r = cmd('request_landing', telemetry_id=req('GET', '/api/telemetry')[1]['telemetry_id'])
    assert r['accepted']
    time.sleep(0.5)
    r = cmd('cancel_mission')
    assert r['accepted'] and r['state'] == 'HOVER'
    ok('request_landing(UNKNOWN_TELEMETRY 거부 → 승인) → hover → request_landing → cancel_mission(HOVER)')

    st, fr, _ = req('GET', '/api/frames')
    assert fr['bridge_frame'] == 'GAZEBO_WORLD_ENU' and fr['local_frame']['identity'] is True
    assert fr['vehicle_origin']['z_offset_m'] == 0.24
    ok(f"/api/frames → pad origin {fr['landing_pad_origin_in_gazebo_world']}, yaw {fr['local_frame']['yaw_deg']}")

    st, cam, _ = req('GET', '/api/camera')
    assert len(cam['intrinsics']['K']) == 9 and cam['tf']['base_to_camera_link']['xyz_m'] == [0.0, 0.0, -0.14]
    st, jpg, hd = req('GET', '/api/camera/snapshot.jpg', raw=True)
    from PIL import Image
    im = Image.open(io.BytesIO(jpg))
    assert im.format == 'JPEG' and 'X-Image-Stamp' in hd
    for path in ('/camera', '/stream/perception.mjpg'):
        with urllib.request.urlopen(BASE + path, timeout=5) as resp:
            assert resp.headers['Content-Type'].startswith('multipart/x-mixed-replace')
            chunk = b''
            while chunk.count(b'--frame') < 2:
                chunk += resp.read1(65536)
        assert b'Content-Type: image/jpeg' in chunk and b'X-Image-Stamp' in chunk
    ok(f"/api/camera (K, TF), snapshot {im.size}, MJPEG /camera, /stream/perception.mjpg")

    st, html, _ = req('GET', '/', raw=True)
    assert st == 200 and b'Vertiport HTTP Bridge Viewer' in html
    r = urllib.request.Request(BASE + '/api/command', method='OPTIONS')
    with urllib.request.urlopen(r) as resp:
        assert resp.status == 204
    ok('GET / (뷰어), OPTIONS preflight(CORS)')
    print('\nALL HTTP CHECKS PASSED')


if __name__ == '__main__':
    main()
