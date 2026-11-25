"""dashboard_api (대시보드 HTTP v1 형식) 테스트."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vertiport_sim import dashboard_api as d  # noqa: E402
from vertiport_sim import frames as f  # noqa: E402
from vertiport_sim.telemetry_core import build_telemetry  # noqa: E402

DASHBOARD_FIELDS = {   # 대시보드가 보낸 예시의 필드와 자료형
    'telemetry_id': str, 'timestamp': str, 'frame': str, 'id': str, 'x': float, 'y': float, 'z': float,
    'altitude_m': float, 'roll': float, 'pitch': float, 'yaw': float, 'attitude_convention': str,
    'speed': float, 'battery': float, 'voltage': float, 'flight_mode': str, 'armed': bool, 'emergency': bool,
    'latitude': float, 'longitude': float, 'gps': str, 'satellites': int, 'mavlink': str,
    'camera_stream_url': str,
}


def sample(yaw_ned=0.0, z_ned=-3.76, battery=True):
    q = f.euler_to_quat_wxyz(0.0, 0.0, yaw_ned)
    return build_telemetry(
        pos_world=(8.0, 2.0, 0.24 - z_ned), vel_world=(0.1, 0.0, -0.06), q_frd_ned_wxyz=q, t_px4_us=52_000_000,
        pos_valid=True, armed=True, nav_state=14, landed=False, failsafe=False,
        commander={'state': 'HOLD', 'target_m': None}, telemetry_id='sample-1942', t_sample=1795545296.123,
        battery={'remaining_pct': 72.5, 'voltage_v': 15.4, 'warning': 0} if battery else None,
        global_position={'lat': 35.8889, 'lon': 128.6103, 'alt_amsl_m': 54.0},
        gps={'fix_type': 3, 'satellites': 12, 'eph_m': 0.8, 'epv_m': 1.2})


def test_fields_and_types_match_dashboard():
    out = d.to_dashboard_telemetry(sample(), 'X500-01', 'http://127.0.0.1:8080/camera', link_ok=True)
    for k, t in DASHBOARD_FIELDS.items():
        assert k in out, k
        assert isinstance(out[k], t), (k, out[k])
    assert list(out)[:len(DASHBOARD_FIELDS)] == list(DASHBOARD_FIELDS), '필드 순서도 예시와 동일'
    assert out['frame'] == 'GAZEBO_WORLD_ENU' and out['attitude_convention'] == 'ENU_FLU_RZYX_DEG'
    assert re.fullmatch(r'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z', out['timestamp'])
    assert out['timestamp'].endswith('56.123Z')
    assert out['flight_mode'] == 'HOLD' and out['telemetry_id'] == 'sample-1942'
    assert out['gps'] == '3D_FIX' and out['mavlink'] == 'CONNECTED'
    assert d.to_dashboard_telemetry(sample(), 'X500-01', None, link_ok=False)['mavlink'] == 'DISCONNECTED'
    assert d.gps_fix_name(6) == 'RTK_FIXED' and d.gps_fix_name(None) is None


def test_units():
    out = d.to_dashboard_telemetry(sample(yaw_ned=0.0), 'X500-01', None)
    assert abs(out['yaw'] - 90.0) < 0.01, 'NED 북쪽 = ENU yaw 90 deg'
    assert abs(out['z'] - 4.0) < 1e-6 and abs(out['altitude_m'] - 3.76) < 1e-6
    assert abs(out['ext']['z_bottom'] - 3.76) < 1e-6   # 대시보드 모델 원점(바닥) 기준 z
    assert abs(out['speed'] - math.sqrt(0.1 ** 2 + 0.06 ** 2)) < 0.01


def test_nulls_when_unavailable():
    out = d.to_dashboard_telemetry(sample(battery=False), 'X500-01', None)
    assert out['battery'] is None and out['voltage'] is None
    e = d.empty_telemetry('X500-01', 'u')
    assert e['x'] is None and e['flight_mode'] == 'DISCONNECTED' and e['frame'] == 'GAZEBO_WORLD_ENU'
    assert e['gps'] is None and e['mavlink'] == 'DISCONNECTED'


def test_local_frame():
    ident = d.LocalFrame()
    assert ident.to_world(1, 2, 3) == (1, 2, 3)
    lf = d.LocalFrame((10.0, 5.0, 0.0), 90.0)
    w = lf.to_world(1.0, 0.0, 2.0)                # local x = world 북쪽
    assert abs(w[0] - 10.0) < 1e-9 and abs(w[1] - 6.0) < 1e-9 and w[2] == 2.0
    back = lf.from_world(*w)
    assert all(abs(a - b) < 1e-9 for a, b in zip(back, (1.0, 0.0, 2.0)))


def test_translate_commands():
    lf, log = d.LocalFrame(), d.TelemetryLog()
    log.add('sample-1', 100.0)
    c, _ = d.translate_command({'request_id': 'r1', 'command': 'set_target',
                                'parameters': {'frame': 'GAZEBO_WORLD_ENU', 'x': 5, 'y': 3, 'z': 4}}, lf, log, 101)
    assert c == {'command': 'GOTO', 'x': 5.0, 'y': 3.0, 'z': 4.0}
    assert d.translate_command({'command': 'hover', 'parameters': {}}, lf, log, 101)[0] == {'command': 'HOLD'}
    assert d.translate_command({'command': 'cancel_mission'}, lf, log, 101)[0] == {'command': 'CANCEL'}
    c, _ = d.translate_command({'command': 'request_landing', 'parameters': {'telemetry_id': 'sample-1'}},
                               lf, log, 101)
    assert c == {'command': 'LAND', 'x': 0.0, 'y': 0.0}
    c, _ = d.translate_command({'command': 'set_target', 'x': 1, 'y': 2, 'z': 3}, lf, log, 101)   # 평탄 형식도 허용
    assert c['command'] == 'GOTO'
    for body, code in [
        ({'command': 'fly'}, 'UNKNOWN_COMMAND'),
        ({}, 'INVALID_REQUEST'),
        ({'command': 'set_target', 'parameters': {'x': 1, 'y': 2}}, 'INVALID_PARAMETERS'),
        ({'command': 'set_target', 'parameters': {'frame': 'WGS84', 'x': 1, 'y': 2, 'z': 3}}, 'UNSUPPORTED_FRAME'),
        ({'command': 'request_landing', 'parameters': {}}, 'INVALID_PARAMETERS'),
        ({'command': 'request_landing', 'parameters': {'telemetry_id': 'nope'}}, 'UNKNOWN_TELEMETRY'),
    ]:
        try:
            d.translate_command(body, lf, log, 101)
        except d.CommandError as e:
            assert e.code == code, (body, e.code)
        else:
            raise AssertionError(body)
    d.translate_command({'command': 'request_landing', 'parameters': {'telemetry_id': 'sample-1'}}, lf, log, 102.9)
    try:
        d.translate_command({'command': 'request_landing', 'parameters': {'telemetry_id': 'sample-1'}}, lf, log, 103.1)
    except d.CommandError as e:
        assert e.code == 'STALE_TELEMETRY'
    else:
        raise AssertionError('3초 넘은 telemetry_id 는 STALE_TELEMETRY')


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
