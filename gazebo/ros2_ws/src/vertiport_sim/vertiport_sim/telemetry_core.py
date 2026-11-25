"""Telemetry JSON 생성 (ROS 의존성 없음).

docs/INTERFACE.md 의 'telemetry' 규격을 만드는 유일한 함수.
규격을 바꾸면 여기와 INTERFACE.md 를 같이 고칠 것 (SCHEMA_VERSION 올리기).
"""

import math
import time
from typing import Optional, Sequence

from . import frames

SCHEMA_VERSION = '1.1'

NAV_STATE_NAMES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
    5: 'AUTO_RTL', 6: 'POSITION_SLOW', 8: 'ALTITUDE_CRUISE', 10: 'ACRO', 12: 'DESCEND',
    13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND',
    19: 'AUTO_FOLLOW_TARGET', 20: 'AUTO_PRECLAND', 21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
}
NAV_STATE_OFFBOARD = 14
NAV_STATE_AUTO_LAND = 18


def _r(v, n: int = 3):
    # JSON 에 NaN/Inf 가 들어가면 브라우저 JSON.parse 가 실패하므로 None 으로 보낸다
    if v is None:
        return None
    v = float(v)
    return (round(v, n) + 0.0) if math.isfinite(v) else None   # +0.0: -0.0 → 0.0


def build_telemetry(
    *,
    pos_world: Sequence[float],
    vel_world: Sequence[float],
    q_frd_ned_wxyz: Optional[Sequence[float]],
    t_px4_us: int,
    pos_valid: bool,
    armed: bool,
    nav_state: Optional[int],
    landed: Optional[bool],
    failsafe: Optional[bool],
    commander: Optional[dict],
    dist_bottom: Optional[float] = None,
    source: str = 'sim',
    t_now: Optional[float] = None,
    telemetry_id: Optional[str] = None,
    t_sample: Optional[float] = None,
    battery: Optional[dict] = None,
    global_position: Optional[dict] = None,
    gps: Optional[dict] = None,
) -> dict:
    """내부 ROS 토픽 /vertiport/telemetry 용 JSON (rad, 중첩 구조).
    대시보드 HTTP 형식(deg, 평탄 구조)은 dashboard_api.to_dashboard_telemetry() 가 이걸 변환한다."""
    if q_frd_ned_wxyz is not None:
        r, p, y = frames.quat_wxyz_to_euler(q_frd_ned_wxyz)
        roll, pitch, yaw = frames.rpy_frd_ned_to_flu_enu(r, p, y)
        qw, qx, qy, qz = frames.euler_to_quat_wxyz(roll, pitch, yaw)
        quat = {'x': _r(qx, 5), 'y': _r(qy, 5), 'z': _r(qz, 5), 'w': _r(qw, 5)}
    else:
        roll = pitch = yaw = None
        quat = None

    state = commander.get('state') if commander else None
    target = commander.get('target_m') if commander else None
    return {
        'type': 'telemetry',
        'schema': SCHEMA_VERSION,
        'telemetry_id': telemetry_id,
        't': _r(time.time() if t_now is None else t_now, 3),
        't_sample': _r(t_sample, 3),     # PX4 위치 샘플을 받은 실제 시각 (Unix)
        't_px4': _r(t_px4_us / 1e6, 4),
        'frame': 'ENU',
        'origin': 'pad',
        'source': source,
        'position_m': {'x': _r(pos_world[0]), 'y': _r(pos_world[1]), 'z': _r(pos_world[2])},
        'altitude_m': _r(pos_world[2]),
        'rpy_rad': {'roll': _r(roll, 4), 'pitch': _r(pitch, 4), 'yaw': _r(yaw, 4)},
        'orientation_q': quat,   # body FLU → World ENU, {x,y,z,w}
        'velocity_mps': {'x': _r(vel_world[0]), 'y': _r(vel_world[1]), 'z': _r(vel_world[2])},
        'state': state or 'NO_COMMANDER',
        'target_m': (None if not target else
                     {'x': target[0], 'y': target[1], 'z': target[2]}),
        'last_command': commander.get('last_command') if commander else None,
        'valid': bool(pos_valid),
        'px4': {
            'armed': bool(armed),
            'nav_state': nav_state,
            'mode': NAV_STATE_NAMES.get(nav_state, str(nav_state)) if nav_state is not None else None,
            'landed': landed,
            'failsafe': failsafe,
            'dist_bottom_m': None if dist_bottom is None else _r(dist_bottom),
        },
        'battery': battery,                   # {remaining_pct, voltage_v, warning} 또는 None
        'global_position': global_position,   # {lat, lon, alt_amsl_m} 또는 None
        'gps': gps,                           # {fix_type, satellites, eph_m, epv_m} 또는 None
    }
