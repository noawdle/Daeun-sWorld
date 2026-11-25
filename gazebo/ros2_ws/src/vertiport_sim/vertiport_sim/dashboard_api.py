"""대시보드 HTTP API v1 형식 변환 (ROS 의존성 없음 → 단위 테스트 가능).

대시보드 코드(gazebo_bridge_client.py, telemetry_provider.py)가 파싱하는 필드 이름·자료형에 맞춘다.
  - 좌표: GAZEBO_WORLD_ENU (X East, Y North, Z Up, m)
  - 자세: deg, ENU_FLU_RZYX_DEG  (R = Rz(yaw)·Ry(pitch)·Rx(roll), 기체 FLU → World ENU)
  - 시간: ISO 8601 UTC (밀리초, 'Z')
  - 못 주는 값: null
내부 ROS 토픽(/vertiport/telemetry, rad·중첩 구조)은 그대로 두고 여기서만 변환한다.
"""

import math
from collections import OrderedDict
from datetime import datetime, timezone
from typing import Optional, Tuple

API_VERSION = '1'
FRAME = 'GAZEBO_WORLD_ENU'
LOCAL_FRAME = 'VERTIPORT_LOCAL_XYZ_UP'
ATTITUDE_CONVENTION = 'ENU_FLU_RZYX_DEG'

# 대시보드 명령 → Commander 명령
DASHBOARD_COMMANDS = {
    'set_target': 'GOTO',
    'hover': 'HOLD',
    'request_landing': 'LAND',
    'cancel_mission': 'CANCEL',
}
NATIVE_COMMANDS = ('GOTO', 'HOLD', 'LAND', 'RESUME', 'CANCEL')   # 판단 프로그램·테스트 도구용 (확장)

BATTERY_WARNING_CRITICAL = 2
VEHICLE_ORIGIN_Z_OFFSET_M = 0.24   # 기체 중심(base_link) - 기체 바닥(스키드) 높이 차 (X500)

# PX4 SensorGps.fix_type → 대시보드 gps 문자열
GPS_FIX_NAMES = {0: 'NO_GPS', 1: 'NO_FIX', 2: '2D_FIX', 3: '3D_FIX', 4: 'DGPS', 5: 'RTK_FLOAT',
                 6: 'RTK_FIXED', 8: 'EXTRAPOLATED'}


def gps_fix_name(fix_type) -> Optional[str]:
    if fix_type is None:
        return None
    return GPS_FIX_NAMES.get(int(fix_type), 'UNKNOWN')


def iso_utc(t: Optional[float]) -> Optional[str]:
    if t is None:
        return None
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def _deg(v):
    return None if v is None else round(math.degrees(v), 2)


def _r(v, n=3):
    return None if v is None else round(float(v), n) + 0.0


# ---------------------------------------------------------------- 좌표계
class LocalFrame:
    """VERTIPORT_LOCAL_XYZ_UP ↔ GAZEBO_WORLD_ENU.

    local = Rz(-yaw) · (world - origin)     world = origin + Rz(yaw) · local
    origin = Landing Pad 중심의 Gazebo 월드 좌표, yaw = Gazebo X축에서 local X축까지 회전 (CCW +, deg)
    이 world 파일은 패드를 Gazebo 원점에 두었으므로 기본값 origin=(0,0,0), yaw=0 → 두 좌표계가 같다.
    """

    def __init__(self, origin=(0.0, 0.0, 0.0), yaw_deg: float = 0.0, name: str = LOCAL_FRAME):
        self.origin = tuple(float(c) for c in origin)
        self.yaw_deg = float(yaw_deg)
        self.name = name
        a = math.radians(self.yaw_deg)
        self._c, self._s = math.cos(a), math.sin(a)

    def to_world(self, x, y, z):
        ox, oy, oz = self.origin
        return (ox + self._c * x - self._s * y, oy + self._s * x + self._c * y, oz + z)

    def from_world(self, x, y, z):
        ox, oy, oz = self.origin
        dx, dy = x - ox, y - oy
        return (self._c * dx + self._s * dy, -self._s * dx + self._c * dy, z - oz)

    def describe(self) -> dict:
        return {
            'bridge_frame': FRAME,
            'bridge_axes': {'x': 'East', 'y': 'North', 'z': 'Up'}, 'units': {'length': 'm', 'angle': 'deg'},
            'landing_pad_origin_in_gazebo_world': {'x': self.origin[0], 'y': self.origin[1], 'z': self.origin[2]},
            'local_frame': {
                'name': self.name, 'origin_in_gazebo_world': {'x': self.origin[0], 'y': self.origin[1],
                                                              'z': self.origin[2]},
                'yaw_deg': self.yaw_deg,
                'rule': 'local = Rz(-yaw)·(world - origin);  world = origin + Rz(yaw)·local',
                'identity': self.origin == (0.0, 0.0, 0.0) and self.yaw_deg == 0.0,
            },
            'vehicle_origin': {
                'telemetry_xyz': 'base_link (기체 중심) = Gazebo 모델 pose',
                'dashboard_model_origin': 'skid bottom (기체 바닥)',
                'z_offset_m': VEHICLE_ORIGIN_Z_OFFSET_M,
                'rule': 'dashboard_z = telemetry_z - z_offset_m ;  set_target_z = dashboard_z + z_offset_m',
            },
            'threejs': 'three.x = x, three.y = z, three.z = -y',
            'attitude_convention': ATTITUDE_CONVENTION,
            'attitude_rule': 'R = Rz(yaw)·Ry(pitch)·Rx(roll), body FLU → world ENU; yaw 0 = East, CCW +; '
                             'pitch + = nose down; roll + = right wing down',
        }


# ---------------------------------------------------------------- 텔레메트리
def empty_telemetry(vehicle_id: str, camera_url: Optional[str]) -> dict:
    keys = ('telemetry_id', 'timestamp', 'x', 'y', 'z', 'altitude_m', 'roll', 'pitch', 'yaw', 'speed',
            'battery', 'voltage', 'armed', 'emergency', 'latitude', 'longitude', 'gps', 'satellites', 'mavlink')
    out = {k: None for k in keys}
    out.update({'frame': FRAME, 'id': vehicle_id, 'attitude_convention': ATTITUDE_CONVENTION,
                'flight_mode': 'DISCONNECTED', 'mavlink': 'DISCONNECTED', 'camera_stream_url': camera_url})
    return _ordered(out)


_ORDER = ('telemetry_id', 'timestamp', 'frame', 'id', 'x', 'y', 'z', 'altitude_m', 'roll', 'pitch', 'yaw',
          'attitude_convention', 'speed', 'battery', 'voltage', 'flight_mode', 'armed', 'emergency',
          'latitude', 'longitude', 'gps', 'satellites', 'mavlink', 'camera_stream_url')


def _ordered(d: dict) -> dict:
    out = {k: d.get(k) for k in _ORDER}
    for k, v in d.items():
        if k not in out:
            out[k] = v
    return out


def to_dashboard_telemetry(tel: dict, vehicle_id: str, camera_url: Optional[str],
                           pad_z: float = 0.0, gear_height: float = VEHICLE_ORIGIN_Z_OFFSET_M,
                           link_ok: Optional[bool] = None) -> dict:
    """내부 telemetry(JSON) → 대시보드 텔레메트리.

    x, y, z     : base_link(기체 중심)의 Gazebo 월드 좌표 = Gazebo 모델 pose 와 동일. 지상 정지 시 z ≈ 0.24
    altitude_m  : Landing Pad 면 기준 기체 바닥(착륙장치) 높이 = z - pad_z - gear_height → 착지 시 ≈ 0
    gps         : PX4 fix_type 문자열 ("3D_FIX" 등)
    mavlink     : 비행제어기 링크 상태 "CONNECTED" / "DISCONNECTED"
                  (실제 링크는 MAVLink 가 아니라 uXRCE-DDS. 대시보드 필드 이름에 맞춘 것)
    """
    p, r, v = tel['position_m'], tel['rpy_rad'], tel['velocity_mps']
    speed = math.sqrt(v['x'] ** 2 + v['y'] ** 2 + v['z'] ** 2)
    px4 = tel.get('px4') or {}
    bat = tel.get('battery') or {}
    glob = tel.get('global_position') or {}
    gps = tel.get('gps') or {}
    state = tel.get('state')
    flight_mode = state if state and state != 'NO_COMMANDER' else (px4.get('mode') or 'UNKNOWN')
    emergency = bool(px4.get('failsafe')) or (bat.get('warning') or 0) >= BATTERY_WARNING_CRITICAL
    if link_ok is None:
        link_ok = bool(tel.get('connected'))
    out = {
        'telemetry_id': tel.get('telemetry_id'),
        'timestamp': iso_utc(tel.get('t_sample') or tel.get('t')),
        'frame': FRAME,
        'id': vehicle_id,
        'x': _r(p['x']), 'y': _r(p['y']), 'z': _r(p['z']),
        'altitude_m': _r(p['z'] - pad_z - gear_height),
        'roll': _deg(r['roll']), 'pitch': _deg(r['pitch']), 'yaw': _deg(r['yaw']),
        'attitude_convention': ATTITUDE_CONVENTION,
        'speed': _r(speed, 2),
        'battery': bat.get('remaining_pct'),
        'voltage': bat.get('voltage_v'),
        'flight_mode': flight_mode,
        'armed': bool(px4.get('armed')),
        'emergency': emergency,
        'latitude': glob.get('lat'),
        'longitude': glob.get('lon'),
        'gps': gps_fix_name(gps.get('fix_type')),
        'satellites': gps.get('satellites'),
        'mavlink': 'CONNECTED' if link_ok else 'DISCONNECTED',
        'camera_stream_url': camera_url,
        # ---- 확장 필드 (대시보드가 무시해도 됨)
        'ext': {
            'valid': tel.get('valid'), 'connected': tel.get('connected'), 'source': tel.get('source'),
            'orientation_q': tel.get('orientation_q'),
            'velocity_mps': v, 'ground_speed_mps': _r(math.hypot(v['x'], v['y']), 2),
            'target': tel.get('target_m'), 'px4_mode': px4.get('mode'), 'landed': px4.get('landed'),
            'commander': tel.get('commander'), 'last_command': tel.get('last_command'),
            't_sim': tel.get('t_px4'), 'gps_fix_type': gps.get('fix_type'),
            'z_bottom': _r(p['z'] - gear_height), 'vehicle_origin_z_offset_m': gear_height,
            'altitude_amsl_m': glob.get('alt_amsl_m'), 'battery_warning': bat.get('warning'),
        },
    }
    return _ordered(out)


# ---------------------------------------------------------------- 텔레메트리 ID 기록 (request_landing 검증용)
class TelemetryLog:
    def __init__(self, maxlen: int = 3000):
        self.maxlen = maxlen
        self.items: 'OrderedDict[str, float]' = OrderedDict()

    def add(self, tid: Optional[str], t: Optional[float]):
        if not tid or tid in self.items:
            return
        self.items[tid] = t or 0.0
        while len(self.items) > self.maxlen:
            self.items.popitem(last=False)

    def get(self, tid) -> Optional[float]:
        return self.items.get(tid)


# ---------------------------------------------------------------- 명령 변환
class CommandError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code, self.message = code, message


def _params(body: dict) -> dict:
    p = body.get('parameters', body.get('params'))
    if p is None:
        p = {k: v for k, v in body.items() if k not in ('command', 'request_id', 'command_id')}
    if not isinstance(p, dict):
        raise CommandError('INVALID_PARAMETERS', 'parameters must be an object')
    return p


def _num(p, k):
    try:
        v = float(p[k])
    except (KeyError, TypeError, ValueError):
        raise CommandError('INVALID_PARAMETERS', f'parameter "{k}" (number) required') from None
    if not math.isfinite(v):
        raise CommandError('INVALID_PARAMETERS', f'parameter "{k}" must be finite')
    return v


def translate_command(body: dict, local: LocalFrame, tlog: TelemetryLog, now: float,
                      pad_xy=(0.0, 0.0), landing_max_age_s: float = 3.0) -> Tuple[dict, str]:
    """대시보드 요청 → (Commander 명령 dict, 설명). 잘못된 요청은 CommandError."""
    if not isinstance(body, dict):
        raise CommandError('INVALID_REQUEST', 'body must be a JSON object')
    raw = body.get('command')
    if not isinstance(raw, str) or not raw.strip():
        raise CommandError('INVALID_REQUEST', '"command" (string) required')
    name = raw.strip()
    p = _params(body)

    if name.lower() in DASHBOARD_COMMANDS:
        name = name.lower()
        native = DASHBOARD_COMMANDS[name]
        if name == 'set_target':
            frame = str(p.get('frame') or FRAME)
            x, y, z = _num(p, 'x'), _num(p, 'y'), _num(p, 'z')
            if frame == FRAME:
                wx, wy, wz = x, y, z
            elif frame == local.name:
                wx, wy, wz = local.to_world(x, y, z)
            else:
                raise CommandError('UNSUPPORTED_FRAME', f'frame must be {FRAME} or {local.name} (got {frame})')
            cmd = {'command': native, 'x': wx, 'y': wy, 'z': wz}
            if p.get('yaw') is not None:
                cmd['yaw'] = math.radians(_num(p, 'yaw'))      # 대시보드 deg → Commander rad
            return cmd, f'set_target {frame} ({x:.2f}, {y:.2f}, {z:.2f}) → world ({wx:.2f}, {wy:.2f}, {wz:.2f})'
        if name == 'request_landing':
            tid = p.get('telemetry_id')
            if not tid:
                raise CommandError('INVALID_PARAMETERS', 'parameter "telemetry_id" required')
            t = tlog.get(str(tid))
            if t is None:
                raise CommandError('UNKNOWN_TELEMETRY', f'unknown telemetry_id {tid}')
            if now - t > landing_max_age_s:
                raise CommandError('STALE_TELEMETRY',
                                   f'telemetry {tid} is {now - t:.1f}s old (max {landing_max_age_s:.0f}s)')
            return ({'command': native, 'x': float(pad_xy[0]), 'y': float(pad_xy[1])},
                    f'request_landing on landing pad ({pad_xy[0]:.2f}, {pad_xy[1]:.2f}) based on {tid}')
        return {'command': native}, name

    if name.upper() in NATIVE_COMMANDS:          # 확장: Commander 명령 직접 사용
        cmd = {'command': name.upper()}
        for k in ('x', 'y', 'z', 'yaw'):
            if k in p:
                cmd[k] = p[k]
        return cmd, name.upper()

    raise CommandError('UNKNOWN_COMMAND',
                       f'command must be one of {list(DASHBOARD_COMMANDS)} (got {name})')
