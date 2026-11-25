"""Commander 상태기계 (ROS 의존성 없음 → 단위 테스트/오프라인 시뮬레이션 가능).

입력  : 상위 명령(GOTO / HOLD / LAND / RESUME / CANCEL) + 현재 기체 상태(World ENU)
출력  : World ENU 위치 setpoint, yaw setpoint, PX4 요청(OFFBOARD/ARM/DISARM/LAND)

이륙 전용 명령은 없다. 지상에서 GOTO 를 받으면 현재 위치에서 목표 고도까지 수직 이륙한 뒤 이동한다.

ROS 노드(commander_node.py)는 이 클래스를 감싸서
  - PX4 토픽 구독 → VehicleState 생성
  - step() 결과를 World→PX4 local NED 로 변환해서 퍼블리시
만 담당한다.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]

# ------------------------------------------------------------------ 상태 이름 (인터페이스 규격)
DISCONNECTED = 'DISCONNECTED'  # PX4 데이터 미수신
IDLE = 'IDLE'                  # 지상, 대기 (disarm)
TAKEOFF = 'TAKEOFF'            # 이륙 중
GOTO = 'GOTO'                  # 목표 좌표로 이동 중
HOVER = 'HOVER'                # 목표 도착 후 제자리 비행 (명령 대기)
HOLD = 'HOLD'                  # 명령에 의한 일시정지 (RESUME 으로 이어가기 가능)
LANDING = 'LANDING'            # 패드 정렬(ALIGN) / 하강(DESCEND)
TOUCHDOWN = 'TOUCHDOWN'        # 최종 접지 (PX4 Land 모드)
LANDED = 'LANDED'              # 착지 완료 (disarm 대기)
OVERRIDE = 'OVERRIDE'          # PX4/조종자가 제어권을 가져감 → Commander 개입 중단

ALL_STATES = (DISCONNECTED, IDLE, TAKEOFF, GOTO, HOVER, HOLD, LANDING, TOUCHDOWN, LANDED, OVERRIDE)
OFFBOARD_STATES = (TAKEOFF, GOTO, HOVER, HOLD, LANDING)

PHASE_ALIGN = 'ALIGN'
PHASE_DESCEND = 'DESCEND'

REQ_OFFBOARD = 'OFFBOARD'
REQ_ARM = 'ARM'
REQ_LAND = 'LAND'
REQ_DISARM = 'DISARM'

COMMANDS = ('GOTO', 'HOLD', 'LAND', 'RESUME', 'CANCEL')


@dataclass
class VehicleState:
    connected: bool = False
    pos: Vec3 = (0.0, 0.0, 0.0)      # World ENU [m]
    vel: Vec3 = (0.0, 0.0, 0.0)      # World ENU [m/s]
    yaw: float = 0.0                 # ENU yaw [rad]
    armed: bool = False
    offboard: bool = False           # PX4 nav_state == OFFBOARD
    auto_land: bool = False          # PX4 nav_state == AUTO_LAND
    landed: bool = True
    failsafe: bool = False
    pos_valid: bool = True


@dataclass
class StepOutput:
    setpoint: Optional[Vec3]          # World ENU, None 이면 setpoint 송신 안 함
    yaw: float
    requests: List[str] = field(default_factory=list)


DEFAULTS = dict(
    min_takeoff_alt=1.5,     # 지상에서 GOTO 받으면 최소 이 고도까지 먼저 상승
    max_speed_xy=1.5,        # carrot(setpoint) 수평 이동 속도 [m/s]
    max_speed_z=1.0,         # carrot 수직 이동 속도 [m/s]
    descent_rate=0.5,        # 착륙 하강 속도 [m/s]
    arrive_tol=0.25,         # 도착 판정 거리 [m]
    align_tol=0.25,          # 착륙 정렬 판정 수평 거리 [m]
    touchdown_alt=0.5,       # 이 고도 이하에서 PX4 Land 모드로 최종 접지 [m]
    pad_xy=(0.0, 0.0),       # LAND 좌표 생략 시 사용할 패드 위치 (World)
    xy_limit=60.0,           # 지오펜스: |x|,|y| 최대 [m]
    z_min=1.0,               # GOTO 최소 고도 [m]
    z_max=30.0,              # GOTO 최대 고도 [m]
    warmup_cycles=10,        # OFFBOARD 요청 전 setpoint 선행 송신 횟수
    request_retry_s=1.0,     # OFFBOARD/ARM/LAND 재요청 간격
    arm_timeout_s=10.0,      # 이 시간 안에 arm/offboard 안 되면 실패 처리
    override_grace_s=1.0,    # offboard 이탈이 이 시간 이상 지속되면 OVERRIDE
)


def command_name(cmd: dict) -> str:
    """명령 이름. 규격 필드는 'command' ('cmd' 도 호환용으로 허용)."""
    return str(cmd.get('command', cmd.get('cmd', '')) or '').strip().upper()


def result_code(ok: bool, reason: str) -> str:
    """명령 처리 결과 코드 (대시보드 API 의 code 필드)."""
    if ok:
        return 'ACCEPTED'
    r = reason or ''
    if 'unknown command' in r:
        return 'UNKNOWN_COMMAND'
    if 'px4_not_connected' in r:
        return 'NOT_CONNECTED'
    if 'geofence' in r or 'must be in' in r:
        return 'OUT_OF_BOUNDS'
    if r.startswith('invalid'):
        return 'INVALID_PARAMETERS'
    return 'REJECTED_STATE'


def _finite(*vals) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def _dist_xy(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class CommanderCore:
    def __init__(self, **cfg):
        self.cfg = dict(DEFAULTS)
        self.cfg.update({k: v for k, v in cfg.items() if v is not None})
        self.state = DISCONNECTED
        self.phase: Optional[str] = None
        self.target: Optional[Vec3] = None       # 최종 목표 (World)
        self.sp: Optional[Vec3] = None           # 현재 carrot setpoint (World)
        self.yaw_sp: float = 0.0
        self.land_xy: Optional[Tuple[float, float]] = None
        self.pending: Optional[dict] = None      # TAKEOFF 후 이어서 할 동작
        self.paused: Optional[dict] = None       # HOLD 직전 동작 (RESUME 용)
        self.message = ''
        self.last_cmd: Optional[dict] = None     # 마지막으로 받은 명령 (승인 여부 포함)
        self._want_disarm = False
        self._cycles = 0
        self._last_req: Dict[str, float] = {}
        self._active_since: Optional[float] = None
        self._offboard_seen = False
        self._armed_seen = False
        self._entering = False      # True: OFFBOARD/ARM 진입을 요청 중 (아직 확인 전)
        self._not_offboard_since: Optional[float] = None
        self._vs = VehicleState()

    # ================================================================ 명령 처리
    def observe(self, vs: VehicleState):
        """명령 처리 직전에 최신 기체 상태를 반영 (step 사이에 명령이 들어올 때)."""
        self._vs = vs

    def handle_command(self, cmd: dict, now: Optional[float] = None) -> Tuple[bool, str]:
        """상위 명령 처리. 반환: (accepted, reason)"""
        now = time.monotonic() if now is None else now
        if not isinstance(cmd, dict):
            return False, 'invalid: not an object'
        name = command_name(cmd)
        if name not in COMMANDS:
            ok, reason = False, f'invalid: unknown command "{name}" (allowed: {", ".join(COMMANDS)})'
        elif self.state == DISCONNECTED:
            ok, reason = False, 'rejected: px4_not_connected'
        else:
            handler = getattr(self, f'_cmd_{name.lower()}')
            ok, reason = handler(cmd, self._vs, self._is_airborne(), now)
        self.last_cmd = {'command_id': cmd.get('command_id'), 'command': name, 'accepted': ok, 'reason': reason}
        for k in ('x', 'y', 'z', 'yaw'):
            if k in cmd:
                self.last_cmd[k] = cmd[k]
        if ok:
            self.message = reason
        return ok, reason

    def _parse_xyz(self, cmd, need_z=True):
        try:
            x = float(cmd['x'])
            y = float(cmd['y'])
            z = float(cmd['z']) if need_z else None
        except (KeyError, TypeError, ValueError):
            return None, 'invalid: x, y' + (', z' if need_z else '') + ' required (numbers)'
        if not _finite(x, y) or (need_z and not _finite(z)):
            return None, 'invalid: non-finite value'
        lim = self.cfg['xy_limit']
        if abs(x) > lim or abs(y) > lim:
            return None, f'rejected: outside geofence |x|,|y| <= {lim}'
        if need_z and not (self.cfg['z_min'] <= z <= self.cfg['z_max']):
            return None, f'rejected: z must be in [{self.cfg["z_min"]}, {self.cfg["z_max"]}]'
        return (x, y, z), ''

    def _parse_yaw(self, cmd) -> Optional[float]:
        yaw = cmd.get('yaw')
        if yaw is None:
            return None
        try:
            yaw = float(yaw)
        except (TypeError, ValueError):
            return None
        return yaw if math.isfinite(yaw) else None

    def _begin_offboard_session(self, now):
        """지상/OVERRIDE 에서 새로 제어를 시작할 때 호출."""
        if self.state in (IDLE, LANDED, OVERRIDE, TOUCHDOWN) or self.sp is None:
            self.sp = tuple(self._vs.pos)
        self._active_since = now
        self._last_req.clear()
        self._not_offboard_since = None
        self._entering = True
        if self.state in (IDLE, LANDED):
            self._offboard_seen = False
            self._armed_seen = False

    def _cmd_goto(self, cmd, vs, airborne, now):
        xyz, err = self._parse_xyz(cmd)
        if xyz is None:
            return False, err
        yaw = self._parse_yaw(cmd)
        if not airborne:
            if self.state not in (IDLE, LANDED, TAKEOFF):
                return False, f'rejected: busy ({self.state})'
            if self.state != TAKEOFF:
                self._begin_offboard_session(now)
                self.yaw_sp = vs.yaw if yaw is None else yaw
                self.state = TAKEOFF
                self.target = (vs.pos[0], vs.pos[1], max(xyz[2], self.cfg['min_takeoff_alt']))
            self.pending = {'cmd': 'GOTO', 'target': xyz, 'yaw': yaw}
            self.paused = None
            return True, f'takeoff then goto ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})'
        if self.state == OVERRIDE:
            self._begin_offboard_session(now)
        if yaw is not None:
            self.yaw_sp = yaw
        self.state = GOTO
        self.phase = None
        self.target = xyz
        self.pending = None
        self.paused = None
        return True, f'goto ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})'

    def _cmd_hold(self, cmd, vs, airborne, now):
        if self.state == HOLD:
            return True, 'already holding'   # 중복 HOLD 는 무시 (paused 유지)
        if not airborne and self.state not in (TAKEOFF,):
            return False, f'rejected: not airborne ({self.state})'
        if self.state == OVERRIDE:
            return False, 'rejected: override active (send GOTO/LAND to retake control)'
        snapshot_state = self.state
        if snapshot_state == TOUCHDOWN:      # 최종 접지 중 HOLD → 하강 단계로 되돌림
            snapshot_state, phase = LANDING, PHASE_DESCEND
            self._begin_offboard_session(now)
        else:
            phase = self.phase
        self.paused = {
            'state': snapshot_state, 'phase': phase, 'target': self.target,
            'land_xy': self.land_xy, 'pending': self.pending,
        } if snapshot_state in (TAKEOFF, GOTO, LANDING) else None
        self.state = HOLD
        self.phase = None
        self.sp = tuple(vs.pos)
        self.target = tuple(vs.pos)
        self.pending = None
        return True, 'hold at current position'

    def _cmd_resume(self, cmd, vs, airborne, now):
        if self.state != HOLD:
            return False, f'rejected: not in HOLD ({self.state})'
        if not self.paused:
            return False, 'rejected: nothing to resume'
        p = self.paused
        self.paused = None
        self.state = p['state']
        self.phase = p['phase']
        self.target = p['target']
        self.land_xy = p['land_xy']
        self.pending = p['pending']
        if self.state == LANDING and self.phase == PHASE_DESCEND:
            # 현재 고도에서 하강 재개
            self.target = (self.land_xy[0], self.land_xy[1], self._descent_floor())
        return True, f'resume {self.state}' + (f'/{self.phase}' if self.phase else '')

    def _cmd_cancel(self, cmd, vs, airborne, now):
        """진행 중 동작(이동·착륙·HOLD 대기 동작)을 취소하고 현재 위치에서 HOVER. RESUME 불가."""
        if self.state in (IDLE, LANDED):
            return False, f'rejected: nothing to cancel ({self.state})'
        if self.state == OVERRIDE:
            return False, 'rejected: override active'
        if self.state == TAKEOFF and vs.landed:
            # 아직 이륙 전 → 이륙 취소, disarm
            self._to_idle('takeoff canceled')
            self._want_disarm = True
            return True, 'takeoff canceled → disarm'
        if self.state == TOUCHDOWN:
            self._begin_offboard_session(now)
        self.state = HOVER
        self.phase = None
        self.sp = tuple(vs.pos)
        self.target = tuple(vs.pos)
        self.pending = None
        self.paused = None
        return True, 'canceled → hover at current position'

    def _cmd_land(self, cmd, vs, airborne, now):
        if 'x' in cmd or 'y' in cmd:
            xyz, err = self._parse_xyz(cmd, need_z=False)
            if xyz is None:
                return False, err
            lx, ly = xyz[0], xyz[1]
        else:
            lx, ly = self.cfg['pad_xy']
        if not airborne:
            return False, f'rejected: not airborne ({self.state})'
        if self.state == LANDING:   # 착륙 중 LAND 재수신 → 목표 좌표 보정 (비전 기반 정밀화)
            self.land_xy = (lx, ly)
            if self.phase == PHASE_ALIGN:
                self.target = (lx, ly, self.target[2])
            else:
                self.target = (lx, ly, self._descent_floor())
            return True, f'land target updated ({lx:.2f}, {ly:.2f})'
        if self.state == OVERRIDE:
            self._begin_offboard_session(now)
        self.state = LANDING
        self.phase = PHASE_ALIGN
        self.land_xy = (lx, ly)
        align_z = max(vs.pos[2], self.cfg['touchdown_alt'] + 1.0)
        self.target = (lx, ly, align_z)
        self.pending = None
        self.paused = None
        return True, f'land at ({lx:.2f}, {ly:.2f})'

    # ================================================================ 주기 실행
    def step(self, vs: VehicleState, now: float, dt: float) -> StepOutput:
        self._vs = vs
        req: List[str] = []
        if not vs.connected:
            if self.state != DISCONNECTED:
                self.message = 'px4 connection lost'
            self.state = DISCONNECTED
            self._cycles = 0
            return StepOutput(None, self.yaw_sp, req)

        self._cycles += 1
        if self.state == DISCONNECTED:
            # 연결 직후: 비행 중이면 제어권을 빼앗지 않는다
            if vs.armed and not vs.landed:
                self.state = OVERRIDE
                self.message = 'connected while airborne → OVERRIDE (send a command to take control)'
            else:
                self.state = IDLE
                self.message = 'connected'
            self.sp = tuple(vs.pos)
            self.yaw_sp = vs.yaw

        if vs.armed:
            self._armed_seen = True

        # ---- 지상 상태 전이
        if self.state in OFFBOARD_STATES + (TOUCHDOWN, LANDED) and self._armed_seen and not vs.armed:
            self._to_idle('disarmed')
        if self.state == TOUCHDOWN and vs.landed:
            self.state = LANDED
            self.message = 'landed'
        if self.state == LANDED and not vs.armed:
            self._to_idle('landed & disarmed')

        # ---- OFFBOARD/ARM 진입 확인 & 타임아웃
        if self.state in OFFBOARD_STATES and self._entering:
            if vs.offboard and vs.armed:
                self._entering = False
                self._offboard_seen = True
                self._not_offboard_since = None
            elif self._active_since is not None and now - self._active_since > self.cfg['arm_timeout_s']:
                self._entering = False
                if vs.armed and not vs.landed:
                    self.state = OVERRIDE
                    self.phase = None
                    self.paused = None
                    self.pending = None
                    self.message = 'offboard re-entry timeout → OVERRIDE'
                else:
                    self._to_idle('arm/offboard timeout - check preflight (NAV_DLL_ACT, COM_RCL_EXCEPT)')

        # ---- OVERRIDE 감지: 진입 확인 후 PX4 가 OFFBOARD 를 벗어나면 (조종자 모드 변경/failsafe)
        #      Commander 는 다시 OFFBOARD 를 요청하지 않는다 → 조종자와 제어권 다툼 방지
        if self.state in OFFBOARD_STATES and not self._entering and self._offboard_seen:
            if vs.offboard:
                self._not_offboard_since = None
            else:
                if self._not_offboard_since is None:
                    self._not_offboard_since = now
                if now - self._not_offboard_since > self.cfg['override_grace_s']:
                    self.state = OVERRIDE
                    self.phase = None
                    self.paused = None
                    self.pending = None
                    self.message = 'PX4 left OFFBOARD (pilot/failsafe) → OVERRIDE'

        # ---- 상태별 setpoint
        if self._want_disarm:
            if not vs.armed:
                self._want_disarm = False
            elif self._retry_ok(REQ_DISARM, now):
                req.append(REQ_DISARM)

        if self.state in (IDLE, LANDED, OVERRIDE, TOUCHDOWN):
            self.sp = tuple(vs.pos)  # 언제든 매끄럽게 재진입할 수 있도록 현재 위치 추종
            if self.state == TOUCHDOWN and not vs.auto_land and self._retry_ok(REQ_LAND, now):
                req.append(REQ_LAND)
            return StepOutput(self.sp, self.yaw_sp, req)

        if self.state == TAKEOFF:
            self._advance(dt, self.cfg['max_speed_z'])
            if self._arrived(vs):
                if self.pending and self.pending.get('cmd') == 'GOTO':
                    p = self.pending
                    self.pending = None
                    self.state = GOTO
                    self.target = p['target']
                    if p.get('yaw') is not None:
                        self.yaw_sp = p['yaw']
                    self.message = 'takeoff done → goto'
                else:
                    self.state = HOVER
                    self.message = 'takeoff done'
        elif self.state == GOTO:
            self._advance(dt, self.cfg['max_speed_z'])
            if self._arrived(vs):
                self.state = HOVER
                self.message = 'arrived'
        elif self.state in (HOVER, HOLD):
            pass  # sp 고정
        elif self.state == LANDING:
            if self.phase == PHASE_ALIGN:
                self._advance(dt, self.cfg['max_speed_z'])
                if self.sp == self.target and _dist_xy(vs.pos, self.target) < self.cfg['align_tol']:
                    self.phase = PHASE_DESCEND
                    self.target = (self.land_xy[0], self.land_xy[1], self._descent_floor())
                    self.message = 'aligned → descend'
            if self.phase == PHASE_DESCEND:
                # 수평 오차가 크면 하강을 멈추고 수평만 보정
                if _dist_xy(vs.pos, self.target) < 2.0 * self.cfg['align_tol']:
                    self._advance(dt, self.cfg['descent_rate'])
                else:
                    sx, sy, sz = self._advance_xy_only(dt)
                    self.sp = (sx, sy, sz)
                if vs.pos[2] <= self.cfg['touchdown_alt']:
                    self.state = TOUCHDOWN
                    self.phase = None
                    self.message = 'touchdown (PX4 Land)'
                    req.append(REQ_LAND)
                    self._last_req[REQ_LAND] = now

        # ---- OFFBOARD/ARM 요청
        if self.state in OFFBOARD_STATES and self._entering and self._cycles > self.cfg['warmup_cycles']:
            if not vs.offboard and self._retry_ok(REQ_OFFBOARD, now):
                req.append(REQ_OFFBOARD)
            if not vs.armed and self._retry_ok(REQ_ARM, now):
                req.append(REQ_ARM)
        return StepOutput(self.sp, self.yaw_sp, req)

    # ================================================================ helpers
    def _is_airborne(self) -> bool:
        vs = self._vs
        return bool(vs.armed and not vs.landed) or self.state in (GOTO, HOVER, HOLD, LANDING)

    def _descent_floor(self) -> float:
        return self.cfg['touchdown_alt'] - 0.2

    def _to_idle(self, why: str):
        self.state = IDLE
        self.phase = None
        self.target = None
        self.pending = None
        self.paused = None
        self._active_since = None
        self._offboard_seen = False
        self._armed_seen = False
        self._entering = False
        self.message = why

    def _retry_ok(self, key: str, now: float) -> bool:
        last = self._last_req.get(key)
        if last is None or now - last >= self.cfg['request_retry_s']:
            self._last_req[key] = now
            return True
        return False

    def _arrived(self, vs: VehicleState) -> bool:
        return self.sp == self.target and _dist3(vs.pos, self.target) < self.cfg['arrive_tol']

    def _advance(self, dt: float, vz: float):
        """carrot setpoint 를 target 쪽으로 속도 제한을 두고 이동."""
        if self.target is None:
            return
        if self.sp is None:
            self.sp = tuple(self._vs.pos)
        sx, sy, _ = self._advance_xy_only(dt)
        dz = self.target[2] - self.sp[2]
        step = vz * dt
        sz = self.target[2] if abs(dz) <= step else self.sp[2] + math.copysign(step, dz)
        self.sp = (sx, sy, sz)

    def _advance_xy_only(self, dt: float):
        tx, ty = self.target[0], self.target[1]
        sx, sy, sz = self.sp
        d = math.hypot(tx - sx, ty - sy)
        step = self.cfg['max_speed_xy'] * dt
        if d <= step:
            return tx, ty, sz
        return sx + (tx - sx) * step / d, sy + (ty - sy) * step / d, sz

    def status(self) -> dict:
        def r(v):
            return None if v is None else [round(c, 3) for c in v]
        return {
            'state': self.state,
            'phase': self.phase,
            'target_m': r(self.target),
            'setpoint_m': r(self.sp),
            'yaw_sp_rad': round(self.yaw_sp, 4),
            'land_xy_m': r(self.land_xy),
            'paused': None if not self.paused else {
                'state': self.paused['state'], 'phase': self.paused['phase']},
            'last_command': self.last_cmd,
            'message': self.message,
        }
