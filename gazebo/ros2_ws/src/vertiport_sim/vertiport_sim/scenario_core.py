"""시나리오(사람/차량/FOD 배치) 로직 - ROS 의존성 없음.

Gazebo 의 UserCommands 시스템 플러그인이 제공하는 서비스를 gz CLI 로 호출한다.
  /world/<world>/create   (gz.msgs.EntityFactory)
  /world/<world>/remove   (gz.msgs.Entity)
  /world/<world>/set_pose (gz.msgs.Pose)
※ world SDF 에 gz-sim-user-commands-system 플러그인이 없으면 이 서비스들이 생성되지 않는다.
"""

import math
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

GZ_PREFIX = 'hz_'   # Gazebo 엔티티 이름 접두사 (시나리오 객체만 일괄 정리하기 위함)

DEFAULT_MODELS = {
    'person': 'model://vp_person',
    'vehicle': 'model://vp_vehicle',
    'fod': 'model://vp_fod',
}


@dataclass
class SceneObject:
    id: str
    type: str
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    uri: Optional[str] = None

    def to_gt(self) -> dict:
        return {'id': self.id, 'type': self.type, 'x': round(self.x, 3), 'y': round(self.y, 3),
                'z': round(self.z, 3), 'yaw': round(self.yaw, 4)}

    @property
    def gz_name(self) -> str:
        return GZ_PREFIX + self.id


def parse_object(d: dict, models: Dict[str, str]) -> SceneObject:
    t = str(d['type']).lower()
    if t not in models and not d.get('uri'):
        raise ValueError(f'unknown object type "{t}" (known: {", ".join(models)})')
    oid = str(d.get('id') or '')
    if not oid:
        raise ValueError('object id required')
    if not all(c.isalnum() or c in '_-' for c in oid):
        raise ValueError(f'invalid id "{oid}" (use letters, digits, _ or -)')
    vals = [float(d.get(k, 0.0)) for k in ('x', 'y', 'z', 'yaw')]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError('non-finite coordinate')
    return SceneObject(oid, t, *vals, uri=d.get('uri') or models.get(t))


def diff_scene(current: Dict[str, SceneObject], target: List[SceneObject]):
    """현재 → 목표 장면으로 가기 위한 (remove 목록, spawn 목록, move 목록)."""
    tmap = {o.id: o for o in target}
    remove = [o for oid, o in current.items() if oid not in tmap or tmap[oid].type != o.type
              or tmap[oid].uri != o.uri]
    removed_ids = {o.id for o in remove}
    spawn, move = [], []
    for o in target:
        cur = current.get(o.id)
        if cur is None or o.id in removed_ids:
            spawn.append(o)
        elif (cur.x, cur.y, cur.z, cur.yaw) != (o.x, o.y, o.z, o.yaw):
            move.append(o)
    return remove, spawn, move


# ------------------------------------------------------------------ gz 요청 문자열
def _quat_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))   # x, y, z, w


def create_request(o: SceneObject) -> str:
    # SDF 안에서는 작은따옴표만 사용 → protobuf text 의 큰따옴표와 충돌하지 않음
    sdf = (f"<sdf version='1.9'><include><uri>{o.uri}</uri><name>{o.gz_name}</name>"
           f"<static>true</static></include></sdf>")
    qx, qy, qz, qw = _quat_yaw(o.yaw)
    return (f'sdf: "{sdf}" name: "{o.gz_name}" allow_renaming: false '
            f'pose {{ position {{ x: {o.x} y: {o.y} z: {o.z} }} '
            f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}')


def remove_request(gz_name: str) -> str:
    return f'name: "{gz_name}" type: MODEL'


def set_pose_request(o: SceneObject) -> str:
    qx, qy, qz, qw = _quat_yaw(o.yaw)
    return (f'name: "{o.gz_name}" position {{ x: {o.x} y: {o.y} z: {o.z} }} '
            f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}')


class GzClient:
    """gz CLI 래퍼. (테스트 시 dry_run=True 로 호출 내용만 기록)"""

    def __init__(self, world: str, timeout_ms: int = 3000, dry_run: bool = False):
        self.world = world
        self.timeout_ms = timeout_ms
        self.dry_run = dry_run
        self.calls: List[list] = []
        self.gz = shutil.which('gz') or 'gz'

    def _service(self, name, reqtype, reptype, req) -> Tuple[bool, str]:
        cmd = [self.gz, 'service', '-s', f'/world/{self.world}/{name}', '--reqtype', reqtype,
               '--reptype', reptype, '--timeout', str(self.timeout_ms), '--req', req]
        self.calls.append(cmd)
        if self.dry_run:
            return True, 'dry-run'
        try:
            p = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=self.timeout_ms / 1000.0 + 2.0)
        except (OSError, subprocess.TimeoutExpired) as e:
            return False, str(e)
        out = (p.stdout + p.stderr).strip()
        ok = p.returncode == 0 and 'data: true' in out
        return ok, out

    def create(self, o: SceneObject):
        return self._service('create', 'gz.msgs.EntityFactory', 'gz.msgs.Boolean', create_request(o))

    def remove(self, gz_name: str):
        return self._service('remove', 'gz.msgs.Entity', 'gz.msgs.Boolean', remove_request(gz_name))

    def set_pose(self, o: SceneObject):
        return self._service('set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean', set_pose_request(o))

    def world_ready(self) -> bool:
        if self.dry_run:
            return True
        try:
            p = subprocess.run([self.gz, 'service', '-l'], capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return False
        return f'/world/{self.world}/create' in p.stdout

    def list_models(self) -> List[str]:
        """현재 world 의 모델 이름 목록 (best-effort)."""
        if self.dry_run:
            return []
        try:
            p = subprocess.run([self.gz, 'model', '--list'], capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return []
        names = []
        for line in p.stdout.splitlines():
            line = line.strip()
            if line.startswith('- '):
                names.append(line[2:].strip())
        return names


# ------------------------------------------------------------------ 시퀀스
@dataclass
class SequenceRunner:
    """시나리오 시퀀스 실행기.

    steps 예시:
      - apply: S1_person
      - wait_s: 8
      - wait_alt_below: 3.0       # 드론 고도가 3m 미만이 될 때까지 대기
      - apply: S0_clear
    """
    name: str
    steps: List[dict]
    idx: int = 0
    wait_until: Optional[float] = None
    done: bool = False
    log: List[str] = field(default_factory=list)

    def step(self, now: float, altitude: Optional[float]) -> Optional[str]:
        """이번에 적용할 시나리오 이름을 반환 (없으면 None)."""
        while not self.done:
            if self.idx >= len(self.steps):
                self.done = True
                return None
            s = self.steps[self.idx]
            if 'apply' in s:
                self.idx += 1
                self.log.append(f'apply {s["apply"]}')
                return str(s['apply'])
            if 'wait_s' in s:
                if self.wait_until is None:
                    self.wait_until = now + float(s['wait_s'])
                if now < self.wait_until:
                    return None
                self.wait_until = None
                self.idx += 1
                continue
            if 'wait_alt_below' in s:
                if altitude is None or altitude >= float(s['wait_alt_below']):
                    return None
                self.idx += 1
                continue
            if 'wait_alt_above' in s:
                if altitude is None or altitude <= float(s['wait_alt_above']):
                    return None
                self.idx += 1
                continue
            raise ValueError(f'unknown sequence step: {s}')
        return None

    def current_step(self) -> Optional[dict]:
        return None if self.done or self.idx >= len(self.steps) else self.steps[self.idx]
