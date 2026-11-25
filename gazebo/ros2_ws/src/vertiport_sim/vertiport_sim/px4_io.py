"""PX4 uXRCE-DDS 토픽 구독 공통 모듈 (Commander / Telemetry Bridge 공용).

- PX4 는 BEST_EFFORT QoS 로 퍼블리시 → 구독도 같은 QoS 를 써야 데이터가 들어온다.
- PX4 v1.16 부터 일부 메시지에 버전 접미사가 붙는다 (예: /fmu/out/vehicle_status_v1).
  config 에 지정한 토픽이 그래프에 없고 다른 버전 토픽이 보이면 자동으로 갈아탄다.
"""

import re
import time
from typing import Callable, Dict, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (BatteryStatus, SensorGps, VehicleAttitude, VehicleGlobalPosition,
                          VehicleLandDetected, VehicleLocalPosition, VehicleStatus)

from . import frames
from .commander_core import VehicleState
from .telemetry_core import NAV_STATE_AUTO_LAND, NAV_STATE_OFFBOARD

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# PX4 v1.17 기준 기본 토픽 (버전 접미사 포함)
DEFAULT_TOPICS = {
    'local_position': '/fmu/out/vehicle_local_position_v1',
    'attitude': '/fmu/out/vehicle_attitude',
    'status': '/fmu/out/vehicle_status_v1',
    'land_detected': '/fmu/out/vehicle_land_detected',
    'battery': '/fmu/out/battery_status_v1',
    'global_position': '/fmu/out/vehicle_global_position',
    'gps': '/fmu/out/vehicle_gps_position',
    'offboard_control_mode': '/fmu/in/offboard_control_mode',
    'trajectory_setpoint': '/fmu/in/trajectory_setpoint',
    'vehicle_command': '/fmu/in/vehicle_command',
}

_SUB_TYPES = {
    'local_position': VehicleLocalPosition,
    'attitude': VehicleAttitude,
    'status': VehicleStatus,
    'land_detected': VehicleLandDetected,
    'battery': BatteryStatus,
    'global_position': VehicleGlobalPosition,
    'gps': SensorGps,
}
# 없어도 동작하는 토픽 (못 찾으면 해당 값만 null)
_OPTIONAL = ('battery', 'global_position', 'gps')

_VER_RE = re.compile(r'^(?P<base>.+?)(_v(?P<ver>\d+))?$')


def declare_topic_params(node: Node, prefix: str = 'topics.') -> Dict[str, str]:
    out = {}
    for key, default in DEFAULT_TOPICS.items():
        out[key] = node.declare_parameter(prefix + key, default).value
    return out


def find_topic_variant(name: str, graph_names) -> Optional[str]:
    """name 과 base 가 같은 버전 토픽 중 가장 높은 버전을 반환 (없으면 None)."""
    base = _VER_RE.match(name).group('base')
    best, best_ver = None, -1
    for g in graph_names:
        m = _VER_RE.match(g)
        if m and m.group('base') == base:
            ver = int(m.group('ver') or 0)
            if ver > best_ver:
                best, best_ver = g, ver
    return best


class Px4Listener:
    """PX4 상태 토픽 구독 → VehicleState / 원시값 제공."""

    def __init__(self, node: Node, topics: Dict[str, str],
                 transform: frames.LocalWorldTransform, timeout_s: float = 1.0,
                 on_local_position: Optional[Callable] = None):
        self.node = node
        self.topics = dict(topics)
        self.tf = transform
        self.timeout_s = timeout_s
        self.on_local_position = on_local_position
        self.lpos: Optional[VehicleLocalPosition] = None
        self.att: Optional[VehicleAttitude] = None
        self.status: Optional[VehicleStatus] = None
        self.land: Optional[VehicleLandDetected] = None
        self.battery: Optional[BatteryStatus] = None
        self.global_pos: Optional[VehicleGlobalPosition] = None
        self.gps: Optional[SensorGps] = None
        self.t_lpos = 0.0
        self.t_lpos_wall = 0.0      # 마지막 위치 샘플을 받은 실제 시각 (Unix) = 텔레메트리 샘플 생성 시각
        self.lpos_seq = 0
        self._subs = {}
        for key in _SUB_TYPES:
            self._subscribe(key, self.topics[key])
        self._check_deadline = time.monotonic() + 30.0
        self._check_timer = node.create_timer(2.0, self._check_topics)

    # ------------------------------------------------------------ subscriptions
    def _subscribe(self, key: str, name: str):
        if key in self._subs:
            self.node.destroy_subscription(self._subs[key])
        cb = getattr(self, f'_cb_{key}')
        self._subs[key] = self.node.create_subscription(_SUB_TYPES[key], name, cb, PX4_QOS)
        self.topics[key] = name

    def _published_topics(self):
        # 자기 자신의 구독도 그래프에 잡히므로 '퍼블리셔가 있는' 토픽만 본다
        return [n for n, _ in self.node.get_topic_names_and_types() if self.node.count_publishers(n) > 0]

    def _check_topics(self):
        """설정한 토픽에 퍼블리셔가 없으면 버전 접미사만 다른 토픽을 찾아 재구독."""
        graph = self._published_topics()
        all_ok = True
        for key in _SUB_TYPES:
            name = self.topics[key]
            if name in graph:
                continue
            all_ok = False
            alt = find_topic_variant(name, graph)
            if alt and alt != name:
                self.node.get_logger().warn(
                    f'[topic] {name} 퍼블리셔 없음 → {alt} 로 재구독 (PX4/px4_msgs 버전 확인 권장)')
                self._subscribe(key, alt)
        if all_ok or time.monotonic() > self._check_deadline:
            if not all_ok:
                miss_keys = [k for k in _SUB_TYPES if self.topics[k] not in graph]
                missing = [self.topics[k] for k in miss_keys]
                required = [k for k in miss_keys if k not in _OPTIONAL]
                log = self.node.get_logger().error if required else self.node.get_logger().warn
                log(
                    f'[topic] PX4 토픽을 찾지 못함: {missing} - MicroXRCEAgent/PX4 실행 여부 확인')
            self._check_timer.cancel()

    def resolve_publisher_topic(self, name: str) -> str:
        """PX4 가 구독 중인 /fmu/in 토픽 이름 확인 (이미 보이면 버전 변형 탐색)."""
        graph = [n for n, _ in self.node.get_topic_names_and_types() if self.node.count_subscribers(n) > 0]
        if name in graph:
            return name
        alt = find_topic_variant(name, graph)
        if alt and alt != name:
            self.node.get_logger().warn(f'[topic] {name} → {alt} 로 퍼블리시')
            return alt
        return name

    # ------------------------------------------------------------ callbacks
    def _cb_local_position(self, msg):
        self.lpos = msg
        self.t_lpos = time.monotonic()
        self.t_lpos_wall = time.time()
        self.lpos_seq += 1
        if self.on_local_position:
            self.on_local_position(msg)

    def _cb_attitude(self, msg):
        self.att = msg

    def _cb_status(self, msg):
        self.status = msg

    def _cb_land_detected(self, msg):
        self.land = msg

    def _cb_battery(self, msg):
        self.battery = msg

    def _cb_global_position(self, msg):
        self.global_pos = msg

    def _cb_gps(self, msg):
        self.gps = msg

    # ------------------------------------------------------------ accessors
    @property
    def connected(self) -> bool:
        return self.lpos is not None and time.monotonic() - self.t_lpos < self.timeout_s

    def pos_world(self):
        if self.lpos is None:
            return (0.0, 0.0, 0.0)
        return self.tf.local_ned_to_world((self.lpos.x, self.lpos.y, self.lpos.z))

    def vel_world(self):
        if self.lpos is None:
            return (0.0, 0.0, 0.0)
        return self.tf.vel_ned_to_world((self.lpos.vx, self.lpos.vy, self.lpos.vz))

    def yaw_world(self) -> float:
        if self.att is not None:
            _, _, yaw_ned = frames.quat_wxyz_to_euler(self.att.q)
            return frames.yaw_ned_to_enu(yaw_ned)
        if self.lpos is not None:
            return frames.yaw_ned_to_enu(self.lpos.heading)
        return 0.0

    @property
    def armed(self) -> bool:
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def nav_state(self) -> Optional[int]:
        return None if self.status is None else int(self.status.nav_state)

    @property
    def landed(self) -> bool:
        return True if self.land is None else bool(self.land.landed)

    def vehicle_state(self) -> VehicleState:
        return VehicleState(
            connected=self.connected,
            pos=self.pos_world(),
            vel=self.vel_world(),
            yaw=self.yaw_world(),
            armed=self.armed,
            offboard=self.nav_state == NAV_STATE_OFFBOARD,
            auto_land=self.nav_state == NAV_STATE_AUTO_LAND,
            landed=self.landed,
            failsafe=bool(self.status.failsafe) if self.status is not None else False,
            pos_valid=bool(self.lpos and self.lpos.xy_valid and self.lpos.z_valid),
        )


def declare_origin_param(node: Node):
    """PX4 local 원점(=스폰 위치)의 World ENU 좌표. run_sim.sh 의 SPAWN_X/Y 와 같아야 함."""
    v = node.declare_parameter('origin_offset_enu', [0.0, 0.0, 0.0]).value
    return frames.LocalWorldTransform([float(c) for c in v])
