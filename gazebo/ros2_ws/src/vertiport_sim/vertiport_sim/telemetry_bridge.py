"""Telemetry Bridge: PX4 상태 → 팀 규격 JSON + TF.

출력
  /vertiport/telemetry   (std_msgs/String JSON, rate_hz)  ← HTTP Bridge 가 /api/telemetry 로 제공
  /tf                     map → base_link          (World ENU / body FLU, rate_hz)
  /tf_static              base_link → camera_link → camera_optical_frame

TF 타임스탬프
  tf_stamp_source = px4  : PX4 타임스탬프 사용 (SITL 에서는 Gazebo 시뮬레이션 시간 = 카메라 header.stamp 와 같은 시간축)
  tf_stamp_source = ros  : 노드 시계 (실기체)
"""

import json
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import frames
from .px4_io import Px4Listener, declare_origin_param, declare_topic_params
from .telemetry_core import build_telemetry

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _tf(parent, child, stamp, xyz, q_wxyz) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    t.transform.rotation.w = float(q_wxyz[0])
    t.transform.rotation.x = float(q_wxyz[1])
    t.transform.rotation.y = float(q_wxyz[2])
    t.transform.rotation.z = float(q_wxyz[3])
    return t


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('vertiport_telemetry_bridge')
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.source = self.declare_parameter('source', 'sim').value
        self.stamp_src = self.declare_parameter('tf_stamp_source', 'px4').value
        self.f_map = self.declare_parameter('frames.map', 'map').value
        self.f_base = self.declare_parameter('frames.base', 'base_link').value
        self.f_cam = self.declare_parameter('frames.camera', 'camera_link').value
        self.f_opt = self.declare_parameter('frames.camera_optical', 'camera_optical_frame').value
        cam_xyz = self.declare_parameter('camera_mount_xyz', list(frames.CAMERA_MOUNT_XYZ)).value
        cam_rpy = self.declare_parameter('camera_mount_rpy', list(frames.CAMERA_MOUNT_RPY)).value

        self.tf = declare_origin_param(self)
        self.px4 = Px4Listener(self, declare_topic_params(self), self.tf)
        self.commander = None
        self.t_commander = 0.0
        self._last_seq = -1
        # telemetry_id = sample-<n>: n 은 실행 시각 기반으로 시작 → 재시작해도 겹치지 않음
        self._sample_n = int(time.time()) % 1_000_000 * 1000

        self.pub_tel = self.create_publisher(String, '/vertiport/telemetry', 10)
        self.create_subscription(String, '/vertiport/commander_state', self._on_commander, LATCHED)

        self.tf_pub = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        zero = Time(sec=0, nanosec=0)
        self.tf_static.sendTransform([
            _tf(self.f_base, self.f_cam, zero, cam_xyz, frames.euler_to_quat_wxyz(*cam_rpy)),
            _tf(self.f_cam, self.f_opt, zero, (0.0, 0.0, 0.0), frames.Q_CAMLINK_FROM_OPTICAL_WXYZ),
        ])

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'Telemetry bridge: {self.rate_hz:.0f} Hz, TF {self.f_map}→{self.f_base}→{self.f_cam}→{self.f_opt}, '
            f'origin_offset_enu={self.tf.offset}')

    def _on_commander(self, msg):
        try:
            self.commander = json.loads(msg.data)
            self.t_commander = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _stamp(self, t_px4_us: int) -> Time:
        if self.stamp_src == 'px4':
            return Time(sec=int(t_px4_us // 1_000_000), nanosec=int((t_px4_us % 1_000_000) * 1000))
        ns = self.get_clock().now().nanoseconds
        return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))

    def _extras(self):
        b, g, gps = self.px4.battery, self.px4.global_pos, self.px4.gps
        battery = None
        if b is not None and b.connected:
            battery = {'remaining_pct': round(float(b.remaining) * 100, 1) if b.remaining >= 0 else None,
                       'voltage_v': round(float(b.voltage_v), 2) if b.voltage_v > 0 else None,
                       'warning': int(b.warning)}
        glob = None
        if g is not None and g.lat_lon_valid:
            glob = {'lat': round(float(g.lat), 7), 'lon': round(float(g.lon), 7),
                    'alt_amsl_m': round(float(g.alt), 2) if g.alt_valid else None}
        gpsd = None
        if gps is not None:
            gpsd = {'fix_type': int(gps.fix_type), 'satellites': int(gps.satellites_used),
                    'eph_m': round(float(gps.eph), 2), 'epv_m': round(float(gps.epv), 2)}
        return battery, glob, gpsd

    def _tick(self):
        if self.px4.lpos is None or self.px4.lpos_seq == self._last_seq:
            return      # 새 PX4 샘플이 없으면 발행하지 않음 (같은 샘플에 새 ID 를 붙이지 않기 위해)
        self._last_seq = self.px4.lpos_seq
        self._sample_n += 1
        battery, glob, gpsd = self._extras()
        cmdr = self.commander if time.monotonic() - self.t_commander < 2.0 else None
        lp = self.px4.lpos
        tel = build_telemetry(
            pos_world=self.px4.pos_world(),
            vel_world=self.px4.vel_world(),
            q_frd_ned_wxyz=None if self.px4.att is None else list(self.px4.att.q),
            t_px4_us=int(lp.timestamp),
            pos_valid=bool(lp.xy_valid and lp.z_valid),
            armed=self.px4.armed,
            nav_state=self.px4.nav_state,
            landed=None if self.px4.land is None else bool(self.px4.land.landed),
            failsafe=None if self.px4.status is None else bool(self.px4.status.failsafe),
            commander=cmdr,
            dist_bottom=float(lp.dist_bottom) if lp.dist_bottom_valid else None,
            source=self.source,
            telemetry_id=f'sample-{self._sample_n}',
            t_sample=self.px4.t_lpos_wall,
            battery=battery, global_position=glob, gps=gpsd,
        )
        tel['connected'] = self.px4.connected
        if cmdr:
            tel['commander'] = {k: cmdr.get(k) for k in ('phase', 'paused', 'message')}
        self.pub_tel.publish(String(data=json.dumps(tel, ensure_ascii=False)))

        q = tel['orientation_q']
        if q is not None:
            p = tel['position_m']
            self.tf_pub.sendTransform(_tf(self.f_map, self.f_base, self._stamp(int(lp.timestamp)),
                                          (p['x'], p['y'], p['z']), (q['w'], q['x'], q['y'], q['z'])))


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
