"""가짜 PX4 SITL + 파라미터 로더 (e2e 테스트/라이브 데모 공용). fake_ros 가 sys.path 에 있어야 함."""
import math

import rclpy
import yaml
from px4_msgs.msg import (BatteryStatus, OffboardControlMode, SensorGps, TrajectorySetpoint, VehicleAttitude,
                          VehicleCommand, VehicleGlobalPosition, VehicleLandDetected, VehicleLocalPosition,
                          VehicleStatus)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from vertiport_sim import frames


def load_params(path, overrides):
    cfg = yaml.safe_load(open(path))

    def flat(d, pre=''):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, pre + k + '.'))
            else:
                out[pre + k] = v
        return out
    common = flat(cfg['/**']['ros__parameters'])
    nodes = {k for k in cfg if k != '/**'} | set(overrides) | {'vertiport_perception_stub'}
    for node in nodes:
        p = dict(common)
        p.update(flat(cfg.get(node, {}).get('ros__parameters', {})))
        p.update(overrides.get(node, {}))
        rclpy.PARAMS[node] = p


class FakePx4(Node):
    """PX4 SITL 흉내: offboard/arm/land 명령에 반응하는 1차 지연 기체 (NED local)."""
    QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def __init__(self):
        super().__init__('fake_px4')
        self.p = [0.0, 0.0, 0.0]   # NED
        self.v = [0.0, 0.0, 0.0]
        self.yaw = 0.0             # NED (북)
        self.armed = False
        self.nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.sp = None
        self.t_ocm = -10.0
        self.t_land = None
        self.cmd_log = []
        self.pub_lp = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.QOS)
        self.pub_att = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', self.QOS)
        # 일부러 버전 접미사 없이 퍼블리시 → 노드의 자동 탐색 기능 검증
        self.pub_st = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', self.QOS)
        self.pub_ld = self.create_publisher(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.QOS)
        self.pub_bat = self.create_publisher(BatteryStatus, '/fmu/out/battery_status_v1', self.QOS)
        self.pub_glob = self.create_publisher(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.QOS)
        self.pub_gps = self.create_publisher(SensorGps, '/fmu/out/vehicle_gps_position', self.QOS)
        self.battery = 1.0
        self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode', self._ocm, self.QOS)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self._tsp, self.QOS)
        self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self._cmd, self.QOS)
        self.create_timer(0.02, self._step)

    def _ocm(self, m):
        assert m.position and not m.velocity
        self.t_ocm = rclpy.VT[0]

    def _tsp(self, m):
        assert all(math.isfinite(c) for c in m.position), m.position
        assert all(math.isnan(c) for c in m.velocity)
        self.sp = list(m.position)
        self.sp_yaw = m.yaw

    def _cmd(self, m):
        self.cmd_log.append(m.command)
        assert m.from_external and m.target_system == 1
        if m.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and m.param1 == 1 and m.param2 == 6:
            if rclpy.VT[0] - self.t_ocm < 0.5:
                self.nav = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        elif m.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM and m.param1 == 1:
            self.armed = True
        elif m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.nav = VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    def _step(self):
        dt = 0.02
        t = rclpy.VT[0]
        if self.nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD and t - self.t_ocm > 0.5:
            self.nav = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER   # offboard loss failsafe
        prev = list(self.p)
        if self.armed and self.nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.sp:
            for i in range(3):
                self.p[i] += (self.sp[i] - self.p[i]) * min(1.0, 3.0 * dt)
        elif self.armed and self.nav == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.p[2] = min(0.0, self.p[2] + 0.7 * dt)
        self.p[2] = min(0.0, self.p[2])
        self.v = [(self.p[i] - prev[i]) / dt for i in range(3)]
        landed = self.p[2] > -0.03
        if landed and self.armed and self.nav == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.t_land = self.t_land or t
            if t - self.t_land > 2.0:
                self.armed = False
                self.nav = VehicleStatus.NAVIGATION_STATE_POSCTL
                self.t_land = None
        ts = int(t * 1e6)
        self.pub_lp.publish(VehicleLocalPosition(
            timestamp=ts, xy_valid=True, z_valid=True, x=self.p[0], y=self.p[1], z=self.p[2],
            vx=self.v[0], vy=self.v[1], vz=self.v[2], heading=self.yaw))
        self.pub_att.publish(VehicleAttitude(timestamp=ts, q=list(frames.euler_to_quat_wxyz(0, 0, self.yaw))))
        self.pub_st.publish(VehicleStatus(timestamp=ts, arming_state=2 if self.armed else 1, nav_state=self.nav))
        self.pub_ld.publish(VehicleLandDetected(timestamp=ts, landed=landed))
        if self.armed:
            self.battery = max(0.0, self.battery - 0.0005 * dt)
        self.pub_bat.publish(BatteryStatus(timestamp=ts, connected=True, voltage_v=14.0 + 2.8 * self.battery,
                                           remaining=self.battery, warning=0))
        lat = 35.8889 + self.p[0] / 111320.0
        lon = 128.6103 + self.p[1] / (111320.0 * math.cos(math.radians(35.8889)))
        self.pub_glob.publish(VehicleGlobalPosition(timestamp=ts, lat=lat, lon=lon, alt=50.0 - self.p[2],
                                                    lat_lon_valid=True, alt_valid=True))
        self.pub_gps.publish(SensorGps(timestamp=ts, fix_type=3, satellites_used=12, eph=0.8, epv=1.2))


class FakeCamera(Node):
    """하향 카메라 흉내: 640x480 rgb8 그라데이션 + CameraInfo (stamp = 가상/실제 시간)."""

    def __init__(self, hz=10.0, w=640, h=480):
        super().__init__('fake_camera')
        self.w, self.h = w, h
        self.pub = self.create_publisher(Image, '/camera/down/image_raw', 5)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/down/camera_info', 5)
        row = bytes((x * 255 // w) for x in range(w)) * 3
        self.data = bytes(bytearray(row[i // 3 + (i % 3) * w] for i in range(w * 3))) * h
        self.n = 0
        self.create_timer(1.0 / hz, self._tick)

    def _tick(self):
        t = rclpy.VT[0]
        m = Image()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        m.header.frame_id = 'camera_link'
        m.width, m.height, m.step, m.encoding = self.w, self.h, self.w * 3, 'rgb8'
        m.data = self.data
        self.pub.publish(m)
        ci = CameraInfo()
        ci.header = m.header
        ci.width, ci.height = self.w, self.h
        fx = (self.w / 2) / math.tan(1.74 / 2)
        ci.k = [fx, 0.0, self.w / 2, 0.0, fx, self.h / 2, 0.0, 0.0, 1.0]
        ci.d = [0.0] * 5
        ci.distortion_model = 'plumb_bob'
        self.pub_info.publish(ci)
