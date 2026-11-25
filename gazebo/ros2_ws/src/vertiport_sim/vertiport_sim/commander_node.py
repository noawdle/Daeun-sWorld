"""Commander ROS2 노드.

  /vertiport/command (std_msgs/String, JSON)  ──▶  CommanderCore  ──▶  PX4 (/fmu/in/...)
                                                       │
  /vertiport/commander_state (JSON, 10 Hz)  ◀──────────┤
  /vertiport/command_ack     (JSON)          ◀─────────┘

- World ENU ↔ PX4 local NED 변환은 여기서만 수행한다.
- OffboardControlMode + TrajectorySetpoint 를 연결된 동안 항상 rate_hz 로 송신한다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from . import frames
from .commander_core import (DEFAULTS, REQ_ARM, REQ_DISARM, REQ_LAND, REQ_OFFBOARD, CommanderCore,
                             command_name, result_code)
from .px4_io import PX4_QOS, Px4Listener, declare_origin_param, declare_topic_params

RELIABLE_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
NAN = float('nan')


class CommanderNode(Node):
    def __init__(self):
        super().__init__('vertiport_commander')
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.target_system = int(self.declare_parameter('target_system', 1).value)
        self.tf = declare_origin_param(self)
        topics = declare_topic_params(self)

        cfg = {}
        for key, default in DEFAULTS.items():
            if isinstance(default, tuple):
                cfg[key] = tuple(self.declare_parameter(key, list(default)).value)
            else:
                cfg[key] = self.declare_parameter(key, default).value
        self.core = CommanderCore(**cfg)

        self.px4 = Px4Listener(self, topics, self.tf)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, self.px4.resolve_publisher_topic(topics['offboard_control_mode']), PX4_QOS)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, self.px4.resolve_publisher_topic(topics['trajectory_setpoint']), PX4_QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, self.px4.resolve_publisher_topic(topics['vehicle_command']), PX4_QOS)

        self.pub_state = self.create_publisher(String, '/vertiport/commander_state', RELIABLE_LATCHED)
        self.pub_ack = self.create_publisher(String, '/vertiport/command_ack', 10)
        self.create_subscription(String, '/vertiport/command', self._on_command, 10)

        self._id_seq = 0
        self._last_t = time.monotonic()
        self._state_div = max(1, int(round(self.rate_hz / 10.0)))
        self._tick_n = 0
        self._last_logged_state = None
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'Commander ready: origin_offset_enu={self.tf.offset}, pad_xy={self.core.cfg["pad_xy"]}')

    # ------------------------------------------------------------------ commands
    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError('not an object')
        except (json.JSONDecodeError, ValueError) as e:
            self._ack({}, False, f'invalid: json ({e})')
            return
        if not cmd.get('command_id'):
            self._id_seq += 1
            cmd['command_id'] = f'cmd-{int(time.time() * 1000)}-{self._id_seq}'
        # 명령 처리 전 최신 기체 상태 반영
        self.core.observe(self.px4.vehicle_state())
        ok, reason = self.core.handle_command(cmd, time.monotonic())
        self._ack(cmd, ok, reason)
        level = self.get_logger().info if ok else self.get_logger().warn
        level(f'[cmd] {cmd} → {"ACCEPTED" if ok else "REJECTED"}: {reason}')
        self._publish_state()

    def _ack(self, cmd, ok, reason):
        out = {'type': 'command_ack', 't': round(time.time(), 3),
               'command_id': cmd.get('command_id'), 'command': command_name(cmd) or None,
               'accepted': ok, 'code': result_code(ok, reason), 'reason': reason, 'state': self.core.state}
        self.pub_ack.publish(String(data=json.dumps(out, ensure_ascii=False)))

    # ------------------------------------------------------------------ loop
    def _tick(self):
        now = time.monotonic()
        dt = min(0.2, max(1e-3, now - self._last_t))
        self._last_t = now
        vs = self.px4.vehicle_state()
        out = self.core.step(vs, now, dt)

        if vs.connected and out.setpoint is not None:
            self._publish_offboard(out.setpoint, out.yaw)
        for r in out.requests:
            if r == REQ_OFFBOARD:
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif r == REQ_ARM:
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            elif r == REQ_DISARM:
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            elif r == REQ_LAND:
                self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info(f'[px4] request {r}')

        if self.core.state != self._last_logged_state:
            self.get_logger().info(f'[state] {self._last_logged_state} → {self.core.state} '
                                   f'({self.core.message})')
            self._last_logged_state = self.core.state
            self._publish_state()
        self._tick_n += 1
        if self._tick_n % self._state_div == 0:
            self._publish_state()

    def _publish_state(self):
        st = self.core.status()
        st.update({'type': 'commander_state', 't': round(time.time(), 3), 'frame': 'ENU'})
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))

    # ------------------------------------------------------------------ PX4 out
    def _ts_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard(self, sp_world, yaw_enu):
        m = OffboardControlMode()
        m.timestamp = self._ts_us()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_mode.publish(m)

        x, y, z = self.tf.world_to_local_ned(sp_world)
        s = TrajectorySetpoint()
        s.timestamp = m.timestamp
        s.position = [float(x), float(y), float(z)]
        s.velocity = [NAN, NAN, NAN]
        s.acceleration = [NAN, NAN, NAN]
        s.jerk = [NAN, NAN, NAN]
        s.yaw = float(frames.yaw_enu_to_ned(yaw_enu))
        s.yawspeed = NAN
        self.pub_sp.publish(s)

    def _vehicle_command(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        c = VehicleCommand()
        c.timestamp = self._ts_us()
        c.command = int(command)
        c.param1, c.param2, c.param3, c.param4 = float(p1), float(p2), float(p3), float(p4)
        if command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            # param4 = yaw(NaN: 현재 유지), param5/6 = lat/lon(NaN: 현재 위치)
            c.param4, p5, p6, p7 = NAN, NAN, NAN, NAN
        c.param5, c.param6, c.param7 = float(p5), float(p6), float(p7)
        c.target_system = self.target_system
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
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
