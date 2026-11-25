"""시나리오 매니저 ROS2 노드.

입력  /vertiport/scenario_cmd   (std_msgs/String JSON)
        {"op": "apply",    "name": "S1_person"}
        {"op": "spawn",    "type": "fod", "id": "fod_9", "x": 0.4, "y": -0.3}
        {"op": "move",     "id": "person_1", "x": 2.0, "y": 1.0}
        {"op": "remove",   "id": "fod_9"}
        {"op": "clear"}
        {"op": "sequence", "name": "demo_person_hold"}
        {"op": "stop_sequence"}
        {"op": "list"}
출력  /vertiport/ground_truth    (JSON, 2 Hz + 변경 즉시, latched)
      /vertiport/scenario_status (JSON, 명령 결과)
"""

import json
import os
import queue
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .scenario_core import (DEFAULT_MODELS, GZ_PREFIX, GzClient, SequenceRunner, diff_scene,
                            parse_object)

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ScenarioNode(Node):
    def __init__(self):
        super().__init__('vertiport_scenario')
        default_file = os.path.join(get_package_share_directory('vertiport_sim'), 'config', 'scenarios.yaml')
        self.world = self.declare_parameter('world_name', 'vertiport').value
        path = self.declare_parameter('scenario_file', default_file).value
        self.initial = self.declare_parameter('initial_scenario', 'S0_clear').value
        gt_rate = float(self.declare_parameter('gt_rate_hz', 2.0).value)
        pad_xy = self.declare_parameter('pad_xy', [0.0, 0.0]).value
        pad_size = float(self.declare_parameter('pad_size_m', 3.0).value)
        dry_run = bool(self.declare_parameter('dry_run', False).value)

        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        self.models = dict(DEFAULT_MODELS)
        self.models.update(cfg.get('models') or {})
        self.scenarios = cfg.get('scenarios') or {}
        self.sequences = cfg.get('sequences') or {}
        self.pad = {'id': 'pad_1', 'x': float(pad_xy[0]), 'y': float(pad_xy[1]), 'z': 0.0,
                    'size_m': pad_size}
        self.get_logger().info(f'scenario file: {path} ({len(self.scenarios)} scenarios, '
                               f'{len(self.sequences)} sequences)')

        self.gz = GzClient(self.world, dry_run=dry_run)
        self.lock = threading.Lock()
        self.scene = {}             # id → SceneObject (Gazebo 에 실제로 생성된 것)
        self.active_name = None
        self.ready = False
        self.seq = None
        self.altitude = None
        self.jobs = queue.Queue()

        self.pub_gt = self.create_publisher(String, '/vertiport/ground_truth', LATCHED)
        self.pub_status = self.create_publisher(String, '/vertiport/scenario_status', 10)
        self.create_subscription(String, '/vertiport/scenario_cmd', self._on_cmd, 10)
        self.create_subscription(String, '/vertiport/telemetry', self._on_telemetry, 10)
        self.create_timer(1.0 / gt_rate, self._publish_gt)
        self.create_timer(0.2, self._tick_sequence)

        threading.Thread(target=self._worker, daemon=True).start()
        self.jobs.put(('startup', None))

    # ------------------------------------------------------------ ROS 입력
    def _on_telemetry(self, msg: String):
        try:
            self.altitude = json.loads(msg.data).get('altitude_m')
        except (json.JSONDecodeError, AttributeError):
            pass

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            op = str(cmd.get('op', '')).lower()
        except (json.JSONDecodeError, AttributeError) as e:
            self._status(False, f'invalid json: {e}')
            return
        if op == 'list':
            self._status(True, 'list', scenarios={k: (v or {}).get('description', '')
                                                  for k, v in self.scenarios.items()},
                         sequences=list(self.sequences))
            return
        if op == 'sequence':
            name = cmd.get('name')
            if name not in self.sequences:
                self._status(False, f'unknown sequence "{name}"', op=op)
                return
            self.seq = SequenceRunner(name, list(self.sequences[name]))
            self._status(True, f'sequence {name} started', op=op)
            return
        if op == 'stop_sequence':
            self.seq = None
            self._status(True, 'sequence stopped', op=op)
            return
        if op not in ('apply', 'spawn', 'move', 'remove', 'clear'):
            self._status(False, f'unknown op "{op}"', op=op)
            return
        self.jobs.put((op, cmd))

    def _tick_sequence(self):
        if self.seq is None or not self.ready:
            return
        try:
            name = self.seq.step(time.monotonic(), self.altitude)
        except ValueError as e:
            self._status(False, str(e), op='sequence')
            self.seq = None
            return
        if name:
            self.jobs.put(('apply', {'op': 'apply', 'name': name, 'from_sequence': self.seq.name}))
        if self.seq.done:
            self._status(True, f'sequence {self.seq.name} finished', op='sequence')
            self.seq = None

    # ------------------------------------------------------------ worker (gz 호출은 여기서만)
    def _worker(self):
        while rclpy.ok():
            try:
                op, cmd = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if op == 'startup':
                    self._startup()
                elif not self.ready:
                    self._status(False, 'gazebo world not ready yet', op=op)
                elif op == 'apply':
                    self._apply(cmd)
                elif op == 'spawn':
                    o = parse_object(cmd, self.models)
                    with self.lock:
                        target = list(self.scene.values()) + [o]
                    self._sync(target, f'spawn {o.id}')
                elif op == 'move':
                    with self.lock:
                        cur = self.scene.get(str(cmd.get('id')))
                    if cur is None:
                        raise ValueError(f'no object "{cmd.get("id")}"')
                    o = parse_object({**cur.to_gt(), 'uri': cur.uri, **cmd, 'type': cur.type}, self.models)
                    with self.lock:
                        target = [o if x.id == o.id else x for x in self.scene.values()]
                    self._sync(target, f'move {o.id}')
                elif op == 'remove':
                    oid = str(cmd.get('id'))
                    with self.lock:
                        if oid not in self.scene:
                            raise ValueError(f'no object "{oid}"')
                        target = [x for x in self.scene.values() if x.id != oid]
                    self._sync(target, f'remove {oid}')
                elif op == 'clear':
                    self._sync([], 'clear')
                    self.active_name = None
            except (ValueError, KeyError, TypeError) as e:
                self._status(False, str(e), op=op)

    def _startup(self):
        self.get_logger().info(f'waiting for Gazebo world "{self.world}" (UserCommands services)...')
        t0 = time.monotonic()
        while rclpy.ok() and not self.gz.world_ready():
            if time.monotonic() - t0 > 120:
                self.get_logger().error(
                    f'/world/{self.world}/create 서비스 없음 - world 이름 또는 UserCommands 플러그인 확인')
                t0 = time.monotonic()
            time.sleep(2.0)
        # 이전 실행에서 남은 시나리오 객체 정리
        for name in self.gz.list_models():
            if name.startswith(GZ_PREFIX):
                ok, _ = self.gz.remove(name)
                self.get_logger().info(f'cleanup leftover {name}: {ok}')
        self.ready = True
        self.get_logger().info('Gazebo world ready')
        if self.initial:
            self._apply({'name': self.initial})

    def _apply(self, cmd):
        name = cmd.get('name')
        if name not in self.scenarios:
            raise ValueError(f'unknown scenario "{name}"')
        objs = [parse_object(d, self.models) for d in ((self.scenarios[name] or {}).get('objects') or [])]
        if self._sync(objs, f'apply {name}', extra={'from_sequence': cmd.get('from_sequence')}):
            self.active_name = name

    def _sync(self, target, label, extra=None) -> bool:
        with self.lock:
            current = dict(self.scene)
        remove, spawn, move = diff_scene(current, target)
        errors = []
        for o in remove:
            ok, out = self.gz.remove(o.gz_name)
            with self.lock:
                self.scene.pop(o.id, None)   # 실패해도 장부에서는 제거 (다음 apply 에서 재시도 방지)
            if not ok:
                errors.append(f'remove {o.id}: {out}')
        for o in spawn:
            ok, out = self.gz.create(o)
            if ok:
                with self.lock:
                    self.scene[o.id] = o
            else:
                errors.append(f'create {o.id}: {out}')
        for o in move:
            ok, out = self.gz.set_pose(o)
            if ok:
                with self.lock:
                    self.scene[o.id] = o
            else:
                errors.append(f'move {o.id}: {out}')
        ok = not errors
        self._status(ok, label if ok else f'{label} - errors: {errors}', **(extra or {}))
        (self.get_logger().info if ok else self.get_logger().error)(
            f'[scenario] {label}: -{len(remove)} +{len(spawn)} ~{len(move)}' + ('' if ok else f' {errors}'))
        self._publish_gt()
        return ok

    # ------------------------------------------------------------ 출력
    def _publish_gt(self):
        with self.lock:
            objs = [o.to_gt() for o in self.scene.values()]
        gt = {'type': 'ground_truth', 't': round(time.time(), 3), 'frame': 'ENU', 'origin': 'pad',
              'scenario': self.active_name, 'world_ready': self.ready, 'pad': self.pad,
              'objects': sorted(objs, key=lambda o: o['id'])}
        if self.seq is not None:
            gt['sequence'] = {'name': self.seq.name, 'step': self.seq.current_step()}
        self.pub_gt.publish(String(data=json.dumps(gt, ensure_ascii=False)))

    def _status(self, ok, message, **kw):
        out = {'type': 'scenario_status', 't': round(time.time(), 3), 'ok': ok, 'message': message,
               'scenario': self.active_name}
        out.update({k: v for k, v in kw.items() if v is not None})
        self.pub_status.publish(String(data=json.dumps(out, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()
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
