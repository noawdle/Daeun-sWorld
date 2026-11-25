"""모의 안전판단 노드 (Ground Truth 기반) - 실제 판단 프로그램이 오기 전 테스트용.

입력  /vertiport/ground_truth, /vertiport/commander_state
출력  /vertiport/decision (JSON)  ← 실제 판단 프로그램도 같은 규격으로 퍼블리시
      /vertiport/command  (act:=true 일 때 HOLD / RESUME, auto_mission:=true 일 때 이륙→접근→착륙)

auto_mission 흐름
  IDLE → GOTO(패드 상공, approach_alt: 지상이면 수직 이륙 후 이동) → [LAND_OK 이면] LAND → 완료
  ※ 사람·차량은 AI 탐지 범위가 아니므로, 이 노드는 Gazebo 시나리오 정답(ground truth)으로 판단한다.
  접근(GOTO)·착륙(LANDING) 중 판단이 LAND_OK 가 아니면 HOLD, 다시 LAND_OK 가 되면 RESUME
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import judge_core as jc

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MockJudge(Node):
    def __init__(self):
        super().__init__('vertiport_mock_judge')
        self.act = bool(self.declare_parameter('act', True).value)
        self.auto_mission = bool(self.declare_parameter('auto_mission', False).value)
        self.approach_alt = float(self.declare_parameter('approach_alt', 4.0).value)
        self.rules = {k: float(self.declare_parameter(k, v).value) for k, v in jc.DEFAULT_RULES.items()}

        self.gt = None
        self.cmdr = None
        self.decision = None
        self.held_by_me = False
        self.mission = 'WAIT_IDLE' if self.auto_mission else 'OFF'
        self._last_sent = 0.0
        self._seq = 0

        self.pub_dec = self.create_publisher(String, '/vertiport/decision', 10)
        self.pub_cmd = self.create_publisher(String, '/vertiport/command', 10)
        self.create_subscription(String, '/vertiport/ground_truth', self._on_gt, LATCHED)
        self.create_subscription(String, '/vertiport/commander_state', self._on_cmdr, LATCHED)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'Mock judge (GT 기반): act={self.act}, auto_mission={self.auto_mission}')

    def _on_gt(self, msg):
        self.gt = json.loads(msg.data)

    def _on_cmdr(self, msg):
        self.cmdr = json.loads(msg.data)

    def _send(self, cmd: dict):
        cmd['source'] = 'mock_judge'
        self._seq += 1
        cmd['command_id'] = f'judge-{self._seq}'
        self.pub_cmd.publish(String(data=json.dumps(cmd)))
        self._last_sent = time.monotonic()
        self.get_logger().info(f'[judge] → {cmd}')

    def _tick(self):
        if self.gt is None:
            return
        res = jc.evaluate(self.gt.get('objects', []), self.gt.get('pad', {}), self.rules)
        out = {'type': 'decision', 't': round(time.time(), 3), 'source': 'mock_judge_gt',
               'frame': 'ENU', **res}
        self.pub_dec.publish(String(data=json.dumps(out, ensure_ascii=False)))
        if res['decision'] != self.decision:
            self.get_logger().info(f'[judge] decision {self.decision} → {res["decision"]} {res["reasons"]}')
        self.decision = res['decision']

        if not self.cmdr or time.monotonic() - self._last_sent < 1.0:
            return
        st = self.cmdr.get('state')
        safe = self.decision == jc.LAND_OK

        # ---- 안전 게이트: 접근/착륙 중 위험 → HOLD, 해제 → RESUME
        if self.act:
            # 자동 미션이 끝난 뒤(DONE)에도 수동 명령으로 접근·착륙하면 계속 안전 게이트를 적용
            if not safe and st in ('TAKEOFF', 'GOTO', 'LANDING', 'TOUCHDOWN'):
                self._send({'command': 'HOLD'})
                self.held_by_me = True
                return
            if safe and st == 'HOLD' and self.held_by_me:
                self._send({'command': 'RESUME'})
                self.held_by_me = False
                return
            if st != 'HOLD':
                self.held_by_me = False

        # ---- 자동 미션
        pad = self.gt.get('pad', {})
        if self.mission == 'WAIT_IDLE' and st == 'IDLE':
            self._send({'command': 'GOTO', 'x': pad.get('x', 0.0), 'y': pad.get('y', 0.0), 'z': self.approach_alt})
            self.mission = 'APPROACH'
        elif self.mission == 'APPROACH' and st == 'HOVER' and safe:
            self._send({'command': 'LAND', 'x': pad.get('x', 0.0), 'y': pad.get('y', 0.0)})
            self.mission = 'LANDING'
        elif self.mission == 'LANDING' and st in ('LANDED', 'IDLE'):
            self.get_logger().info('[judge] mission complete')
            self.mission = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = MockJudge()
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
