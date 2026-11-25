"""End-to-end 오프라인 검증: 실제 노드 코드 + 가짜 rclpy/PX4 + 가상 시간.

검증 항목
  1. config/vertiport.yaml 파라미터가 각 노드에 타입 오류 없이 로드되는가
  2. PX4 토픽 이름 버전 자동 탐색 (가짜 PX4 는 일부러 /fmu/out/vehicle_status 를 무접미사로 퍼블리시)
  3. mock_judge auto_mission: 이륙 → 패드 접근 → 착륙
  4. demo_person_hold 시퀀스: 하강 중 사람 등장 → HOLD → 사람 사라짐 → RESUME → 착지
  5. telemetry JSON 이 규격 키를 모두 갖고, 패드 위(원점)에 착지했는가
실행: python3 test/e2e_fake_ros.py
"""

import json
import math
import os
import sys
import time

_real_sleep = time.sleep

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'fake_ros'))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

import rclpy  # noqa: E402  (fake)
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# ---- 가상 시간 주입
time.monotonic = lambda: rclpy.VT[0]
time.time = lambda: 1.7e9 + rclpy.VT[0]

from fake_px4 import FakePx4, load_params  # noqa: E402
import tf2_ros  # noqa: E402  (fake)

SPAWN = (-6.0, 0.0)


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        self.tel, self.dec, self.acks, self.states = [], [], [], []
        self.create_subscription(String, '/vertiport/telemetry', lambda m: self.tel.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/vertiport/decision', lambda m: self.dec.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/vertiport/command_ack', lambda m: self.acks.append(json.loads(m.data)), 10)
        self.pub_scn = self.create_publisher(String, '/vertiport/scenario_cmd', 10)

    def state(self):
        return self.tel[-1]['state'] if self.tel else None


def main():
    rclpy.reset()
    rclpy.init()
    cfg = os.path.join(HERE, '..', 'config', 'vertiport.yaml')
    origin = [SPAWN[0], SPAWN[1], 0.24]
    load_params(cfg, {
        'vertiport_commander': {'origin_offset_enu': origin},
        'vertiport_telemetry_bridge': {'origin_offset_enu': origin, 'source': 'sim'},
        'vertiport_scenario': {'origin_offset_enu': origin, 'dry_run': True},
        'vertiport_mock_judge': {'origin_offset_enu': origin, 'auto_mission': True},
    })
    from vertiport_sim.commander_node import CommanderNode
    from vertiport_sim.mock_judge_node import MockJudge
    from vertiport_sim.scenario_node import ScenarioNode
    from vertiport_sim.telemetry_bridge import TelemetryBridge

    px4 = FakePx4()
    nodes = [CommanderNode(), TelemetryBridge(), ScenarioNode(), MockJudge()]
    rec = Recorder()

    # 시나리오 시퀀스 시작 (scenario worker 스레드가 startup 을 마칠 시간을 줌)
    rclpy.advance(1.0)
    for _ in range(50):
        if nodes[2].ready:
            break
        _real_sleep(0.05)
    assert nodes[2].ready, 'scenario manager not ready'
    rec.pub_scn.publish(String(data=json.dumps({'op': 'sequence', 'name': 'demo_person_hold'})))

    history = []
    held_seen = resumed = False
    t_end = 120.0
    while rclpy.VT[0] < t_end:
        rclpy.advance(0.1)
        # scenario 워커 스레드가 처리할 시간
        if not nodes[2].jobs.empty():
            for _ in range(40):
                if nodes[2].jobs.empty():
                    break
                _real_sleep(0.01)
            _real_sleep(0.02)
        st = rec.state()
        if not history or history[-1] != st:
            history.append(st)
            print(f't={rclpy.VT[0]:6.1f}s  state={st:<10} alt={rec.tel[-1]["altitude_m"]:.2f} '
                  f'pos=({rec.tel[-1]["position_m"]["x"]:.2f},{rec.tel[-1]["position_m"]["y"]:.2f}) '
                  f'decision={rec.dec[-1]["decision"] if rec.dec else None}')
        if st == 'HOLD':
            held_seen = True
        if held_seen and st == 'LANDING':
            resumed = True
        if st == 'IDLE' and 'LANDING' in history and resumed:
            break

    tel = rec.tel[-1]
    print('\nstate history:', ' → '.join(history))
    print('final telemetry:', json.dumps(tel, ensure_ascii=False)[:400], '...')
    errs = [m for (_, lvl, m) in rclpy.LOG if lvl == 'ERROR']
    warns = [m for (_, lvl, m) in rclpy.LOG if lvl == 'WARN']

    # ---- 검증
    for k in ('t', 'frame', 'position_m', 'rpy_rad', 'orientation_q', 'velocity_mps', 'state', 'altitude_m',
              't_px4', 'px4', 'last_command'):
        assert k in tel, k
    assert tel['frame'] == 'ENU'
    assert any('vehicle_status' in w and '재구독' in w for w in warns), '토픽 버전 자동 탐색 경고가 없음'
    assert held_seen, '사람 등장 시 HOLD 가 없었음'
    assert resumed, 'HOLD 후 착륙 재개가 없었음'
    assert history[-1] == 'IDLE', history
    d = math.hypot(tel['position_m']['x'], tel['position_m']['y'])  # noqa
    assert d < 0.4, f'패드 중심에서 {d:.2f} m 벗어나 착지'
    assert abs(tel['rpy_rad']['yaw'] - math.pi / 2) < 1e-3, 'NED yaw 0(북) → ENU yaw +pi/2'
    q = tel['orientation_q']
    assert abs(q['z'] - math.sin(math.pi / 4)) < 1e-3 and abs(q['w'] - math.cos(math.pi / 4)) < 1e-3
    assert abs(tel['position_m']['z'] - 0.24) < 0.01, '착지 시 base_link 높이 0.24 m'
    # TF
    names = {(t.header.frame_id, t.child_frame_id) for t in tf2_ros.STATIC}
    assert names == {('base_link', 'camera_link'), ('camera_link', 'camera_optical_frame')}, names
    last = tf2_ros.SENT[-1]
    assert (last.header.frame_id, last.child_frame_id) == ('map', 'base_link')
    assert abs(last.header.stamp.sec + last.header.stamp.nanosec * 1e-9 - tel['t_px4']) < 0.1, 'TF stamp = PX4 시간축'
    print('last_command:', tel['last_command'])
    assert tel['last_command']['command'] == 'RESUME' and tel['last_command']['accepted']
    assert not errs, errs
    assert {176, 400, 21} <= set(px4.cmd_log), px4.cmd_log   # SET_MODE, ARM, NAV_LAND 모두 송신
    decisions = [x['decision'] for x in rec.dec]
    assert 'HOLD' in decisions and decisions[-1] == 'LAND_OK'
    # 통계
    print(f'\nOK  착지 오차 {d:.2f} m, 텔레메트리 {len(rec.tel)}건 '
          f'({len(rec.tel) / rclpy.VT[0]:.1f} Hz), ack {len(rec.acks)}건, 경고 {len(warns)}건')
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
