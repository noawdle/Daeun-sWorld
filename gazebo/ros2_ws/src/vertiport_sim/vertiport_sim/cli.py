"""터미널용 명령 도구.

  ros2 run vertiport_sim vp_cmd goto 5 3 4 [yaw_rad]     # 지상이면 수직 이륙 후 이동
  ros2 run vertiport_sim vp_cmd hold | resume | cancel
  ros2 run vertiport_sim vp_cmd land [x y]

  ros2 run vertiport_sim vp_scenario list
  ros2 run vertiport_sim vp_scenario apply S1_person
  ros2 run vertiport_sim vp_scenario spawn fod fod_9 0.4 -0.3
  ros2 run vertiport_sim vp_scenario move person_1 2.0 1.0
  ros2 run vertiport_sim vp_scenario remove fod_9
  ros2 run vertiport_sim vp_scenario clear
  ros2 run vertiport_sim vp_scenario sequence demo_person_hold
"""

import json
import sys
import time

import rclpy
from std_msgs.msg import String


def _send_and_wait(topic: str, payload: dict, reply_topic: str, timeout=3.0, match=None):
    rclpy.init()
    node = rclpy.create_node('vp_cli')
    replies = []

    def on_reply(m):
        if match is None or match(m.data):
            replies.append(m.data)
    node.create_subscription(String, reply_topic, on_reply, 10)
    pub = node.create_publisher(String, topic, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print(f'[warn] {topic} 구독자가 없음 (노드가 실행 중인지 확인)')
    pub.publish(String(data=json.dumps(payload)))
    t0 = time.monotonic()
    while not replies and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(replies[0] if replies else '(응답 없음)')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if replies else 1


def _usage():
    print(__doc__)
    return 2


def cmd_main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if not a:
        return _usage()
    c = a[0].upper()
    try:
        if c == 'GOTO':
            p = {'command': c, 'x': float(a[1]), 'y': float(a[2]), 'z': float(a[3])}
            if len(a) > 4:
                p['yaw'] = float(a[4])
        elif c in ('HOLD', 'RESUME', 'CANCEL'):
            p = {'command': c}
        elif c == 'LAND':
            p = {'command': c} if len(a) < 3 else {'command': c, 'x': float(a[1]), 'y': float(a[2])}
        else:
            return _usage()
    except (IndexError, ValueError):
        return _usage()
    p['source'] = 'cli'
    p['command_id'] = f'cli-{int(time.time() * 1000)}'
    return _send_and_wait('/vertiport/command', p, '/vertiport/command_ack',
                          match=lambda r: json.loads(r).get('command_id') == p['command_id'])


def scenario_main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if not a:
        return _usage()
    op = a[0].lower()
    try:
        if op in ('list', 'clear', 'stop_sequence'):
            p = {'op': op}
        elif op in ('apply', 'sequence'):
            p = {'op': op, 'name': a[1]}
        elif op == 'spawn':
            p = {'op': op, 'type': a[1], 'id': a[2], 'x': float(a[3]), 'y': float(a[4])}
            if len(a) > 5:
                p['yaw'] = float(a[5])
        elif op == 'move':
            p = {'op': op, 'id': a[1], 'x': float(a[2]), 'y': float(a[3])}
        elif op == 'remove':
            p = {'op': op, 'id': a[1]}
        else:
            return _usage()
    except (IndexError, ValueError):
        return _usage()
    return _send_and_wait('/vertiport/scenario_cmd', p, '/vertiport/scenario_status', timeout=10.0)


if __name__ == '__main__':
    sys.exit(cmd_main())
