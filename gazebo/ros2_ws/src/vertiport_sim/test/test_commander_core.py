"""CommanderCore 오프라인 시뮬레이션 테스트 (ROS/PX4 없이 실행).

간단한 가상 기체: armed+OFFBOARD 이면 setpoint 를 1차 지연으로 추종,
AUTO_LAND 이면 0.7 m/s 로 하강, 접지 후 2초 뒤 자동 disarm.

실행:  python3 -m pytest test/  (또는 python3 test/test_commander_core.py)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vertiport_sim import commander_core as cc  # noqa: E402


class FakeVehicle:
    def __init__(self, pos=(-6.0, 0.0, 0.0)):
        self.pos = list(pos)
        self.armed = False
        self.mode = 'POSCTL'
        self.landed = True
        self.t_landed = 0.0
        self.sp = None

    def apply(self, out: cc.StepOutput, now):
        for r in out.requests:
            if r == cc.REQ_OFFBOARD and self.sp is not None:
                self.mode = 'OFFBOARD'
            elif r == cc.REQ_ARM:
                self.armed = True
            elif r == cc.REQ_LAND:
                self.mode = 'AUTO_LAND'
            elif r == cc.REQ_DISARM and self.landed:
                self.armed = False
                self.mode = 'POSCTL'
        if out.setpoint is not None:
            self.sp = out.setpoint

    def update(self, dt, now):
        if self.armed and self.mode == 'OFFBOARD' and self.sp is not None:
            k = min(1.0, 3.0 * dt)
            for i in range(3):
                self.pos[i] += (self.sp[i] - self.pos[i]) * k
            self.pos[2] = max(0.0, self.pos[2])
        elif self.armed and self.mode == 'AUTO_LAND':
            self.pos[2] = max(0.0, self.pos[2] - 0.7 * dt)
        if self.pos[2] <= 0.02:
            if not self.landed:
                self.t_landed = now
            self.landed = True
            if self.armed and self.mode == 'AUTO_LAND' and now - self.t_landed > 2.0:
                self.armed = False
        else:
            self.landed = False

    def state(self):
        return cc.VehicleState(
            connected=True, pos=tuple(self.pos), armed=self.armed,
            offboard=self.mode == 'OFFBOARD', auto_land=self.mode == 'AUTO_LAND',
            landed=self.landed)


def run(core, veh, seconds, t0, dt=0.05, until=None):
    t = t0
    end = t0 + seconds
    while t < end:
        out = core.step(veh.state(), t, dt)
        veh.apply(out, t)
        veh.update(dt, t)
        t += dt
        if until and until(core, veh):
            break
    return t


def make():
    core = cc.CommanderCore(pad_xy=(0.0, 0.0))
    veh = FakeVehicle()
    t = run(core, veh, 1.0, 0.0)
    assert core.state == cc.IDLE
    return core, veh, t


def test_vertical_takeoff_goto_land():
    core, veh, t = make()
    ok, _ = core.handle_command({'command': 'GOTO', 'x': -6, 'y': 0, 'z': 4}, t)
    assert ok
    t = run(core, veh, 20, t, until=lambda c, v: c.state == cc.HOVER)
    assert core.state == cc.HOVER and abs(veh.pos[2] - 4) < 0.3

    ok, _ = core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)
    assert ok and core.state == cc.GOTO
    t = run(core, veh, 30, t, until=lambda c, v: c.state == cc.HOVER)
    assert core.state == cc.HOVER and math.hypot(veh.pos[0], veh.pos[1]) < 0.3

    ok, _ = core.handle_command({'command': 'LAND'}, t)
    assert ok and core.state == cc.LANDING
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.IDLE)
    assert core.state == cc.IDLE and not veh.armed
    assert math.hypot(veh.pos[0], veh.pos[1]) < 0.4


def test_goto_from_ground_autotakeoff():
    core, veh, t = make()
    ok, _ = core.handle_command({'command': 'GOTO', 'x': 5, 'y': 3, 'z': 4}, t)
    assert ok and core.state == cc.TAKEOFF
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    assert core.state == cc.HOVER
    assert abs(veh.pos[0] - 5) < 0.3 and abs(veh.pos[1] - 3) < 0.3


def test_hold_resume_during_descent():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    core.handle_command({'command': 'LAND'}, t)
    t = run(core, veh, 30, t, until=lambda c, v: c.phase == cc.PHASE_DESCEND and v.pos[2] < 2.5)
    ok, _ = core.handle_command({'command': 'HOLD'}, t)
    assert ok and core.state == cc.HOLD
    z_hold = veh.pos[2]
    t = run(core, veh, 5, t)
    assert abs(veh.pos[2] - z_hold) < 0.3, 'HOLD 중 고도 유지'
    # 중복 HOLD 는 paused 를 덮어쓰지 않음
    ok, _ = core.handle_command({'command': 'HOLD'}, t)
    assert ok and core.paused['state'] == cc.LANDING
    ok, _ = core.handle_command({'command': 'RESUME'}, t)
    assert ok and core.state == cc.LANDING and core.phase == cc.PHASE_DESCEND
    t = run(core, veh, 30, t, until=lambda c, v: c.state == cc.IDLE)
    assert core.state == cc.IDLE


def test_hold_during_touchdown_returns_to_offboard():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 3}, t)
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    core.handle_command({'command': 'LAND'}, t)
    t = run(core, veh, 30, t, until=lambda c, v: c.state == cc.TOUCHDOWN)
    t = run(core, veh, 0.2, t)
    ok, _ = core.handle_command({'command': 'HOLD'}, t)
    assert ok and core.state == cc.HOLD
    t = run(core, veh, 3, t)
    assert veh.mode == 'OFFBOARD' and core.state == cc.HOLD and not veh.landed
    ok, _ = core.handle_command({'command': 'RESUME'}, t)
    assert ok
    t = run(core, veh, 30, t, until=lambda c, v: c.state == cc.IDLE)
    assert core.state == cc.IDLE


def test_override_when_pilot_switches_mode():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': -6, 'y': 0, 'z': 4}, t)
    t = run(core, veh, 20, t, until=lambda c, v: c.state == cc.HOVER)
    veh.mode = 'POSCTL'   # 조종자가 모드 변경
    t = run(core, veh, 2.0, t)
    assert core.state == cc.OVERRIDE
    assert veh.mode == 'POSCTL', 'Commander 가 OFFBOARD 를 다시 빼앗으면 안 됨'
    ok, _ = core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)   # 명시적 재획득
    assert ok
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    assert core.state == cc.HOVER and veh.mode == 'OFFBOARD'


def test_land_target_refinement():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    core.handle_command({'command': 'LAND', 'x': 0.0, 'y': 0.0}, t)
    t = run(core, veh, 3, t)
    ok, msg = core.handle_command({'command': 'LAND', 'x': 0.6, 'y': -0.4}, t)   # 비전 기반 보정
    assert ok and 'updated' in msg
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.IDLE)
    assert abs(veh.pos[0] - 0.6) < 0.4 and abs(veh.pos[1] + 0.4) < 0.4


def test_rejections():
    core, veh, t = make()
    assert not core.handle_command({'command': 'HOLD'}, t)[0]          # 지상
    assert not core.handle_command({'command': 'LAND'}, t)[0]          # 지상
    assert not core.handle_command({'command': 'RESUME'}, t)[0]
    assert not core.handle_command({'command': 'FLY'}, t)[0]
    assert not core.handle_command({'command': 'TAKEOFF', 'z': 4}, t)[0]   # 규격에서 제외된 명령
    assert not core.handle_command({'command': 'CANCEL'}, t)[0]            # 지상에서는 취소할 것 없음
    assert core.last_cmd['accepted'] is False and core.last_cmd['command'] == 'CANCEL'
    assert not core.handle_command({'command': 'GOTO', 'x': 500, 'y': 0, 'z': 4}, t)[0]
    assert not core.handle_command({'command': 'GOTO', 'x': 1, 'y': 0, 'z': 0.2}, t)[0]
    assert not core.handle_command({'command': 'GOTO', 'x': 'a', 'y': 0, 'z': 4}, t)[0]


def test_command_id_and_alias():
    core, veh, t = make()
    ok, _ = core.handle_command({'cmd': 'goto', 'x': 0, 'y': 0, 'z': 3, 'command_id': 'abc-1'}, t)
    assert ok and core.last_cmd['command_id'] == 'abc-1' and core.last_cmd['command'] == 'GOTO'


def test_cancel_during_landing_hovers_and_is_not_resumable():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.HOVER)
    core.handle_command({'command': 'LAND'}, t)
    t = run(core, veh, 30, t, until=lambda c, v: c.phase == cc.PHASE_DESCEND and v.pos[2] < 3.0)
    ok, _ = core.handle_command({'command': 'CANCEL'}, t)
    assert ok and core.state == cc.HOVER
    z = veh.pos[2]
    t = run(core, veh, 5, t)
    assert abs(veh.pos[2] - z) < 0.3 and core.state == cc.HOVER
    assert not core.handle_command({'command': 'RESUME'}, t)[0]


def test_cancel_after_hold_clears_paused():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 10, 'y': 0, 'z': 4}, t)
    t = run(core, veh, 40, t, until=lambda c, v: c.state == cc.GOTO and v.pos[0] > -2)
    core.handle_command({'command': 'HOLD'}, t)
    assert core.paused is not None
    ok, _ = core.handle_command({'command': 'CANCEL'}, t)
    assert ok and core.paused is None and core.state == cc.HOVER


def test_cancel_before_liftoff_disarms():
    core, veh, t = make()
    core.handle_command({'command': 'GOTO', 'x': 0, 'y': 0, 'z': 4}, t)
    veh.armed = True                    # 지상에서 arm 된 직후(이륙 전) 상황을 재현
    assert veh.landed and core.state == cc.TAKEOFF
    ok, msg = core.handle_command({'command': 'CANCEL'}, t)
    assert ok and core.state == cc.IDLE
    t = run(core, veh, 3, t)
    assert not veh.armed, 'CANCEL 후 disarm 되어야 함'


def test_arm_timeout():
    core = cc.CommanderCore()
    veh = FakeVehicle()
    t = run(core, veh, 1.0, 0.0)
    core.handle_command({'command': 'GOTO', 'x': -6, 'y': 0, 'z': 4}, t)
    # apply() 를 호출하지 않음 = arm 요청을 무시하는 기체 (preflight 실패 상황)
    t2 = t
    for _ in range(int(12 / 0.05)):
        core.step(veh.state(), t2, 0.05)
        t2 += 0.05
    assert core.state == cc.IDLE and 'timeout' in core.message


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
