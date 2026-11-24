"""로컬 DEMO 임무 상태 머신. 비행 명령은 생성하지 않습니다."""

from dataclasses import dataclass, field


STAGES = ("대기", "이륙", "버티포트 이동", "패드 상공 Hover", "Landing Pad 탐색",
          "FOD 안전 확인", "패드 중심 정렬", "하강", "착륙 완료")
PAD = (0.0, 0.0, 0.0)
ORIGIN = (-7.0, -5.0, 0.0)


@dataclass
class DemoMission:
    stage: int = 0
    started: float = 0.0
    prechecked: bool = False
    paused_at: float | None = None
    requested: bool = False
    pad_confirmed: bool = False
    target: tuple = (0.0, 0.0, 3.0)
    path: list = field(default_factory=lambda: [ORIGIN])
    hold_position: tuple | None = None

    def set_target(self, target):
        if self.stage not in (0, 8):
            raise ValueError("대기 또는 착륙 완료 상태에서 목표를 적용할 수 있습니다.")
        self.target = target

    def start(self, now):
        if not self.prechecked or self.stage not in (0, 8) or self.target[2] <= 0:
            return False
        self.stage, self.started = 1, now
        self.requested = self.pad_confirmed = False
        self.paused_at = None
        self.path = [ORIGIN]
        self.hold_position = None
        return True

    def cancel(self):
        self.stage = 0
        self.paused_at = None
        self.requested = self.pad_confirmed = self.prechecked = False
        self.path = [ORIGIN]
        self.hold_position = None

    def toggle_pause(self, now):
        if self.paused_at is None:
            self.paused_at = now
        else:
            self.started += now - self.paused_at
            self.paused_at = None

    def position(self, now):
        x, y, z = self.target
        elapsed = (self.paused_at if self.paused_at is not None else now) - self.started
        span = {1: 2, 2: 4, 6: 2, 7: 3}.get(self.stage, 1)
        p = min(1, max(0, elapsed / span))
        start_x, start_y, _ = ORIGIN
        hold_x, hold_y, hold_z = self.hold_position or self.target
        return (
            ORIGIN, (start_x, start_y, z*p),
            (start_x+(x-start_x)*p, start_y+(y-start_y)*p, z),
            (x, y, z), (x, y, z), (hold_x, hold_y, hold_z),
            (hold_x*(1-p), hold_y*(1-p), hold_z+(z-hold_z)*p),
            (PAD[0], PAD[1], z*(1-p)), PAD,
        )[self.stage]

    def tick(self, now, safety):
        if self.paused_at is not None:
            return None
        elapsed = now - self.started
        if self.stage in (6, 7) and safety != "SAFE":
            self.hold_position = self.position(now)
            self.path.append(self.hold_position)
            self.stage, self.started, self.requested = 5, now, False
            return "하강 보류", "DEMO 안전 조건 재확인 및 착륙 재요청 필요"
        advance = {1: elapsed >= 2, 2: elapsed >= 4, 3: elapsed >= 2,
                   4: self.pad_confirmed, 5: self.requested and safety == "SAFE",
                   6: elapsed >= 2, 7: elapsed >= 3}
        if not advance.get(self.stage):
            return None
        self.path.append(self.position(now))
        self.stage += 1
        self.started = now
        return {3: ("Hover 도달", "DEMO 목표 좌표 도달"),
                5: ("Landing Pad 확인 상태", "DEMO 운영자 확인"),
                7: ("하강 시작", "DEMO 안전 조건 충족"),
                8: ("착륙 완료", "DEMO 임무 완료")}.get(self.stage)
