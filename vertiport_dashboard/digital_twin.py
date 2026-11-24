"""3D 뷰어에 전달하는 위치, 자세 및 텔레메트리 유효시간."""

from collections import deque
from datetime import datetime, timezone
from math import atan2, degrees

from gazebo_bridge_client import finite_number
from telemetry_provider import TELEMETRY_MAX_AGE_SECONDS


ATTITUDE_CONVENTION = "ENU_FLU_RZYX_DEG"
TRACE_LIMIT = 180


class TwinTrace:
    def __init__(self):
        self.source = None
        self.points = deque(maxlen=TRACE_LIMIT)
        self.latest_timestamp = None

    def update(self, source, position, timestamp):
        if self.source != source or position is None:
            self.source = source
            self.points.clear()
            self.latest_timestamp = None
        if position is not None and (self.latest_timestamp is None or timestamp > self.latest_timestamp):
            if not self.points or self.points[-1] != position:
                self.points.append(position)
            self.latest_timestamp = timestamp
        return list(self.points)


def bridge_view(drone, transform):
    if not drone["fresh"]:
        status = {
            "DISCONNECTED": "연결 끊김", "TIMEOUT": "연결 끊김", "STALE_TELEMETRY": "데이터 지연",
        }.get(drone["error"], drone["connection"])
        return None, None, 0, status
    remaining = (TELEMETRY_MAX_AGE_SECONDS -
                 (datetime.now(timezone.utc) - datetime.fromisoformat(drone["timestamp"])).total_seconds())
    if remaining <= 0:
        return None, None, 0, "데이터 지연"
    position, attitude = None, None
    raw = tuple(drone[axis] for axis in ("x", "y", "z"))
    if transform is not None and all(finite_number(value) for value in raw):
        converted = transform.to_local(raw)
        if all(finite_number(value) for value in converted):
            position = converted
            angles = tuple(drone[axis] for axis in ("roll", "pitch", "yaw"))
            if drone.get("attitude_convention") == ATTITUDE_CONVENTION and all(finite_number(v) for v in angles):
                attitude = (angles[0] % 360, angles[1] % 360, (angles[2] % 360 - transform.yaw_deg % 360) % 360)
    return position, attitude, max(1, round(remaining * 1000)), drone["connection"]


def demo_attitude(mission):
    origin = mission.path[0]
    x, y, _ = mission.target
    if mission.stage >= 6:
        return (0, 0, degrees(atan2(-y, -x))) if x or y else (0, 0, 0)
    return 0, 0, degrees(atan2(y - origin[1], x - origin[0]))
