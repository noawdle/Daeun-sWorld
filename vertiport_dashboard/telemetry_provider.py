"""드론 시뮬레이터와 후속 텔레메트리 공급자의 대시보드 계약."""

from datetime import datetime, timezone
from threading import Lock
from typing import Protocol
from urllib.parse import urlsplit

from drone_simulator import get_drone_state
from gazebo_bridge_client import BRIDGE_FRAME, BridgeError, finite_number


TELEMETRY_MAX_AGE_SECONDS = 3
_NUMBERS = ("x", "y", "z", "altitude_m", "speed", "roll", "pitch", "yaw",
            "battery", "voltage", "latitude", "longitude", "satellites")
_STRINGS = ("id", "flight_mode", "gps", "mavlink", "telemetry_id", "attitude_convention")
_FLAGS = ("armed", "emergency", "landing_requested")


def unavailable_telemetry(source, connection="연결 대기", *, error=None, bridge_online=False):
    return dict.fromkeys((*_NUMBERS, *_STRINGS, *_FLAGS, "camera_stream_url", "timestamp", "frame")) | {
        "source": source, "connection": connection, "error": error, "bridge_online": bridge_online,
        "fresh": False, "simulation_ready": False,
    }


class TelemetryProvider(Protocol):
    def read(self, elapsed: float) -> dict: ...


class DemoTelemetryProvider:
    def __init__(self, scenario=None, manual=None):
        self.scenario = scenario
        self.manual = manual

    def read(self, elapsed):
        state = get_drone_state(elapsed, self.scenario) if self.scenario else self.manual
        return unavailable_telemetry("시뮬레이션 데이터", "DEMO 작동") | {
            **{key: value for key, value in state.items() if key in (*_NUMBERS, *_STRINGS, *_FLAGS)},
            "flight_mode": state.get("flight_phase"), "fresh": True,
        }


class GazeboTelemetryProvider:
    def __init__(self, client):
        self.client = client
        self.source = "Mock Bridge · 시뮬레이션 데이터" if client.mock else "Gazebo Bridge"
        self._latest = unavailable_telemetry(self.source)
        self._lock = Lock()

    def read(self, elapsed=0):
        health = None
        try:
            health = self.client.health()
            state = self._parse(self.client.telemetry())
            state.update(bridge_online=True, simulation_ready=health.get("simulation_ready") is True)
        except BridgeError as error:
            connection = "Bridge 연결 · 텔레메트리 대기" if health else "연결 대기"
            state = unavailable_telemetry(self.source, connection, error=error.code, bridge_online=bool(health))
        with self._lock:
            self._latest = state
        return dict(state)

    def latest(self):
        """카메라 저장 스레드는 마지막 유효 상태만 사용하고 HTTP를 호출하지 않습니다."""
        with self._lock:
            state = dict(self._latest)
        if state["fresh"] and not self._fresh_timestamp(state["timestamp"]):
            return unavailable_telemetry(self.source, "Bridge 연결 · 텔레메트리 대기",
                                         error="STALE_TELEMETRY", bridge_online=state["bridge_online"])
        return state

    @staticmethod
    def _fresh_timestamp(value):
        try:
            timestamp = datetime.fromisoformat(value)
            if timestamp.utcoffset() is None:
                return False
            age = (datetime.now(timezone.utc) - timestamp).total_seconds()
            return -1 <= age <= TELEMETRY_MAX_AGE_SECONDS
        except (TypeError, ValueError, OverflowError):
            return False

    def _parse(self, payload):
        timestamp = payload.get("timestamp")
        if not self._fresh_timestamp(timestamp):
            raise BridgeError("STALE_TELEMETRY", "최신 텔레메트리가 필요합니다.")
        if payload.get("frame") != BRIDGE_FRAME:
            raise BridgeError("UNKNOWN_FRAME", "지원하지 않는 텔레메트리 좌표계입니다.")
        state = unavailable_telemetry(self.source, "Mock 응답" if self.client.mock else "Bridge 응답")
        for key in _NUMBERS:
            value = payload.get(key)
            if finite_number(value):
                state[key] = value
        if state["battery"] is not None and not 0 <= state["battery"] <= 100:
            state["battery"] = None
        for key in _STRINGS:
            value = payload.get(key)
            state[key] = value.strip() if isinstance(value, str) and value.strip() else None
        for key in _FLAGS:
            value = payload.get(key)
            state[key] = value if type(value) is bool else None
        stream_url = payload.get("camera_stream_url")
        if isinstance(stream_url, str):
            try:
                address = urlsplit(stream_url)
                if (address.scheme in ("http", "https", "rtsp", "rtsps", "webrtc") and address.hostname
                        and address.username is None and address.password is None):
                    state["camera_stream_url"] = stream_url
            except ValueError:
                pass
        state.update(timestamp=timestamp, frame=BRIDGE_FRAME, fresh=True)
        return state


# 기존 import를 사용하는 도구와 호환합니다.
DemoTelemetry = DemoTelemetryProvider
