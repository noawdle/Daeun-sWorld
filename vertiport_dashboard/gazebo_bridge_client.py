"""Versioned HTTP client for an external simulation bridge."""

import json
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from http.client import HTTPException
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener
from uuid import uuid4


LOCAL_FRAME = "VERTIPORT_LOCAL_XYZ_UP"
BRIDGE_FRAME = "GAZEBO_WORLD_ENU"
MAX_RESPONSE_BYTES = 64 * 1024


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class BridgeError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


@dataclass(frozen=True)
class CoordinateTransform:
    """Scale-preserving XY rotation then translation; both frames are Z-up."""

    yaw_deg: float
    offset_m: tuple

    def __post_init__(self):
        if len(self.offset_m) != 3 or not all(finite_number(value) for value in (self.yaw_deg, *self.offset_m)):
            raise ValueError("변환 설정에는 유한한 회전각과 세 개의 오프셋이 필요합니다.")

    @classmethod
    def from_json(cls, raw):
        settings = json.loads(raw)
        if not isinstance(settings, dict) or settings.get("source_frame") != LOCAL_FRAME or settings.get("target_frame") != BRIDGE_FRAME:
            raise ValueError("변환 좌표계가 일치해야 합니다.")
        yaw, offset = settings.get("yaw_deg"), settings.get("offset_m")
        if not isinstance(offset, list) or len(offset) != 3:
            raise ValueError("offset_m에는 세 개의 값이 필요합니다.")
        return cls(yaw, tuple(offset))

    def to_bridge(self, point):
        x, y, z = point
        yaw = math.radians(self.yaw_deg)
        dx, dy, dz = self.offset_m
        return (math.cos(yaw)*x - math.sin(yaw)*y + dx,
                math.sin(yaw)*x + math.cos(yaw)*y + dy, z + dz)

    def to_local(self, point):
        dx, dy, dz = self.offset_m
        x, y, z = (value - offset for value, offset in zip(point, (dx, dy, dz)))
        yaw = math.radians(self.yaw_deg)
        return (math.cos(yaw)*x + math.sin(yaw)*y, -math.sin(yaw)*x + math.cos(yaw)*y, z)


class GazeboBridgeClient:
    def __init__(self, base_url=None, *, mock=False, timeout=0.65, transform=None, enable_commands=False):
        self.base_url = (base_url or os.getenv("VERTIPORT_BRIDGE_URL") or "http://127.0.0.1:8765").rstrip("/")
        try:
            address = urlsplit(self.base_url)
            valid = (address.scheme in ("http", "https") and address.hostname
                     and not (address.username or address.password or address.query or address.fragment)
                     and not address.path and (address.port is None or address.port > 0))
        except ValueError:
            valid = False
        if not mock and not valid:
            raise BridgeError("INVALID_URL", "Bridge 주소에 HTTP(S) 호스트와 포트를 설정해 주세요.")
        self.mock = mock
        self.timeout = timeout
        self.transform = transform
        self.enable_commands = mock or enable_commands
        self.commands = []
        self._opener = build_opener(_NoRedirect)

    def _request(self, path, payload=None):
        if self.mock:
            return self._mock_response(path, payload)
        body = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf-8")
        request = Request(self.base_url + path, data=body,
                          headers={"Accept": "application/json", "Content-Type": "application/json"})
        try:
            try:
                with self._opener.open(request, timeout=self.timeout) as response:
                    return self._decode(response.read(MAX_RESPONSE_BYTES + 1))
            except HTTPError as error:
                with error:
                    if payload is not None and error.code in (400, 409, 422, 503):
                        result = self._decode(error.read(MAX_RESPONSE_BYTES + 1))
                        if result.get("accepted") is False:
                            return result
                raise BridgeError(f"HTTP_{error.code}", f"Bridge HTTP {error.code}") from error
        except (OSError, HTTPException) as error:
            reason = getattr(error, "reason", error)
            if isinstance(reason, TimeoutError):
                raise BridgeError("TIMEOUT", "Bridge 응답 시간이 초과되었습니다.") from error
            raise BridgeError("DISCONNECTED", "Bridge 서버에 연결할 수 없습니다.") from error

    @staticmethod
    def _decode(body):
        try:
            if len(body) > MAX_RESPONSE_BYTES:
                raise ValueError
            payload = json.loads(body)
            if not isinstance(payload, dict):
                raise ValueError
        except (ValueError, RecursionError) as error:
            raise BridgeError("INVALID_RESPONSE", "Bridge JSON 응답 형식이 올바르지 않습니다.") from error
        return payload

    def health(self):
        result = self._request("/api/health")
        if result.get("api_version") != "1" or result.get("status") != "ok":
            raise BridgeError("NOT_READY", "Bridge가 호환되는 준비 상태를 보고하지 않았습니다.")
        return result

    def telemetry(self):
        return self._request("/api/telemetry")

    def _command(self, command, parameters):
        if not self.enable_commands:
            raise BridgeError("COMMANDS_DISABLED", "Bridge 명령 전송이 비활성화되어 있습니다.")
        request_id = uuid4().hex
        result = self._request("/api/command", {
            "request_id": request_id, "command": command, "parameters": parameters,
        })
        command_id = result.get("command_id")
        if (result.get("request_id") != request_id or type(result.get("accepted")) is not bool
                or (result["accepted"] and (not isinstance(command_id, str) or not command_id.strip()))):
            raise BridgeError("INVALID_RESPONSE", "명령 응답의 요청 ID 또는 승인 정보가 올바르지 않습니다.")
        return result

    def send_target(self, x, y, z):
        from map_controller import local_target

        point = local_target(x, y, z)
        if self.transform is None:
            raise BridgeError("FRAME_UNCONFIRMED", "좌표 변환 규칙 확인이 필요합니다.")
        transformed = self.transform.to_bridge(point)
        if not all(math.isfinite(value) for value in transformed):
            raise BridgeError("INVALID_COORDINATES", "변환된 목표 좌표가 유효하지 않습니다.")
        return self._command("set_target", dict(zip(("x", "y", "z"), transformed)) | {"frame": BRIDGE_FRAME})

    def hover(self):
        return self._command("hover", {})

    def request_landing(self, *, safety, telemetry_id):
        if not self.mock:
            raise BridgeError("CAMERA_UNVERIFIED", "Bridge 하향 영상과 안전 판단을 연결한 후 착륙 요청을 활성화할 수 있습니다.")
        if safety != "SAFE" or not isinstance(telemetry_id, str) or not telemetry_id.strip():
            raise BridgeError("LANDING_BLOCKED", "최신 SAFE 판단과 텔레메트리 확인이 필요합니다.")
        return self._command("request_landing", {"telemetry_id": telemetry_id})

    def cancel_mission(self):
        return self._command("cancel_mission", {})

    def _mock_response(self, path, payload):
        if path == "/api/health":
            return {"api_version": "1", "status": "ok", "simulation_ready": True}
        if path == "/api/telemetry":
            return {
                "telemetry_id": uuid4().hex, "timestamp": datetime.now(timezone.utc).isoformat(),
                "frame": BRIDGE_FRAME, "id": "MOCK-X500", "x": 0.0, "y": 0.0, "z": 3.0,
                "altitude_m": 3.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "speed": 0.0,
                "attitude_convention": "ENU_FLU_RZYX_DEG",
                "battery": 70.0, "flight_mode": "HOLD", "armed": True, "emergency": False,
                "camera_stream_url": None,
            }
        self.commands.append(payload)
        return {"request_id": payload["request_id"], "accepted": True,
                "command_id": f'mock-{payload["request_id"]}', "code": "ACCEPTED", "message": "Mock 승인"}
