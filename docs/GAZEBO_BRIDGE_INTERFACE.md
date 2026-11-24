# Gazebo Bridge HTTP 인터페이스 제안 · v1

Streamlit 대시보드가 HTTP로 팀원이 운영하는 Bridge에 연결합니다. Bridge가 MAVSDK/ROS2를 통해 PX4 SITL과 Gazebo X500을 연결합니다. 이 문서는 팀 협의용 규격이며 대시보드에는 Bridge 서버가 포함되지 않습니다.

## 주소와 설정

기본 주소는 `http://127.0.0.1:8765`입니다. 서버 주소가 달라지면 Streamlit을 시작하기 전에 환경변수로 설정합니다.

```powershell
$env:VERTIPORT_BRIDGE_URL = 'http://127.0.0.1:8765'
```

Bridge와 대시보드는 같은 개발 장비 또는 접근이 제한된 시뮬레이션 네트워크에서 운영합니다. 대시보드는 HTTP 리다이렉트를 따르지 않습니다. 기본 동작은 조회 전용입니다. 팀이 시뮬레이션 명령을 승인한 뒤에만 `VERTIPORT_BRIDGE_ENABLE_COMMANDS=1`을 설정합니다. 해당 설정은 목표 좌표, Hover, 취소에 적용됩니다. **HTTP 착륙 요청은 Bridge 하향 영상과 FOD 안전 판단의 연결을 검증할 때까지 차단됩니다.**

## 엔드포인트

| 메서드 | 경로 | 용도 |
| --- | --- | --- |
| GET | `/api/health` | API 버전과 시뮬레이션 준비 상태 |
| GET | `/api/telemetry` | 최신 단일 X500 샘플 |
| POST | `/api/command` | 명령 제출 및 접수 승인 여부 |

요청과 응답은 UTF-8 JSON 객체입니다. 대시보드는 요청별 0.65초 소켓 타임아웃과 64 KiB 응답 제한을 적용합니다. 활성 GAZEBO 화면에서 약 1초 간격으로 health와 telemetry를 순서대로 조회합니다. Bridge는 최소 약 1 Hz로 최신 샘플을 발행하고, 실제 샘플 생성 시각을 유지해야 합니다.

### Health 응답

```json
{"api_version": "1", "status": "ok", "simulation_ready": true}
```

`simulation_ready: true`는 시뮬레이션 대상과 Bridge가 명령을 받을 준비가 됐음을 뜻합니다. Bridge만 켜져 있거나 대상이 연결되지 않은 경우 `false`를 반환합니다.

### 텔레메트리 응답

```json
{
  "telemetry_id": "sample-1942",
  "timestamp": "2026-11-25T02:34:56.123Z",
  "frame": "GAZEBO_WORLD_ENU",
  "id": "X500-01",
  "x": 8.0, "y": 2.0, "z": 4.0,
  "altitude_m": 3.0,
  "roll": -1.2, "pitch": 1.7, "yaw": 90.0,
  "attitude_convention": "ENU_FLU_RZYX_DEG",
  "speed": 0.12, "battery": 72.5, "voltage": 15.4,
  "flight_mode": "HOLD", "armed": true, "emergency": false,
  "latitude": null, "longitude": null, "gps": null, "satellites": null, "mavlink": null,
  "camera_stream_url": "rtsp://127.0.0.1:8554/x500/downward"
}
```

- `timestamp`은 시간대가 포함된 ISO 8601 형식, `telemetry_id`는 샘플 식별자입니다. 3초보다 오래됐거나 1초보다 미래인 데이터는 제외합니다. 서버와 대시보드 시계를 동기화합니다.
- `x/y/z`는 월드 원점 기준 m, `altitude_m`은 착륙패드 지면 기준 고도 m입니다. `roll/pitch/yaw`는 도, `speed`는 m/s, `battery`는 0~100%, `voltage`는 V입니다. `yaw`는 ENU +X에서 +Y 방향으로 증가합니다. PX4 NED의 자세와 위치 변환은 Bridge에서 처리합니다.
- 제공하지 않는 값은 `null`로 보냅니다. 영상 연동은 이 단계에서 주소만 전달하며, 대시보드는 해당 주소를 자동으로 열거나 분석하지 않습니다. 화면에 표시할 주소에는 인증 정보가 포함되지 않아야 합니다.
- 3D 자세를 사용하는 경우 팀이 합의한 자세 변환으로 `attitude_convention: "ENU_FLU_RZYX_DEG"`를 표기합니다. ENU 월드, FLU 기체, 도 단위 `Rz(yaw) Ry(pitch) Rx(roll)`입니다. 생략하거나 다른 규격을 보내면 3D 뷰어는 자세를 미확인으로 표시합니다. [3D 좌표와 자세](3D_DIGITAL_TWIN.md#좌표와-자세)

### 명령 요청

```json
{
  "request_id": "1c79d9c0e18a4754968969d09e228f36",
  "command": "set_target",
  "parameters": {"frame": "GAZEBO_WORLD_ENU", "x": 8.0, "y": 2.0, "z": 4.0}
}
```

| 명령 | parameters |
| --- | --- |
| `set_target` | `frame`, `x`, `y`, `z` |
| `hover` | `{}` |
| `request_landing` | `{"telemetry_id": "sample-1942"}` |
| `cancel_mission` | `{}` |

명령은 버튼을 누를 때만 제출하며 자동으로 재전송하지 않습니다. 같은 `request_id`로 이미 처리한 요청이 도착하면 Bridge는 중복 실행 없이 동일 결과를 반환하는 방식으로 구현합니다. 팀은 승인 응답과 실행 결과를 구분합니다. 착륙을 연결할 때 Bridge는 텔레메트리 ID와 최신 시뮬레이션·착륙 사전조건을 독립적으로 확인하고 조건이 달라지면 거절해야 합니다.

### 명령 응답

```json
{
  "request_id": "1c79d9c0e18a4754968969d09e228f36",
  "accepted": true,
  "command_id": "cmd-312",
  "code": "ACCEPTED",
  "message": "Queued by simulation bridge"
}
```

거절할 때도 `request_id`를 그대로 넣고 `accepted: false`, `command_id: null`과 사유를 반환합니다. 대시보드는 승인 여부와 명령 ID를 표시합니다. 승인된 명령에는 비어 있지 않은 ID가 필요합니다.

| HTTP | 코드 예시 |
| --- | --- |
| 200/202 | `ACCEPTED` |
| 400 | `INVALID_REQUEST` |
| 409 | `INVALID_FLIGHT_STATE`, `LANDING_BLOCKED` |
| 422 | `UNKNOWN_FRAME`, `INVALID_COORDINATES` |
| 503 | `BRIDGE_NOT_READY` |

대시보드 자체 오류에는 `TIMEOUT`, `DISCONNECTED`, `INVALID_RESPONSE`, `STALE_TELEMETRY`, `FRAME_UNCONFIRMED` 등이 있습니다. 연결 실패나 텔레메트리 이상 시 이전 위치와 수치를 비우고 새 명령을 차단합니다. 타임아웃은 미실행을 보장하지 않으므로 새 명령을 제출하기 전에 Bridge 상태와 당시 요청 처리를 확인합니다.

## 좌표 합의 및 Mock

지도 프레임은 `VERTIPORT_LOCAL_XYZ_UP`이고 원점은 지상의 패드 중심입니다. 대시보드와 Bridge 모두 미터 및 Z-up 좌표를 사용합니다. **두 원점이나 축은 기본값으로 일치한다고 보지 않습니다.** 팀이 확정한 XY 회전과 XYZ 이동을 설정합니다.

```powershell
# 예시입니다. 팀이 검증한 파라미터로 교체합니다.
$env:VERTIPORT_BRIDGE_TRANSFORM = '{"source_frame":"VERTIPORT_LOCAL_XYZ_UP","target_frame":"GAZEBO_WORLD_ENU","yaw_deg":90,"offset_m":[10,-2,1]}'
```

방향은 대시보드에서 Bridge로, 회전은 +Z축 주위의 반시계 각도입니다. `p_bridge = Rz(yaw_deg) · p_local + offset_m`. 예를 들어 로컬 `(4, 2, 3)`은 위 설정에서 Bridge `(8, 2, 4)`가 됩니다. 지도 위치 표시는 역변환하며, 규칙이 없으면 Bridge 좌표만 표시하고 목표 전송 및 지도상의 기체 표시를 보류합니다. 다른 축 체계, 축 반전이나 배율이 필요하면 팀 협의 후 규격과 변환 구현을 수정합니다.

GAZEBO에서 **Mock Bridge**를 켜면 HTTP를 사용하지 않는 인메모리 응답을 사용합니다. **Mock 테스트 좌표 변환 사용**을 켜야 목표 전송이 가능하며, 위 예시 변환이 적용됩니다. Mock 착륙 요청에는 최신 WebRTC FOD 0개, **Mock 패드 확인**, 유효한 Mock 기체 상태와 샘플 ID가 필요합니다. REAL은 미연동 상태입니다.
