# 스마트 버티포트 시뮬레이션 — 통합 인터페이스 규격 v1.3

> 팀 이세지 · 진로탐색수업 "[항공드론] 스마트 버티포트 안전관리 시스템 시뮬레이션 개발"
> **Gazebo → 통합 프로그램 → HTTP Bridge → 3D 대시보드** 연결 규격.
> HTTP API 는 대시보드 코드(`gazebo_bridge_client.py`, `telemetry_provider.py`)가 파싱하는 필드 이름·자료형을 그대로 따른다 (**API v1**).

## 0-A. 대시보드 담당과의 합의 사항 (2026-11-25 확정)

| 항목 | 합의 |
|---|---|
| 좌표계 | Gazebo·Bridge·대시보드 모두 **ENU** (X = East, Y = North, Z = Up). VERTIPORT_LOCAL_XYZ_UP = GAZEBO_WORLD_ENU, **XY 회전 0°, 원점 오프셋 (0, 0, 0)** |
| Three.js 표시 | `three.x = x`, `three.y = z`, `three.z = −y` (대시보드 기존 변환과 같음) |
| Z 기준점 | 텔레메트리 `z` = 기체 중심, 대시보드 모델 원점 = 스키드 바닥 → **대시보드가 0.24 m 오프셋을 적용**: 표시 `z_dash = z − 0.24`, `set_target.z = z_dash + 0.24`. 값은 `GET /api/frames` 의 `vehicle_origin.z_offset_m` 에서 읽는다 (`ext.z_bottom` 에도 이미 뺀 값이 있음) |
| altitude_m | 패드 면 기준 기체 바닥 높이 (그대로 표시) |
| 자세 | `roll/pitch/yaw` deg, `attitude_convention: "ENU_FLU_RZYX_DEG"`. rad 값은 보내지 않음 |
| 비행 상태 | `flight_mode` 필드 (10개 값, enum 제한 없음) |
| timestamp | 실제 UTC (`…Z`). 대시보드가 KST 로 바꿔서 표시 |
| gps | 문자열: `NO_GPS`, `NO_FIX`, `2D_FIX`, `3D_FIX`, `DGPS`, `RTK_FLOAT`, `RTK_FIXED` (수신 전이면 null) |
| mavlink | 문자열: `CONNECTED` / `DISCONNECTED` = 비행제어기 링크 상태 (실제 링크는 uXRCE-DDS) |
| 명령 | set_target / hover / request_landing / cancel_mission, 모두 `request_id` + `parameters` 객체. 응답은 HTTP 200 + `accepted`(bool) + `code` + `command_id` + `request_id` |
| request_landing | 가장 최근 `telemetry_id` 를 전달하고, **유효시간 3 초** (Bridge 도 3 초로 맞춤). 대시보드는 카메라 안전판단 연동 전까지 착륙 버튼을 막아 둠 |
| RESUME | 지금은 사용하지 않음. hover 뒤에는 set_target 이나 request_landing 을 새로 보냄 |
| 카메라 | 대시보드(Windows)에서 `<img src="{camera_stream_url}">` 로 연결 예정. Gazebo PC(Ubuntu) LAN IP 사용 |
| 확장 필드 | `ext`, `components`, `state` 등은 대시보드가 무시함 (보내도 됨) |

## 0. 대시보드 요구사항 반영 현황

| 요구 | 반영 | 위치 |
|---|---|---|
| `GET /api/health` → `api_version`, `status`, `simulation_ready` | ✅ | §3.1 |
| `GET /api/telemetry` 24개 필드, 이름·자료형·순서 동일 | ✅ 테스트로 필드·자료형·순서 검증 | §3.2 |
| x/y/z Gazebo 월드 좌표, altitude_m 패드 기준 | ✅ | §1, §3.2 |
| roll/pitch/yaw deg, `ENU_FLU_RZYX_DEG` | ✅ | §1.3 |
| timestamp ISO 8601 = 실제 샘플 생성 시각, telemetry_id 샘플별 고유 | ✅ PX4 위치 샘플 수신 시각, 새 샘플에만 새 ID | §3.2 |
| 못 주는 값은 null | ✅ battery·위치·GPS 등은 수신 전이면 null | §3.2 |
| 1 Hz 이상 | ✅ 20 Hz | §3.2 |
| `POST /api/command`: set_target / hover / request_landing / cancel_mission | ✅ | §3.3 |
| 응답 `request_id`, `accepted`, `command_id`, `code`, `message` | ✅ | §3.3 |
| `/camera/down/image_raw`, `camera_info`, TF, camera_stream_url, 브라우저 재생 | ✅ `GET /camera` (MJPEG) | §5 |
| Bridge 좌표계 GAZEBO_WORLD_ENU ↔ 대시보드 VERTIPORT_LOCAL_XYZ_UP 변환 규칙 | ✅ 패드 = Gazebo 원점, 회전 0 → 현재는 같은 좌표 | §1.2, `GET /api/frames` |
| Ubuntu 버전 확정 | ✅ 22.04 → ROS 2 Humble 조합 | README §1 |

---

## 1. 좌표계

### 1.1 Bridge 좌표계 `GAZEBO_WORLD_ENU`
X = East, Y = North, Z = Up, 단위 m. Gazebo world 좌표 그대로.
- PX4 v1.17 `GZBridge.cpp` 의 변환이 `NED = (y_gz, x_gz, −z_gz)` 이므로, Bridge 의 `ENU = (y_ned, x_ned, −z_ned) + 스폰 오프셋` 은 Gazebo world 좌표와 정확히 같다.

### 1.2 Landing Pad 원점과 `VERTIPORT_LOCAL_XYZ_UP` 변환

| 항목 | 값 |
|---|---|
| Landing Pad 중심의 Gazebo 월드 좌표 | **(0, 0, 0)** — `vertiport.sdf` 에서 패드를 world 원점에 둠 |
| Gazebo X축 대비 패드 축 회전 | **0°** |
| 이륙 대기 위치 (stand) | (−6, 0, 0) |

변환식 (`origin` = 패드 중심, `yaw` = Gazebo X축에서 local X축까지 반시계 각도)
```
local = Rz(−yaw) · (world − origin)
world = origin + Rz(yaw) · local
```
현재 `origin = (0,0,0)`, `yaw = 0` 이다. 따라서 **대시보드 local 축이 X = 동, Y = 북, Z = 위라면 두 좌표계는 같은 값**이다.
대시보드 local 축이 다르면(예: X 가 활주 방향) `vertiport.yaml` 의 `local_frame_yaw_deg` 만 바꾸면 된다. 그러면 `set_target` 에 `frame: "VERTIPORT_LOCAL_XYZ_UP"` 로 보낸 좌표도 자동으로 변환된다. 현재 값은 `GET /api/frames` 로 확인한다.

### 1.3 자세 `ENU_FLU_RZYX_DEG`

| 항목 | 정의 |
|---|---|
| 기체 좌표계 | FLU: x = 기수, y = 왼쪽, z = 위 |
| 회전 | **R = Rz(yaw) · Ry(pitch) · Rx(roll)**, 기체 FLU → World ENU, 단위 deg |
| yaw | 0 = 기수가 동쪽(+X), **반시계 +**, 90 = 북쪽 |
| pitch | +Y(왼쪽) 축 오른손 회전 → **기수 내림 +** |
| roll | +X(기수) 축 오른손 회전 → **오른쪽 날개 내림 +** |
| 쿼터니언 | `ext.orientation_q {x,y,z,w}` (같은 회전, 짐벌락 없음) |

Three.js(Y-up) 로 옮길 때: `position.set(x, z, −y)`, `quaternion.set(q.x, q.z, −q.y, q.w)`. 드론 모델은 기수 = 모델 +X 로 둔다.

### 1.4 기체 기준점

| 필드 | 의미 | 지상 정지 시 |
|---|---|---|
| `x, y, z` | 기체 중심(base_link)의 Gazebo 월드 좌표 = Gazebo 모델 pose | z ≈ **0.24** |
| `altitude_m` | Landing Pad 면 기준 기체 바닥(착륙장치) 높이 = `z − pad_z − 0.24` | ≈ **0.00** |
| `set_target` 의 z | `x, y, z` 와 같은 기준 (기체 중심 높이) | — |

---

## 2. 전체 구조

```
Gazebo/PX4 (ROS 2)                              통합 프로그램                 HTTP Bridge :8080              3D 대시보드
 /vertiport/telemetry (20 Hz) ──────────────┬──▶ landing_pad·FOD 탐지 ───┐   GET  /api/health          ◀── gazebo_bridge_client.py
 /tf map→base_link→camera_optical_frame ────┤   픽셀 → 월드 변환         ├──▶ GET  /api/telemetry          telemetry_provider.py
 /camera/down/image_raw · camera_info ──────┘   /vertiport/perception ───┘   POST /api/command
 /vertiport/ground_truth (시나리오 정답) ──────────────────────────────────▶ GET  /api/scene, /api/perception
 Commander ◀── /vertiport/command ◀───────────────────────────────────────── GET  /camera (MJPEG)
```
HTTP Bridge(`http_bridge` 노드)는 ROS 토픽을 모아 대시보드 형식으로 바꾸기만 한다. 좌표·단위 변환이 여기서 한 번만 일어나므로 대시보드는 ROS 를 몰라도 된다.

---

## 3. HTTP API v1

Base URL `http://<Gazebo PC IP>:8080`. JSON UTF-8, CORS 허용(`*`, OPTIONS preflight 지원).

### 3.1 `GET /api/health`
```json
{
  "api_version": "1",
  "status": "ok",
  "simulation_ready": true,
  "timestamp": "2026-11-24T17:34:56.123Z",
  "components": { "telemetry": {"ok": true, "age_s": 0.03}, "px4": {...}, "commander": {...},
                  "gazebo_world": {...}, "camera": {...}, "perception": {...}, "decision": {...} }
}
```
| 필드 | 값 |
|---|---|
| `status` | `"ok"` = 준비 완료 + 카메라 정상 · `"degraded"` = 텔레메트리는 오지만 일부 비정상 · `"down"` = 텔레메트리 없음 |
| `simulation_ready` | 텔레메트리 1초 이내 + PX4 연결·위치 유효 + Commander 동작 + Gazebo world 준비 → 명령을 받을 수 있는 상태 |
| `components` | 확장 필드 (진단용, 무시해도 됨) |

### 3.2 `GET /api/telemetry`
```json
{
  "telemetry_id": "sample-543526003",
  "timestamp": "2026-11-24T18:05:27.008Z",
  "frame": "GAZEBO_WORLD_ENU",
  "id": "X500-01",
  "x": -6.0, "y": 0.0, "z": 0.24,
  "altitude_m": 0.0,
  "roll": 0.0, "pitch": 0.0, "yaw": 90.0,
  "attitude_convention": "ENU_FLU_RZYX_DEG",
  "speed": 0.0,
  "battery": 100.0,
  "voltage": 16.8,
  "flight_mode": "IDLE",
  "armed": false,
  "emergency": false,
  "latitude": 35.8889, "longitude": 128.6103,
  "gps": "3D_FIX",
  "satellites": 12,
  "mavlink": "CONNECTED",
  "camera_stream_url": "http://192.168.0.10:8080/camera",
  "ext": { "orientation_q": {...}, "velocity_mps": {...}, "ground_speed_mps": 0.0, "target": null,
           "px4_mode": "POSCTL", "landed": true, "commander": {...}, "last_command": {...},
           "t_sim": 52.84, "gps_fix_type": 3, "altitude_amsl_m": 50.0, "battery_warning": 0, ... }
}
```
(테스트에서 실제로 나온 응답. 필드 순서도 대시보드 예시와 같음)

| 필드 | 형 | 출처 / 정의 |
|---|---|---|
| `telemetry_id` | str | `sample-<n>`. **PX4 위치 샘플이 새로 들어올 때마다** 새 ID (같은 샘플에 ID 를 다시 쓰지 않음) |
| `timestamp` | str | ISO 8601 **UTC**(`Z`), ms 단위. 그 위치 샘플을 받은 실제 시각 (KST = UTC + 9시간) |
| `frame` | str | `"GAZEBO_WORLD_ENU"` |
| `id` | str | `"X500-01"` (`vehicle_id` 파라미터) |
| `x, y, z` | float m | §1.4 |
| `altitude_m` | float m | §1.4 |
| `roll, pitch, yaw` | float deg | §1.3 |
| `speed` | float m/s | 3D 속도 크기 √(vx²+vy²+vz²). 수평 속도는 `ext.ground_speed_mps` |
| `battery` | float % / null | PX4 `battery_status.remaining × 100` (SITL 은 모의 배터리) |
| `voltage` | float V / null | PX4 `battery_status.voltage_v` |
| `flight_mode` | str | Commander 상태: `DISCONNECTED`, `IDLE`, `TAKEOFF`, `GOTO`, `HOVER`, `HOLD`, `LANDING`, `TOUCHDOWN`, `LANDED`, `OVERRIDE` (§4.3). Commander 가 없으면 PX4 모드 이름 |
| `armed` | bool | PX4 arming 상태 |
| `emergency` | bool | PX4 failsafe 활성 **또는** 배터리 경고 CRITICAL 이상 |
| `latitude, longitude` | float deg / null | PX4 `vehicle_global_position` (SITL 기준점: 경북대 부근) |
| `gps` | str / null | PX4 `vehicle_gps_position.fix_type` → `NO_GPS`/`NO_FIX`/`2D_FIX`/`3D_FIX`/`DGPS`/`RTK_FLOAT`/`RTK_FIXED`. GPS 메시지 수신 전이면 null |
| `satellites` | int / null | PX4 `vehicle_gps_position.satellites_used` |
| `mavlink` | str | 비행제어기 링크 상태 `CONNECTED` / `DISCONNECTED` (텔레메트리가 1 초 안에 들어오고 PX4 가 연결돼 있으면 CONNECTED). 실제 링크는 MAVLink 대신 uXRCE-DDS |
| `camera_stream_url` | str | 요청한 Host 기준 `http://<host>/camera`. 고정하려면 `public_base_url` 파라미터 |
| `ext` | object | 확장 필드 (대시보드가 무시해도 됨) |

- 갱신: 20 Hz (최소 요구 1 Hz).
- 텔레메트리를 한 번도 받지 못했을 때: 모든 값이 `null` 이고 `frame`, `id`, `attitude_convention`, `camera_stream_url` 만 채워진다. `flight_mode` 와 `mavlink` 는 `"DISCONNECTED"` 이다.
- 대시보드 모델(바닥 원점)에 그릴 때는 `z − 0.24` 를 쓰거나 `ext.z_bottom` 을 그대로 쓴다 (§0-A).

### 3.3 `POST /api/command`

**요청** — `parameters` 객체 형식이 기본이다 (`parameters` 없이 최상위에 써도 받는다)
```json
{"request_id": "dash-0012", "command": "set_target",
 "parameters": {"frame": "GAZEBO_WORLD_ENU", "x": 5.0, "y": 3.0, "z": 4.0}}
```
| command | parameters | 동작 (Commander 명령) |
|---|---|---|
| `set_target` | `frame`, `x`, `y`, `z` (선택 `yaw` deg) | 목표 좌표로 이동 → 도착하면 `HOVER`. **지상이면 수직 이륙 후 이동** (GOTO). `frame` = `GAZEBO_WORLD_ENU` 또는 `VERTIPORT_LOCAL_XYZ_UP` (생략 시 GAZEBO_WORLD_ENU) |
| `hover` | `{}` | 현재 위치 정지 → `HOLD` (진행 중 동작 기억) |
| `request_landing` | `telemetry_id` | 착륙 요청. telemetry_id 는 Bridge 가 보낸 것이어야 하고 3 초 이내여야 함 → 패드 중심 상공 정렬 → 하강 → 접지 (LAND) |
| `cancel_mission` | `{}` | 진행 중 동작 취소 → 현재 위치 `HOVER`. 이륙 전이면 이륙 취소·disarm (CANCEL) |

**응답** (HTTP 200)
```json
{"request_id": "dash-0012", "accepted": true, "command_id": "cmd-4352605", "code": "ACCEPTED",
 "message": "set_target GAZEBO_WORLD_ENU (5.00, 3.00, 4.00) → world (5.00, 3.00, 4.00): goto (5.0, 3.0, 4.0)",
 "state": "GOTO"}
```
| code | accepted | 의미 |
|---|---|---|
| `ACCEPTED` | true | 접수됨 |
| `INVALID_REQUEST` | false | JSON 객체가 아니거나 `command` 가 없음 (본문이 JSON 이 아니면 HTTP 400) |
| `UNKNOWN_COMMAND` | false | 위 4개 명령이 아님 |
| `INVALID_PARAMETERS` | false | 필수 파라미터가 없거나 숫자가 아님 |
| `UNSUPPORTED_FRAME` | false | frame 이 GAZEBO_WORLD_ENU / VERTIPORT_LOCAL_XYZ_UP 이 아님 |
| `OUT_OF_BOUNDS` | false | \|x\|,\|y\| > 60 m 또는 z 가 1–30 m 범위 밖 |
| `UNKNOWN_TELEMETRY` | false | request_landing 의 telemetry_id 를 Bridge 가 모름 |
| `STALE_TELEMETRY` | false | telemetry_id 가 3 초보다 오래됨 (`landing_request_max_age_s`) |
| `REJECTED_STATE` | false | 지금 상태에서 할 수 없음 (예: 지상에서 hover, 지상에서 request_landing) |
| `NOT_CONNECTED` | false | PX4 연결 없음 |
| `COMMANDER_UNAVAILABLE` | false | Commander 노드가 실행 중이 아님 |
| `TIMEOUT` | false | Commander 가 2 초 안에 응답하지 않음 |

- 응답은 본문이 JSON 이 아닐 때(400)를 빼고 **항상 HTTP 200** 이다. 결과는 `accepted` 와 `code` 로 판단한다.
- `command_id` (`cmd-<n>`) 는 Bridge 가 명령마다 새로 만든다. 텔레메트리의 `ext.last_command.command_id` 로 마지막 명령 처리 결과를 확인할 수 있다.
- `state` 는 확장 필드로, 명령 처리 직후의 Commander 상태다.
- 확장: 판단 프로그램과 테스트 도구는 Commander 명령(`GOTO`, `HOLD`, `LAND`, `RESUME`, `CANCEL`)을 직접 보낼 수도 있다. 이 경우 yaw 는 rad 이다.

### 3.4 `GET /camera` — 카메라 스트림
- **MJPEG** (`multipart/x-mixed-replace`). 브라우저에서 `<img src="http://<host>:8080/camera">` 로 바로 재생된다 (`<video>` 태그가 아님).
- 기본 10 fps, 가로 960 px 로 축소 (`stream_fps`, `stream_max_width`, `jpeg_quality` 로 조정).
- 프레임마다 `X-Image-Stamp` 헤더(영상 timestamp, s)가 붙는다.
- 그 밖에 `GET /api/camera/snapshot.jpg` (최신 1장), `GET /stream/perception.mjpg` (통합 프로그램이 탐지 박스를 그린 영상)가 있다.

### 3.5 `GET /api/camera` — 카메라 정보
```json
{
  "topic_image": "/camera/down/image_raw", "topic_info": "/camera/down/camera_info",
  "camera_stream_url": "http://<host>/camera",
  "image": {"stamp": 52.8333, "frame_id": "camera_link", "width": 1280, "height": 960, "encoding": "rgb8",
            "fps": 29.8, "age_s": 0.02},
  "intrinsics": {"width": 1280, "height": 960, "K": [539.94, 0, 640, 0, 539.94, 480, 0, 0, 1],
                 "D": [0, 0, 0, 0, 0], "distortion_model": "plumb_bob", "frame_id": "camera_link", "hfov_deg": 99.69},
  "tf": {"tree": "map → base_link → camera_link → camera_optical_frame",
         "base_to_camera_link": {"xyz_m": [0, 0, -0.14], "rpy_deg": [0, 90.0, 0], "q_xyzw": {...}},
         "camera_link_to_optical_q_xyzw": {"x": -0.5, "y": 0.5, "z": -0.5, "w": 0.5},
         "R_base_from_optical": [[0, -1, 0], [-1, 0, 0], [0, 0, -1]],
         "image_orientation": "image up = vehicle nose (forward), image right = vehicle right"}
}
```

### 3.6 `GET /api/frames` — 좌표계 정의
§1 의 내용(Bridge 좌표계, 패드 원점, local 변환식·yaw, 자세 규칙, 기체 기준점)을 JSON 으로 준다. 대시보드는 시작할 때 한 번 읽어서 `local_frame.identity` 가 true 인지 확인하면 된다.

### 3.7 `GET /api/scene`, `GET /api/perception` (3D 트윈 확장)
- `/api/scene`:
  - `drone` = `/api/telemetry` 와 같은 객체
  - `landing_pad {id, x, y, z, yaw, size_m, perceived}`
  - `fod[]` = AI 인식 결과
  - `scenario_objects[] {id, type, x, y, z, yaw(deg), source: "scenario"}` = Gazebo 에 배치한 사람·차량·FOD 의 정답 위치
  - `decision`, `camera`
- `/api/perception`: 통합 프로그램의 `/vertiport/perception` 을 그대로 준다 (§6). 아직 데이터가 없으면 `{"available": false, "landing_pad": null, "fod": []}`.
- 좌표는 모두 GAZEBO_WORLD_ENU, 각도는 deg 이다. perception 안의 좌표만 `position_m {x,y,z}` 형식이다.

### 3.8 (테스트용) `POST /api/scenario`, `GET /`
- `POST /api/scenario {"op": "apply", "name": "S3_fod"}` → 시나리오를 바꾼다 (§7).
- `GET /` → 점검용 뷰어. 지도, 카메라, 명령 버튼이 있다.

---

## 4. Commander

### 4.1 대시보드 명령 ↔ Commander
| 대시보드 | Commander | HOLD 와의 관계 |
|---|---|---|
| set_target | GOTO | HOLD 상태에서도 바로 새 목표로 이동 |
| hover | HOLD | 직전 동작 기억 |
| request_landing | LAND (패드 중심) | HOLD 상태에서 보내면 착륙 진행 |
| cancel_mission | CANCEL | 직전 동작 버림 → HOVER |

### 4.2 착륙 순서
ALIGN (현재 고도로 패드 상공 정렬, 오차 0.25 m 이내) → DESCEND (0.5 m/s 하강) → TOUCHDOWN (z ≤ 0.5 m 에서 PX4 Land 모드) → LANDED → IDLE (disarm)

### 4.3 상태 (`flight_mode`)
| 상태 | 의미 | 승인되는 대시보드 명령 |
|---|---|---|
| `DISCONNECTED` | PX4 데이터 없음 | — |
| `IDLE` | 지상 대기 | set_target |
| `TAKEOFF` | 수직 이륙 중 | set_target, hover, request_landing, cancel_mission |
| `GOTO` | 이동 중 | 〃 |
| `HOVER` | 도착 또는 취소 후 제자리 | 〃 |
| `HOLD` | hover 명령으로 정지 | set_target, request_landing, cancel_mission |
| `LANDING` | 착륙 (`ext.commander.phase` = ALIGN / DESCEND) | set_target, hover, request_landing(보정), cancel_mission |
| `TOUCHDOWN` | 최종 접지 | hover, cancel_mission |
| `LANDED` | 착지, disarm 대기 | — |
| `OVERRIDE` | 조종자나 PX4 failsafe 가 제어권을 가져감 | set_target, request_landing (명시적으로 제어권을 다시 가져올 때) |

---

## 5. 카메라 (ROS)

| 항목 | 값 |
|---|---|
| 영상 | `/camera/down/image_raw` (sensor_msgs/Image, rgb8, 1280×960, 30 Hz) |
| 내부 파라미터 | `/camera/down/camera_info` — K 이론값 fx = fy ≈ 539.9, cx = 640, cy = 480, 왜곡 없음 |
| 화각 | 수평 99.7°, 수직 83.3° |
| 영상 방향 | 영상 위쪽 = 기체 기수, 영상 오른쪽 = 기체 오른쪽 |
| TF | `map → base_link` (동적 20 Hz) → `camera_link` (고정 xyz 0,0,−0.14 / rpy 0,90°,0) → `camera_optical_frame` (고정, OpenCV 관례) |
| Timestamp | 영상 `header.stamp` = Gazebo 시뮬레이션 시간 = TF stamp = telemetry `ext.t_sim` |
| 브라우저 스트림 | `camera_stream_url` = `http://<host>:8080/camera` |
| 지면 범위 | 고도 4 m 에서 9.5 × 7.1 m (0.74 cm/px), 3 m 패드 ≈ 405 px, 0.3 m FOD ≈ 40 px |

- 픽셀 → 지면 좌표 참고 구현: `vertiport_sim/camera_geometry.py`. 영상 시각(`t_img`)과 가장 가까운 기체 자세를 써서 계산한다.
- 실기체: Jetson 카메라 노드가 같은 토픽명으로 발행하고, `camera_info` 에는 캘리브레이션한 K·D 를 넣는다.

## 6. perception (통합 → `/vertiport/perception`, `std_msgs/String` JSON)
AI 탐지 범위는 **landing_pad, fod** 만이다.
```json
{"type": "perception", "schema": "1.1", "t": 1764000000.30, "t_img": 52.8333, "source": "integration",
 "frame": "GAZEBO_WORLD_ENU",
 "image": {"width": 1280, "height": 960, "frame_id": "camera_link"},
 "drone_pose": {"position_m": {...}, "orientation_q": {...}, "t_px4": 52.83},
 "landing_pad": {"detected": true, "conf": 0.94, "bbox_xyxy": [455, 270, 849, 673], "center_px": {"u": 652.1, "v": 471.8},
                 "position_m": {"x": 0.03, "y": -0.02, "z": 0.0}, "gt_id": "pad_1", "gt_error_m": 0.04},
 "fod": [{"id": "fod_1", "class": "fod", "conf": 0.71, "bbox_xyxy": [684, 505, 721, 531], "center_px": {"u": 702.4, "v": 518.0},
          "position_m": {"x": 0.58, "y": -0.37, "z": 0.0}, "gt_id": "fod_1", "gt_error_m": 0.04}],
 "latency_ms": 23.5}
```
- 필수: `type`, `t`, `t_img`, `source`, `landing_pad` (없으면 `{"detected": false}`), `fod` (없으면 `[]`)
- `gt_id`, `gt_error_m` 은 시뮬레이션 검증용이다.
- 선택: bbox 를 그린 영상을 `/vertiport/perception/image` 로 발행하면 `/stream/perception.mjpg` 로 볼 수 있다.
- 통합 프로그램이 준비되기 전에는 `perception_stub` 노드(시나리오 정답을 투영한 가짜 탐지)가 같은 형식으로 발행한다.

## 7. 시나리오 객체 (Gazebo, AI 범위 밖)
사람·차량은 AI 가 탐지하지 않는다. Gazebo 에 배치한 정답 위치로만 존재하고, `/api/scene` 의 `scenario_objects` 로 전달된다.

| 시나리오 | 내용 | 기대 판단 |
|---|---|---|
| S0_clear | 위험요소 없음 | 착륙 가능 |
| S1_person | 패드 2.9 m 거리 사람 | 대기 |
| S2_vehicle | 패드 5.8 m 거리 차량 | 접근 제한 |
| S3_fod | 패드 위 FOD | 착륙 금지 |
| S4_fod_edge | 패드 가장자리 FOD | 착륙 금지 |
| S5_mixed | 사람 + 차량 + FOD | 착륙 금지 |
| S6_person_far | 11 m 밖 사람 | 착륙 가능 |

제어 (`/vertiport/scenario_cmd` 또는 `POST /api/scenario`): `apply`, `spawn`, `move`, `remove`, `clear`, `sequence`, `stop_sequence`, `list`

## 8. decision (판단 → `/vertiport/decision`)
```json
{"type": "decision", "t": 1764000000.2, "source": "judge_v1", "decision": "NO_LAND", "label_ko": "착륙 금지",
 "reasons": [{"decision": "NO_LAND", "type": "fod", "id": "fod_1", "dist_m": 0.72}]}
```
`LAND_OK` (착륙 가능) · `HOLD` (대기) · `RESTRICTED` (접근 제한) · `NO_LAND` (착륙 금지). 심각도는 NO_LAND > RESTRICTED > HOLD > LAND_OK 순이다. 테스트용 `mock_judge` 가 같은 형식으로 발행한다.

## 9. ROS 2 내부 토픽 (참고)

| 토픽 | 내용 |
|---|---|
| `/vertiport/telemetry` | 내부 텔레메트리 JSON (rad, 중첩 구조, `telemetry_id`, `t_sample`). HTTP 는 이것을 v1 형식으로 변환한다 |
| `/vertiport/command`, `/vertiport/command_ack` | Commander 명령 `{"command": "GOTO", ..., "command_id"}` / 결과 `{command_id, accepted, code, reason, state}` |
| `/vertiport/commander_state` | Commander 상태 (latched) |
| `/vertiport/ground_truth` | 시나리오 정답 좌표 (latched) |
| `/vertiport/perception`, `/vertiport/perception/image` | 통합 출력 |
| `/vertiport/decision` | 판단 출력 |
| `/tf`, `/tf_static` | §5 |

## 10. 시뮬레이션 ↔ 실기체
기체 상태·제어(`/fmu/*`), 카메라(`/camera/down/*`), HTTP API 는 모두 같다. 그래서 **대시보드 코드를 고칠 필요가 없다.**
실기체 실행은 `sim.launch.py sim:=false` 이다. 이때 텔레메트리의 `ext.source` 는 "real", TF stamp 는 ROS 시계, `scenario_objects` 는 빈 배열이 된다.

## 11. 변경 이력
| 버전 | 날짜 | 내용 |
|---|---|---|
| 1.0 | 2026-11-25 | 최초 작성 |
| 1.1 | 2026-11-25 | HTTP Bridge, command_id, TF, 쿼터니언, AI 범위 FOD+패드 |
| 1.2 | 2026-11-25 | 대시보드 코드 기준 HTTP API v1 (필드명·자료형·deg·ISO 시간·명령 4종·code), `/camera`, `/api/frames`, 배터리·GPS, Ubuntu 22.04 확정 |
| 1.3 | 2026-11-25 | 대시보드 담당과 합의: ENU 통일·회전 0°, Z 오프셋 0.24 m 는 대시보드가 적용, gps/mavlink 문자열, request_landing 유효 3 초 |
