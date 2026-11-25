# Vertiport Gazebo — X500 비행환경 · 버티포트 · 하향카메라 · 외부 제어 · 실시간 상태 출력

진로탐색수업 팀 이세지 — **[항공드론] 스마트 버티포트 안전관리 시스템 시뮬레이션**의 Gazebo/PX4 파트와 대시보드 연결용 HTTP Bridge.

```
Gazebo/PX4 (ROS 2) ──▶ 통합 프로그램 (패드·FOD 탐지, 픽셀→월드) ──▶ HTTP Bridge :8080 ──▶ 3D 대시보드 (Three.js)

[입력]  POST /api/command   set_target / hover / request_landing / cancel_mission → request_id·accepted·command_id·code·message
[출력]  GET  /api/telemetry  x,y,z (GAZEBO_WORLD_ENU, m) · altitude_m · roll/pitch/yaw (deg) · speed · battery · flight_mode ...
        GET  /api/health     api_version · status · simulation_ready
        GET  /camera         하향 카메라 MJPEG (camera_stream_url)
        GET  /api/frames · /api/camera · /api/scene · /api/perception
```
HTTP API 는 대시보드 코드(`gazebo_bridge_client.py`, `telemetry_provider.py`)의 필드 규격(**API v1**)을 그대로 따른다. 데이터 형식·좌표계·TF 의 기준은 **[docs/INTERFACE.md](docs/INTERFACE.md)** (v1.3) 이다. 대시보드와 합의한 내용은 §0-A 에 있다.

---

## 1. 버전

### 1.1 확정 기록

| 구성요소 | 고정 버전 | 비고 |
|---|---|---|
| Ubuntu | **22.04 LTS** ✅ 확정 (2026-11-25) | Gazebo 담당 PC |
| ROS 2 | **Humble** ✅ | 22.04 에 맞는 조합 |
| Gazebo | **Harmonic** (gz-sim 8.x) | PX4 v1.16 부터 공식 지원 |
| PX4-Autopilot | **v1.17.0** | 2026-04 안정 릴리스 |
| px4_msgs | **release/1.17** | PX4 v1.17.0 의 메시지 235개와 정의가 같은 것을 확인함 |
| Micro-XRCE-DDS-Agent | **v2.4.3** | PX4 클라이언트는 v2.x 전용 (v3.x 쓰면 안 됨) |
| ros_gz | `ros-humble-ros-gzharmonic` | Humble 기본 `ros-humble-ros-gz` 는 Fortress 용이라 쓰면 안 됨 |
| 기체 모델 | `gz_x500_mono_cam_down` (airframe 4014) | PX4 기본 제공 하향 카메라 X500 |
| Python 패키지 | python3-yaml, python3-pil | HTTP Bridge 는 표준 라이브러리 `http.server` 사용 |

| 확정 기록 | 값 |
|---|---|
| 확정 조합 | **Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic + PX4 v1.17.0 + px4_msgs release/1.17 + XRCE Agent v2.4.3** |
| 확정일 | 2026-11-25 (팀 합의, Gazebo PC = Ubuntu 22.04) |
| Gazebo 담당 PC 사양 | ____ (`check_env.sh` 출력의 CPU / GPU / RAM 기록) |
| `verify_versions.sh` 전부 ✅ 날짜 | ____ |

> **22.04 + Humble 을 기본으로 한 이유**: PX4 ROS 2 문서의 공식 권장 조합이고, 실기체에 쓸 Jetson(JetPack 6)도 Ubuntu 22.04 라서 시뮬레이션과 실기체가 같은 ROS 배포판을 쓰게 된다.
> PX4 v1.18.0 도 나와 있지만 자료가 많은 v1.17 로 고정했다. 올릴 때는 `PX4_TAG` 와 `PX4_MSGS_BRANCH` 를 **함께** 바꾼다.

모든 버전은 **`versions.env` 한 곳**에서 관리하고, 스크립트는 이 파일을 읽는다.

### 1.2 참고: Ubuntu 버전별 조합 (나중에 PC 가 바뀔 때)

| Gazebo PC | ROS 2 | ros_gz 패키지 | `versions.env` 에 쓸 값 |
|---|---|---|---|
| Ubuntu 22.04 | Humble | `ros-humble-ros-gzharmonic` | `UBUNTU_VERSION=22.04`, `ROS_DISTRO=humble` (기본값) |
| Ubuntu 24.04 | Jazzy | `ros-jazzy-ros-gz` | `UBUNTU_VERSION=24.04`, `ROS_DISTRO=jazzy` |
| 20.04 이하 / WSL | — | — | 지원 안 함 → 22.04 네이티브 설치 권장 |

PX4 v1.17.0, px4_msgs release/1.17, Gazebo Harmonic, XRCE Agent v2.4.3 은 두 조합에서 같다.

### 1.3 설치 후 확인
1. `bash scripts/check_env.sh` → 모두 [OK] 인지 확인하고 PC 사양을 위 표에 적는다.
2. `bash scripts/install.sh` → 마지막에 나오는 `verify_versions.sh` 결과가 모두 ✅ 인지 확인하고 날짜를 적는다.

---

## 2. 설치

### 2.1 스크립트 설치 (권장)
```bash
git clone <this repo> ~/vertiport_gazebo && cd ~/vertiport_gazebo
bash scripts/check_env.sh          # 환경 점검 (아무것도 바꾸지 않음)
bash scripts/install.sh            # 30~60분, 디스크 약 15 GB
bash scripts/verify_versions.sh    # 버전 · 메시지 정의 일치 확인
```
코드를 고친 뒤 워크스페이스만 다시 빌드하려면 `bash scripts/install.sh --only-ws` 를 쓴다.

### 2.2 수동 설치 (스크립트가 하는 일, Ubuntu 22.04 기준)
```bash
# ① ROS 2 Humble
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools python3-yaml python3-pil tmux

# ② PX4 v1.17.0 + Gazebo Harmonic (ubuntu.sh 가 Gazebo 까지 설치)
git clone --recursive --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools
cd ~/PX4-Autopilot && make px4_sitl

# ③ ROS ↔ Gazebo 브리지 (Harmonic 용)
sudo apt install -y ros-humble-ros-gzharmonic          # 24.04/Jazzy: ros-jazzy-ros-gz

# ④ Micro XRCE-DDS Agent v2.4.3
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig /usr/local/lib/

# ⑤ 워크스페이스
cd ~/vertiport_gazebo/ros2_ws/src && git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
cd ~/vertiport_gazebo/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# ⑥ 버티포트 world 를 PX4 worlds 폴더에 링크
ln -sfn ~/vertiport_gazebo/sim/worlds/vertiport.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/vertiport.sdf
```

---

## 3. 실행

```bash
bash scripts/run_sim.sh            # tmux: agent / px4 / ros / shell 창
bash scripts/run_sim.sh --demo     # + 자동 미션, 가짜 탐지(perception_stub), 하강 중 사람 등장 → HOLD → RESUME
bash scripts/run_sim.sh --headless # Gazebo GUI 없이
bash scripts/run_sim.sh --kill     # 종료
```
- tmux 창 이동은 `Ctrl+b` 다음 숫자 키. 세션에서 나올 때는 `Ctrl+b d`, 다시 들어갈 때는 `tmux attach -t vp`.
- **대시보드 접속**: `http://<Gazebo PC IP>:8080` (같은 네트워크면 다른 PC 에서도 접속 가능)
- **점검 뷰어**: 브라우저에서 `http://localhost:8080/` 을 연다. 지도, 카메라, 명령 버튼이 있다.

터미널 3개로 직접 실행하려면:
```bash
# [1] DDS Agent
MicroXRCEAgent udp4 -p 8888
# [2] PX4 + Gazebo
export GZ_SIM_RESOURCE_PATH=~/vertiport_gazebo/sim/models:$GZ_SIM_RESOURCE_PATH
cd ~/PX4-Autopilot && PX4_GZ_WORLD=vertiport PX4_GZ_MODEL_POSE="-6,0,0,0,0,0" \
  PX4_PARAM_COM_RCL_EXCEPT=4 make px4_sitl gz_x500_mono_cam_down
# PX4 가 뜨면 pxh> 에서 (QGC 없이 arm 하려면 필수, 환경변수로는 적용 안 됨)
param set NAV_DLL_ACT 0
# [3] ROS 2 노드 (commander, telemetry_bridge, http_bridge, scenario_manager, gz 카메라 브리지)
source /opt/ros/humble/setup.bash && source ~/vertiport_gazebo/ros2_ws/install/setup.bash
ros2 launch vertiport_sim sim.launch.py spawn_x:=-6.0 spawn_y:=0.0
```
launch 옵션: `http_port:=8080`, `mock_judge:=true`, `auto_mission:=true`, `perception_stub:=true`, `sim:=false`(실기체).

## 4. 사용 예

```bash
# HTTP (대시보드와 같은 방식)
curl http://localhost:8080/api/health
curl http://localhost:8080/api/telemetry
curl -X POST http://localhost:8080/api/command -H 'Content-Type: application/json' \
     -d '{"request_id":"t1","command":"set_target","parameters":{"frame":"GAZEBO_WORLD_ENU","x":5,"y":3,"z":4}}'
curl -X POST http://localhost:8080/api/command -d '{"request_id":"t2","command":"hover","parameters":{}}'
TID=$(curl -s http://localhost:8080/api/telemetry | python3 -c 'import sys,json;print(json.load(sys.stdin)["telemetry_id"])')
curl -X POST http://localhost:8080/api/command -d "{\"request_id\":\"t3\",\"command\":\"request_landing\",\"parameters\":{\"telemetry_id\":\"$TID\"}}"
curl -X POST http://localhost:8080/api/command -d '{"request_id":"t4","command":"cancel_mission","parameters":{}}'
curl http://localhost:8080/api/frames
python3 examples/http_client_example.py --mission     # set_target 패드 상공 → request_landing
# 카메라: 브라우저에서 http://localhost:8080/camera

# ROS 2 CLI
ros2 run vertiport_sim vp_cmd goto 5 3 4               # 지상이면 수직 이륙 후 이동
ros2 run vertiport_sim vp_cmd hold
ros2 run vertiport_sim vp_cmd resume
ros2 run vertiport_sim vp_cmd cancel
ros2 run vertiport_sim vp_cmd land                     # 패드 중심, 또는: land 0.3 -0.2
ros2 run vertiport_sim vp_scenario apply S3_fod
ros2 run vertiport_sim vp_scenario sequence demo_person_hold
ros2 run rqt_image_view rqt_image_view /camera/down/image_raw
ros2 run tf2_ros tf2_echo map camera_optical_frame
```

## 5. 처음 실행할 때 확인할 것

| 항목 | 기대값 |
|---|---|
| `GET /api/health` | `status: "ok"`, `simulation_ready: true`, camera fps 약 30 |
| 지상 정지 시 telemetry | `x,y,z ≈ (-6.00, 0.00, 0.24)`, `altitude_m ≈ 0.00` |
| `yaw` | 약 0° (스폰 yaw 0 = 기수 동쪽). Gazebo 화면에서 기수가 +X 쪽인지 |
| `battery`, `latitude`, `satellites` | null 이 아닌 값 (PX4 SITL 모의값) |
| set_target (0, 0, 4) 후 | Gazebo 에서 패드 바로 위, `/camera` 영상 중앙에 패드가 보임 |
| 영상 위쪽 | 기체 기수 방향 |
| `tf2_echo map camera_optical_frame` | 수평 hover 시 z 축이 아래(−Z)를 향함 |
| `--demo` | `/api/perception` 의 `landing_pad.gt_error_m` < 0.1 m |

## 6. 안전 시나리오 테스트

| 상황 | 재현 | 기대 동작 |
|---|---|---|
| 패드 안전 → 접근·착륙 | `apply S0_clear` → set_target 0 0 4 → request_landing | ALIGN → DESCEND → TOUCHDOWN → IDLE |
| 사람 탐지 → Hover | 하강 중 `apply S1_person` | 판단 HOLD → hover → 고도 유지 |
| 차량 탐지 → 접근 제한 | 접근 중 `apply S2_vehicle` | 판단 RESTRICTED → hover |
| FOD 탐지 → 착륙 금지 | `apply S3_fod` 후 request_landing | 판단 NO_LAND → 패드 상공 대기 |
| 위험요소 해제 → 착륙 재개 | `apply S0_clear` | 판단 LAND_OK → request_landing (또는 mock_judge 의 RESUME) → 이어서 하강 |
| 운영자 중단 | 아무 때나 cancel_mission | 현재 위치 HOVER |

사람·차량은 AI 탐지 범위가 아니어서 Gazebo 시나리오 객체(ground truth)로만 존재하고, 대시보드에는 `/api/scene` 의 `scenario_objects` 로 표시된다. `mock_judge` 는 ground truth 를 기준으로 판단하는 테스트용 노드다. 실제 판단 프로그램이 `/vertiport/decision`·`/vertiport/command` 를 쓰게 되면 끈다.

## 7. 구조

```
vertiport_gazebo/
├── versions.env                       ← 팀 고정 버전 (단일 기준)
├── docs/INTERFACE.md                  ← 통합 데이터 규격 v1.1
├── scripts/  check_env.sh · install.sh · verify_versions.sh · run_sim.sh
├── sim/worlds/vertiport.sdf           ← 원점 = 패드 중심, ENU, UserCommands 포함
├── sim/models/  vp_pad(텍스처 교체 가능) · vp_stand · vp_person · vp_vehicle · vp_fod
├── ros2_ws/src/vertiport_sim/
│   ├── vertiport_sim/
│   │   ├── commander_core.py / commander_node.py   GOTO·HOLD·LAND·RESUME·CANCEL → PX4 Offboard
│   │   ├── telemetry_bridge.py                     PX4 → /vertiport/telemetry + TF
│   │   ├── http_bridge.py                          대시보드 HTTP API v1 + /camera MJPEG + 뷰어
│   │   ├── dashboard_api.py                        API v1 필드 변환·명령 매핑·좌표계 (ROS 없이 테스트)
│   │   ├── scenario_core.py / scenario_node.py     사람·차량·FOD 배치, ground truth
│   │   ├── camera_geometry.py                      픽셀 ↔ 지면 좌표 (참고 구현)
│   │   ├── perception_stub.py                      GT 투영 가짜 탐지 (통합 프로그램 자리 표시)
│   │   ├── mock_judge_node.py / judge_core.py      GT 기반 모의 판단
│   │   ├── frames.py                               ENU↔NED, 쿼터니언, Three.js 변환, 카메라 TF 상수
│   │   └── cli.py                                  vp_cmd / vp_scenario
│   ├── www/viewer.html                             점검 뷰어 (GET /)
│   ├── launch/sim.launch.py · config/vertiport.yaml · config/scenarios.yaml
│   └── test/                                       단위 + 가짜 ROS/PX4 통합 테스트
└── examples/http_client_example.py
```

## 8. 역할 분담

| 파트 | 범위 | 인터페이스 |
|---|---|---|
| **Gazebo/PX4** | PX4 SITL + X500, 버티포트 world, 하향 카메라 토픽, 위치·자세·속도·상태 출력, TF, Commander, 사람/차량/FOD 시나리오, Ground Truth | 출력: telemetry, TF, camera, ground_truth / 입력: `/vertiport/command` |
| **AI** | 하향 영상 → **landing_pad, FOD** 탐지 (class · conf · bbox) | → 통합 |
| **통합 (란)** | 영상 + 기체 자세 수신, 패드/FOD 모델 연결, 픽셀→월드 변환, GT 대비 오차 검증, HTTP Bridge 로 전달, 최종 통합 | `/vertiport/perception` 발행 (INTERFACE §6) |
| **대시보드** | Three.js 3D 트윈 (드론·패드·FOD·시나리오 객체), 판단 결과, 카메라 영상 | HTTP Bridge API v1 (`/api/*`, `/camera`) |

## 9. 테스트 (ROS 없는 PC 에서도 실행 가능)

```bash
cd ros2_ws/src/vertiport_sim
python3 -m pytest test/test_frames.py test/test_commander_core.py test/test_scenario_judge.py test/test_dashboard_api.py
python3 test/e2e_fake_ros.py     # 실제 노드 코드 + 가짜 ROS/PX4: 자동 미션 + 사람 HOLD/RESUME (가상 시간)
python3 test/http_fake_ros.py    # 대시보드 API v1 전체 (필드·자료형·순서, 명령 4종·code) + MJPEG + CORS (실시간 약 30초)
```
**실제 PX4 SITL·Gazebo 에서는 아직 돌려보지 않았다.** 처음 실행할 때 §5 체크리스트와 §10 트러블슈팅을 같이 본다.

## 10. 트러블슈팅

| 증상 | 확인 |
|---|---|
| `/api/health` 의 px4 가 false | Agent 실행 여부, `ros2 topic list \| grep fmu`, pxh> `uxrce_dds_client status` |
| `vehicle_status_v1` 관련 경고 | PX4 v1.16+ 메시지 버전 접미사. 노드가 맞는 토픽을 자동으로 찾아 재구독한다. 경고가 계속 나면 PX4 와 px4_msgs 버전을 맞춘다 |
| `Preflight Fail: No connection to the GCS`, 명령은 ACCEPTED 인데 안 뜨고 10초 뒤 IDLE | pxh> 에서 `param set NAV_DLL_ACT 0` (run_sim.sh 는 자동 설정). `PX4_PARAM_NAV_DLL_ACT=0` 환경변수는 기본값과 같아서 airframe 기본값 2 에 덮어씌워짐. QGroundControl 을 켜두는 것도 방법 |
| 그 밖의 `arm/offboard timeout` | pxh> 에서 `commander check` 로 거부 이유 확인 |
| 땅에 있을 때 yaw 가 0° 가 아님 (10° 이상) | 지자기 편각 불일치. world 의 `magnetic_field` 가 GPS 기준점과 맞는지 확인 (vertiport.sdf 는 대구 기준으로 맞춤) |
| camera 가 false / 스트림이 안 나옴 | `gz topic -l \| grep image` 로 실제 이름 확인 → `camera_gz_prefix:=...` 로 지정. Pillow 설치 확인 |
| 카메라가 느리거나 검은 화면 | GPU 드라이버 확인 (ogre2). VM·WSL 은 느림 |
| 대시보드(Windows)에서 연결 안 됨 | Gazebo PC(Ubuntu)의 LAN IP 로 접속하고 있는지, 방화벽 `sudo ufw allow 8080/tcp`, 두 PC 가 같은 네트워크인지 확인. Windows 에서 `curl http://<IP>:8080/api/health` 로 먼저 확인 |
| 대시보드 카메라 안 나옴 | `camera_stream_url` 을 `<img src>` 로 넣었는지 확인 (`<video>`, streamlit-webrtc 로는 재생 안 됨). 브라우저 주소창에 URL 을 직접 열어 확인 |
| 시나리오 객체가 안 생김 | `gz service -l \| grep /world/vertiport/create` 확인 (UserCommands 필요) |
| 위치가 패드 기준으로 어긋남 | `PX4_GZ_MODEL_POSE` 와 launch 의 `spawn_x/spawn_y` 가 같은지 확인 (run_sim.sh 는 `SPAWN_X/SPAWN_Y` 하나로 맞춤) |
| 패드 인식률이 낮음 | `sim/models/vp_pad/materials/textures/pad.png` 를 실제 패드를 위에서 찍은 정사각 사진으로 교체 |
