#!/usr/bin/env bash
# =============================================================================
#  시뮬레이션 실행 (tmux 세션 'vp' 에 창 3개)
#    [agent]  MicroXRCEAgent udp4 -p 8888
#    [px4]    PX4 SITL + Gazebo (vertiport world, X500 하향 카메라)
#    [ros]    ros2 launch vertiport_sim sim.launch.py
#
#    bash scripts/run_sim.sh               # 기본
#    bash scripts/run_sim.sh --demo        # 자동 미션 + 가짜 탐지(perception_stub) + demo_person_hold 시나리오
#    bash scripts/run_sim.sh --headless    # Gazebo GUI 없이 (카메라 렌더링은 동작)
#    bash scripts/run_sim.sh --kill        # 종료
#  창 이동: Ctrl+b 후 0/1/2,  세션 나가기: Ctrl+b d,  다시 붙기: tmux attach -t vp
# =============================================================================
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../versions.env
source "$REPO/versions.env"

# 스폰 위치 = 이륙 대기 위치 (World ENU). sim.launch.py 의 spawn_x/spawn_y 로 그대로 전달됨
SPAWN_X=${SPAWN_X:--6.0}
SPAWN_Y=${SPAWN_Y:-0.0}
SESSION=vp
DEMO=0; HEADLESS_FLAG=""
for a in "$@"; do
  case "$a" in
    --demo) DEMO=1;;
    --headless) HEADLESS_FLAG="HEADLESS=1";;
    --kill) tmux kill-session -t $SESSION 2>/dev/null; pkill -9 -f "gz sim" || true; pkill -9 -x px4 || true
            pkill -9 -f MicroXRCEAgent || true; pkill -f "ros2 launch vertiport_sim" || true; echo "stopped"; exit 0;;
    -h|--help) sed -n 2,14p "$0"; exit 0;;
    *) echo "unknown option $a"; exit 2;;
  esac
done

[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || { echo "PX4 SITL 빌드 없음 → bash scripts/install.sh"; exit 1; }
[ -f "$REPO/ros2_ws/install/setup.bash" ] || { echo "워크스페이스 빌드 없음 → bash scripts/install.sh --only-ws"; exit 1; }
ln -sfn "$REPO/sim/worlds/vertiport.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/vertiport.sdf"

# PX4 ↔ Gazebo 공통 환경
#  - GZ_SIM_RESOURCE_PATH: 우리 모델(vp_pad, vp_person ...) 경로. PX4 gz_env.sh 가 뒤에 자기 경로를 덧붙임
#  - PX4_GZ_MODEL_POSE  : 스폰 위치 (x,y,z,roll,pitch,yaw) - yaw 0 = 기체 정면이 동쪽(+x)
#  - QGC/RC 없이 SITL 에서 arm 하기 위한 파라미터 (실기체에는 적용하지 말 것!)
#       COM_RCL_EXCEPT=4  Offboard 모드에서 RC 끊김 failsafe 예외   ← PX4_PARAM_ 환경변수로 설정
#       NAV_DLL_ACT=0     GCS 연결 필요 조건 끔                      ← 환경변수로는 안 먹힘 (기본값 0 과 같아서
#                         airframe 의 set-default 2 에 덮어씌워짐) → PX4 기동 후 px4-param 으로 설정
ENV_PX4="export GZ_SIM_RESOURCE_PATH=$REPO/sim/models:\${GZ_SIM_RESOURCE_PATH:-}; \
export PX4_GZ_WORLD=$PX4_GZ_WORLD PX4_GZ_MODEL_POSE='$SPAWN_X,$SPAWN_Y,0,0,0,0' \
PX4_PARAM_COM_RCL_EXCEPT=4 $HEADLESS_FLAG"
ENV_ROS="set +u; source /opt/ros/$ROS_DISTRO/setup.bash; source $REPO/ros2_ws/install/setup.bash; \
export GZ_SIM_RESOURCE_PATH=$REPO/sim/models:\${GZ_SIM_RESOURCE_PATH:-}"

LAUNCH_ARGS="spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y world:=$PX4_GZ_WORLD model:=${PX4_SIM_MODEL#gz_}"
[ $DEMO -eq 1 ] && LAUNCH_ARGS="$LAUNCH_ARGS mock_judge:=true auto_mission:=true perception_stub:=true"

CMD_AGENT="MicroXRCEAgent udp4 -p 8888"
CMD_PX4="$ENV_PX4; cd $PX4_DIR && make px4_sitl $PX4_SIM_MODEL"
WAIT_GZ="echo 'Gazebo world 대기 중...'; until gz topic -l 2>/dev/null | grep -q '^/world/$PX4_GZ_WORLD/clock'; do sleep 1; done; sleep 3"
PX4_PARAM_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4-param"
# tmux → sh -c → bash -c 로 두 번 해석되므로 $ 를 쓰지 않는 형태로 작성 (변수·$(...) 금지)
SET_PARAMS="for t in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30; do $PX4_PARAM_BIN set NAV_DLL_ACT 0 >/dev/null 2>&1 && echo 'PX4: NAV_DLL_ACT=0 설정 (GCS 없이 arm 허용)' && break; sleep 1; done"
CMD_ROS="$ENV_ROS; $WAIT_GZ; $SET_PARAMS; ros2 launch vertiport_sim sim.launch.py $LAUNCH_ARGS"
CMD_DEMO="$ENV_ROS; echo '시나리오 매니저 대기 중...'; until ros2 topic list 2>/dev/null | grep -q '^/vertiport/ground_truth$'; do sleep 1; done; \
ros2 run vertiport_sim vp_scenario sequence demo_person_hold; exec bash"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux 가 없어서 수동 실행 안내만 출력합니다. 터미널 3개에서 각각:"
  echo; echo "[1] $CMD_AGENT"; echo; echo "[2] $CMD_PX4"; echo "    → PX4 가 뜨면 pxh> 에서: param set NAV_DLL_ACT 0"; echo; echo "[3] $CMD_ROS"; exit 0
fi

tmux kill-session -t $SESSION 2>/dev/null || true
if pgrep -f "gz sim" >/dev/null || pgrep -x px4 >/dev/null; then
  echo "이전에 실행된 Gazebo/PX4 가 남아 있어 종료합니다"
  pkill -9 -f "gz sim" || true; pkill -9 -x px4 || true; pkill -9 -f MicroXRCEAgent || true; sleep 2
fi
tmux new-session -d -s $SESSION -n agent "$CMD_AGENT; exec bash"
tmux new-window -t $SESSION -n px4 "bash -c \"$CMD_PX4\"; exec bash"
tmux new-window -t $SESSION -n ros "bash -c \"$CMD_ROS\"; exec bash"
if [ $DEMO -eq 1 ]; then
  tmux new-window -t $SESSION -n demo "bash -c \"$CMD_DEMO\""
fi
tmux new-window -t $SESSION -n shell "bash -c \"$ENV_ROS; echo '명령 예: ros2 run vertiport_sim vp_cmd goto 0 0 4   (지상이면 자동 이륙)'; exec bash\""
tmux select-window -t $SESSION:px4
echo "tmux 세션 '$SESSION' 시작 (창: agent / px4 / ros$( [ $DEMO -eq 1 ] && echo ' / demo') / shell)"
echo "대시보드 API: http://localhost:8080/api/health   점검 뷰어: http://localhost:8080/"
exec tmux attach -t $SESSION
