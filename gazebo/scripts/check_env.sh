#!/usr/bin/env bash
# Gazebo 담당 PC 환경 점검 (읽기 전용 - 아무것도 설치/변경하지 않음)
#   bash scripts/check_env.sh
# 출력 결과를 팀 채널에 공유한 뒤 versions.env 의 UBUNTU_VERSION / ROS_DISTRO 를 확정한다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../versions.env
source "$REPO/versions.env"

ok()   { printf '  \033[32m[OK]\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33m[WARN]\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m[NO]\033[0m   %s\n' "$*"; }
hdr()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

hdr "OS / 하드웨어"
if [ -r /etc/os-release ]; then . /etc/os-release; fi
UB="${VERSION_ID:-unknown}"
echo "  OS        : ${PRETTY_NAME:-unknown} (kernel $(uname -r), $(uname -m))"
echo "  CPU       : $(nproc) cores, $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2 | xargs)"
echo "  RAM       : $(free -g 2>/dev/null | awk '/Mem:/{print $2" GB"}')"
echo "  Disk(\$HOME): $(df -h "$HOME" 2>/dev/null | awk 'NR==2{print $4" free"}')"
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "  GPU       : $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1)"
else
  echo "  GPU       : $(lspci 2>/dev/null | grep -i -m1 -E 'vga|3d' | cut -d: -f3 | xargs)"
  warn "NVIDIA 드라이버 없음 - 내장그래픽이면 카메라 센서 렌더링(ogre2)이 느릴 수 있음"
fi
if grep -qi microsoft /proc/version 2>/dev/null; then warn "WSL 감지 - Gazebo GUI/GPU 렌더링 제약 있음 (네이티브 Ubuntu 권장)"; fi
[ "$(free -g | awk '/Mem:/{print $2}')" -lt 8 ] 2>/dev/null && warn "RAM 8GB 미만 - PX4 빌드/Gazebo 가 느릴 수 있음"

hdr "권장 조합 판단"
case "$UB" in
  22.04) REC=humble; ok "Ubuntu 22.04 → ROS 2 Humble + Gazebo Harmonic (PX4 공식 권장, Jetson JetPack 6 과 동일 OS)";;
  24.04) REC=jazzy;  ok "Ubuntu 24.04 → ROS 2 Jazzy + Gazebo Harmonic";;
  20.04) REC=none;   bad "Ubuntu 20.04 → Gazebo Harmonic/PX4 v1.17 미지원. 22.04 로 업그레이드 필요";;
  *)     REC=none;   bad "Ubuntu $UB 는 이 프로젝트에서 검증하지 않은 버전";;
esac
echo "  versions.env : UBUNTU_VERSION=$UBUNTU_VERSION, ROS_DISTRO=$ROS_DISTRO"
if [ "$UB" = "$UBUNTU_VERSION" ] && [ "$REC" = "$ROS_DISTRO" ]; then
  ok "versions.env 와 이 PC 가 일치"
elif [ "$REC" != none ]; then
  warn "versions.env 수정 필요 → UBUNTU_VERSION=$UB, ROS_DISTRO=$REC (팀 합의 후)"
fi

hdr "설치 상태"
if [ -d "/opt/ros/$ROS_DISTRO" ]; then ok "ROS 2 $ROS_DISTRO 설치됨"; else bad "ROS 2 $ROS_DISTRO 없음"; fi
other=$(find /opt/ros -mindepth 1 -maxdepth 1 -type d ! -name "$ROS_DISTRO" -printf '%f ' 2>/dev/null)
[ -n "$other" ] && warn "다른 ROS 배포판도 있음: $other (한 터미널에 두 개를 source 하지 말 것)"
if command -v gz >/dev/null 2>&1; then
  GZV=$(gz sim --versions 2>/dev/null | head -1)
  case "$GZV" in 8.*) ok "Gazebo Harmonic (gz-sim $GZV)";; *) warn "gz-sim $GZV - Harmonic(8.x) 아님";; esac
else bad "Gazebo(gz) 없음"; fi
if dpkg -l 2>/dev/null | grep -q -E "ros-$ROS_DISTRO-ros-gz(harmonic)?-bridge|ros-$ROS_DISTRO-ros-gz "; then ok "ros_gz 브리지 설치됨"; else bad "ros_gz 브리지 없음"; fi
if dpkg -l 2>/dev/null | grep -q "ros-$ROS_DISTRO-ros-gz-bridge" && [ "$ROS_DISTRO" = humble ]; then
  warn "Humble 기본 ros-humble-ros-gz(=Fortress용) 감지 → ros-humble-ros-gzharmonic 으로 교체 필요"
fi
if [ -d "$PX4_DIR/.git" ]; then
  T=$(git -C "$PX4_DIR" describe --tags --always 2>/dev/null)
  [ "$T" = "$PX4_TAG" ] && ok "PX4 $T ($PX4_DIR)" || warn "PX4 $T ≠ 고정값 $PX4_TAG ($PX4_DIR)"
  [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] && ok "PX4 SITL 빌드됨" || warn "PX4 SITL 아직 빌드 안 됨"
else bad "PX4-Autopilot 없음 ($PX4_DIR)"; fi
if command -v MicroXRCEAgent >/dev/null 2>&1; then ok "MicroXRCEAgent: $(command -v MicroXRCEAgent)"; else bad "MicroXRCEAgent 없음"; fi
M="$REPO/ros2_ws/src/px4_msgs"
if [ -d "$M/.git" ]; then
  B=$(git -C "$M" rev-parse --abbrev-ref HEAD)
  [ "$B" = "$PX4_MSGS_BRANCH" ] && ok "px4_msgs $B" || warn "px4_msgs $B ≠ 고정값 $PX4_MSGS_BRANCH"
else bad "px4_msgs 없음 ($M)"; fi
python3 -c "import PIL" 2>/dev/null && ok "python3 Pillow (카메라 MJPEG)" || bad "python3 Pillow 없음 (sudo apt install python3-pil)"
python3 -c "import yaml" 2>/dev/null && ok "python3 yaml" || bad "python3 yaml 없음"
command -v tmux >/dev/null && ok "tmux" || warn "tmux 없음 (run_sim.sh 가 수동 실행 안내로 대체)"
echo
