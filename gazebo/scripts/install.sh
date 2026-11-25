#!/usr/bin/env bash
# =============================================================================
#  버티포트 시뮬레이션 설치 스크립트 (versions.env 의 고정 버전으로 설치)
#
#    bash scripts/install.sh              # 전체 설치 (ROS 2, PX4+Gazebo, ros_gz, Agent, 워크스페이스)
#    bash scripts/install.sh --only-ws    # 워크스페이스만 다시 빌드 (코드 수정 후)
#    옵션: --skip-ros --skip-px4 --skip-agent --skip-ws
#
#  소요: 처음 설치 30~60분 (PX4 빌드 포함), 디스크 약 15 GB
# =============================================================================
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../versions.env
source "$REPO/versions.env"

SKIP_ROS=0; SKIP_PX4=0; SKIP_AGENT=0; SKIP_WS=0
for a in "$@"; do
  case "$a" in
    --skip-ros) SKIP_ROS=1;; --skip-px4) SKIP_PX4=1;; --skip-agent) SKIP_AGENT=1;; --skip-ws) SKIP_WS=1;;
    --only-ws) SKIP_ROS=1; SKIP_PX4=1; SKIP_AGENT=1;;
    -h|--help) sed -n 2,11p "$0"; exit 0;;
    *) echo "unknown option $a"; exit 2;;
  esac
done

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { printf '\033[31mERROR: %s\033[0m\n' "$*"; exit 1; }

# ---------------------------------------------------------------- 0. OS 확인
. /etc/os-release
[ "${VERSION_ID}" = "$UBUNTU_VERSION" ] || die "이 PC 는 Ubuntu ${VERSION_ID}, versions.env 는 $UBUNTU_VERSION.
  scripts/check_env.sh 결과를 보고 versions.env 를 팀 합의로 수정한 뒤 다시 실행하세요."
case "$UBUNTU_VERSION:$ROS_DISTRO" in
  22.04:humble|24.04:jazzy) ;;
  *) die "지원하지 않는 조합: Ubuntu $UBUNTU_VERSION + ROS $ROS_DISTRO";;
esac
[ "$(id -u)" -ne 0 ] || die "root 가 아닌 일반 사용자로 실행하세요 (sudo 는 스크립트가 필요할 때 요청)"
echo "고정 버전: Ubuntu $UBUNTU_VERSION / ROS 2 $ROS_DISTRO / Gazebo $GZ_RELEASE / PX4 $PX4_TAG / px4_msgs $PX4_MSGS_BRANCH / XRCE Agent $XRCE_AGENT_TAG"

PIP_FLAGS="--user"
[ "$UBUNTU_VERSION" = "24.04" ] && PIP_FLAGS="--user --break-system-packages"

step "기본 패키지"
sudo apt-get update
sudo apt-get install -y git curl wget tmux build-essential cmake python3-pip \
  python3-yaml python3-pil python3-pytest software-properties-common lsb-release gnupg

# ---------------------------------------------------------------- 1. ROS 2
if [ $SKIP_ROS -eq 0 ]; then
  step "ROS 2 $ROS_DISTRO"
  if [ ! -d "/opt/ros/$ROS_DISTRO" ]; then
    sudo add-apt-repository -y universe
    # ROS 공식 apt 소스 패키지 (키 만료 문제를 해결한 2025년 이후 방식)
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [ -n "$ROS_APT_SOURCE_VERSION" ] || die "ros-apt-source 버전 조회 실패 (네트워크 확인)"
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-$VERSION_CODENAME}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update
    sudo apt-get install -y "ros-$ROS_DISTRO-desktop" ros-dev-tools
  else
    echo "이미 설치됨: /opt/ros/$ROS_DISTRO"
  fi
fi
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

# ---------------------------------------------------------------- 2. PX4 + Gazebo Harmonic
if [ $SKIP_PX4 -eq 0 ]; then
  step "PX4-Autopilot $PX4_TAG (+ Gazebo $GZ_RELEASE 는 PX4 ubuntu.sh 가 설치)"
  if [ ! -d "$PX4_DIR/.git" ]; then
    git clone --recursive --branch "$PX4_TAG" https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  else
    CUR=$(git -C "$PX4_DIR" describe --tags --always)
    if [ "$CUR" != "$PX4_TAG" ]; then
      echo "PX4 $CUR → $PX4_TAG 로 전환"
      git -C "$PX4_DIR" fetch --tags
      git -C "$PX4_DIR" checkout "$PX4_TAG"
      git -C "$PX4_DIR" submodule update --init --recursive
      (cd "$PX4_DIR" && make distclean)
    fi
  fi
  bash "$PX4_DIR/Tools/setup/ubuntu.sh"
  # ROS 2 rosidl 은 empy 3.x 필요 (PX4 문서 권장 버전)
  # shellcheck disable=SC2086
  pip3 install $PIP_FLAGS -U empy==3.3.4 pyros-genmsg setuptools
  gz sim --versions | head -1 | grep -q '^8\.' || die "Gazebo Harmonic(8.x) 설치 확인 실패"
  (cd "$PX4_DIR" && make px4_sitl)
fi

# ---------------------------------------------------------------- 3. ros_gz (ROS ↔ Gazebo 브리지)
step "ros_gz 브리지 ($ROS_DISTRO + Harmonic)"
if [ "$ROS_DISTRO" = humble ]; then
  # Humble 기본 ros-gz 는 Fortress 용 → Harmonic 용 패키지로 설치
  sudo apt-get install -y ros-humble-ros-gzharmonic
else
  sudo apt-get install -y ros-jazzy-ros-gz
fi

# ---------------------------------------------------------------- 4. Micro XRCE-DDS Agent
if [ $SKIP_AGENT -eq 0 ]; then
  step "Micro-XRCE-DDS-Agent $XRCE_AGENT_TAG"
  if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    [ -d "$XRCE_DIR" ] || git clone -b "$XRCE_AGENT_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$XRCE_DIR"
    mkdir -p "$XRCE_DIR/build"
    (cd "$XRCE_DIR/build" && cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig /usr/local/lib/)
  else
    echo "이미 설치됨: $(command -v MicroXRCEAgent)"
  fi
fi

# ---------------------------------------------------------------- 5. ROS 2 워크스페이스
if [ $SKIP_WS -eq 0 ]; then
  step "워크스페이스 (px4_msgs $PX4_MSGS_BRANCH + vertiport_sim)"
  M="$REPO/ros2_ws/src/px4_msgs"
  if [ ! -d "$M/.git" ]; then
    git clone -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git "$M"
  else
    B=$(git -C "$M" rev-parse --abbrev-ref HEAD)
    if [ "$B" != "$PX4_MSGS_BRANCH" ]; then
      git -C "$M" fetch origin "$PX4_MSGS_BRANCH"
      git -C "$M" checkout "$PX4_MSGS_BRANCH"
      rm -rf "$REPO/ros2_ws/build/px4_msgs" "$REPO/ros2_ws/install/px4_msgs"
    fi
  fi
  (cd "$REPO/ros2_ws" && colcon build --symlink-install)
fi

# ---------------------------------------------------------------- 6. 버티포트 world 연결
step "버티포트 world 를 PX4 Gazebo worlds 폴더에 링크"
if [ -d "$PX4_DIR/Tools/simulation/gz/worlds" ]; then
  ln -sfn "$REPO/sim/worlds/vertiport.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/vertiport.sdf"
  echo "  $PX4_DIR/Tools/simulation/gz/worlds/vertiport.sdf → $REPO/sim/worlds/vertiport.sdf"
else
  echo "  (PX4 가 없어 건너뜀)"
fi

step "완료 - 버전 검증"
bash "$REPO/scripts/verify_versions.sh" || true
cat <<EOF

다음 단계:
  bash scripts/run_sim.sh                       # 시뮬레이션 실행
  bash scripts/run_sim.sh --demo                # 자동 이륙→접근→착륙 + 사람 등장 시나리오
EOF
