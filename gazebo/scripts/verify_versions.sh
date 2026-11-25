#!/usr/bin/env bash
# 설치된 버전이 versions.env 와 일치하는지 + px4_msgs 메시지 정의가 PX4 와 같은지 검사
#   bash scripts/verify_versions.sh      (모두 일치하면 exit 0)
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../versions.env
source "$REPO/versions.env"
FAIL=0
row() { printf '  %-14s %-16s %-22s %s\n' "$1" "$2" "$3" "$4"; }
chk() { if [ "$2" = "$3" ]; then row "$1" "$2" "$3" "✅"; else row "$1" "$2" "$3" "❌"; FAIL=1; fi; }

. /etc/os-release 2>/dev/null || true
echo
row "항목" "고정값" "설치됨" ""
row "----" "------" "------" ""
chk "Ubuntu" "$UBUNTU_VERSION" "${VERSION_ID:-?}"
chk "ROS 2" "$ROS_DISTRO" "$( [ -d /opt/ros/$ROS_DISTRO ] && echo $ROS_DISTRO || ls /opt/ros 2>/dev/null | head -1)"
GZV=$(gz sim --versions 2>/dev/null | head -1)
chk "Gazebo" "$GZ_RELEASE" "$(case "$GZV" in 8.*) echo harmonic;; "") echo none;; *) echo "gz-sim $GZV";; esac)"
chk "PX4" "$PX4_TAG" "$(git -C "$PX4_DIR" describe --tags --always 2>/dev/null || echo none)"
chk "px4_msgs" "$PX4_MSGS_BRANCH" "$(git -C "$REPO/ros2_ws/src/px4_msgs" rev-parse --abbrev-ref HEAD 2>/dev/null || echo none)"
AG=$(git -C "$XRCE_DIR" describe --tags --always 2>/dev/null || echo "?")
command -v MicroXRCEAgent >/dev/null || AG=none
chk "XRCE Agent" "$XRCE_AGENT_TAG" "$AG"

# px4_msgs 의 .msg 와 PX4 소스의 .msg 를 주석/공백 무시하고 비교
if [ -d "$PX4_DIR/msg" ] && [ -d "$REPO/ros2_ws/src/px4_msgs/msg" ]; then
  python3 - "$PX4_DIR/msg" "$REPO/ros2_ws/src/px4_msgs/msg" <<'EOF' || FAIL=1
import glob, os, re, sys
px4, msgs = sys.argv[1], sys.argv[2]
def norm(p):
    return [re.sub(r'\s+', ' ', l.split('#')[0].strip()) for l in open(p) if l.split('#')[0].strip()]
bad, missing = [], []
files = sorted(glob.glob(os.path.join(msgs, '*.msg')))
for f in files:
    n = os.path.basename(f)
    c = [x for x in (os.path.join(px4, n), os.path.join(px4, 'versioned', n)) if os.path.exists(x)]
    if not c:
        missing.append(n)
    elif norm(f) != norm(c[0]):
        bad.append(n)
print(f'\n  메시지 정의 비교: px4_msgs {len(files)}개 중 불일치 {len(bad)}개, PX4에 없음 {len(missing)}개')
if bad:
    print('   ❌ 불일치:', ', '.join(bad[:15]))
if missing:
    print('   ⚠️ PX4에 없음:', ', '.join(missing[:15]))
sys.exit(1 if bad else 0)
EOF
fi
echo
[ $FAIL -eq 0 ] && echo "  ✅ 모든 버전이 팀 고정값과 일치" || echo "  ❌ 불일치 항목 있음 → scripts/install.sh 재실행 또는 versions.env 확인"
exit $FAIL
