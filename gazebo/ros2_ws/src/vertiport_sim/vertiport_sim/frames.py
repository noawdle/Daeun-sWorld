"""좌표계 변환 유틸리티 (ROS 의존성 없음 → 단위 테스트 가능).

팀 공통 좌표계 (docs/INTERFACE.md 참고)
  - World frame : ENU, 원점 = 착륙패드 중심, x=East, y=North, z=Up
  - Body frame  : FLU (x=Forward, y=Left, z=Up)
  - 각도        : rad, yaw 0 = +x(East), 반시계(CCW) +

PX4 내부 좌표계
  - Local frame : NED (x=North, y=East, z=Down), 원점 = EKF 초기화 위치(=스폰 위치)
  - Body frame  : FRD (x=Forward, y=Right, z=Down)

NED/FRD 값은 이 모듈과 Commander/Telemetry 노드 안에서만 다루고,
팀 인터페이스(ROS 토픽 JSON, HTTP API)에는 절대 노출하지 않는다.
"""

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]


def wrap_pi(a: float) -> float:
    """각도를 [-pi, pi) 범위로."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- 벡터 (위치/속도)
def ned_to_enu(v: Sequence[float]) -> Vec3:
    """NED 벡터 → ENU 벡터. (N, E, D) → (E, N, -D)"""
    n, e, d = float(v[0]), float(v[1]), float(v[2])
    return (e, n, -d)


def enu_to_ned(v: Sequence[float]) -> Vec3:
    """ENU 벡터 → NED 벡터. (E, N, U) → (N, E, -U)"""
    e, n, u = float(v[0]), float(v[1]), float(v[2])
    return (n, e, -u)


# ---------------------------------------------------------------- 자세
def quat_wxyz_to_euler(q: Sequence[float]) -> Vec3:
    """쿼터니언 [w, x, y, z] → (roll, pitch, yaw), ZYX(Tait-Bryan) 순서.

    PX4 VehicleAttitude.q 는 [w, x, y, z] 순서이며 FRD body → NED 회전이다.
    """
    w, x, y, z = (float(c) for c in q)
    # roll (x축)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y축) - 짐벌락 근처 클램프
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z축)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def euler_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """(roll, pitch, yaw) ZYX → 쿼터니언 [w, x, y, z]. (테스트/디지털트윈용)"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """NED yaw(북=0, 시계+) → ENU yaw(동=0, 반시계+)."""
    return wrap_pi(math.pi / 2.0 - yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ENU yaw → NED yaw. (자기 자신의 역변환과 같은 식)"""
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def rpy_frd_ned_to_flu_enu(roll: float, pitch: float, yaw: float) -> Vec3:
    """PX4 자세(FRD→NED 오일러) → 팀 공통 자세(FLU→ENU 오일러).

    roll  : 오른쪽 날개 내려감이 + 로 두 관례에서 동일
    pitch : FRD 는 기수 들림 +, FLU 는 기수 내림 + → 부호 반전
    yaw   : 북 기준 시계방향 → 동 기준 반시계방향
    """
    return (wrap_pi(roll), -pitch, yaw_ned_to_enu(yaw))


# ---------------------------------------------------------------- PX4 local ↔ World
class LocalWorldTransform:
    """PX4 local NED ↔ 팀 World ENU 변환.

    PX4 local 원점은 EKF가 초기화된 위치(SITL에서는 기체 스폰 위치)이고,
    축 방향은 북/동으로 정렬되어 있다. Gazebo world 는 ENU 이며
    PX4 world 파일의 spherical_coordinates 가 ENU/heading 0 이므로
    두 좌표계는 '평행이동'만 다르다.

        world_enu = origin_offset_enu + ned_to_enu(local_ned)

    origin_offset_enu = PX4 local 원점(스폰 위치)의 World 좌표
    """

    def __init__(self, origin_offset_enu: Sequence[float] = (0.0, 0.0, 0.0)):
        self.offset = tuple(float(c) for c in origin_offset_enu)
        if len(self.offset) != 3:
            raise ValueError('origin_offset_enu must have 3 elements')

    def local_ned_to_world(self, p_ned: Sequence[float]) -> Vec3:
        e, n, u = ned_to_enu(p_ned)
        ox, oy, oz = self.offset
        return (e + ox, n + oy, u + oz)

    def world_to_local_ned(self, p_world: Sequence[float]) -> Vec3:
        ox, oy, oz = self.offset
        return enu_to_ned((p_world[0] - ox, p_world[1] - oy, p_world[2] - oz))

    # 속도는 평행이동 영향 없음
    @staticmethod
    def vel_ned_to_world(v_ned: Sequence[float]) -> Vec3:
        return ned_to_enu(v_ned)


# ---------------------------------------------------------------- 쿼터니언 유틸
def quat_mul(a, b):
    """쿼터니언 곱 a⊗b, [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_to_matrix(q):
    """[w, x, y, z] → 3x3 회전행렬 (child 축을 parent 좌표로 표현)."""
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def quat_xyzw(q_wxyz) -> dict:
    """[w, x, y, z] → 인터페이스 JSON 형식 {"x","y","z","w"} (Three.js/ROS 와 같은 순서)."""
    w, x, y, z = q_wxyz
    return {'x': x, 'y': y, 'z': z, 'w': w}


# ---------------------------------------------------------------- Three.js (Y-up) 변환
# ENU(z-up) → Three.js(y-up):  three = (x, z, -y)
# 같은 축 변환을 회전에 적용하면 쿼터니언은 (qx, qz, -qy, qw)
def enu_to_threejs_position(p):
    return (p[0], p[2], -p[1])


def enu_to_threejs_quat_xyzw(qx, qy, qz, qw):
    return (qx, qz, -qy, qw)


# ---------------------------------------------------------------- 하향 카메라 장착 정보
# x500_mono_cam_down: 카메라 링크가 base_link 대비 pitch +90° 로 장착되어 광축이 아래를 향함.
# OpenCV optical frame (x=영상 오른쪽, y=영상 아래, z=광축) 의 각 축을 body FLU 로 표현한 회전행렬.
#   optical x (영상 오른쪽) = body 오른쪽  = -y_FLU
#   optical y (영상 아래)   = body 뒤쪽    = -x_FLU
#   optical z (광축)        = body 아래    = -z_FLU
# 즉 영상 위쪽 = 기체 앞(Forward).
R_BODY_FLU_FROM_CAM_OPTICAL = (
    (0.0, -1.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

# TF 체인 (PX4 Gazebo 모델 x500_mono_cam_down 의 SDF 에서 계산한 값)
#   x500_base 모델: base_link 가 모델 원점 위 0.24 m
#   mono_cam include pose: (0, 0, 0.10, roll 0, pitch 1.5707, yaw 0)
#   → base_link 기준 camera_link = (0, 0, -0.14), pitch +90°  (Gazebo 카메라 관례: x = 광축)
BASE_LINK_HEIGHT = 0.24                              # 지상 정지 시 base_link 높이 [m]
CAMERA_MOUNT_XYZ = (0.0, 0.0, -0.14)                 # base_link → camera_link [m]
CAMERA_MOUNT_RPY = (0.0, 1.5707, 0.0)                # base_link → camera_link [rad]
# camera_link(x 광축) → camera_optical_frame(z 광축, x 오른쪽, y 아래): ROS 표준 고정 회전
Q_CAMLINK_FROM_OPTICAL_WXYZ = (0.5, -0.5, 0.5, -0.5)
