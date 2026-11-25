"""하향 카메라 기하 - 픽셀 ↔ 지면 월드 좌표 (참고 구현, ROS 의존성 없음).

통합 파트가 bbox 픽셀을 월드 좌표로 바꿀 때 쓰는 계산을 TF 규격(INTERFACE.md §5)과
같은 정의로 구현한 것. 그대로 써도 되고 검증용 기준으로만 써도 된다.

  pixel_to_ground(u, v, K, drone_pos, drone_q)  →  (x, y, z=ground_z) World ENU
  world_to_pixel(p, K, drone_pos, drone_q)       →  (u, v) 또는 None(카메라 뒤)

drone_pos : telemetry.position_m (base_link, World ENU)
drone_q   : telemetry.orientation_q {x,y,z,w} (body FLU → World ENU)
K         : camera_info.k (3x3 행 우선 9개) - fx=K[0], fy=K[4], cx=K[2], cy=K[5]
왜곡은 고려하지 않음 (Gazebo 카메라는 왜곡 없음. 실기체는 먼저 cv2.undistortPoints 적용).
"""

from typing import Optional, Sequence, Tuple

from . import frames


def _mat_vec(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(R):
    return tuple(tuple(R[j][i] for j in range(3)) for i in range(3))


def _q_wxyz(q) -> Tuple[float, float, float, float]:
    if isinstance(q, dict):
        return (q['w'], q['x'], q['y'], q['z'])
    return tuple(q)


def camera_pose_world(drone_pos, drone_q, mount_xyz=frames.CAMERA_MOUNT_XYZ):
    """(카메라 광학중심 World 좌표, R_world_from_optical)"""
    R_wb = frames.quat_to_matrix(_q_wxyz(drone_q))
    off = _mat_vec(R_wb, mount_xyz)
    p = drone_pos if not isinstance(drone_pos, dict) else (drone_pos['x'], drone_pos['y'], drone_pos['z'])
    c = (p[0] + off[0], p[1] + off[1], p[2] + off[2])
    R_bo = frames.R_BODY_FLU_FROM_CAM_OPTICAL
    R_wo = tuple(tuple(sum(R_wb[i][k] * R_bo[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    return c, R_wo


def pixel_to_ground(u: float, v: float, K: Sequence[float], drone_pos, drone_q,
                    ground_z: float = 0.0) -> Optional[Tuple[float, float, float]]:
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    c, R_wo = camera_pose_world(drone_pos, drone_q)
    d = _mat_vec(R_wo, ((u - cx) / fx, (v - cy) / fy, 1.0))
    if d[2] >= -1e-9:          # 광선이 지면을 향하지 않음
        return None
    s = (ground_z - c[2]) / d[2]
    if s <= 0:
        return None
    return (c[0] + s * d[0], c[1] + s * d[1], ground_z)


def world_to_pixel(p: Sequence[float], K: Sequence[float], drone_pos, drone_q) -> Optional[Tuple[float, float]]:
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    c, R_wo = camera_pose_world(drone_pos, drone_q)
    pc = _mat_vec(_transpose(R_wo), (p[0] - c[0], p[1] - c[1], p[2] - c[2]))
    if pc[2] <= 1e-9:
        return None
    return (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy)
