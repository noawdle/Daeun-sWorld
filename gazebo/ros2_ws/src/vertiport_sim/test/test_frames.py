"""frames.py 좌표변환 테스트."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vertiport_sim import camera_geometry as cg  # noqa: E402
from vertiport_sim import frames as f  # noqa: E402


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_vector_roundtrip():
    for _ in range(100):
        v = [random.uniform(-10, 10) for _ in range(3)]
        assert close(f.enu_to_ned(f.ned_to_enu(v)), v)


def test_altitude_sign():
    # PX4 NED z=-4 (위로 4m) → World ENU z=+4
    assert close(f.ned_to_enu((0, 0, -4)), (0, 0, 4))
    # 팀 좌표 target_z=4 → PX4 z=-4
    assert close(f.enu_to_ned((5, 3, 4)), (3, 5, -4))


def test_yaw_conventions():
    # 북쪽을 보는 기체: NED yaw 0 → ENU yaw +90°
    assert abs(f.yaw_ned_to_enu(0.0) - math.pi / 2) < 1e-9
    # 동쪽을 보는 기체: NED yaw +90° → ENU yaw 0
    assert abs(f.yaw_ned_to_enu(math.pi / 2)) < 1e-9
    for _ in range(100):
        y = random.uniform(-math.pi, math.pi)
        assert abs(f.wrap_pi(f.yaw_enu_to_ned(f.yaw_ned_to_enu(y)) - y)) < 1e-9


def test_quat_euler_roundtrip():
    for _ in range(200):
        r, p, y = random.uniform(-3, 3), random.uniform(-1.5, 1.5), random.uniform(-3, 3)
        q = f.euler_to_quat_wxyz(r, p, y)
        r2, p2, y2 = f.quat_wxyz_to_euler(q)
        assert abs(f.wrap_pi(r - r2)) < 1e-6 and abs(p - p2) < 1e-6 and abs(f.wrap_pi(y - y2)) < 1e-6


def test_rpy_frd_ned_to_flu_enu():
    # 기수 들림 (FRD pitch +0.1) → FLU pitch -0.1
    r, p, y = f.rpy_frd_ned_to_flu_enu(0.0, 0.1, 0.0)
    assert abs(p + 0.1) < 1e-9 and abs(y - math.pi / 2) < 1e-9


def test_local_world_offset():
    tf = f.LocalWorldTransform((-6.0, 0.0, 0.0))   # 스폰 위치 = World (-6, 0)
    # 스폰 지점에서 동쪽 6m, 위로 4m 이동 = 패드 상공
    assert close(tf.local_ned_to_world((0.0, 6.0, -4.0)), (0.0, 0.0, 4.0))
    for _ in range(100):
        w = [random.uniform(-20, 20) for _ in range(3)]
        assert close(tf.local_ned_to_world(tf.world_to_local_ned(w)), w)


def test_camera_extrinsic_is_rotation():
    R = f.R_BODY_FLU_FROM_CAM_OPTICAL
    # 직교 + det = +1
    for i in range(3):
        for j in range(3):
            dot = sum(R[k][i] * R[k][j] for k in range(3))
            assert abs(dot - (1.0 if i == j else 0.0)) < 1e-12
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    assert abs(det - 1.0) < 1e-12


def _apply(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def test_camera_tf_chain_matches_extrinsic():
    q = f.quat_mul(f.euler_to_quat_wxyz(*f.CAMERA_MOUNT_RPY), f.Q_CAMLINK_FROM_OPTICAL_WXYZ)
    R = f.quat_to_matrix(q)
    for i in range(3):
        assert close(R[i], f.R_BODY_FLU_FROM_CAM_OPTICAL[i], 1e-3)


def test_threejs_conversion_consistent():
    # ENU 에서 회전 후 Three 로 옮긴 것 == Three 로 옮긴 뒤 Three 쿼터니언으로 회전한 것
    for _ in range(100):
        r, p, y = random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-3, 3)
        w, x, yy, z = f.euler_to_quat_wxyz(r, p, y)
        v = [random.uniform(-1, 1) for _ in range(3)]
        a = f.enu_to_threejs_position(_apply(f.quat_to_matrix((w, x, yy, z)), v))
        tx, ty, tz, tw = f.enu_to_threejs_quat_xyzw(x, yy, z, w)
        b = _apply(f.quat_to_matrix((tw, tx, ty, tz)), f.enu_to_threejs_position(v))
        assert close(a, b, 1e-9)


def test_quat_matches_rpy_convention():
    # yaw 90° = 기수가 북(+y)
    R = f.quat_to_matrix(f.euler_to_quat_wxyz(0, 0, math.pi / 2))
    assert close(_apply(R, (1, 0, 0)), (0, 1, 0), 1e-9)
    # pitch +0.3 (FLU) = 기수 내림 → 기체 x축의 z 성분이 음수
    R = f.quat_to_matrix(f.euler_to_quat_wxyz(0, 0.3, 0))
    assert _apply(R, (1, 0, 0))[2] < 0


K = [539.94, 0, 640, 0, 539.94, 480, 0, 0, 1]


def test_level_hover_center_pixel_hits_below_camera():
    q = f.quat_xyzw(f.euler_to_quat_wxyz(0, 0, 0.0))
    g = cg.pixel_to_ground(640, 480, K, (1.0, 2.0, 4.0), q)
    assert close(g, (1.0, 2.0, 0.0), 1e-9)   # 수평 자세: 영상 중심 = 바로 아래


def test_image_up_is_drone_forward():
    # yaw 0 → 기수 동쪽(+x). 영상 위쪽(v 작음) 점은 +x 쪽 지면
    q = f.quat_xyzw(f.euler_to_quat_wxyz(0, 0, 0.0))
    g = cg.pixel_to_ground(640, 100, K, (0.0, 0.0, 4.0), q)
    assert g[0] > 0.5 and abs(g[1]) < 1e-9
    g = cg.pixel_to_ground(1200, 480, K, (0.0, 0.0, 4.0), q)   # 영상 오른쪽 = 기체 오른쪽 = -y
    assert g[1] < -0.5 and abs(g[0]) < 1e-9


def test_project_unproject_roundtrip_tilted():
    for _ in range(200):
        q = f.quat_xyzw(f.euler_to_quat_wxyz(random.uniform(-.3, .3), random.uniform(-.3, .3),
                                             random.uniform(-3, 3)))
        pos = (random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(1.5, 8))
        p = (pos[0] + random.uniform(-1.5, 1.5), pos[1] + random.uniform(-1.5, 1.5), 0.0)
        uv = cg.world_to_pixel(p, K, pos, q)
        assert uv is not None
        g = cg.pixel_to_ground(uv[0], uv[1], K, pos, q)
        assert close(g, p, 1e-6)


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
