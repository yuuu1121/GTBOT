"""imu_drift_shim의 yaw 회전이 roll·pitch를 건드리지 않는지 확인.

심은 IMU 자세에 헤딩 오차만 주입해야 한다. 쿼터니언 곱 순서를 틀리면(오른쪽 곱)
body z축 회전이 되어 기울어진 자세에서 roll·pitch가 함께 오염된다 — 그러면 판
검출 기하까지 흔들려 드리프트 실험이 아니라 다른 실험이 된다.
"""
import numpy as np
from gtbot_formation.imu_drift_shim import yaw_rotate


def rpy_to_q(r, p, y):
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def q_to_rpy(x, y, z, w):
    return (np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def test_yaw_rotate_preserves_roll_pitch():
    rng = np.random.default_rng(0)
    wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
    for _ in range(500):
        r, p = rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)   # 파랑 수준 기울기
        y, d = rng.uniform(-np.pi, np.pi), rng.uniform(-1.0, 1.0)
        r2, p2, y2 = q_to_rpy(*yaw_rotate(*rpy_to_q(r, p, y), d))
        assert abs(wrap(y2 - (y + d))) < 1e-9, 'yaw가 지정한 만큼 돌지 않았다'
        assert abs(r2 - r) < 1e-9 and abs(p2 - p) < 1e-9, 'roll·pitch가 오염됐다'
