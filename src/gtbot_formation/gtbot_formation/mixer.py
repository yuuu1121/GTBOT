"""검증된 믹서(2026-08-06 실측): [E,W,N,S], +X=[0,0,-a,+a], +Y=[-a,+a,0,0], yaw=[a,a,a,a].

yaw 방향 실측: [a,a,a,a]의 a>0은 yaw를 **감소**시킨다(sw=+0.1에서 -215 deg/4 s,
정상상태 yaw rate ≈ -12 rad/s per unit sw, 1차 지연 τ≈0.5 s). 따라서 yaw를 yaw0로
되돌리는 제어법은 syaw = kpsi*(yaw - yaw0) — 부호를 뒤집으면 정귀환이 되어 발산한다.
"""
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def world_to_body(vx, vy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * vx + s * vy, -s * vx + c * vy])


def setpoints(e_body, syaw, kv):
    sx = float(np.clip(kv * e_body[0], -0.7, 0.7))
    sy = float(np.clip(kv * e_body[1], -0.7, 0.7))
    sw = float(np.clip(syaw, -0.3, 0.3))
    out = np.array([-sy + sw, +sy + sw, -sx + sw, +sx + sw])
    return np.clip(out, -1.0, 1.0)
