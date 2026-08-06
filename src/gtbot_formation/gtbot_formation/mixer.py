"""검증된 믹서(2026-08-06 실측): [E,W,N,S], +X=[0,0,-a,+a], +Y=[-a,+a,0,0], yaw=[a,a,a,a]."""
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
    sx, sy = kv * e_body[0], kv * e_body[1]
    out = np.array([-sy + syaw, +sy + syaw, -sx + syaw, +sx + syaw])
    return np.clip(out, -1.0, 1.0)
