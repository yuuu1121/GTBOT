import numpy as np
from gtbot_formation.perception_core import cluster_2d, marker_split, marker_heading

def synth_pole(cx, cy, h_lo, h_hi, n=30, r=0.025, seed=0):
    rng = np.random.default_rng(seed)
    ang = rng.uniform(0, 2*np.pi, n)
    h = rng.uniform(h_lo, h_hi, n)
    return np.stack([cx + r*np.cos(ang), cy + r*np.sin(ang), h], axis=1)

def test_cluster_2d_separates_three_robots():
    pts = np.vstack([synth_pole(0.9, 0.0, 0.25, 0.38),
                     synth_pole(-0.45, 0.75, 0.25, 0.38, seed=1),
                     synth_pole(-0.45, -0.75, 0.25, 0.38, seed=2)])
    clusters = cluster_2d(pts[:, :2])
    assert len(clusters) == 3
    assert sorted(len(c) for c in clusters) == [30, 30, 30]

def test_marker_split_and_heading():
    yaw = np.deg2rad(30)   # 로봇 (1.0, 0.5), yaw 30° — aux = center + 0.15*(cos,sin)
    center = synth_pole(1.0, 0.5, 0.25, 0.38)
    aux = synth_pole(1.0 + 0.15*np.cos(yaw), 0.5 + 0.15*np.sin(yaw), 0.25, 0.30, seed=3)
    xyz = np.vstack([center, aux])
    res = marker_split(xyz, xyz[:, 2])
    assert res is not None
    c_xy, a_xy = res
    assert np.linalg.norm(c_xy - [1.0, 0.5]) < 0.04
    est = marker_heading(c_xy, a_xy)
    assert abs(np.arctan2(np.sin(est - yaw), np.cos(est - yaw))) < np.deg2rad(10)

def test_marker_split_rejects_single_pole():
    only_center = synth_pole(0.0, 0.0, 0.30, 0.40)
    assert marker_split(only_center, only_center[:, 2]) is None
