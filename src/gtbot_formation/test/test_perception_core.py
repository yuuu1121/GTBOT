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

def _synth_marker(cx, cy, yaw, main_top=0.40, aux_drop=0.08, seed_c=0, seed_a=3):
    # main pole top=main_top, aux pole top=main_top-aux_drop (fix round 1 실측: 두 기둥
    # 높이차 ~0.08m). h_max 기준 상대 밴딩이므로 절대 높이값 자체는 임의로 둬도 무방.
    center = synth_pole(cx, cy, main_top - 0.15, main_top, seed=seed_c)
    aux = synth_pole(cx + 0.15*np.cos(yaw), cy + 0.15*np.sin(yaw),
                      main_top - aux_drop - 0.12, main_top - aux_drop, seed=seed_a)
    return np.vstack([center, aux])

def test_marker_split_and_heading():
    yaw = np.deg2rad(30)   # 로봇 (1.0, 0.5), yaw 30° — aux = center + 0.15*(cos,sin)
    xyz = _synth_marker(1.0, 0.5, yaw)
    res = marker_split(xyz, xyz[:, 2])
    assert res is not None
    c_xy, a_xy = res
    assert np.linalg.norm(c_xy - [1.0, 0.5]) < 0.04
    est = marker_heading(c_xy, a_xy)
    assert abs(np.arctan2(np.sin(est - yaw), np.cos(est - yaw))) < np.deg2rad(10)

def test_marker_split_robust_to_z_miscalibration():
    # fix round 1: h_max 기준 상대 밴딩이므로 z_water_offset 오차(전 포인트에 동일 오프셋)가
    # 그대로 상쇄돼야 한다 — 임의 오프셋(+0.7)을 h 전체에 더해도 결과가 바뀌지 않아야 함.
    yaw = np.deg2rad(-50)
    xyz = _synth_marker(-0.3, 0.8, yaw)
    xyz_shifted = xyz.copy()
    xyz_shifted[:, 2] += 0.7
    res = marker_split(xyz, xyz[:, 2])
    res_shifted = marker_split(xyz_shifted, xyz_shifted[:, 2])
    assert res is not None and res_shifted is not None
    assert np.allclose(res[0], res_shifted[0]) and np.allclose(res[1], res_shifted[1])

def test_marker_split_rejects_single_pole():
    only_center = synth_pole(0.0, 0.0, 0.25, 0.40)
    assert marker_split(only_center, only_center[:, 2]) is None
