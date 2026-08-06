import numpy as np
from gtbot_formation.perception_core import (cluster_2d, lidar_to_world,
                                             marker_split, marker_heading)

def synth_pole(cx, cy, h_lo, h_hi, n=30, r=0.015, seed=0):
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

def _synth_marker(cx, cy, yaw, main_top=0.442, aux_drop=0.10, seed_c=0, seed_a=3):
    # round 4 실측 기하: MastMain top 0.442 m(bottom 0.142), MastAux top 0.342 m(bottom 0.122),
    # 간격 0.15 m. 노드 사전필터(h>0.2)를 통과한 구간만 넣는다.
    center = synth_pole(cx, cy, 0.20, main_top, seed=seed_c)
    aux = synth_pole(cx + 0.15*np.cos(yaw), cy + 0.15*np.sin(yaw),
                      0.20, main_top - aux_drop, seed=seed_a)
    return np.vstack([center, aux])

def test_lidar_to_world_frame_convention():
    # round 4 실측: 클라우드는 FLU(y 왼쪽·z 위), platform body는 NED — 월드 정렬은
    # y 미러 후 R(+yaw). 알려진 월드 점을 라이다 관측값으로 역산해 왕복이 맞는지 본다.
    yaw, z_off = np.deg2rad(30.0), 0.198
    world_xy, height = np.array([2.0, 1.0]), 0.44
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.array([[c, s], [-s, c]]) @ world_xy          # R(-yaw) = world -> body(NED)
    flu = np.array([[body[0], -body[1], height - z_off]])  # NED -> FLU
    xy, h = lidar_to_world(flu, yaw, z_off)
    assert np.allclose(xy[0], world_xy, atol=1e-9)
    assert np.isclose(h[0], height)

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
    only_center = synth_pole(0.0, 0.0, 0.20, 0.442)
    assert marker_split(only_center, only_center[:, 2]) is None

def test_marker_split_ignores_hull_points():
    # round 4 실측: 선체 상단은 마스트 상단보다 0.30 m 낮다. 사전필터를 뚫고 들어와도
    # 상대 밴딩 밖이라 중앙/보조 추정에 섞이지 않아야 한다.
    yaw = np.deg2rad(120)
    xyz = _synth_marker(1.5, -0.7, yaw)
    rng = np.random.default_rng(7)
    hull = np.column_stack([1.5 + rng.uniform(-0.25, 0.25, 200),
                            -0.7 + rng.uniform(-0.25, 0.25, 200),
                            rng.uniform(0.0, 0.147, 200)])
    res = marker_split(xyz, xyz[:, 2])
    res_h = marker_split(np.vstack([xyz, hull]), np.concatenate([xyz[:, 2], hull[:, 2]]))
    assert res is not None and res_h is not None
    assert np.allclose(res[0], res_h[0]) and np.allclose(res[1], res_h[1])
