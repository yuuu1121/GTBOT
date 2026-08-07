import numpy as np
from gtbot_formation.perception_core import (cluster_2d, lidar_to_world,
                                             marker_split, marker_heading,
                                             marker_center, kf_step, assign_tracks)

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

def _synth_marker(cx, cy, yaw, main_top=0.442, aux_drop=0.08, seed_c=0, seed_a=3,
                  offset=(0.0, 0.0)):
    # round 5 재배치 기하: 두 기둥이 로봇 원점 기준 body ±0.08 m 대칭(간격 0.16), 상단
    # 높이차 0.08(MastMain 스팬 -0.535..-0.225 / MastAux -0.455..-0.225).
    # (cx, cy)는 **로봇 원점**이고 main은 그 뒤쪽(-X), aux는 앞쪽(+X)에 놓인다.
    # 노드 사전필터(h>0.2)를 통과한 구간만 넣는다.
    # offset: 마스트 중점의 body 위치(씬 최종본은 (0,-0.10)) — 월드에서는 yaw로 회전해 실린다.
    ct, st = np.cos(yaw), np.sin(yaw)
    ox, oy = ct*offset[0] - st*offset[1], st*offset[0] + ct*offset[1]
    mx, my = cx + ox, cy + oy                       # 두 기둥의 중점(월드)
    dx, dy = 0.08*ct, 0.08*st
    center = synth_pole(mx - dx, my - dy, 0.20, main_top, seed=seed_c)
    aux = synth_pole(mx + dx, my + dy, 0.20, main_top - aux_drop, seed=seed_a)
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
    # 로봇 원점 (1.0, 0.5), yaw 30°. 위치 추정은 두 기둥의 **중점**(= 원점), 헤딩은 main→aux.
    yaw = np.deg2rad(30)
    xyz = _synth_marker(1.0, 0.5, yaw)
    res = marker_split(xyz, xyz[:, 2])
    assert res is not None
    c_xy, a_xy = res
    assert np.linalg.norm(c_xy - [1.0 - 0.08*np.cos(yaw), 0.5 - 0.08*np.sin(yaw)]) < 0.04
    assert np.linalg.norm(marker_center(c_xy, a_xy, (0.0, 0.0)) - [1.0, 0.5]) < 0.04
    est = marker_heading(c_xy, a_xy)
    assert abs(np.arctan2(np.sin(est - yaw), np.cos(est - yaw))) < np.deg2rad(10)

def test_marker_split_excludes_aux_top_from_main_band():
    """상단 높이차가 0.08로 줄어 main_band 상한 0.09는 aux 상단을 물어들인다(회귀 가드)."""
    yaw = 0.0
    xyz = _synth_marker(0.0, 0.0, yaw)
    c_ok, _ = marker_split(xyz, xyz[:, 2])                          # 기본 main_band=(0, 0.07)
    c_bad, _ = marker_split(xyz, xyz[:, 2], main_band=(0.0, 0.09))
    assert abs(c_ok[0] - (-0.08)) < 0.02                            # main 기둥에 정확히 붙음
    assert c_bad[0] > c_ok[0] + 0.01                                # aux 쪽으로 끌려감

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


def _run_kf(z, dt, q=0.5, r=0.025):
    x = np.array([z[0][0], z[0][1], 0.0, 0.0])
    P = np.diag([r**2, r**2, 0.5**2, 0.5**2])
    out = []
    for zk in z[1:]:
        x, P = kf_step(x, P, zk, dt, q, r)
        out.append(x.copy())
    return np.array(out)

def test_kf_velocity_converges_and_beats_finite_difference():
    """상수속도 궤적 + 측정잡음: KF 속도가 참값에 수렴하고 유한차분보다 분산이 훨씬 작아야 한다.

    유한차분은 측정잡음을 1/dt로 증폭한다(sigma ~ r*sqrt(2)/dt). 이것이 rel_vel SNR~1의
    정체였고 LP 릴레이 부호를 노이즈가 정하게 만들었다(round 4~5 실측)."""
    dt, r = 0.1, 0.025
    v_true = np.array([0.20, -0.10])
    rng = np.random.default_rng(0)
    n = 300
    truth = np.array([v_true * (k * dt) for k in range(n)])
    z = truth + rng.normal(0, r, (n, 2))

    est = _run_kf(z, dt, r=r)
    fd = np.diff(z, axis=0) / dt                       # 현행 유한차분(EMA 이전 원신호)

    tail = est[len(est)//2:, 2:]                       # 수렴 후 구간
    assert np.linalg.norm(tail.mean(0) - v_true) < 0.02        # 바이어스 없이 수렴
    kf_std = tail.std(0).mean()
    fd_std = fd[len(fd)//2:].std(0).mean()
    assert kf_std < fd_std / 5, (kf_std, fd_std)               # 분산이 유의하게 작다
    assert fd_std > 0.2                                        # 유한차분 잡음 규모(~r*sqrt2/dt) 확인

def test_kf_position_tracks_measurement_but_smoother():
    dt, r = 0.1, 0.025
    rng = np.random.default_rng(1)
    n = 200
    truth = np.array([[1.0 + 0.05*k*dt, -0.5] for k in range(n)])
    z = truth + rng.normal(0, r, (n, 2))
    est = _run_kf(z, dt, r=r)
    pos_err = np.linalg.norm(est[n//2:, :2] - truth[n//2+1:], axis=1)
    raw_err = np.linalg.norm(z[n//2:] - truth[n//2:], axis=1)
    assert pos_err.mean() < raw_err.mean()             # 위치도 생측정보다 정확


def test_marker_center_undoes_body_offset():
    """씬 최종본: 두 기둥이 body (±0.08, -0.10)에 실려 중점 != 로봇 원점.

    보정을 빼면 위치 추정에 0.10 m 상수 편향이 남아 S4 예산(0.1 m)을 통째로 먹는다."""
    off = (0.0, -0.10)
    for yaw_deg in (0.0, 30.0, -120.0, 175.0):
        yaw = np.deg2rad(yaw_deg)
        xyz = _synth_marker(1.2, -0.4, yaw, offset=off)
        c_xy, a_xy = marker_split(xyz, xyz[:, 2])
        est = marker_center(c_xy, a_xy, off)
        assert np.linalg.norm(est - [1.2, -0.4]) < 0.04, (yaw_deg, est)
        naive = 0.5*(c_xy + a_xy)                              # 보정 누락 시
        assert np.linalg.norm(naive - [1.2, -0.4]) > 0.08      # 0.10 m 편향이 실제로 생긴다
        est_yaw = marker_heading(c_xy, a_xy)                   # 헤딩은 오프셋과 무관
        assert abs(np.arctan2(np.sin(est_yaw - yaw), np.cos(est_yaw - yaw))) < np.deg2rad(10)


def test_assign_tracks_no_preemption_starvation():
    """선점 기아 재현: 두 트랙의 최근접이 같은 검출일 때 탐욕은 한 트랙을 굶긴다.

    round 11 실측 결함 — 탐욕은 argmin 후 점유돼 있으면 그 클러스터를 **버려서**
    남은 트랙이 자기 차선책 검출을 받지 못하고 영구 미아가 됐다."""
    tracks = [np.array([0.0, 0.0]), np.array([0.30, 0.0])]
    dets = [np.array([0.25, 0.0]), np.array([0.80, 0.0])]   # det0가 두 트랙 모두의 최근접
    asg = assign_tracks(dets, tracks, gate=0.6)
    assert asg[0] is not None and asg[1] is not None        # 둘 다 배정 = 기아 없음
    assert asg[0] != asg[1]
    # 전역 최적: 총비용 |0.25|+|0.5| = 0.75 < 대안 |0.05|+|0.8(게이트밖)| -> track1이 det1
    assert asg == [0, 1]

def test_assign_tracks_respects_gate_and_leaves_unassigned():
    tracks = [np.array([0.0, 0.0]), np.array([5.0, 0.0])]
    dets = [np.array([0.1, 0.0])]
    asg = assign_tracks(dets, tracks, gate=0.6)
    assert asg == [0, None]                                  # 게이트 밖은 미배정

def test_assign_tracks_prefers_assignment_over_none():
    tracks = [np.array([0.0, 0.0])]
    dets = [np.array([0.55, 0.0])]                           # 게이트 안이면 항상 배정이 유리
    assert assign_tracks(dets, tracks, gate=0.6) == [0]
