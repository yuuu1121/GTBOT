import numpy as np
from sim.scenario import Scenario
from sim.utility import phi_robot, z2_vector, surface_distances

def _state(positions, velocities):
    X = np.zeros(12)
    X[:6] = np.array(positions).ravel(); X[6:] = np.array(velocities).ravel()
    return X

def test_phi1_guard_zero_velocity():
    sc = Scenario()
    X = _state([(-7, 3), (0, 7), (7, -4)], [(0, 0)] * 3)   # 원논문 X(0): 속도 0
    phis = phi_robot(X, 0, sc)
    assert phis[0] == 0.0 and np.isfinite(phis).all()      # 0/0=NaN 가드 (스펙)

def test_phi3_peak_at_target():
    sc = Scenario()
    X = _state([sc.targets[0], (0, 7), (7, -4)], [(0.1, 0)] * 3)
    assert np.isclose(phi_robot(X, 0, sc)[2], 1.0)          # 식 2.9: |r*|=0 → 1

def test_phi5_phi6_formulas():
    sc = Scenario()
    X = _state([(0, 0), (9, 0), (0, -8)], [(0.1, 0)] * 3)
    phis = phi_robot(X, 0, sc)
    diw = 11 - 0 - 2                                        # 벽 표면거리 9
    assert np.isclose(phis[4], np.log(1 + 6 * np.exp(-40 * diw)))
    dij = min(9.0, 8.0) - 4.0                               # 최근접 로봇 표면거리 (2Rr 규약)
    assert np.isclose(phis[5], np.log(1 + 10 * np.exp(-20 * dij)))

def test_z2_order_and_dim():
    sc = Scenario()
    X = _state([(-7, 3), (0, 7), (7, -4)], [(1, 0)] * 3)
    z2 = z2_vector(X, sc)
    assert z2.shape == (18,)
    assert np.allclose(z2[:6], phi_robot(X, 0, sc))         # 로봇 블록 순서 고정

def test_surface_distances():
    sc = Scenario()
    X = _state([(0, 0), (9, 0), (0, -8)], [(0, 0)] * 3)
    dr, dw = surface_distances(X, sc)
    assert np.isclose(dr, 8.0 - 4.0) and np.isclose(dw, 11 - 9 - 2)

def test_phi7_peak_at_formation():
    from sim.scenario import e2_scenario
    sc = e2_scenario(use_phi7_weight=True)
    X = np.zeros(12); X[:6] = np.asarray(sc.targets).ravel()   # 목표 = 편대 형상
    for i in range(3):
        assert np.isclose(phi_robot(X, i, sc)[6], 2.0, atol=1e-3)   # deg(i)=2, 각 exp(0)=1

def test_phi8_leader_repulsion():
    """φ⁸: 상대좌표 원점(리더)에 접근할수록 증가, 기준은 leader_standoff 표면거리."""
    sc = Scenario(phi_terms=(1, 2, 3, 4, 5, 6, 8), leader_standoff=0.47, robot_radius=0.25)
    far = _state([(3.0, 0), (0, 7), (7, -4)], [(0.1, 0)] * 3)
    near = _state([(0.5, 0), (0, 7), (7, -4)], [(0.1, 0)] * 3)
    assert phi_robot(near, 0, sc)[6] > phi_robot(far, 0, sc)[6]
    d = 0.5 - 0.47                                          # 표면거리 = |pos| - standoff
    assert np.isclose(phi_robot(near, 0, sc)[6], np.log(1 + 10 * np.exp(-20 * d)))

def test_phi8_absent_when_not_in_phi_terms():
    """순수 가산: φ⁸ 미포함 시나리오(E1~E4)의 z2는 불변 — 재검증 불요의 근거."""
    sc = Scenario()                                          # phi_terms=(1..6), leader_standoff=0.0
    X = _state([(0.1, 0), (0, 7), (7, -4)], [(0.1, 0)] * 3)  # 원점 근접이라도 영향 없어야
    assert z2_vector(X, sc).shape == (18,)
    assert len(phi_robot(X, 0, sc)) == 6

def test_phi1_v_floor_bounds_gradient_and_is_inert_by_default():
    """φ¹ 분모 하한: 기본 0.0이면 기존과 동일, >0이면 |v|→0 발산을 유계화한다.

    기본이 0.0이라 기존 수치 캠페인(E1~E4)의 z2는 비트 단위로 불변 — 순수 가산의 근거."""
    from sim.experiment import analytic_c
    base = Scenario(phi_terms=(1, 2, 3, 4, 5, 6))
    slow = _state([(-7, 3), (0, 7), (7, -4)], [(1e-4, 0)] * 3)   # 거의 정지 = 발산 조건
    assert np.allclose(phi_robot(slow, 0, base), phi_robot(slow, 0, Scenario(phi_terms=(1, 2, 3, 4, 5, 6), phi1_v_floor=0.0)))
    guarded = Scenario(phi_terms=(1, 2, 3, 4, 5, 6), phi1_v_floor=0.05)
    c_raw = analytic_c(slow, base, np.tile(base.w_robot, 3))
    c_grd = analytic_c(slow, guarded, np.tile(guarded.w_robot, 3))
    assert np.max(np.abs(c_grd)) < np.max(np.abs(c_raw)) / 10      # 발산이 실제로 눌린다

def test_phi1_v_floor_inactive_at_normal_speed():
    """평시 상대속도(0.05~0.4 m/s 대역 위)에서는 하한이 걸리지 않아 결과가 같아야 한다."""
    fast = _state([(-7, 3), (0, 7), (7, -4)], [(0.3, 0.2)] * 3)   # |v| = 0.36 > floor
    a = phi_robot(fast, 0, Scenario(phi_terms=(1, 2, 3, 4, 5, 6)))
    b = phi_robot(fast, 0, Scenario(phi_terms=(1, 2, 3, 4, 5, 6), phi1_v_floor=0.05))
    assert np.allclose(a, b)

def test_phi9_is_negative_squared_error_and_zero_at_target():
    sc = Scenario(phi_terms=(1, 2, 3, 4, 5, 6, 9))
    at = _state([sc.targets[0], (0, 7), (7, -4)], [(0.1, 0)] * 3)
    assert np.isclose(phi_robot(at, 0, sc)[6], 0.0)              # 목표점에서 0
    off = np.array(sc.targets[0]) + np.array([0.3, -0.4])       # 오차 0.5
    st = _state([off, (0, 7), (7, -4)], [(0.1, 0)] * 3)
    assert np.isclose(phi_robot(st, 0, sc)[6], -0.25)            # -|err|^2

def test_phi9_gradient_is_linear_in_error():
    """∂φ⁹/∂pos = -2·r* — 오차에 선형. 이것이 QP에 결핍됐던 '크기 있는 복원력'이다."""
    sc = Scenario(phi_terms=(9,), w_robot=np.array([1.0]))
    g = []
    for d in (0.5, 1.0, 2.0):
        off = np.array(sc.targets[0]) + np.array([d, 0.0])
        X = _state([off, (0, 7), (7, -4)], [(0.1, 0)] * 3)
        h = 1e-5
        Xp = X.copy(); Xp[0] += h
        g.append((phi_robot(Xp, 0, sc)[0] - phi_robot(X, 0, sc)[0]) / h)
    assert np.allclose(g, [-1.0, -2.0, -4.0], atol=1e-3)         # -2d, 선형
    assert abs(g[2] / g[0] - 4.0) < 1e-2                          # 4배 오차 -> 4배 복원력

def test_phi9_absent_when_not_in_phi_terms():
    """순수 가산: 기존 수치 캠페인(E1~E4) 시나리오의 z2는 비트 불변."""
    sc = Scenario()                                               # phi_terms=(1..6)
    X = _state([(3.0, 1.0), (0, 7), (7, -4)], [(0.1, 0)] * 3)
    assert len(phi_robot(X, 0, sc)) == 6 and z2_vector(X, sc).shape == (18,)
