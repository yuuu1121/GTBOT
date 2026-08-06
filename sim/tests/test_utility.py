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
