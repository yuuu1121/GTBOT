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
