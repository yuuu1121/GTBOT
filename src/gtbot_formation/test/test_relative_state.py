import numpy as np
from gtbot_formation.relative_state import assemble, make_scenario, OFFSETS, FORMATION

def test_assemble_layout():                 # sim/utility._pos_vel 레이아웃: [pos(2N); vel(2N)]
    L = (np.array([10.0, 5.0]), np.array([0.2, 0.0]))
    F = [(np.array([8.0, 6.0]), np.array([0.2, 0.1])),
         (np.array([8.0, 4.0]), np.array([0.0, 0.0])),
         (np.array([6.3, 5.0]), np.array([0.2, 0.0]))]
    X = assemble(L, F)
    assert X.shape == (12,)
    assert np.allclose(X[:2], [-2.0, 1.0])          # p1 - pL
    assert np.allclose(X[6:8], [0.0, 0.1])          # v1 - vL
    assert np.allclose(X[8:10], [-0.2, 0.0])        # v2 - vL

def test_offsets_form_triangle():
    O = np.array(OFFSETS)
    for (i, j), d in FORMATION.items():
        assert abs(np.linalg.norm(O[i] - O[j]) - d) < 0.01

def test_centered_formation_geometry():
    O = np.array(OFFSETS)
    assert np.allclose(np.linalg.norm(O, axis=1), 0.866, atol=0.001)   # platform 중심 반지름
    assert set(FORMATION.values()) == {1.5}

def test_scenario_stonefish_scale():
    sc = make_scenario()
    assert sc.n_robots == 3 and sc.phi_terms == (1, 2, 3, 4, 5, 6, 7, 8, 9)
    assert sc.leader_standoff == 0.47 and sc.m == 27   # φ⁸·φ⁹ 추가로 21 -> 27
    assert len(sc.w_robot) == 9 and sc.w_robot[-1] == 50.0   # φ⁹ 가중(프로브 3점에서 채택)
    assert sc.dt == 0.05 and sc.u_max == 0.3
    assert sc.robot_radius == 0.5   # 물리 0.22가 아니라 LiDAR 트랙 분리한계(중심간 0.9 m)를 덮는 값
    assert np.allclose(sc.targets, OFFSETS)
    assert sc.formation == FORMATION and sc.sigma == 1.0
