import numpy as np
from sim.scenario import e1_scenario
from sim.experiment import run_phase1, operating_point

def test_phase1_smoke_no_violation_and_finite():
    sc = e1_scenario()
    res = run_phase1(sc, "bilinear")
    assert res.eps_log.shape == (300, 18) and np.isfinite(res.eps_log).all()
    assert res.viol_flags["robot"] is False and res.viol_flags["wall"] is False  # 1-based면 무충돌 (:1216)

def test_operating_point_phi_at_target():
    sc = e1_scenario()
    z10, z20 = operating_point(sc)
    assert np.allclose(z10[:6], np.asarray(sc.targets).ravel())
    assert np.isclose(z20[2], 1.0) and z20[0] == 0.0   # 목표에서 phi3=1, phi1=0(속도 0 가드)

def test_phase1_episodic_resets_state():
    import dataclasses
    from sim.scenario import e3_input, IMPROVED_EPISODES, improved_episode_state
    sc = dataclasses.replace(e1_scenario(), ks=150)
    eps = [improved_episode_state(i) for i in range(len(IMPROVED_EPISODES))]
    res = run_phase1(sc, "linear", input_fn=lambda k: e3_input(k, 3), episodes=eps, p_reset=False)
    assert np.allclose(res.X_log[0], eps[0])
    assert np.allclose(res.X_log[75], eps[1])      # k=76 직전 리셋이 X_log[75]에 반영
    assert np.isfinite(res.eps_log).all()

def test_phase1_episode_len_param():
    import dataclasses
    from sim.scenario import e3_input, IMPROVED_EPISODES, improved_episode_state
    sc = dataclasses.replace(e1_scenario(), ks=100)
    eps = [improved_episode_state(i) for i in range(len(IMPROVED_EPISODES))]
    res = run_phase1(sc, "linear", input_fn=lambda k: e3_input(k, 3),
                      episodes=eps, episode_len=25, p_reset=False)
    assert np.allclose(res.X_log[25], eps[1])      # k=26 직전 리셋이 X_log[25]에 반영 (25스텝 주기)

def test_analytic_c_points_toward_target():
    import numpy as np, dataclasses
    from sim.scenario import e1_scenario
    from sim.experiment import analytic_c
    sc = dataclasses.replace(e1_scenario(), w_robot=np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    # 로봇1이 목표 (-4.5, 0)의 오른쪽 (-1.5, 0)에 정지 — phi3만 보상하면 -x 가속이 유리
    X = np.zeros(12); X[:6] = [-1.5, 0.0, 3.0, 4.0, 3.0, -4.0]   # 로봇2·3은 목표 위(기여 0 근방)
    c = analytic_c(X, sc, sc.w_full)
    assert c[0] < 0        # ax1 음수 방향(목표 쪽)이 목적함수 증가
