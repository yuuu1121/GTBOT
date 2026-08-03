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
