import numpy as np
from sim.rls import RLS

def test_posterior_identity():
    """εa = ε·ρ/(ρ+ζᵀPζ) — RLS 구현 정합성 (architect 리뷰 항등식, 잔차<1e-10)."""
    rng = np.random.default_rng(1)
    r = RLS(12, 4, rho=1e-4, p0=100.0)
    for _ in range(20):
        zeta, y = rng.normal(size=12), rng.normal(size=4)
        m2 = 1e-4 + zeta @ (r.P @ zeta)          # 갱신 전 P 사용
        eps, eps_a = r.update(zeta, y)
        assert np.max(np.abs(eps_a - eps * 1e-4 / m2)) < 1e-10

def test_recovers_synthetic_linear_system():
    rng = np.random.default_rng(2)
    theta_true = rng.normal(size=(8, 3))
    r = RLS(8, 3, rho=1e-4, p0=100.0)
    for _ in range(300):
        zeta = rng.normal(size=8)
        r.update(zeta, theta_true.T @ zeta)
    assert np.max(np.abs(r.theta - theta_true)) < 1e-6

def test_nan_guard_skips_update():
    r = RLS(4, 2)
    theta_before = r.theta.copy()
    eps, eps_a = r.update(np.array([1.0, np.nan, 0, 0]), np.zeros(2))
    assert eps_a is None and np.array_equal(r.theta, theta_before)
