import numpy as np
from sim.control import input_objective, solve_input
from sim.koopman import build_zeta, zeta_dim

def test_c_extraction_matches_full_eval():
    """c·U+const ≡ wᵀΘ̂ᵀζ(U) — bilinear LP에서 가장 틀리기 쉬운 지점 (스펙 게이트 0-4)."""
    rng = np.random.default_rng(3)
    z1d, z2d = rng.normal(size=12), rng.normal(size=18)
    theta = rng.normal(size=(zeta_dim(3, 18), 18)); w = rng.normal(size=18)
    c, const = input_objective(theta, z1d, z2d, w, 6)
    for _ in range(5):
        U = rng.normal(size=6)
        full = w @ (theta.T @ build_zeta(z1d, z2d, U))
        assert np.isclose(c @ U + const, full)

def test_solve_is_bang_bang():
    c = np.array([1.0, -2.0, 0.0, 3.0])
    U, status = solve_input(c, -4.0, 3.0)
    assert status == "ok"
    assert np.allclose(U, [3.0, -4.0, 0.0, 3.0])   # c>0→u_max, c<0→u_min, c≈0→0

def test_nan_guard():
    U, status = solve_input(np.array([np.nan, 1.0]), -4.0, 3.0)
    assert status == "nan_guard" and np.allclose(U, 0.0)
