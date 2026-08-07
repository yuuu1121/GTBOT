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

def test_qp_reg_matches_lp_sign_when_reg_small():
    """λ→작음: 폐형식 해가 박스에 붙어 LP 꼭짓점과 같은 부호·크기가 된다."""
    c = np.array([0.4, -1.2, 0.0, 2.5, -0.05, 0.9])
    lp, _ = solve_input(c, -0.3, 0.3)
    qp, _ = solve_input(c, -0.3, 0.3, reg=1e-6)
    nz = np.abs(c) > 1e-9
    assert np.allclose(qp[nz], lp[nz])

def test_qp_reg_is_linear_when_reg_large():
    """λ→큼: 박스에 안 닿아 u = c/λ 선형(릴레이 아님 = 크기 정보 보존)."""
    c = np.array([0.4, -1.2, 0.0, 2.5, -0.05, 0.9])
    reg = 1000.0
    qp, status = solve_input(c, -0.3, 0.3, reg=reg)
    assert status == "ok"
    assert np.allclose(qp, c / reg)

def test_qp_reg_respects_box():
    c = np.array([50.0, -50.0, 0.1])
    qp, _ = solve_input(c, -0.3, 0.3, reg=1.0)
    assert np.all(qp >= -0.3) and np.all(qp <= 0.3)
    assert qp[0] == 0.3 and qp[1] == -0.3

def test_qp_reg_zero_is_exactly_legacy_lp():
    """input_reg=0이면 기존 경로와 비트 동일 — E1~E4 수치 캠페인 무영향의 근거."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        c = rng.normal(0, 2, 6)
        assert np.allclose(solve_input(c, -4.0, 3.0)[0], solve_input(c, -4.0, 3.0, reg=0.0)[0])

def test_qp_reg_nan_guard():
    U, status = solve_input(np.array([np.nan, 1.0]), -0.3, 0.3, reg=10.0)
    assert status == "nan_guard" and np.allclose(U, 0.0)
