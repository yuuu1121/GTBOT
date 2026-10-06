import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'gtbot_formation'))
from gtbot_formation.relative_state import make_scenario
from sim.lifted import PolyPhiDict, LiftedModel, zeta, fit_lifted, identification_data, build_mpc_model, in_trust_region
from sim.experiment import analytic_c
from sim.control import solve_input
from sim.dynamics import ab_matrices
from sim.experiment import operating_point
from sim.utility import z2_vector

SC = make_scenario()


def test_dictionary_layout_and_exact_utility():
    D = PolyPhiDict(SC, deg=3, phi_deg=2)
    assert D.n == 1 + 3 * 34 + 27 + 3 * 9 * 14 == 508   # [1; poly3×3; φ27; φ_r⊗poly2×3]
    X = operating_point(SC)[0] + 0.1
    assert abs(D.g @ D.lift(X) - SC.w_full @ z2_vector(X, SC)) < 1e-12   # J = gᵀz 정확


def test_model_blocks_match_theta():
    """K·L·N 분해가 Θᵀζ 와 일치하고, c_1step 이 유한차분과 맞는다."""
    D = PolyPhiDict(SC, deg=1, phi_deg=0); rng = np.random.default_rng(0)
    theta = rng.normal(size=(D.n + 6 + D.n * 6, D.n)); M = LiftedModel(D, theta)
    z, u = rng.normal(size=D.n), rng.normal(size=6)
    assert np.allclose(M.step(z, u), theta.T @ zeta(z, u))
    e = 1e-6
    cf = [(M.g @ M.step(z, e * np.eye(6)[l]) - M.g @ M.step(z, np.zeros(6))) / e for l in range(6)]
    assert np.allclose(M.c_1step(z), cf, atol=1e-6)


def test_mpc_gradient_matches_finite_difference():
    D = PolyPhiDict(SC, deg=1, phi_deg=0); rng = np.random.default_rng(1)
    theta = 0.05 * rng.normal(size=(D.n + 6 + D.n * 6, D.n)); M = LiftedModel(D, theta)
    z0 = rng.normal(size=D.n); H = 3; v = rng.uniform(-0.3, 0.3, 6 * H); up = rng.uniform(-0.3, 0.3, 6)
    def J(v):
        us = v.reshape(H, 6); z = z0; s = 0.0
        for u in us: z = M.step(z, u); s += M.g @ z
        return s - 0.5 * float(v @ v) - 1.5 * float((np.diff(np.vstack([up[None], us]), axis=0) ** 2).sum())
    # mpc() 내부 f 를 직접 뽑기 위해 maxiter=0 으로 한 번 호출해 함수값·기울기만 검산
    from scipy.optimize import approx_fprime
    import types
    f = None
    def capture(fun, x0, **kw): nonlocal f; f = fun; return types.SimpleNamespace(x=x0)
    import sim.lifted as L; orig = L.minimize; L.minimize = capture
    try: M.mpc(z0, H, v.reshape(H, 6), 1.0, -0.3, 0.3, rho=3.0, u_prev=up)
    finally: L.minimize = orig
    val, grad = f(v)
    assert abs(val + J(v)) < 1e-9
    assert np.allclose(grad, -approx_fprime(v, J, 1e-7), atol=1e-4)


def test_koopman_mpc_settles_on_nominal_plant():
    """사전·식별·MPC 전체(노드 control() 과 같은 구조: 신뢰 영역 밖은 analytic): 이상 이중적분기에서 초기 오차
    0.3 m 와 1.0 m(초기 시드 3 = 장벽 안쪽 출발) 둘 다 편대를 붙들고 정지한다(적합 ~10 s)."""
    M = build_mpc_model(SC); A, B = ab_matrices(3, SC.dt); z10, _ = operating_point(SC)
    for init, seed in ((0.3, 0), (1.0, 3)):
        X = z10 + np.concatenate([np.random.default_rng(seed).normal(0, init, 6), np.zeros(6)]); plan = np.zeros((3, 6)); u_prev = np.zeros(6)
        for _ in range(800):
            if in_trust_region(X, SC):
                plan = M.mpc(M.D.lift(X), 3, np.vstack([plan[1:], plan[-1:]]), SC.input_reg, SC.u_min, SC.u_max, rho=3.0, u_prev=u_prev)
            else:
                plan[:] = solve_input(analytic_c(X, SC, SC.w_full), SC.u_min, SC.u_max, reg=SC.input_reg)[0]
            u_prev = plan[0]; X = A @ X + B @ u_prev
        err = np.linalg.norm(X[:6].reshape(3, 2) - SC.targets, axis=1).max()
        assert err < 0.1 and np.abs(X[6:]).max() < 0.02, (init, err)
