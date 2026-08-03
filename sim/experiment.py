from dataclasses import dataclass
import numpy as np
from .dynamics import ab_matrices
from .koopman import build_zeta, build_zeta_linear, zeta_dim
from .rls import RLS
from .scenario import table1_input
from .utility import z2_vector, surface_distances

@dataclass
class Phase1Result:
    rls: object; X_final: np.ndarray; X_log: np.ndarray; U_log: np.ndarray
    eps_log: np.ndarray; eps_a_log: np.ndarray
    viol_flags: dict; trP: float; zeta_rank: int; zPz_log: np.ndarray

def operating_point(sc):
    z10 = sc.desired_state()
    return z10, z2_vector(z10, sc)

def _zeta(sc, model, X, z2, U, z10, z20):
    if model == "linear":
        return build_zeta_linear(X, z2, U)
    return build_zeta(X - z10, z2 - z20, U)

def make_rls(sc, model):
    dim = (4 * sc.n_robots + sc.m + 2 * sc.n_robots) if model == "linear" else zeta_dim(sc.n_robots, sc.m)
    return RLS(dim, sc.m, sc.rls_rho, sc.p0)

def run_phase1(sc, model, input_fn=table1_input):
    A, B = ab_matrices(sc.n_robots, sc.dt)
    z10, z20 = operating_point(sc)
    rls = make_rls(sc, model)
    X = sc.phase1_init_state()
    ks, m = sc.ks, sc.m
    X_log = np.zeros((ks + 1, 4 * sc.n_robots)); X_log[0] = X
    U_log = np.zeros((ks, 2 * sc.n_robots))
    eps_log = np.zeros((ks, m)); eps_a_log = np.zeros((ks, m)); zPz_log = np.zeros(ks)
    viol = {"robot": False, "wall": False}
    zeta_samples = []
    for k in range(1, ks + 1):                      # 1-based (스펙)
        U = input_fn(k)
        z2 = z2_vector(X, sc)
        zeta = _zeta(sc, model, X, z2, U, z10, z20)
        X_next = A @ X + B @ U
        z2_next = z2_vector(X_next, sc)
        zPz_log[k - 1] = zeta @ rls.P @ zeta
        eps, eps_a = rls.update(zeta, z2_next)
        eps_log[k - 1] = eps
        eps_a_log[k - 1] = eps_a if eps_a is not None else np.nan
        if k % sc.p_reset[model] == 0:
            rls.reset_P()                            # linear 130 / bilinear 75 (:1448)
        dr, dw = surface_distances(X_next, sc)
        viol["robot"] = viol["robot"] or dr <= 0
        viol["wall"] = viol["wall"] or dw <= 0
        if k % 3 == 0:
            zeta_samples.append(zeta)                # 랭크 계산용 표본 (메모리 절약)
        X_log[k] = X_next; U_log[k - 1] = U
        X = X_next
    rank = int(np.linalg.matrix_rank(np.array(zeta_samples)))
    return Phase1Result(rls, X, X_log, U_log, eps_log, eps_a_log, viol,
                        float(np.trace(rls.P)), rank, zPz_log)

def open_loop_eval(sc, model, rls, X_log, U_log, start, horizon):
    """frozen Θ̂로 z(2)를 재귀 예측 (z(1)은 참값 — 동역학 정확). 반환: |예측-참| (horizon, m)."""
    z10, z20 = operating_point(sc)
    z2_hat = z2_vector(X_log[start], sc)
    err = np.zeros((horizon, sc.m))
    for t in range(horizon):
        k = start + t
        zeta = _zeta(sc, model, X_log[k], z2_hat, U_log[k], z10, z20)
        z2_hat = rls.theta.T @ zeta
        err[t] = np.abs(z2_hat - z2_vector(X_log[k + 1], sc))
    return err
