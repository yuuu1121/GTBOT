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
    return build_zeta(X - z10, z2 - z20, U, reduced=sc.reduced_lifting)

def make_rls(sc, model):
    dim = (4 * sc.n_robots + sc.m + 2 * sc.n_robots) if model == "linear" \
        else zeta_dim(sc.n_robots, sc.m, reduced=sc.reduced_lifting)
    return RLS(dim, sc.m, sc.rls_rho, sc.p0)

def run_phase1(sc, model, input_fn=table1_input, episodes=None, p_reset=True):
    A, B = ab_matrices(sc.n_robots, sc.dt)
    z10, z20 = operating_point(sc)
    rls = make_rls(sc, model)
    X = sc.phase1_init_state() if episodes is None else episodes[0]
    ks, m = sc.ks, sc.m
    X_log = np.zeros((ks + 1, 4 * sc.n_robots)); X_log[0] = X
    U_log = np.zeros((ks, 2 * sc.n_robots))
    eps_log = np.zeros((ks, m)); eps_a_log = np.zeros((ks, m)); zPz_log = np.zeros(ks)
    viol = {"robot": False, "wall": False}
    zeta_samples = []
    for k in range(1, ks + 1):                      # 1-based (스펙)
        if episodes is not None and k > 1 and (k - 1) % 75 == 0:
            X = episodes[((k - 1) // 75) % len(episodes)]      # 에피소드 경계 순간이동 리셋
            X_log[k - 1] = X
        U = input_fn(k)
        z2 = z2_vector(X, sc)
        zeta = _zeta(sc, model, X, z2, U, z10, z20)
        X_next = A @ X + B @ U
        z2_next = z2_vector(X_next, sc)
        zPz_log[k - 1] = zeta @ rls.P @ zeta
        eps, eps_a = rls.update(zeta, z2_next)
        eps_log[k - 1] = eps
        eps_a_log[k - 1] = eps_a if eps_a is not None else np.nan
        if p_reset and k % sc.p_reset[model] == 0:
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

def frozen_1step_eval(sc, model, rls, X_log, U_log):
    """동결 Θ̂로 teacher-forced 1-step 예측 (z(2)는 매 k 참값 — 재귀 아님). 반환: 오차 (ks, m)."""
    z10, z20 = operating_point(sc)
    ks = U_log.shape[0]
    err = np.zeros((ks, sc.m))
    for k in range(ks):
        z2 = z2_vector(X_log[k], sc)
        zeta = _zeta(sc, model, X_log[k], z2, U_log[k], z10, z20)
        z2_pred = rls.theta.T @ zeta
        err[k] = z2_pred - z2_vector(X_log[k + 1], sc)
    return err

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

@dataclass
class Phase2Result:
    X_log: np.ndarray; U_log: np.ndarray
    min_robot_surf: float; min_wall_surf: float
    final_target_dists: np.ndarray; min_target_dists: np.ndarray; lp_events: list

def run_phase2(sc, rls, model, init_pos, targets, iters):
    """제어 평가 (:1450~1453): 초기·목표 재설정, 동작점 재계산, LP 입력, RLS 갱신 지속.
    reached 판정은 지평 내 최소 거리 기준(스펙 "제어 지평 내 0.5m 이내") — 그리디 1-step LP는
    감속항이 없어 목표를 지나쳐 관통하는 것이 정상 거동이므로 최종 스텝만으로는 과소평가된다."""
    import dataclasses
    from .control import input_objective, solve_input
    sc2 = dataclasses.replace(sc, targets=np.asarray(targets))
    A, B = ab_matrices(sc2.n_robots, sc2.dt)
    z10, z20 = operating_point(sc2)
    n_in = 2 * sc2.n_robots
    tgt_arr = np.asarray(targets)
    X = np.zeros(4 * sc2.n_robots); X[:n_in] = np.asarray(init_pos).ravel()
    X_log = np.zeros((iters + 1, 4 * sc2.n_robots)); X_log[0] = X
    U_log = np.zeros((iters, n_in))
    min_dr, min_dw, events = np.inf, np.inf, []
    min_target_dists = np.linalg.norm(X[:n_in].reshape(-1, 2) - tgt_arr, axis=1)
    for k in range(1, iters + 1):
        z2 = z2_vector(X, sc2)
        if model == "bilinear":
            c, _ = input_objective(rls.theta, X - z10, z2 - z20, sc2.w_full, n_in, reduced=sc2.reduced_lifting)
        else:   # linear도 U-affine — 평가 추출 동일 패턴 (1회 사용, 인라인)
            base = rls.theta.T @ build_zeta_linear(X, z2, np.zeros(n_in))
            eye = np.eye(n_in)
            c = np.array([sc2.w_full @ (rls.theta.T @ build_zeta_linear(X, z2, eye[l]) - base)
                          for l in range(n_in)])
        U, status = solve_input(c, sc2.u_min, sc2.u_max)
        if status != "ok":
            events.append((k, status))
        zeta = _zeta(sc2, model, X, z2, U, z10, z20)
        X_next = A @ X + B @ U
        rls.update(zeta, z2_vector(X_next, sc2))       # 갱신 지속 (:1453)
        dr, dw = surface_distances(X_next, sc2)
        min_dr, min_dw = min(min_dr, dr), min(min_dw, dw)
        min_target_dists = np.minimum(min_target_dists,
                                       np.linalg.norm(X_next[:n_in].reshape(-1, 2) - tgt_arr, axis=1))
        X_log[k] = X_next; U_log[k - 1] = U
        X = X_next
    pos = X[:n_in].reshape(-1, 2)
    dists = np.linalg.norm(pos - tgt_arr, axis=1)
    return Phase2Result(X_log, U_log, float(min_dr), float(min_dw), dists, min_target_dists, events)
