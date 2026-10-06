from dataclasses import dataclass, replace
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

def run_phase1(sc, model, input_fn=table1_input, episodes=None, p_reset=True, episode_len=75):
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
        if episodes is not None and k > 1 and (k - 1) % episode_len == 0:
            X = episodes[((k - 1) // episode_len) % len(episodes)]     # 에피소드 경계 순간이동 리셋
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

def analytic_c(X, sc, w_full, h=1e-4):
    """유한차분 참 1-step 그래디언트: c_l = [w·z2(AX+B·h·e_l) − w·z2(AX)]/h (해석적 팔 — 모델 불요)."""
    A, B = ab_matrices(sc.n_robots, sc.dt)
    n_in = 2 * sc.n_robots
    base_X = A @ X
    base = w_full @ z2_vector(base_X, sc)
    c = np.zeros(n_in)
    for l in range(n_in):
        c[l] = (w_full @ z2_vector(base_X + B[:, l] * h, sc) - base) / h
    return c

def drop_phi(sc, term):
    """효용에서 φ^term 을 뺀 시나리오(phi_terms·w_robot 동기)."""
    keep = [i for i, t in enumerate(sc.phi_terms) if t != term]
    return replace(sc, phi_terms=tuple(sc.phi_terms[i] for i in keep), w_robot=sc.w_robot[keep])

# model 팔(원논문 형태: φ 사전만, 전체 bilinear ζ, 데이터 적합 Θ, 1-step QP)의 적합 설정.
# φ⁴(목표점 위치·속도 폭 0.05 m 스파이크, 가중 2)는 뺀다 — 해석적 팔 성능은 그대로(정상 0.079 m)이고
# 사전 불변성 I_C 0.136→0.029(tools/koopman_consistency_index.py). 궤적 위 재적합 3회 누적:
# 적합 시드 3개 × 사각 리더 시드 2개에서 정상 0.118~0.136 m(해석적 0.079), 단일 재적합은
# 0.14~0.23 m로 흔들렸다(2026-10-06 스윕). 적합에 약 40 s.
MODEL_ARM_FIT = dict(n=3000, refits=3, episodes=60)

def model_scenario(sc):
    return drop_phi(sc, 4)

def nominal_theta(sc, n=6000, sp=0.3, sv=0.15, lam=0.1, refits=1, seed=0, episodes=30):
    """공칭 bilinear Θ: 동작점 둘레 표본에 대한 배치 정칙화 최소자승(원논문 식 3.36의 비반복형).

    원논문 RLS는 Θ(k0)=Θ0를 '선택된 초기 추정'으로 둔다(식 3.37 아래). Θ0=0 + 워밍업 수백 표본은
    미결정계라 제어 불가였다(sim-results '개선 식별 5라운드'). 여기서는 공칭 모델(효용함수 +
    이중적분기)로 동작 영역을 직접 표본화해 Θ를 정한다.
    - U는 ±쌍으로 넣는다: U-홀수 성분(= 제어가 읽는 결합 블록)이 상태 전용 잔차와 직교 분리된다.
    - refits: Θ 제어 + 디더로 공칭 폐루프를 굴려 방문 상태에서 재적합(가우시안 표본은 장벽 안쪽
      처럼 폐루프가 가지 않는 곳에 질량을 써서 평형 편향이 컸다: 정착 오차 0.40 -> 0.17 m).
      원논문의 '궤적 위 식별'에 해당. 방문 표본은 라운드마다 누적하고(episodes×200틱/라운드),
      광역 표본 n/4를 닻으로 섞는다 — 누적 전에는 적합 시드마다 0.14~0.23 m로 흔들렸다.
    - lam: 1e-6(종전 p0=100 상당)이면 미가진 방향이 풀려 평형 위치가 시드마다 크게 흔들린다.
    원 사전(sc.grad_lift=False)의 한계(실측, 잡음 0.02·지연 0.15 s, 적합 시드 4개): 발산은 없으나
    edge 오차 중앙 0.23~0.32 m·최대 0.24~0.39 m로 0.3 m 게이트를 넘고 시드에 민감하다(해석적
    팔 0.155/0.176). sc.grad_lift=True면 0.149/0.192로 해석적 팔과 같아진다 — 이때는 refits=0.
    """
    from .control import input_objective, solve_input
    rng = np.random.default_rng(seed)
    A, B = ab_matrices(sc.n_robots, sc.dt)
    z10, z20 = operating_point(sc)
    n_in, n_pos = 2 * sc.n_robots, 2 * sc.n_robots

    def sample(k, sp_, sv_):
        return [z10 + np.concatenate([rng.normal(0, sp_, n_pos), rng.normal(0, sv_, n_pos)])
                for _ in range(k)]

    def fit(Xs):
        Z, Y = [], []
        for X in Xs:
            U, z2 = rng.uniform(sc.u_min, sc.u_max, n_in), z2_vector(X, sc)
            for s in (1.0, -1.0):
                Z.append(_zeta(sc, "bilinear", X, z2, s * U, z10, z20))
                Y.append(z2_vector(A @ X + B @ (s * U), sc))
        Z, Y = np.array(Z), np.array(Y)
        return np.linalg.solve(Z.T @ Z + lam * np.eye(Z.shape[1]), Z.T @ Y)

    theta = fit(sample(n, sp, sv))
    visited = []                                    # 재적합 궤적 표본은 라운드마다 누적한다(시드 분산 완화)
    for _ in range(refits):
        for X in sample(episodes, 0.5, 0.05):
            for _ in range(200):
                c, _ = input_objective(theta, X - z10, z2_vector(X, sc) - z20, sc.w_full, n_in,
                                       reduced=sc.reduced_lifting)
                U, _ = solve_input(c, sc.u_min, sc.u_max, reg=sc.input_reg)
                X = A @ X + B @ np.clip(U + rng.uniform(-0.1, 0.1, n_in), sc.u_min, sc.u_max)
                if np.abs(X[:n_pos] - z10[:n_pos]).max() > 5.0:
                    break
                visited.append(X)
        theta = fit(sample(n // 4, sp, sv) + visited)   # 광역 표본은 안정성 닻으로 유지
    return theta

@dataclass
class Phase2Result:
    X_log: np.ndarray; U_log: np.ndarray
    min_robot_surf: float; min_wall_surf: float
    final_target_dists: np.ndarray; min_target_dists: np.ndarray; lp_events: list

def run_phase2(sc, rls, model, init_pos, targets, iters, controller="model"):
    """제어 평가 (:1450~1453): 초기·목표 재설정, 동작점 재계산, LP 입력, RLS 갱신 지속.
    reached 판정은 지평 내 최소 거리 기준(스펙 "제어 지평 내 0.5m 이내") — 그리디 1-step LP는
    감속항이 없어 목표를 지나쳐 관통하는 것이 정상 거동이므로 최종 스텝만으로는 과소평가된다.
    controller="analytic": 유한차분 참 1-step 그래디언트로 c 산출, rls 불요(None 허용, ζ·갱신 스킵)."""
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
        if controller == "analytic":
            c = analytic_c(X, sc2, sc2.w_full)
        elif model == "bilinear":
            c, _ = input_objective(rls.theta, X - z10, z2 - z20, sc2.w_full, n_in, reduced=sc2.reduced_lifting)
        else:   # linear도 U-affine — 평가 추출 동일 패턴 (1회 사용, 인라인)
            base = rls.theta.T @ build_zeta_linear(X, z2, np.zeros(n_in))
            eye = np.eye(n_in)
            c = np.array([sc2.w_full @ (rls.theta.T @ build_zeta_linear(X, z2, eye[l]) - base)
                          for l in range(n_in)])
        # QP 정규화를 sim 하네스에서도 재현 가능하게 배선(M-1). 기본 input_reg=0.0이면
        # 기존 LP 경로와 비트 동일이라 E1~E4 수치 캠페인은 무영향 — 기존 회귀가 증거다.
        U, status = solve_input(c, sc2.u_min, sc2.u_max, reg=sc2.input_reg)
        if status != "ok":
            events.append((k, status))
        if rls is not None and controller == "model":
            zeta = _zeta(sc2, model, X, z2, U, z10, z20)
        X_next = A @ X + B @ U
        if rls is not None and controller == "model":
            rls.update(zeta, z2_vector(X_next, sc2))   # 갱신 지속 (:1453)
        dr, dw = surface_distances(X_next, sc2)
        min_dr, min_dw = min(min_dr, dr), min(min_dw, dw)
        min_target_dists = np.minimum(min_target_dists,
                                       np.linalg.norm(X_next[:n_in].reshape(-1, 2) - tgt_arr, axis=1))
        X_log[k] = X_next; U_log[k - 1] = U
        X = X_next
    pos = X[:n_in].reshape(-1, 2)
    dists = np.linalg.norm(pos - tgt_arr, axis=1)
    return Phase2Result(X_log, U_log, float(min_dr), float(min_dw), dists, min_target_dists, events)
