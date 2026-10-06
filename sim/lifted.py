"""불변성 보강 사전 + 입력-bilinear 리프팅 모델 + Koopman MPC (2026-10-06, 설명 노트 §12).

사전 z(X) = [1; 로봇별 단항식(차수 ≤ deg); φ(X); φ_r ⊗ 로봇별 단항식(차수 ≤ phi_deg)]
  - 단항식: 이중적분기는 선형이라 다항식 공간이 Koopman 불변(차수가 닫힘) — 다스텝 예측에 오차가 쌓이지 않는다.
  - φ: 효용항 그대로 두어 J = w·φ = gᵀz 가 정확하다(투영 오차 0). 단항식만으로는 장벽항 φ⁸(e^{-20d})을
    못 맞춰 J RMS 2.4(표준편차 5.1)로 발산했다.
  - φ⊗단항식: e^{-20d}·v 같은 곱이 있어야 장벽항의 다음 스텝이 사전 안에서 선형으로 닫힌다.
    실측 10스텝 J 예측 RMS(궤적 위 J 표준편차 8.2): φ만 4.1 → +poly3 1.4 → +φ⊗x 1.2 → +φ⊗x² 0.50.
모델 z⁺ = Θᵀζ(z, U), ζ = [z; U; z⊗U] → z⁺ = (K + Σ_l U_l N_l) z + L U (입력-bilinear 표준형).
MPC max Σ_h gᵀz_h − λ/2 Σ|U_h|², 박스 제약, L-BFGS-B. 기울기는 수반(adjoint) 역전파 — 리프팅 재귀만
통과하므로 사전의 미분이 필요 없다. dim 508·H=3 에서 틱당 12~20 ms(20 Hz 예산 50 ms 안).
수치 시뮬(원 효용, 잡음 0.02·지연 0.16 s, 리더 사각): analytic 1-step 정상 0.095/코너 0.138 →
Koopman MPC H=3 0.088~0.100/0.112~0.118(적합 시드 3 × 시뮬 시드 2). 참 모델 MPC 상한 0.066/0.087."""
import itertools
import numpy as np
from scipy.optimize import minimize
from .dynamics import ab_matrices
from .utility import z2_vector
from .experiment import analytic_c, operating_point
from .control import solve_input

POS_SCALE, VEL_SCALE = 0.3, 0.15      # 단항식 입력 정규화(동작 영역 폭) — 고차항 조건수 보호
SAT = 1.5                              # 단항식 좌표 포화 x̃ = SAT·tanh(x/SAT): 자리 오차 ~0.45 m·속도 ~0.22 m/s 밖에서 특징이
                                       # 유계가 돼 3차 사전의 외삽 폭주를 막는다. 안쪽(|x|<1)은 x̃≈x. 이걸로 초기 오차 1.0 m 에서도
                                       # 수렴(0.086 m, analytic 0.096)해 신뢰 영역 폴백을 없앴다. 위치 RBF 5×5 사전(dim 856)은 예측은
                                       # 더 좋았으나(h=5 RMS 0.40 vs 0.68) 틱당 116 ms 라 20 Hz 에 못 들어가 쓰지 않았다.


class PolyPhiDict:
    def __init__(self, sc, deg=3, phi_deg=2):
        self.sc = sc
        self.E = np.array([e for e in itertools.product(range(deg + 1), repeat=4) if 0 < sum(e) <= deg])
        self.Ep = (np.array([e for e in itertools.product(range(phi_deg + 1), repeat=4) if 0 < sum(e) <= phi_deg])
                   if phi_deg else np.zeros((0, 4), int))
        self.n_phi = len(sc.phi_terms); self.n_poly = 1 + sc.n_robots * len(self.E)
        self.n = self.n_poly + sc.n_robots * self.n_phi * (1 + len(self.Ep))
        self.g = np.zeros(self.n); self.g[self.n_poly:self.n_poly + sc.n_robots * self.n_phi] = sc.w_full   # J = w·φ

    def lift(self, X):
        N = self.sc.n_robots; pos, vel = X[:2 * N].reshape(N, 2), X[2 * N:].reshape(N, 2)
        xs = [SAT * np.tanh(np.concatenate([(pos[r] - self.sc.targets[r]) / POS_SCALE, vel[r] / VEL_SCALE]) / SAT) for r in range(N)]
        f = [np.ones(1)] + [np.prod(x[None, :] ** self.E, axis=1) for x in xs]
        phi = z2_vector(X, self.sc); f.append(phi)
        for r in range(N):
            if len(self.Ep):
                f.append(np.outer(phi[r * self.n_phi:(r + 1) * self.n_phi], np.prod(xs[r][None, :] ** self.Ep, axis=1)).ravel())
        return np.concatenate(f)


def zeta(z, u):
    return np.concatenate([z, u, np.outer(z, u).ravel()])


class LiftedModel:
    """Θ → K, L, N(6,n,n). M(u) = K + Σ u_l N_l."""
    def __init__(self, D, theta):
        n = D.n; nu = (theta.shape[0] - n) // (n + 1)      # ζ 길이 n + nu + n·nu
        self.D, self.g, self.n, self.nu = D, D.g, n, nu
        self.K = theta[:n].T; self.L = theta[n:n + nu].T
        self.N = np.ascontiguousarray(theta[n + nu:].reshape(n, nu, n).transpose(1, 2, 0))  # ζ 꼬리 (z⊗U).ravel(): 행 i*nu+l → N[l][:, i]
        self.N6 = self.N.reshape(nu, -1)

    def M_of(self, u):
        return self.K + (u @ self.N6).reshape(self.n, self.n)

    def step(self, z, u):
        return self.M_of(u) @ z + self.L @ u

    def c_1step(self, z):
        """c_l = ∂(gᵀz⁺)/∂U_l = gᵀ(L[:,l] + N_l z) — 1-step QP용."""
        return self.g @ self.L + (self.N @ z) @ self.g

    def mpc(self, z0, H, u0, lam, u_min, u_max, rho=0.0, u_prev=None, maxiter=10):
        """max Σ_h gᵀz_h − λ/2 Σ|U_h|² − ρ/2 Σ|U_h − U_{h−1}|²  (U_{−1} = 직전 발행 입력), 박스 제약.
        ρ(입력 변화 패널티)가 없으면 MPC 는 입력을 62 % 포화·부호 반전 0.89 로 써서 이기는 것이라(수치 시뮬),
        Stonefish 에서는 속도 추정이 채터링을 못 따라가 est 속도 오차 RMS 0.15 로 커졌다(1-step 0.017). ρ=3 이면
        포화 0.02·반전 0.43 으로 analytic(0.11·0.70)보다 매끈하고 코너 이득 −8 %는 남는다."""
        nu = self.nu; up0 = np.zeros(nu) if u_prev is None else u_prev

        def f(v):
            us = v.reshape(H, nu); zs = [z0]; Ms = []
            for h in range(H):
                Ms.append(self.M_of(us[h])); zs.append(Ms[-1] @ zs[-1] + self.L @ us[h])
            du = np.diff(np.vstack([up0[None], us]), axis=0)
            J = sum(self.g @ z for z in zs[1:]) - lam / 2 * float(v @ v) - rho / 2 * float((du ** 2).sum())
            adj = np.zeros(self.n); grad = np.zeros((H, nu))
            for h in range(H - 1, -1, -1):
                adj = self.g + adj
                grad[h] = adj @ self.L + (self.N @ zs[h]) @ adj - lam * us[h] - rho * du[h] + (rho * du[h + 1] if h + 1 < H else 0.0)
                adj = Ms[h].T @ adj
            return -J, -grad.ravel()
        r = minimize(f, u0.ravel(), jac=True, method='L-BFGS-B', bounds=[(u_min, u_max)] * (H * nu),
                     options=dict(maxiter=maxiter))
        return r.x.reshape(H, nu)


MPC_RHO = 3.0                          # 입력 변화 패널티(위 mpc 주석)


def identification_data(sc, inits=(0.3,) * 4 + (0.8,) * 6, n_ticks=500, noise=0.05, n_gauss=1500, sp=0.15, sv=0.08,
                        delay=0.16, seed=0):
    """식별용 상태 표본: 공칭 플랜트에서 analytic 제어 폐루프(잡음 0.05로 흔듦, 리더 정지)를 n_traj번 굴린
    방문 상태 + 동작점 둘레 좁은 가우시안. 가우시안만 쓰면 장벽 안쪽 등 폐루프가 가지 않는 곳에 질량을 써서
    적합이 망가진다(nominal_theta 주석과 같은 교훈). inits: 궤적 초기 흩뿌림 — 0.3 m 4개 + 0.8 m 6개. 0.8 m 궤적은
    Stonefish 워밍업 끝 오차(0.6~0.8 m)를 덮기 위해서다; 포화 좌표(SAT) 없이 넓은 데이터만 넣으면 오히려 0.7 m 에서
    발산했고, 포화 좌표와 함께 넣어야 0.3/0.7/1.0 m 모두 수렴한다."""
    rng = np.random.default_rng(seed); N = sc.n_robots
    A, B = ab_matrices(N, sc.dt); AD, BD = ab_matrices(N, delay); z10, _ = operating_point(sc)
    data = []
    for init in inits:
        X = z10 + np.concatenate([rng.normal(0, init, 2 * N), np.zeros(2 * N)])
        q = [np.zeros(2 * N)] * 3; u_prev = np.zeros(2 * N)
        for _ in range(n_ticks):
            Xd = AD @ (X + rng.normal(0, noise, 4 * N)) + BD @ u_prev; data.append(Xd)
            U, _ = solve_input(analytic_c(Xd, sc, sc.w_full), sc.u_min, sc.u_max, reg=sc.input_reg)
            u_prev = U; q.append(U); X = A @ X + B @ q.pop(0)
    data += [z10 + np.concatenate([rng.normal(0, sp, 2 * N), rng.normal(0, sv, 2 * N)]) for _ in range(n_gauss)]
    return data


def fit_lifted(sc, D, data, lam=1e-3, seed=0):
    """입력-bilinear Θ 배치 정칙화 최소자승. 입력은 ±쌍(U-홀수 블록 분리)."""
    rng = np.random.default_rng(seed); A, B = ab_matrices(sc.n_robots, sc.dt); nu = 2 * sc.n_robots
    Z, Y = [], []
    for X in data:
        u = rng.uniform(sc.u_min, sc.u_max, nu); z = D.lift(X)
        for s in (1, -1):
            Z.append(zeta(z, s * u)); Y.append(D.lift(A @ X + B @ (s * u)))
    Z, Y = np.array(Z), np.array(Y)
    theta = np.linalg.solve(Z.T @ Z + lam * np.eye(Z.shape[1]), Z.T @ Y)
    return LiftedModel(D, theta)


def build_mpc_model(sc, deg=3, phi_deg=2, seed=0):
    """노드·도구 공용 진입점: 사전 + 식별 데이터 + 적합. 약 10 s."""
    D = PolyPhiDict(sc, deg, phi_deg)
    return fit_lifted(sc, D, identification_data(sc, seed=seed), seed=seed)
