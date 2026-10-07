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
import itertools, os, hashlib
import numpy as np
from scipy.optimize import minimize
from .dynamics import ab_matrices
from .utility import z2_vector
from .experiment import analytic_c, operating_point
from .control import solve_input

POS_SCALE, VEL_SCALE = 0.3, 0.15      # 단항식 입력 정규화(동작 영역 폭) — 고차항 조건수 보호
SAT = 1.5                              # 단항식 좌표 포화 x̃ = SAT·tanh(x/SAT): 자리 오차 ~0.45 m 밖에서 특징이
SAT_VEL = 3.0                          # 속도 좌표 포화(~0.45 m/s). 위치와 같은 1.5(0.22 m/s)면 큰 오차에서 복귀 속도(0.3~0.6 m/s)가
                                       # 사전에서 보이지 않아 '멀어지는 게 나쁘다'를 모델이 못 보고 폭주했다(시드 4, 초기 0.7 m).
                                       # 유계가 돼 3차 사전의 외삽 폭주를 막는다. 안쪽(|x|<1)은 x̃≈x. 이걸로 초기 오차 1.0 m 에서도
                                       # 수렴(0.086 m, analytic 0.096)해 신뢰 영역 폴백을 없앴다. 위치 RBF 5×5 사전(dim 856)은 예측은
                                       # 더 좋았으나(h=5 RMS 0.40 vs 0.68) 틱당 116 ms 라 20 Hz 에 못 들어가 쓰지 않았다.


class PolyPhiDict:
    def __init__(self, sc, deg=3, phi_deg=2, sat=SAT, sat_vel=SAT_VEL):
        self.sc, self.deg, self.phi_deg, self.sat, self.sat_vel = sc, deg, phi_deg, sat, sat_vel
        self.sat_vec = np.array([sat, sat, sat_vel, sat_vel])
        self.E = np.array([e for e in itertools.product(range(deg + 1), repeat=4) if 0 < sum(e) <= deg])
        self.Ep = (np.array([e for e in itertools.product(range(phi_deg + 1), repeat=4) if 0 < sum(e) <= phi_deg])
                   if phi_deg else np.zeros((0, 4), int))
        self.n_phi = len(sc.phi_terms); self.n_poly = 1 + sc.n_robots * len(self.E)
        self.n = self.n_poly + sc.n_robots * self.n_phi * (1 + len(self.Ep))
        self.g = np.zeros(self.n); self.g[self.n_poly:self.n_poly + sc.n_robots * self.n_phi] = sc.w_full   # J = w·φ
        lin = [next(j for j, e in enumerate(self.E) if e.sum() == 1 and np.argmax(e) == k) for k in range(4)]
        self.idx_lin = np.array([[1 + r * len(self.E) + j for j in lin] for r in range(sc.n_robots)])   # (N,4): z 안의 1차 단항식(포화 좌표) 위치
        N = sc.n_robots; self.idx_X = np.array([[2 * r, 2 * r + 1, 2 * N + 2 * r, 2 * N + 2 * r + 1] for r in range(N)])  # (N,4): 그 좌표의 X 위치

    def lift(self, X):
        N = self.sc.n_robots; pos, vel = X[:2 * N].reshape(N, 2), X[2 * N:].reshape(N, 2)
        xs = [self.sat_vec * np.tanh(np.concatenate([(pos[r] - self.sc.targets[r]) / POS_SCALE, vel[r] / VEL_SCALE]) / self.sat_vec) for r in range(N)]
        f = [np.ones(1)] + [np.prod(x[None, :] ** self.E, axis=1) for x in xs]
        phi = z2_vector(X, self.sc); f.append(phi)
        for r in range(N):
            if len(self.Ep):
                f.append(np.outer(phi[r * self.n_phi:(r + 1) * self.n_phi], np.prod(xs[r][None, :] ** self.Ep, axis=1)).ravel())
        return np.concatenate(f)

    def unlift(self, z):
        """z → X: 1차 단항식(포화 좌표 x̃ = s·tanh(x/s))을 읽어 tanh 역변환. re-projection(Higuchi & Sato 2026 식 사영) 용.
        반환 (X, dX/dx̃ 대각) — 예측 z 가 |x̃| ≥ s 로 벗어나면 0.999s 로 자른다."""
        N = self.sc.n_robots; xt = np.clip(z[self.idx_lin] / self.sat_vec, -0.999, 0.999)
        x = self.sat_vec * np.arctanh(xt); dx = 1.0 / (1.0 - xt ** 2)
        scale = np.array([POS_SCALE, POS_SCALE, VEL_SCALE, VEL_SCALE])
        X = np.zeros(4 * N); X[:2 * N] = (x[:, :2] * POS_SCALE + np.asarray(self.sc.targets)).ravel(); X[2 * N:] = (x[:, 2:] * VEL_SCALE).ravel()
        return X, dx * scale

    def lift_jac_T(self, X, a, eps=1e-6):
        """Jψ(X)ᵀ a — ψ 의 상태 Jacobian 전치곱(12 차원 유한차분, lift 12회)."""
        z0 = self.lift(X); out = np.zeros(len(X))
        for i in range(len(X)):
            Xp = X.copy(); Xp[i] += eps; out[i] = (self.lift(Xp) - z0) @ a / eps
        return out


def zeta(z, u):
    return np.concatenate([z, u, np.outer(z, u).ravel()])


class LiftedModel:
    """Θ → K, L, N(6,n,n). M(u) = K + Σ u_l N_l."""
    def __init__(self, D, theta):
        n = D.n; nu = (theta.shape[0] - n) // (n + 1)      # ζ 길이 n + nu + n·nu
        self.D, self.g, self.n, self.nu, self.theta = D, D.g, n, nu, theta
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

    def mpc(self, z0, H, u0, lam, u_min, u_max, rho=0.0, u_prev=None, maxiter=10, reproject=False):
        """max Σ_h gᵀz_h − λ/2 Σ|U_h|² − ρ/2 Σ|U_h − U_{h−1}|²  (U_{−1} = 직전 발행 입력), 박스 제약.
        ρ(입력 변화 패널티)가 없으면 MPC 는 입력을 62 % 포화·부호 반전 0.89 로 써서 이기는 것이라(수치 시뮬),
        Stonefish 에서는 속도 추정이 채터링을 못 따라가 est 속도 오차 RMS 0.15 로 커졌다(1-step 0.017). ρ=3 이면
        포화 0.02·반전 0.43 으로 analytic(0.11·0.70)보다 매끈하고 코너 이득 −8 %는 남는다.
        reproject=True: 매 스텝 예측 z 를 원 상태로 사영(unlift)하고 다시 lifting 한다(Higuchi & Sato 2026 — 유한 차원 Koopman
        예측기는 lifted 상태 manifold 를 보존하지 않아 다스텝 예측이 벗어난다). 비용 gᵀψ(X_h) 가 예측 상태의 정확한 utility 가 되고
        h=1 J 예측 RMS 0.14→0.09, h≤4 개선. gradient 는 사영 Jacobian(대각) × ψ Jacobian(상태 12차원 FD, lift 12회/스텝)을 거친다."""
        nu = self.nu; up0 = np.zeros(nu) if u_prev is None else u_prev

        def f(v):
            us = v.reshape(H, nu); Ju, gu = self._rollout_grad(z0, us, reproject)
            du = np.diff(np.vstack([up0[None], us]), axis=0)
            J = Ju - lam / 2 * float(v @ v) - rho / 2 * float((du ** 2).sum())
            grad = gu - lam * us - rho * du + rho * np.vstack([du[1:], np.zeros((1, nu))])
            return -J, -grad.ravel()
        r = minimize(f, u0.ravel(), jac=True, method='L-BFGS-B', bounds=[(u_min, u_max)] * (H * nu),
                     options=dict(maxiter=maxiter))
        return r.x.reshape(H, nu)

    def _rollout_grad(self, z0, us, reproject):
        """Σ_h gᵀz_h 와 그 ∂/∂U (H,nu) — forward 롤아웃 + 수반 역전파 한 번. reproject 면 매 스텝 unlift→lift."""
        H = len(us); zs = [z0]; Ms = []; Xs = []; dXs = []
        for h in range(H):
            Ms.append(self.M_of(us[h])); zh = Ms[-1] @ zs[-1] + self.L @ us[h]
            if reproject:
                X, dX = self.D.unlift(zh); Xs.append(X); dXs.append(dX); zh = self.D.lift(X)
            zs.append(zh)
        adj = np.zeros(self.n); grad = np.zeros((H, self.nu))
        for h in range(H - 1, -1, -1):
            adj = self.g + adj                                       # ∂J/∂z_{h+1} (사영 뒤 z)
            if reproject:                                            # ∂J/∂ẑ_{h+1} = JCᵀ Jψᵀ adj — JC 는 1차 단항식 12칸에만 대각
                b = np.zeros(self.n); b[self.D.idx_lin.ravel()] = (self.D.lift_jac_T(Xs[h], adj)[self.D.idx_X] * dXs[h]).ravel(); adj = b
            grad[h] = adj @ self.L + (self.N @ zs[h]) @ adj
            adj = Ms[h].T @ adj
        return sum(self.g @ z for z in zs[1:]), grad

    def mpc_sqp(self, z0, H, u0, lam, u_min, u_max, rho=0.0, u_prev=None, reproject=True, dmax=0.1, iters=1):
        """Folkestad & Burdick 2021 식 SQP — 틱당 선형화 1회 + QP 1회, 이전 계획 shift warm start(u0).
        비용이 z 에 선형(gᵀz)이라 선형화하면 Δu 의 1차식 + λ·ρ 2차식만 남아 모델 호출 없는 작은 QP 가 된다:
            max cᵀΔu − λ/2|u+Δu|² − ρ/2 Σ|(u_h+Δu_h) − (u_{h−1}+Δu_{h−1})|²,  box(u+Δu), |Δu| ≤ dmax(신뢰 영역).
        틱당 비용 = forward 1 + adjoint 1 (re-projection 이면 lift 12H 회) — L-BFGS 10회(10~15 평가)의 1/10."""
        nu = self.nu; up0 = np.zeros(nu) if u_prev is None else u_prev; us = np.asarray(u0, float).copy()
        for _ in range(iters):
            _, c = self._rollout_grad(z0, us, reproject)
            def q(v):
                d = v.reshape(H, nu); u = us + d; du = np.diff(np.vstack([up0[None], u]), axis=0)
                J = float((c * d).sum()) - lam / 2 * float((u ** 2).sum()) - rho / 2 * float((du ** 2).sum())
                g = c - lam * u - rho * du + rho * np.vstack([du[1:], np.zeros((1, nu))])
                return -J, -g.ravel()
            lo = np.maximum(u_min - us, -dmax).ravel(); hi = np.minimum(u_max - us, dmax).ravel()
            r = minimize(q, np.zeros(H * nu), jac=True, method='L-BFGS-B', bounds=list(zip(lo, hi)), options=dict(maxiter=50))
            us = us + r.x.reshape(H, nu)
        return us


MPC_RHO = 3.0                          # 입력 변화 패널티(위 mpc 주석)


def identification_data(sc, inits=(0.3,) * 4 + (0.8,) * 6 + (1.0,) * 6, n_ticks=500, noise=0.05, n_gauss=1500, sp=0.15, sv=0.08,
                        delay=0.16, seed=0):
    """식별용 상태 표본: 공칭 플랜트에서 analytic 제어 폐루프(잡음 0.05로 흔듦, 리더 정지)를 n_traj번 굴린
    방문 상태 + 동작점 둘레 좁은 가우시안. 가우시안만 쓰면 장벽 안쪽 등 폐루프가 가지 않는 곳에 질량을 써서
    적합이 망가진다(nominal_theta 주석과 같은 교훈). inits: 궤적 초기 흩뿌림 — 0.3 m 4개 + 0.8 m 6개 + 1.0 m 6개.
    넓은 궤적은 Stonefish 워밍업 끝 오차(0.6~0.8 m)를 덮기 위해서다; 포화 좌표(SAT) 없이 넓은 데이터만 넣으면 오히려
    0.7 m 에서 발산했고, 포화 좌표와 함께 넣어야 수렴한다 — 그러나 그것도 **초기조건 시드 0 하나**에서만이었다.
    초기조건 시드 1~5(장벽 안쪽·로봇 근접 출발 포함)에서는 적합 시드·데이터 폭·릿지·시드 앙상블·장벽 오버라이드·
    기울기 부호 필터 어느 것으로도 0.7 m 이상에서 발산했다(2026-10-06 수치, Stonefish 시드 1 런 2회 발산).
    결론: 3차 다항식 사전은 자리 둘레 ~0.45 m 안에서만 믿을 수 있고, 밖은 analytic 1-step 이 맡는다
    (in_trust_region). 넓은 궤적 데이터는 경계 근처 적합을 돕는 용도로만 남긴다."""
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


def fit_lifted(sc, D, data, lam=1e-3, seed=0, sub=1):
    """입력-bilinear Θ 배치 정칙화 최소자승. 입력은 ±쌍(U-홀수 블록 분리).
    sub: 예측 스텝 하나가 제어 틱 몇 개인가(입력 ZOH). 이중적분기는 ab_matrices(sub·dt) = A^sub, Σ_{i<sub}A^iB 로 정확.
    sub=3·H=3 이면 지평 0.45 s — 작동기 지연 0.16 s 를 넘겨 관성을 실제로 내다본다(sub=1·H=2 는 0.1 s)."""
    rng = np.random.default_rng(seed); A, B = ab_matrices(sc.n_robots, sub * sc.dt); nu = 2 * sc.n_robots
    Z, Y = [], []
    for X in data:
        u = rng.uniform(sc.u_min, sc.u_max, nu); z = D.lift(X)
        for s in (1, -1):
            Z.append(zeta(z, s * u)); Y.append(D.lift(A @ X + B @ (s * u)))
    Z, Y = np.array(Z), np.array(Y)
    theta = np.linalg.solve(Z.T @ Z + lam * np.eye(Z.shape[1]), Z.T @ Y)
    return LiftedModel(D, theta)


TRUST_POS, TRUST_VEL = 0.45, 0.30     # 리프팅 모델을 믿는 영역(자리 오차·속도). 식별 데이터 질량이 있는 곳.


def in_trust_region(X, sc, pos=TRUST_POS, vel=TRUST_VEL):
    """밖이면 analytic 1-step 으로 내려간다(identification_data 주석의 실측). 사각 주행 중에는 발동하지 않는다."""
    N = sc.n_robots; dp = X[:2 * N].reshape(N, 2) - np.asarray(sc.targets); v = X[2 * N:].reshape(N, 2)
    return bool(np.linalg.norm(dp, axis=1).max() <= pos and np.linalg.norm(v, axis=1).max() <= vel)


def build_mpc_model(sc, deg=3, phi_deg=2, seed=0, sat=SAT, sat_vel=SAT_VEL, lam=None, sub=1, cache_dir=os.path.expanduser('~/.cache/gtbot')):
    """노드·도구 공용 진입점: 사전 + 식별 데이터 + 적합(약 10 s). 같은 (시나리오, 사전, 시드)면 Θ를
    cache_dir 에 저장해 두고 다음 기동부터 즉시 읽는다 — 실기 재시작 시 10 s 적합 대기 제거. cache_dir=None 이면 끈다."""
    D = PolyPhiDict(sc, deg, phi_deg, sat, sat_vel)
    key = hashlib.sha1(repr((deg, phi_deg, sat, sat_vel, lam, seed, POS_SCALE, VEL_SCALE, tuple(sc.phi_terms), tuple(np.round(sc.w_robot, 6)),
                             np.round(np.asarray(sc.targets), 6).tolist(), sc.leader_standoff, sc.dt, sc.u_min, sc.u_max,
                             sc.input_reg, sc.n_robots) + ((sub,) if sub != 1 else ())).encode()).hexdigest()[:16]
    path = cache_dir and os.path.join(cache_dir, f'lifted_theta_{key}.npy')
    if path and os.path.exists(path):
        theta = np.load(path)
        if theta.shape == (D.n + 2 * sc.n_robots + D.n * 2 * sc.n_robots, D.n):
            return LiftedModel(D, theta)
    M = fit_lifted(sc, D, identification_data(sc, seed=seed), seed=seed, sub=sub, **({} if lam is None else dict(lam=lam)))
    if path:
        os.makedirs(cache_dir, exist_ok=True); np.save(path, M.theta)
    return M
