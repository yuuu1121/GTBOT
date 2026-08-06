from dataclasses import dataclass, field
import numpy as np

# 원논문 Table 2 (:1476~1494): case -> ([초기위치], [목표위치])
TABLE2_CASES = {
    "I":   ([(-4.48, 3.3), (2.98, 7.11), (3.65, -5.70)], [(-4.5, 0.0), (3.0, 4.0), (3.0, -4.0)]),
    "II":  ([(-3.1, 3.3), (4.9, 4.6), (0.9, -5.0)],      [(-3.0, 0.5), (5.0, 2.0), (0.5, -3.5)]),
    "III": ([(7.5, 2.5), (-5.1, -2.5), (-5.2, 3.5)],     [(7.5, 0.0), (-5.0, -5.0), (-5.0, 5.0)]),
    "IV":  ([(-3.1, -0.6), (0.0, 2.5), (4.8, 3.0)],      [(-3.5, -3.5), (0.2, 0.0), (5.3, 5.0)]),
    "V":   ([(-6.4, 5.3), (-2.5, 1.8), (3.0, -2.7)],     [(-6.0, 2.4), (-2.5, -1.4), (2.5, -1.4)]),
}
PHASE1_INIT_POS = [(-7.0, 3.0), (0.0, 7.0), (7.0, -4.0)]     # :1077, 속도 0
PHASE1_TARGETS  = [(-4.5, 0.0), (3.0, 4.0), (3.0, -4.0)]     # :1079

def table1_input(k: int) -> np.ndarray:
    """식별 입력 (Table 1, :1096~1106). k는 1-based — 0-based는 벽 관통 아티팩트(스펙)."""
    return np.array([
        2 * np.sin(0.075 * k * np.pi),  3 * np.cos(0.1833 * k * np.pi),
        3 * np.sin(0.14 * k * np.pi),   3 * np.cos(0.095 * k * np.pi),
       -2 * np.sin(0.06 * k * np.pi),  -3 * np.cos(0.092 * k * np.pi)])

def e3_input(k: int, n_robots: int) -> np.ndarray:
    """E3 결정론적 사인파 (스펙 게이트 4): 대역 [0.06,0.19] 균등 분할, x=sin/y=cos 교대."""
    j = np.arange(2 * n_robots)
    amp = (-1.0) ** j * (2 + (j % 2))
    freq = 0.06 + 0.13 * j / (2 * n_robots - 1)
    ph = freq * k * np.pi
    return amp * np.where(j % 2 == 0, np.sin(ph), np.cos(ph))

@dataclass
class Scenario:
    n_robots: int = 3
    phi_terms: tuple = (1, 2, 3, 4, 5, 6)
    targets: np.ndarray = field(default_factory=lambda: np.array(PHASE1_TARGETS))
    dt: float = 0.05
    wall_radius: float = 11.0
    wall_center: np.ndarray = field(default_factory=lambda: np.zeros(2))
    robot_radius: float = 2.0
    u_min: float = -4.0
    u_max: float = 3.0
    p0: float = 100.0
    rls_rho: float = 1e-4
    p_reset: dict = field(default_factory=lambda: {"linear": 130, "bilinear": 75})
    ks: int = 300
    control_iters: int = 30
    eps_guard: float = 1e-6
    w_robot: np.ndarray = field(default_factory=lambda: np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0]))
    formation: dict = field(default_factory=dict)   # {(i,j): d_star}, i<j
    leader_standoff: float = 0.0    # φ⁸ 리더 표면거리 기준(반지름 합). 0 = 항 미사용 시 무영향
    input_reg: float = 0.0          # QP 정규화 λ. 0 = 기존 LP 릴레이 경로 그대로
    sigma: float = 3.0
    reduced_lifting: bool = False   # Task 8b 폴백: bilinear ζ에서 g1·g3 제거(a=433, 결정계)
    v_cruise: float = 4.0   # phi2 순항속도 지시(원논문 값). E2는 1.0 — 이웃거리 무관 4m/s 지시와
                             # phi6 반응거리 0.3m/제동거리 1.0m+ 구조적 불일치를 회피(문서화된 이탈)

    @property
    def n_phi(self): return len(self.phi_terms)
    @property
    def m(self): return self.n_phi * self.n_robots
    @property
    def w_full(self): return np.tile(self.w_robot, self.n_robots)
    def phase1_init_state(self):
        X = np.zeros(4 * self.n_robots)
        X[:2 * self.n_robots] = np.array(PHASE1_INIT_POS).ravel()
        return X
    def desired_state(self):
        X = np.zeros(4 * self.n_robots)
        X[:2 * self.n_robots] = np.asarray(self.targets).ravel()
        return X

def e1_scenario() -> Scenario:
    return Scenario()

# 개선 식별 (Task 8b): 75-스텝 에피소드 8개 — φ³~φ⁶ 전 성분 여기 확보 (스펙 개정 2026-08-04)
# 각 항목: (positions[3][2], velocities[3][2]) — 목표근방·상호근접·벽근처·중원 커버
IMPROVED_EPISODES = [
    ([(-4.2, 0.3), (3.3, 3.7), (2.7, -4.3)], [(0.0, 0.0)] * 3),      # 목표근방 (phi3·phi4 여기)
    ([(0.0, 0.0), (4.6, 0.0), (0.0, -8.0)], [(0.3, 0.0), (-0.3, 0.0), (0.0, 0.3)]),  # 근접쌍 (phi6)
    ([(8.5, 0.0), (-8.5, 0.0), (0.0, 8.5)], [(-0.3, 0.0), (0.3, 0.0), (0.0, -0.3)]), # 벽근처 (phi5)
    ([(-7.0, 3.0), (0.0, 7.0), (7.0, -4.0)], [(0.0, 0.0)] * 3),      # 중원 (Phase I 초기)
    ([(-4.5, 0.5), (3.0, 4.5), (3.5, -3.5)], [(0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)]),# 목표근방+저속
    ([(0.0, 0.0), (4.5, 0.0), (2.25, 3.9)], [(0.0, 0.0)] * 3),       # 근접 삼중 클러스터
    ([(8.0, 0.0), (3.5, 0.0), (-8.0, 0.0)], [(0.0, 0.5), (0.0, -0.5), (0.0, 0.5)]),  # 벽+쌍 혼합
    ([(-5.0, 2.0), (3.0, 5.0), (5.0, -4.0)], [(0.3, 0.3), (-0.3, 0.3), (0.0, -0.5)]),# 중원 산개
]

def improved_episode_state(idx, n_robots=3):
    pos, vel = IMPROVED_EPISODES[idx]
    X = np.zeros(4 * n_robots)
    X[:2 * n_robots] = np.array(pos).ravel(); X[2 * n_robots:] = np.array(vel).ravel()
    return X

def local_episodes(case_init, case_targets, n_episodes=80, n_robots=3):
    """케이스 회랑(init~target) 근방 커버 — 국소 식별용(Task 8b 라운드 4, 결정론적).
    절반(짝수 j)은 저속(위상 오프셋), 절반(홀수 j)은 목표 방향 1/2/3 m/s 이동 속도 —
    제어 궤적의 실제 속도 분포(bang-bang 가속으로 3~4 m/s 도달)를 커버."""
    init = np.array(case_init, dtype=float); tgt = np.array(case_targets, dtype=float)
    direction = tgt - init
    unit = direction / np.linalg.norm(direction, axis=1, keepdims=True)
    eps = []
    for j in range(n_episodes):
        frac = (j % 8) / 7.0
        off = 0.25 * np.array([[np.cos(j + i), np.sin(1.7 * j + i)] for i in range(n_robots)])
        if j % 2 == 0:
            vel = 0.3 * np.array([[np.sin(j + i), np.cos(j - i)] for i in range(n_robots)])
        else:
            vel = (1 + (j % 3)) * unit
        X = np.zeros(4 * n_robots)
        X[:2 * n_robots] = (init + frac * direction + off).ravel()
        X[2 * n_robots:] = vel.ravel()
        eps.append(X)
    return eps

E2_TARGETS   = [(0.0, 3.464), (-3.0, -1.732), (3.0, -1.732)]   # 정삼각형 변 d*=6 (d*-2Rr=2 >= 0.5 충족)
E2_INIT      = [(-6.0, 5.0), (-7.0, -4.0), (6.0, -5.0)]        # 형상 이탈 배치 — J(0)≈151 유의미
E2_FORMATION = {(0, 1): 6.0, (0, 2): 6.0, (1, 2): 6.0}

def e2_scenario(use_phi7_weight: bool) -> Scenario:
    """E2. 두 팔 모두 phi_terms=[1..7](a=1075 동일) — 베이스라인은 w7=0 (스펙 게이트 3 차원 통제)."""
    w7 = 3.0 if use_phi7_weight else 0.0
    return Scenario(
        phi_terms=(1, 2, 3, 4, 5, 6, 7),
        targets=np.array(E2_TARGETS),
        formation=dict(E2_FORMATION), sigma=3.0,
        control_iters=300,                       # 스펙: 원논문 이탈(논문에 명시)
        v_cruise=1.0,                             # 스펙: 원논문 이탈(논문에 명시, control_iters와 동일 문서화)
        w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, w7]))

def e3_scenario(n_robots: int) -> Scenario:
    """E3: 목표 = 반지름 6 원 위 균등 배치, 편대 그래프 = 링 일부 + 노드0 부채꼴 (2N-3 엣지, 최소 강성)."""
    ang = 2 * np.pi * np.arange(n_robots) / n_robots
    targets = np.stack([6 * np.cos(ang), 6 * np.sin(ang)], axis=1)
    if n_robots == 3:
        edges = [(0, 1), (0, 2), (1, 2)]
    else:
        ring = [(i, i + 1) for i in range(n_robots - 1)]              # 개방 체인 N-1개
        fan = [(0, j) for j in range(2, n_robots)]                    # 부채꼴 N-2개 → 계 2N-3
        edges = ring + fan
    formation = {}
    for (a, b) in edges:
        a2, b2 = min(a, b), max(a, b)
        formation[(a2, b2)] = float(np.linalg.norm(targets[a2] - targets[b2]))
    sc = Scenario(n_robots=n_robots, phi_terms=(1, 2, 3, 4, 5, 6, 7), targets=targets,
                  formation=formation, sigma=0.5 * min(formation.values()), control_iters=300,
                  v_cruise=1.0,   # E2와 같은 근거의 문서화된 이탈 — phi2 4m/s 지시와 phi6 반응거리
                                  # 0.3m/제동거리 1.0m+ 구조적 불일치 회피 (scenario.py:47 참조)
                  w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0]))
    init = np.zeros(4 * n_robots)
    init[:2 * n_robots] = (8.0 * np.stack([np.cos(ang + 0.4), np.sin(ang + 0.4)], axis=1)).ravel()
    sc._e3_init = init          # run_e3가 사용
    return sc
