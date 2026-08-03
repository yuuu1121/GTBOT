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
    sigma: float = 3.0

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
                  w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, 3.0]))
    init = np.zeros(4 * n_robots)
    init[:2 * n_robots] = (8.0 * np.stack([np.cos(ang + 0.4), np.sin(ang + 0.4)], axis=1)).ravel()
    sc._e3_init = init          # run_e3가 사용
    return sc
