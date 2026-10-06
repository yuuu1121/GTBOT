"""consistency index I_C = λmax(I − K_F K_B), K_F = Ψ(Y)Ψ(X)⁺, K_B = Ψ(X)Ψ(Y)⁺ (Shi et al. 식 28–29, Haseli & Cortés).
자율계 정의라 흐름을 둘로 둔다: (a) U=0 표류, (b) analytic QP 폐루프 x⁺ = T(x, κ(x))."""
import sys, numpy as np
from dataclasses import replace
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.')
from gtbot_formation.relative_state import make_scenario
from sim.experiment import analytic_c, operating_point
from sim.dynamics import ab_matrices
from sim.utility import z2_vector
from sim.control import solve_input
sc0 = make_scenario(); scg = replace(sc0, grad_lift=True)
A, B = ab_matrices(3, sc0.dt); z10, _ = operating_point(sc0)
rng = np.random.default_rng(0)
def flow(X, mode):
    if mode == 'drift': return A @ X
    U, _ = solve_input(analytic_c(X, sc0, sc0.w_full), -0.3, 0.3, reg=1.0); return A @ X + B @ U
def ic(PX, PY):            # PX, PY: dim × N
    KF = PY @ np.linalg.pinv(PX); KB = PX @ np.linalg.pinv(PY)
    ev = np.linalg.eigvals(np.eye(len(KF)) - KF @ KB)
    return float(np.max(ev.real)), float(np.max(np.abs(ev.imag)))
dicts = {'X만 (선형)': lambda X: X,
         '원 사전 [X; φ27]': lambda X: np.concatenate([X, z2_vector(X, sc0)]),
         '확장 사전 [X; φ27; ψ12]': lambda X: np.concatenate([X, z2_vector(X, scg)]),
         'φ27만': lambda X: z2_vector(X, sc0), 'φ27+ψ12 (X 없이)': lambda X: z2_vector(X, scg)}
for sp, sv, N in ((0.3, 0.15, 3000), (0.1, 0.05, 3000)):
    Xs = [z10 + np.concatenate([rng.normal(0, sp, 6), rng.normal(0, sv, 6)]) for _ in range(N)]
    for mode in ('drift', 'closed'):
        Ys = [flow(X, mode) for X in Xs]
        print(f'== 영역 pos±{sp} vel±{sv}, 흐름={mode}, N={N}')
        for name, psi in dicts.items():
            PX = np.array([psi(X) for X in Xs]).T; PY = np.array([psi(Y) for Y in Ys]).T
            v, im = ic(PX, PY)
            print(f'   {name:24s} dim {PX.shape[0]:3d}  I_C = {v:.4f}  (허수부 최대 {im:.1e})')
