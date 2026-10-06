"""사전 재설계 → Koopman MPC 수치 비교(설명 노트 §12). 플랜트·리더는 koopman_mpc_lookahead.py 와 동일
(상대좌표 이중적분기, 잡음 0.02, 지연 0.16 s, 리더 사각 0.1 m/s·30 s 변).

사전 이름: phi = [1; X; φ] (원논문 꼴), p3phi = +poly3, p3phi1 = +φ⊗x, p3phi2 = +φ⊗x² (sim/lifted 기본).
각 사전에 대해 10스텝 열린 루프 J 예측 RMS, 1-step QP, MPC H=3/5 폐루프를 낸다.
    python3 tools/koopman_dictionary_mpc.py [phi p3phi p3phi1 p3phi2] [--seeds 3]"""
import sys, time, re
import numpy as np
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.')
from gtbot_formation.relative_state import make_scenario
from sim.experiment import analytic_c, operating_point
from sim.dynamics import ab_matrices
from sim.utility import z2_vector
from sim.control import solve_input
from sim.lifted import PolyPhiDict, fit_lifted, identification_data, MPC_RHO
RHO = MPC_RHO                        # 0 으로 두면 입력 변화 패널티 없는 원판 비교

SC = make_scenario(); A, B = ab_matrices(3, SC.dt); AD, BD = ab_matrices(3, 0.16)
Z10, _ = operating_point(SC); W = SC.w_full; UMIN, UMAX, LAM = SC.u_min, SC.u_max, SC.input_reg
J_true = lambda X: float(W @ z2_vector(X, SC))


def leader_vel(t, leg=30.0, v=0.1):
    return v * np.array([(1, 0), (0, 1), (-1, 0), (0, -1)][int(t // leg) % 4], float)


def run(ctrl, M=None, H=1, n=1000, seed=0, noise=0.02):
    rng = np.random.default_rng(seed); X = Z10 + np.concatenate([rng.normal(0, 0.1, 6), np.zeros(6)])
    q = [np.zeros(6)] * 3; u_prev = np.zeros(6); plan = np.zeros((H, 6)); st, tt, tms, us = [], [], [], []
    for k in range(n):
        t = k * SC.dt; Xd = AD @ (X + rng.normal(0, noise, 12)) + BD @ u_prev; t0 = time.perf_counter()
        if ctrl == 'analytic':
            U, _ = solve_input(analytic_c(Xd, SC, W), UMIN, UMAX, reg=LAM)
        elif ctrl == 'qp':
            U, _ = solve_input(M.c_1step(M.D.lift(Xd)), UMIN, UMAX, reg=LAM)
        else:
            plan = M.mpc(M.D.lift(Xd), H, np.vstack([plan[1:], plan[-1:]]), LAM, UMIN, UMAX, rho=RHO, u_prev=u_prev); U = plan[0]
        tms.append(time.perf_counter() - t0)
        us.append(U); u_prev = U; q.append(U); X = A @ X + B @ q.pop(0); X[6:] += np.tile(-(leader_vel(t + SC.dt) - leader_vel(t)), 3)
        st.append(np.linalg.norm(X[:6].reshape(3, 2) - SC.targets, axis=1).max()); tt.append(t)
        if not np.isfinite(X).all() or st[-1] > 20: return None
    st, tt, us = np.array(st), np.array(tt), np.array(us); c_ = (tt >= 30) & (tt < 40); s_ = (tt >= 10) & ~c_
    return np.median(st[s_]), st[c_].max(), 1e3 * np.median(tms), np.mean(np.abs(us) >= UMAX - 1e-3), np.mean(np.sign(us[1:]) * np.sign(us[:-1]) < 0)


def hstep_err(M, Xs, rng, H=10):
    err = np.zeros(H)
    for X in Xs:
        z = M.D.lift(X); x = X.copy()
        for h in range(H):
            u = rng.uniform(UMIN, UMAX, 6); z = M.step(z, u); x = A @ x + B @ u; err[h] += (M.g @ z - J_true(x)) ** 2
    return np.sqrt(err / len(Xs))


FMT = lambda r: 'DIVERGED' if r is None else f'정상 {r[0]:.3f} 코너max {r[1]:.3f} 포화 {r[3]:.2f} 반전 {r[4]:.2f} ({r[2]:.1f} ms/틱)'
SPEC = {'phi': (1, 0), 'p3phi': (3, 0), 'p3phi1': (3, 1), 'p3phi2': (3, 2)}

if __name__ == '__main__':
    args = sys.argv[1:]; seeds = int(args[args.index('--seeds') + 1]) if '--seeds' in args else 1
    names = [a for a in args if a in SPEC] or list(SPEC)
    print('analytic 1-step            ', FMT(run('analytic')), flush=True)
    for name in names:
        for fs in range(seeds):
            rng = np.random.default_rng(fs); data = identification_data(SC, seed=fs)
            t0 = time.time(); D = PolyPhiDict(SC, *SPEC[name]); M = fit_lifted(SC, D, data, seed=fs)
            he = hstep_err(M, data[::7][:300], rng)
            print(f'== {name} (적합 시드 {fs}): dim {D.n}, 적합 {time.time()-t0:.0f}s, J 예측 RMS h=1/5/10 {he[0]:.3f}/{he[4]:.3f}/{he[9]:.3f}', flush=True)
            print(f'   1-step QP (Θ)           {FMT(run("qp", M))}', flush=True)
            for H in (3, 5):
                print(f'   Koopman MPC H={H}         {FMT(run("mpc", M, H))}', flush=True)
