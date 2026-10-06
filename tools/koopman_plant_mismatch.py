"""플랜트 불일치 강건성(수치): 공칭 이중적분기로 적합한 Θ(mpc 팔)가 추력 이득·지연·1차 응답·항력이 다른
플랜트에서 analytic 1-step 대비 어떻게 버티나. 실기 전 '얼마나 벗어나도 되나'를 보기 위한 것.
플랜트: a_act = gain·(지연 delay_ticks 틱, 1차 응답 tau_lag)·U − drag·v. 리더 사각 0.1 m/s.
    python3 tools/koopman_plant_mismatch.py"""
import sys, time
import numpy as np
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.')
from tools.koopman_dictionary_mpc import SC, A, B, AD, BD, Z10, W, UMIN, UMAX, LAM, leader_vel, analytic_c, solve_input, MPC_RHO
from sim.lifted import build_mpc_model

PLANTS = {'공칭': {}, '이득 0.7': dict(gain=0.7), '이득 1.3': dict(gain=1.3), '지연 0.3 s': dict(delay_ticks=6),
          '1차 응답 0.3 s + 항력 0.5': dict(tau_lag=0.3, drag=0.5), '이득 0.5 + 응답 0.3 + 항력 0.5': dict(gain=0.5, tau_lag=0.3, drag=0.5)}


def run(ctrl, M, plant, H=3, n=1000, seed=0, noise=0.02):
    p = dict(gain=1.0, delay_ticks=3, tau_lag=0.0, drag=0.0); p.update(plant)
    rng = np.random.default_rng(seed); X = Z10 + np.concatenate([rng.normal(0, 0.1, 6), np.zeros(6)])
    q = [np.zeros(6)] * max(p['delay_ticks'], 1); a_act = np.zeros(6); u_prev = np.zeros(6); plan = np.zeros((H, 6)); st, tt, us = [], [], []
    for k in range(n):
        t = k * SC.dt; Xd = AD @ (X + rng.normal(0, noise, 12)) + BD @ u_prev
        if ctrl == 'analytic':
            U, _ = solve_input(analytic_c(Xd, SC, W), UMIN, UMAX, reg=LAM)
        else:
            plan = M.mpc(M.D.lift(Xd), H, np.vstack([plan[1:], plan[-1:]]), LAM, UMIN, UMAX, rho=MPC_RHO, u_prev=u_prev); U = plan[0]
        us.append(U); u_prev = U; q.append(U); ud = q.pop(0)
        a_act = ud if p['tau_lag'] <= 0 else a_act + (ud - a_act) * SC.dt / p['tau_lag']
        X = A @ X + B @ (p['gain'] * a_act - p['drag'] * X[6:]); X[6:] += np.tile(-(leader_vel(t + SC.dt) - leader_vel(t)), 3)
        st.append(np.linalg.norm(X[:6].reshape(3, 2) - SC.targets, axis=1).max()); tt.append(t)
        if not np.isfinite(X).all() or st[-1] > 20: return None
    st, tt, us = np.array(st), np.array(tt), np.array(us); c_ = (tt >= 30) & (tt < 40); s_ = (tt >= 10) & ~c_
    return np.median(st[s_]), st[c_].max(), np.mean(np.abs(us) >= UMAX - 1e-3)


FMT = lambda r: 'DIVERGED' if r is None else f'정상 {r[0]:.3f} 코너 {r[1]:.3f} 포화 {r[2]:.2f}'

if __name__ == '__main__':
    M = build_mpc_model(SC)
    print(f'{"플랜트":28s} {"analytic 1-step":32s} {"Koopman MPC H=3":32s}')
    for name, pl in PLANTS.items():
        t0 = time.time(); ra = run('analytic', None, pl); rm = run('mpc', M, pl)
        print(f'{name:28s} {FMT(ra):32s} {FMT(rm):32s} ({time.time()-t0:.0f}s)', flush=True)
