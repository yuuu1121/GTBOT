"""Koopman MPC 시험: 공칭 Θ(확장 사전)로 H스텝 효용을 내다보는 제어 vs 1-step QP.
플랜트: 상대좌표 이중적분기 + 잡음 0.02 + 지연 3틱. 리더는 사각 경로(0.1 m/s, 변 3 m → 30 s마다 90° 코너).
상대좌표에서 리더 가속은 외란 d = −Δv_L 로 들어온다. 'preview'는 그 외란을 미리 아는 경우."""
import sys, time
from dataclasses import replace
import numpy as np
from scipy.optimize import minimize
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.')
from gtbot_formation.relative_state import make_scenario, FORMATION
from sim.experiment import analytic_c, operating_point, nominal_theta
from sim.dynamics import ab_matrices
from sim.utility import z2_vector
from sim.koopman import build_zeta
from sim.control import input_objective, solve_input

SC0 = make_scenario(); SC = replace(SC0, grad_lift=True)
A, B = ab_matrices(3, SC.dt); AD, BD = ab_matrices(3, 0.16)
Z10, Z20 = operating_point(SC); W = SC.w_full; LAM = 1.0; UMAX = 0.3
TH = nominal_theta(SC, n=1500, refits=0)

def leader_vel(t, leg=30.0, v=0.1):
    """사각 경로: 변마다 방향 90° 회전. 월드 방향 상대좌표이므로 편대 목표는 그대로."""
    k = int(t // leg) % 4
    return v * np.array([(1, 0), (0, 1), (-1, 0), (0, -1)][k], float)

def predict_koopman(X, us, dists):
    """Θ로 H스텝 z2 예측. us: (H,6), dists: (H,6) 상대 외란(속도 성분에 더해짐)."""
    z1, z2 = X.copy(), z2_vector(X, SC); J = 0.0
    for h in range(len(us)):
        z2n = TH.T @ build_zeta(z1 - Z10, z2 - Z20, us[h])
        z1 = A @ z1 + B @ us[h]; z1[6:] += dists[h]
        z2 = z2n; J += W @ z2
    return J

def predict_true(X, us, dists):
    z1 = X.copy(); J = 0.0
    for h in range(len(us)):
        z1 = A @ z1 + B @ us[h]; z1[6:] += dists[h]; J += SC0.w_full @ z2_vector(z1, SC0)
    return J

def mpc(X, H, pred, dists, u0):
    f = lambda v: -(pred(X, v.reshape(H, 6), dists) - LAM / 2 * float(v @ v))
    r = minimize(f, u0.ravel(), method='L-BFGS-B', bounds=[(-UMAX, UMAX)] * (6 * H), options=dict(maxiter=10, eps=1e-4))
    return r.x.reshape(H, 6)

def run(ctrl, H=1, preview=False, n=1000, noise=0.02, seed=0):
    rng = np.random.default_rng(seed); X = Z10 + np.concatenate([rng.normal(0, 0.1, 6), np.zeros(6)])
    q = [np.zeros(6)] * 3; u_prev = np.zeros(6); uplan = np.zeros((H, 6)); st, ed, tt = [], [], []
    for k in range(n):
        t = k * SC.dt; Xm = X + rng.normal(0, noise, 12); Xd = AD @ Xm + BD @ u_prev
        if ctrl == 'qp':
            U, _ = solve_input(analytic_c(Xd, SC0, SC0.w_full), -UMAX, UMAX, reg=LAM)
        elif ctrl == 'qp_model':
            c, _ = input_objective(TH, Xd - Z10, z2_vector(Xd, SC) - Z20, W, 6)
            U, _ = solve_input(c, -UMAX, UMAX, reg=LAM)
        else:
            # 지평 내 외란: preview면 리더 속도 변화를 미리 안다, 아니면 0
            if preview:
                dists = np.array([np.tile(-(leader_vel(t + (h + 1) * SC.dt) - leader_vel(t + h * SC.dt)), 3) for h in range(H)])
            else:
                dists = np.zeros((H, 6))
            uplan = mpc(Xd, H, predict_koopman if ctrl == 'mpc_koopman' else predict_true, dists, np.vstack([uplan[1:], uplan[-1:]]))
            U = uplan[0]
        u_prev = U; q.append(U); ua = q.pop(0)
        d = -(leader_vel(t + SC.dt) - leader_vel(t))                    # 실제 외란
        X = A @ X + B @ ua; X[6:] += np.tile(d, 3)
        pos = X[:6].reshape(3, 2); st.append(np.linalg.norm(pos - SC.targets, axis=1).max())
        ed.append(max(abs(np.linalg.norm(pos[i] - pos[j]) - dd) for (i, j), dd in FORMATION.items())); tt.append(t)
        if not np.isfinite(X).all() or st[-1] > 20: return None
    st, ed, tt = map(np.array, (st, ed, tt))
    corner = ((tt % 30.0) < 10.0) & (tt >= 30.0)                        # 각 코너 후 10 s
    steady = (tt >= 10.0) & ~corner
    return dict(st_corner_max=st[corner].max(), st_corner_med=np.median(st[corner]), st_steady_med=np.median(st[steady]),
                ed_corner_max=ed[corner].max(), ed_corner_med=np.median(ed[corner]), ed_steady_med=np.median(ed[steady]))

if __name__ == '__main__':
    cfgs = [('1-step QP (analytic)', dict(ctrl='qp')), ('1-step QP (Koopman Θ)', dict(ctrl='qp_model')),
            ('Koopman MPC H=1', dict(ctrl='mpc_koopman', H=1)), ('Koopman MPC H=3', dict(ctrl='mpc_koopman', H=3)), ('Koopman MPC H=5', dict(ctrl='mpc_koopman', H=5)), ('Koopman MPC H=10', dict(ctrl='mpc_koopman', H=10)),
            ('Koopman MPC H=10 +preview', dict(ctrl='mpc_koopman', H=10, preview=True)),
            ('참 모델 MPC H=5', dict(ctrl='mpc_true', H=5)), ('참 모델 MPC H=10', dict(ctrl='mpc_true', H=10)), ('참 모델 MPC H=10 +preview', dict(ctrl='mpc_true', H=10, preview=True))]
    sel = sys.argv[1:] or [c[0] for c in cfgs]
    for name, kw in cfgs:
        if name not in sel: continue
        t0 = time.time(); r = run(**kw)
        if r is None: print(f'{name:30s} DIVERGED'); continue
        print(f"{name:30s} 코너: station max {r['st_corner_max']:.3f} med {r['st_corner_med']:.3f} | edge max {r['ed_corner_max']:.3f} med {r['ed_corner_med']:.3f} || 정상: station {r['st_steady_med']:.3f} edge {r['ed_steady_med']:.3f}  ({time.time()-t0:.0f}s)", flush=True)
