"""explainer 그림 생성 → draft/fig/. 실행: cd gtbot_ws && python3 <this>"""
import sys, os
from dataclasses import replace
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.')
from gtbot_formation.relative_state import make_scenario, OFFSETS, FORMATION
from sim.experiment import analytic_c, operating_point, nominal_theta
from sim.dynamics import ab_matrices
from sim.utility import z2_vector
from sim.control import input_objective, solve_input

OUT = 'draft/fig'; os.makedirs(OUT, exist_ok=True)
C = {'analytic': '#4C78A8', 'model': '#E45756', 'plain': '#F58518', 'grey': '#9A9A9A', 'ink': '#333333'}
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.color': '#E6E6E6', 'grid.linewidth': 0.6,
                     'font.family': ['Noto Sans CJK HK', 'Baekmuk Dotum', 'DejaVu Sans'], 'axes.unicode_minus': False})

def save(fig, name):
    fig.tight_layout(); fig.savefig(f'{OUT}/{name}.png', dpi=160); plt.close(fig); print('saved', name)

# ---------- fig1: 파이프라인 블록도 ----------
fig, ax = plt.subplots(figsize=(9, 3.6)); ax.axis('off'); ax.set_xlim(0, 10); ax.set_ylim(0, 4)
def box(x, y, w, h, text, fc='#F4F4F4', ec='#555555', fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08', fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs, color=C['ink'])
def arrow(x0, y0, x1, y1):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=12, lw=1.2, color='#555555'))
box(0.1, 1.5, 1.7, 1.0, 'LiDAR 추정\n상대 상태 X')
box(2.2, 1.5, 2.0, 1.0, '지연 보상\n' + r'$X_{pred}=A_\tau X+B_\tau U_{prev}$')
box(4.6, 2.55, 2.4, 0.9, 'analytic 팔\n' + r'$c=\partial J/\partial U$ (유한차분)', fc='#E7EEF7', ec=C['analytic'])
box(4.6, 0.55, 2.4, 0.9, 'model 팔\n' + r'$c=w^\top(\Theta^\top\zeta(e_l)-\Theta^\top\zeta(0))$', fc='#FBE9E8', ec=C['model'])
box(7.4, 1.5, 1.4, 1.0, 'QP\n' + r'$U=\mathrm{clip}(c/\lambda)$')
box(9.0, 1.5, 0.9, 1.0, '속도\n루프', fs=9)
arrow(1.8, 2.0, 2.2, 2.0); arrow(4.2, 2.1, 4.6, 2.9); arrow(4.2, 1.9, 4.6, 1.1)
arrow(7.0, 3.0, 7.4, 2.1); arrow(7.0, 1.0, 7.4, 1.9); arrow(8.8, 2.0, 9.0, 2.0)
ax.text(5.8, 0.25, 'Θ: 공칭 모델 배치 적합(시작 시 1회), 사전 = φ(27) + ψ=∇J(12)', ha='center', fontsize=8.5, color='#666666')
ax.text(5.0, 3.75, '두 팔의 차이는 c의 출처뿐 — 지연 보상·QP·효용 가중은 공통', ha='center', fontsize=9, color=C['ink'])
save(fig, 'fig3_pipeline')

# ---------- fig2: 효용 항 모양 ----------
sc = make_scenario()
fig, axs = plt.subplots(1, 3, figsize=(10, 2.9))
d = np.linspace(0.3, 3.0, 300)
axs[0].plot(d, np.exp(-((d - 1.5) ** 2) / sc.sigma ** 2), color=C['model'], lw=2)
axs[0].axvline(1.5, color=C['grey'], lw=1, ls='--'); axs[0].set_title('φ⁷ 편대 유지 (w=+3)'); axs[0].set_xlabel('로봇 간 거리 [m]')
axs[0].annotate('d* = 1.5 m에서 최대', (1.5, 1.0), (1.8, 0.85), fontsize=8.5, arrowprops=dict(arrowstyle='-', color=C['grey']))
r = np.linspace(0.4, 1.4, 300); dL = r - sc.leader_standoff
axs[1].plot(r, np.log(1 + 10 * np.exp(-20 * dL)), color=C['model'], lw=2)
axs[1].axvline(0.866, color=C['grey'], lw=1, ls='--'); axs[1].set_title('φ⁸ 리더 반발 (w=−32)'); axs[1].set_xlabel('리더까지 거리 [m]')
axs[1].annotate('자리(0.866 m)', (0.866, 0.3), (0.95, 1.2), fontsize=8.5, arrowprops=dict(arrowstyle='-', color=C['grey']))
e = np.linspace(-0.6, 0.6, 300)
axs[2].plot(e, -e ** 2, color=C['model'], lw=2); axs[2].set_title('φ⁹ 위치 복원 (w=+50)'); axs[2].set_xlabel('자리에서 밀려난 거리 [m]')
for a in axs: a.set_ylabel('항 값')
save(fig, 'fig1_utility_terms')

# ---------- fig3: LP 릴레이 vs QP ----------
fig, ax = plt.subplots(figsize=(5, 3))
c = np.linspace(-0.8, 0.8, 400)
ax.plot(c, np.where(c > 0, 0.3, -0.3), color=C['plain'], lw=2, label='LP: 항상 ±0.3 (릴레이)')
ax.plot(c, np.clip(c / 1.0, -0.3, 0.3), color=C['analytic'], lw=2, label='QP (λ=1): clip(c/λ)')
ax.set_xlabel('그래디언트 성분 c_l'); ax.set_ylabel('명령 U_l [m/s²]'); ax.legend(frameon=False, fontsize=8.5)
ax.set_title('같은 c에서 LP와 QP가 내는 명령')
save(fig, 'fig2_lp_vs_qp')

# ---------- fig4: 사전 비교 폐루프 (이상적 플랜트, 잡음 0.02, 지연 0.15 s) ----------
def loop(sc_, theta=None, n=2400, seed=0):
    rng = np.random.default_rng(seed); A, B = ab_matrices(3, sc_.dt); Ad, Bd = ab_matrices(3, 0.16)
    z10, z20 = operating_point(sc_); X = z10 + np.concatenate([rng.normal(0, 0.3, 6), np.zeros(6)])
    q = [np.zeros(6)] * 3; u_prev = np.zeros(6); st, vv = [], []
    for k in range(n):
        Xm = X + rng.normal(0, 0.02, 12); Xd = Ad @ Xm + Bd @ u_prev
        if theta is None: cc = analytic_c(Xd, sc, sc.w_full)
        else: cc, _ = input_objective(theta, Xd - z10, z2_vector(Xd, sc_) - z20, sc_.w_full, 6)
        U, _ = solve_input(cc, -0.3, 0.3, reg=1.0); u_prev = U; q.append(U); X = A @ X + B @ q.pop(0)
        st.append(np.linalg.norm(X[:6].reshape(3, 2) - sc.targets, axis=1).max()); vv.append(np.linalg.norm(X[6:].reshape(3, 2), axis=1).max())
    return np.array(st), np.array(vv)
sc_g = replace(sc, grad_lift=True)
th_grad = nominal_theta(sc_g, n=1500, refits=0)
th_plain = nominal_theta(sc)   # 원 사전은 n=6000+재적합(§7 표와 같은 설정)
runs = [('analytic (참 그래디언트)', loop(sc), C['analytic']), ('model, 원 사전 φ', loop(sc, th_plain), C['plain']), ('model, 확장 사전 φ+ψ', loop(sc_g, th_grad), C['model'])]
fig, axs = plt.subplots(2, 1, figsize=(8, 4.6), sharex=True)
t = np.arange(2400) * 0.05
for name, (st, vv), col in runs:
    axs[0].plot(t, st, color=col, lw=1.4, label=name); axs[1].plot(t, vv, color=col, lw=1.4)
axs[0].set_ylabel('자리 오차 최대 [m]'); axs[1].set_ylabel('상대 속도 최대 [m/s]'); axs[1].set_xlabel('시간 [s]')
axs[0].legend(frameon=False, fontsize=8.5, ncol=3); axs[0].set_title('이상적 이중적분기 + 잡음 0.02 + 지연 0.15 s — 원 사전은 정지하지 못한다')
save(fig, 'fig4_dictionary_comparison')

# ---------- fig5: Stonefish 4x4 사각 런 ----------
d = np.genfromtxt('results/2026-10-06/koopman_model_sq4.csv', delimiter=',', names=True)
tt = d['t'] - d['t'][0]; g = np.stack([d[f'g{i}'] for i in range(12)], 1); pub = d['pub']
act = np.where((pub > 0) & (np.abs(g[:, :6]).sum(1) > 0))[0]; k0 = act[0] + 200
pos = g[k0:, :6].reshape(-1, 3, 2); tc = tt[k0:] - tt[k0]
ed = np.stack([np.abs(np.linalg.norm(pos[:, i] - pos[:, j], axis=1) - dd) for (i, j), dd in FORMATION.items()], 1)
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(tc, np.median(ed, 1), color=C['model'], lw=1.3, label='edge 오차 중앙(3변)')
ax.plot(tc, ed.max(1), color=C['model'], lw=0.8, alpha=0.4, label='edge 오차 최대')
ax.axhline(0.3, color=C['grey'], ls='--', lw=1); ax.text(20, 0.31, 'S6 기준: 중앙 < 0.3', ha='left', va='bottom', fontsize=8, color='#666666')
ax.axvspan(0, 13, color='#EEEEEE', label='정착 대기(리더 정지)')
ax.set_xlabel('제어 진입 후 시간 [s]'); ax.set_ylabel('edge 오차 [m]'); ax.set_ylim(0, 1.0); ax.legend(frameon=False, fontsize=8.5, ncol=3)
ax.set_title('Stonefish, controller:=model, 리더 4×4 m 사각 1.7바퀴 (LiDAR 추정 기반)')
save(fig, 'fig5_stonefish_sq4_edge')

# ---------- fig6: 플랜트 불일치 ----------
plants = ['공칭', '이득 0.7\n+지연 0.3 s', '이득 1.3', '1차 지연 0.3 s\n+항력 0.5', '이득 0.5+지연\n+항력']
ana = [0.157, 0.162, 0.156, 0.162, 0.162]; frz = [0.170, 0.166, 0.171, 0.165, 0.169]
fig, ax = plt.subplots(figsize=(7.5, 3)); x = np.arange(5); w = 0.36
ax.bar(x - w / 2, ana, w, color=C['analytic'], label='analytic'); ax.bar(x + w / 2, frz, w, color=C['model'], label='model (동결 Θ₀, 확장 사전)')
for i in range(5): ax.text(x[i] - w / 2, ana[i] + 0.004, f'{ana[i]:.3f}', ha='center', fontsize=7.5); ax.text(x[i] + w / 2, frz[i] + 0.004, f'{frz[i]:.3f}', ha='center', fontsize=7.5)
ax.set_xticks(x); ax.set_xticklabels(plants, fontsize=8.5); ax.set_ylabel('edge 오차 중앙 [m]'); ax.set_ylim(0, 0.25)
ax.legend(frameon=False, fontsize=8.5); ax.set_title('플랜트 불일치를 넣어도 동결 Θ₀는 거의 안 나빠진다 (수치 시뮬)')
save(fig, 'fig7_plant_mismatch')

# ---------- fig7: 온라인 적응 표류 ----------
fig, ax = plt.subplots(figsize=(6, 2.8)); labels = ['NLMS μ=0.5', 'NLMS μ=0.05', 'RLS λ=0.995']
d0 = [57, 6, 87]; d2 = [51, 6, 100]; x = np.arange(3); w = 0.36
ax.bar(x - w / 2, d0, w, color=C['grey'], label='잡음 0'); ax.bar(x + w / 2, d2, w, color=C['plain'], label='잡음 0.02 m')
ax.set_xticks(x); ax.set_xticklabels(labels); ax.set_ylabel('Θ 결합 블록 표류 [% of max]'); ax.legend(frameon=False, fontsize=8.5)
ax.set_title('제어는 동결, 적응기만 돌렸을 때 Θ 표류 — 잡음이 없어도 흘러간다')
save(fig, 'fig8_adapt_drift')
