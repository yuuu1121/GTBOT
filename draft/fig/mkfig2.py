"""explainer §11·§12 그림 → draft/fig/ (fig9~fig11). 실행: cd gtbot_ws && python3 draft/fig/mkfig2.py
fig9  사전 4종의 h스텝 J 예측 RMS 곡선(§12 표의 곡선판)
fig10 Stonefish 4×4 사각 런 4팔: 변 오차 최대 시계열 + 정착 후 분포
fig11 입력 통계: 포화율·부호 반전율 (수치 ρ=0/3 vs Stonefish)"""
import sys, os
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, 'src/gtbot_formation'); sys.path.insert(0, '.'); sys.path.insert(0, 'tools')
from gtbot_formation.relative_state import FORMATION
OUT = 'draft/fig'
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True,
                     'grid.color': '#E6E6E6', 'grid.linewidth': 0.6, 'font.family': ['Noto Sans CJK HK', 'Baekmuk Dotum', 'DejaVu Sans'],
                     'axes.unicode_minus': False})
COL = {'mpc': '#E45756', 'analytic': '#4C78A8', 'paper': '#54A24B', 'psi': '#B279A2'}

def save(fig, name):
    fig.tight_layout(); fig.savefig(f'{OUT}/{name}.png', dpi=160); plt.close(fig); print('saved', name)

# ---------- fig9: 사전별 h스텝 예측 오차 ----------
def fig9():
    from koopman_dictionary_mpc import SC, SPEC, hstep_err, PolyPhiDict, fit_lifted, identification_data
    data = identification_data(SC, seed=0); fig, ax = plt.subplots(figsize=(6.2, 3.6))
    names = {'phi': '[1; X; φ] (원논문 꼴)', 'p3phi': '+ poly3', 'p3phi1': '+ φ⊗x', 'p3phi2': '+ φ⊗x² (채택)'}
    for (key, (deg, pd)), c in zip(SPEC.items(), ['#9A9A9A', '#F58518', '#54A24B', '#E45756']):
        M = fit_lifted(SC, PolyPhiDict(SC, deg, pd), data, seed=0); he = hstep_err(M, data[::7][:300], np.random.default_rng(5))
        ax.plot(range(1, 11), he, 'o-', color=c, ms=4, label=f'{names[key]}  dim {M.n}')
    ax.set_xlabel('예측 지평 h [스텝, 0.05 s]'); ax.set_ylabel('J 예측 RMS (궤적 위 J 표준편차 8.2)'); ax.set_yscale('log'); ax.legend(fontsize=8.5)
    ax.set_title('사전을 키울수록 다스텝 오차가 쌓이지 않는다 (원 효용, 임의 입력 열린 루프)', fontsize=10)
    save(fig, 'fig9_dictionary_hstep')

# ---------- fig10: Stonefish 4팔 시계열 ----------
def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True); g = np.stack([d[f'g{i}'] for i in range(12)], 1); u = np.stack([d[f'u{i}'] for i in range(6)], 1)
    act = np.where((d['pub'] > 0) & (np.abs(g[:, :6]).sum(1) > 0))[0]; k0 = act[0] + 200
    t = d['t'][k0:] - d['t'][k0]; pos = g[k0:, :6].reshape(-1, 3, 2)
    ed = np.stack([np.abs(np.linalg.norm(pos[:, i] - pos[:, j], axis=1) - dd) for (i, j), dd in FORMATION.items()], 1)
    return t, ed, u[k0:]

RUNS = [('Koopman MPC (기본)', 'mpc', 'results/2026-10-06/koopman_mpc_sq4.csv'), ('analytic 1-step', 'analytic', 'results/2026-10-06/koopman_analytic_sq4.csv'),
        ('Koopman 1-step (논문 꼴)', 'paper', 'results/2026-10-06/koopman_model_paper_sq4.csv'), ('ψ 확장 사전 1-step', 'psi', 'results/2026-10-06/koopman_model_sq4.csv')]

def fig10():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw=dict(width_ratios=[2.6, 1]))
    box, labels, cols = [], [], []
    for name, key, path in RUNS:
        t, ed, u = load(path); m = t > 60; em = ed.max(1)
        a1.plot(t, np.convolve(em, np.ones(20) / 20, 'same'), color=COL[key], lw=1.0, label=name)
        box.append(ed[m].ravel()); labels.append({'mpc': 'MPC', 'analytic': 'analytic', 'paper': '논문 꼴\n1-step', 'psi': 'ψ 확장\n1-step'}[key]); cols.append(COL[key])
    a1.axvline(60, color='#999999', lw=0.8, ls='--'); a1.set_xlabel('제어 진입 후 시간 [s]'); a1.set_ylabel('변 오차 최대 [m] (1 s 이동평균)'); a1.set_ylim(0, 0.8); a1.legend(fontsize=8.5)
    a1.set_title('Stonefish 리더 4×4 m 사각 377 s — 점선 뒤가 정착 후 구간', fontsize=10)
    bp = a2.boxplot(box, showfliers=False, patch_artist=True, widths=0.6)
    for p, c in zip(bp['boxes'], cols): p.set_facecolor(c); p.set_alpha(0.5)
    a2.set_xticks(range(1, 5)); a2.set_xticklabels(labels, fontsize=8.5); a2.set_ylabel('변 오차 (정착 후, 세 변 전부) [m]'); a2.set_title('분포 (상자 = 사분위)', fontsize=10)
    save(fig, 'fig10_stonefish_four_arms')

# ---------- fig11: 입력 통계 ----------
def fig11():
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    rows = [('analytic\n수치', 0.11, 0.70), ('MPC ρ=0\n수치', 0.62, 0.89), ('MPC ρ=3\n수치', 0.02, 0.43)]
    for name, key, path in RUNS[:2]:
        t, ed, u = load(path); m = t > 60; uu = np.abs(u[m]); rows.append((name.replace(' ', '\n') + '\nStonefish', np.mean(uu >= 0.299), np.mean(np.sign(u[m][1:]) * np.sign(u[m][:-1]) < 0)))
    x = np.arange(len(rows)); ax.bar(x - 0.18, [r[1] for r in rows], 0.36, color='#E45756', label='포화율 (|U| = 0.3)'); ax.bar(x + 0.18, [r[2] for r in rows], 0.36, color='#4C78A8', label='틱간 부호 반전율')
    ax.set_xticks(x); ax.set_xticklabels([r[0] for r in rows], fontsize=8); ax.set_ylim(0, 1); ax.legend(fontsize=8.5)
    ax.set_title('MPC 이득의 대부분은 액추에이션이었다 — 입력 변화 패널티 ρ가 뱅뱅을 없앤다', fontsize=10)
    save(fig, 'fig11_input_stats')

if __name__ == '__main__':
    for f in (sys.argv[1:] or ['fig9', 'fig10', 'fig11']): globals()[f]()
