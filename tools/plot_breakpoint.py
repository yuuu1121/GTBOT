"""파괴점 — 검출 누락률 vs est 유효율·판정. 절벽이 얼마나 날카로운지 보이는 것이 목적."""
import json, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'UnDotum'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

R = ['gtbot', 'gtbot2', 'gtbot3']
CFG = [(0, 'base_fix'), (50, 'bp2_d50'), (60, 'bp2_d60'), (70, 'bp2_d70')]
xs, vm, vlo, vhi, npass, ntot = [], [], [], [], [], []
for pct, tag in CFG:
    v, p, n = [], 0, 0
    for r in [1, 2, 3]:
        p4 = f'results/sweep/s4_{tag}_r{r}.json'
        if not os.path.exists(p4):
            continue
        d4 = json.load(open(p4)); v.append(d4['valid_ratio'] * 100); n += 1
        p6 = f'results/sweep/s6_{tag}_r{r}.json'
        p5 = f'results/sweep/s5_{tag}_r{r}.json'
        ok = (d4['gate_s4_pass'] and os.path.exists(p5) and os.path.exists(p6)
              and json.load(open(p5))['gate_s5_pass'] and json.load(open(p6))['gate_s6_pass'])
        p += bool(ok)
    if not v:
        continue
    xs.append(pct); vm.append(np.mean(v)); vlo.append(min(v)); vhi.append(max(v))
    npass.append(p); ntot.append(n)

fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
a = ax[0]
a.errorbar(xs, vm, yerr=[np.array(vm) - vlo, np.array(vhi) - np.array(vm)],
           fmt='o-', capsize=5, c='#1f77b4')
a.axhspan(85.7, 91.1, color='r', alpha=0.12)
a.text(2, 88.4, '통과/실패 경계대\n(실패 최대 85.7% ~ 통과 최소 91.1%)', fontsize=8, color='r', va='center')
a.set_xlabel('검출 누락률 [%]'); a.set_ylabel('est 유효율 [%]')
a.set_title('지각 갱신률 — 누락이 유효율을 끌어내린다', fontsize=11); a.grid(alpha=0.3)

a = ax[1]
col = ['#2ca02c' if p == t else '#d62728' for p, t in zip(npass, ntot)]
a.bar([str(x) for x in xs], [p / t * 100 for p, t in zip(npass, ntot)], color=col, width=0.55)
for i, (p, t) in enumerate(zip(npass, ntot)):
    a.text(i, p / t * 100 + 3, f'{p}/{t}', ha='center', fontsize=10)
a.set_ylim(0, 115); a.set_xlabel('검출 누락률 [%]'); a.set_ylabel('전 게이트 통과 런 [%]')
a.set_title('절벽 — 60% 3/3 통과, 70% 3/3 정착 실패', fontsize=11); a.grid(alpha=0.3, axis='y')

fig.suptitle('파괴점: 검출 누락률 (구성당 3런, 플랫폼 드리프트 포함 실측 IMU 위)', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig('results/figs/breakpoint_dropout.png', dpi=110)
print('saved results/figs/breakpoint_dropout.png')
