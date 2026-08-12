"""복합 조건 + 구성당 3런 -> 반복 산포가 붙은 강건성 요약.

1런짜리 스윕의 한계(런 간 산포를 효과와 구분 못 함)를 메우는 것이 목적이므로,
모든 값은 런 평균 ± 범위(min~max)로 낸다. 표본 3이라 표준편차 대신 범위를 쓴다.
"""
import json, glob, os, re
import numpy as np
import matplotlib; matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'UnDotum'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

WS = '/root/home/gtbot_ws'
SWEEP = f'{WS}/results/sweep'
R3 = ['gtbot', 'gtbot2', 'gtbot3']

# (표시이름, 파일 접두, 설명)
COMPOSITE = [
    ('기준선',   'base',       'bias 0°, 누락 0%, 파고 0'),
    ('복합 약',  'comp_mild',  'bias 0.5°, 누락 5%, 파고 5 cm'),
    ('복합 강',  'comp_harsh', 'bias 1.0°, 누락 15%, 파고 10 cm'),
    ('복합 초과', 'comp_over',  'bias 2.0°, 누락 30%, 파고 20 cm'),
]
SINGLE = [
    ('bias 0.5°', 'bias0.5'), ('bias 1.0°', 'bias1.0'),
    ('누락 5%', 'drop05'), ('누락 15%', 'drop15'),
    ('파고 5 cm', 'wave005'), ('파고 10 cm', 'wave010'),
]


def runs_of(prefix):
    """접두에 해당하는 모든 런의 지표를 모은다. 접미 없는 1차 스윕 파일도 포함."""
    out = []
    tags = sorted(set(
        re.sub(r'^s\d_', '', os.path.basename(f))[:-5]
        for f in glob.glob(f'{SWEEP}/s4_{prefix}*.json')))
    for tag in tags:
        if tag != prefix and not re.fullmatch(rf'{re.escape(prefix)}(_r\d+|r\d+)', tag):
            continue                       # 다른 구성이 접두를 공유하는 경우 배제
        m = {}
        for g, key in (('s4', None), ('s5', None), ('s6', None)):
            p = f'{SWEEP}/{g}_{tag}.json'
            if not os.path.exists(p):
                continue
            d = json.load(open(p))
            if g == 's4':
                m['head'] = np.mean([d['heading_err'][r]['median_deg'] for r in R3])
                m['rmse'] = max(d['pos_rmse'][r] for r in R3)
                for r in R3:
                    m[f'rmse_{r}'] = d['pos_rmse'][r]
                m['valid'] = d['valid_ratio'] * 100
                m['s4'] = d['gate_s4_pass']
            elif g == 's5':
                m['aim'] = max(d[r]['median_deg'] for r in R3)
                m['s5'] = d['gate_s5_pass']
            else:
                m['edge'] = d['median_edge_err_tail30s']
                m['s6'] = d['gate_s6_pass']
        if m:
            m['tag'] = tag
            out.append(m)
    return out


def agg(rs, key):
    v = [r[key] for r in rs if key in r]
    return (np.mean(v), min(v), max(v), len(v)) if v else (np.nan,) * 3 + (0,)


def verdict(rs):
    full = [r for r in rs if all(k in r for k in ('s4', 's5', 's6'))]
    if not full:
        return '미완'
    n = sum(all(r[k] for k in ('s4', 's5', 's6')) for r in full)
    return f'{n}/{len(full)} 통과'


def report(title, items):
    print(f'\n===== {title} =====')
    hdr = (f'{"구성":<12}{"런":>3}  {"S4헤딩[°]":>16} {"S5조준[°]":>16} '
           f'{"S6변오차[m]":>18} {"valid":>7}  판정')
    print(hdr); print('-' * (len(hdr) + 8))
    rows = []
    for name, prefix, *rest in items:
        rs = runs_of(prefix)
        if not rs:
            print(f'{name:<12}{"-":>3}  (결과 없음)'); continue
        h, s, e = agg(rs, 'head'), agg(rs, 'aim'), agg(rs, 'edge')
        vd = agg(rs, 'valid')
        print(f'{name:<12}{len(rs):>3}  {h[0]:6.2f} ({h[1]:.2f}~{h[2]:.2f}) '
              f'{s[0]:6.2f} ({s[1]:.2f}~{s[2]:.2f}) '
              f'{e[0]:7.3f} ({e[1]:.3f}~{e[2]:.3f}) {vd[0]:6.1f}%  {verdict(rs)}')
        rows.append((name, rs, h, s, e, vd))
    return rows


comp_rows = report('복합 조건 (구성당 3런)', COMPOSITE)
sing_rows = report('단일축 (구성당 3런)', SINGLE)

fig, ax = plt.subplots(1, 4, figsize=(20, 5))
labels = [r[0] for r in comp_rows]
x = np.arange(len(labels))
for a, (idx, ylab, gate, ttl) in zip(ax, [
        (2, 'S4 LiDAR 헤딩 [deg]', None, '지각 정밀도'),
        (3, 'S5 조준오차 [deg]', 10.0, '헤딩 추종'),
        (4, 'S6 편대 변 오차 [m]', 0.6, '임무 성능')]):
    mean = [r[idx][0] for r in comp_rows]
    lo = [r[idx][0] - r[idx][1] for r in comp_rows]
    hi = [r[idx][2] - r[idx][0] for r in comp_rows]
    a.errorbar(x, mean, yerr=[lo, hi], fmt='o-', capsize=5, c='#1f77b4')
    if gate:
        a.axhline(gate, ls='--', c='r', lw=1.0, label=f'게이트 {gate}')
        a.legend(fontsize=8)
    a.set_xticks(x); a.set_xticklabels(labels, fontsize=9)
    a.set_ylabel(ylab); a.set_title(ttl, fontsize=12); a.grid(alpha=0.3)
    a.set_ylim(bottom=0)

# 4번째 패널: 로봇별 위치 오차 — 이번 캠페인의 핵심 발견(gtbot 편중)
a = ax[3]
for r, col in zip(R3, ['#d62728', '#2ca02c', '#1f77b4']):
    mean = [agg(row[1], f'rmse_{r}')[0] for row in comp_rows]
    lo = [mean[i] - agg(row[1], f'rmse_{r}')[1] for i, row in enumerate(comp_rows)]
    hi = [agg(row[1], f'rmse_{r}')[2] - mean[i] for i, row in enumerate(comp_rows)]
    a.errorbar(x, mean, yerr=[lo, hi], fmt='o-', capsize=4, c=col, label=r)
a.axhline(0.1, ls='--', c='r', lw=1.0, label='게이트 0.1 m')
a.set_xticks(x); a.set_xticklabels(labels, fontsize=9)
a.set_ylabel('S4 위치 RMSE [m]'); a.set_title('로봇별 위치 오차 — gtbot 편중', fontsize=12)
a.grid(alpha=0.3); a.legend(fontsize=8); a.set_ylim(bottom=0)

fig.suptitle('복합 저하 조건 — 구성당 3런 평균 ± 범위 (실측 IMU 드리프트 + 융합 ON 위)',
             fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(f'{WS}/results/figs/composite_sweep.png', dpi=110)
print('\nsaved results/figs/composite_sweep.png')
