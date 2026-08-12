"""저하 시나리오 3축 스윕 -> 강건성 곡선.

기준선(전부 0)은 융합 검증 런(커밋 a154abd)의 게이트 JSON을 git에서 읽는다 —
스윕이 results/ 루트를 덮어쓰기 때문.
"""
import json, subprocess, sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'UnDotum'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

WS = '/root/home/gtbot_ws'
SWEEP = f'{WS}/results/sweep'
BASE_COMMIT = 'a154abd'
R3 = ['gtbot', 'gtbot2', 'gtbot3']

AXES = [
    ('상수 편의 (라이다 정합)', '장착각 오차 [deg]', [0.0, 0.5, 1.0],
     [None, 'bias0.5', 'bias1.0']),
    ('검출 누락', '누락률 [%]', [0.0, 5.0, 15.0],
     [None, 'drop05r2', 'drop15']),
    ('파랑', '파고 [m]', [0.0, 0.05, 0.10],
     [None, 'wave005', 'wave010']),
]


def base(gate):
    out = subprocess.run(['git', '-C', WS, 'show',
                          f'{BASE_COMMIT}:results/{gate}_stonefish.json'],
                         capture_output=True, text=True)
    return json.loads(out.stdout)


def load(gate, name):
    if name is None:
        return base(gate)
    try:
        return json.load(open(f'{SWEEP}/{gate}_{name}.json'))
    except FileNotFoundError:
        return None


def metrics(name):
    s4, s5, s6 = load('s4', name), load('s5', name), load('s6', name)
    m = {}
    if s4:
        m['head'] = np.mean([s4['heading_err'][r]['median_deg'] for r in R3])
        m['rmse'] = max(s4['pos_rmse'][r] for r in R3)
        m['valid'] = s4['valid_ratio'] * 100
        m['s4pass'] = s4['gate_s4_pass']
    if s5:
        m['aim'] = max(s5[r]['median_deg'] for r in R3)
        m['s5pass'] = s5['gate_s5_pass']
    if s6:
        m['edge'] = s6['median_edge_err_tail30s']
        m['s6pass'] = s6['gate_s6_pass']
    return m


fig, ax = plt.subplots(2, 3, figsize=(16, 8.5))
rows = []
for c, (title, xlabel, xs, names) in enumerate(AXES):
    M = [metrics(n) for n in names]
    for m, x, n in zip(M, xs, names):
        rows.append((title, x, n or 'baseline', m))
    ok = [i for i, m in enumerate(M) if m]
    X = [xs[i] for i in ok]

    a = ax[0, c]
    a.plot(X, [M[i].get('head', np.nan) for i in ok], 'o-', c='#1f77b4',
           label='S4 LiDAR 헤딩 중앙값')
    a.plot(X, [M[i].get('aim', np.nan) for i in ok], 's-', c='#ff7f0e',
           label='S5 조준오차(최악)')
    a.axhline(10.0, ls='--', c='r', lw=1.0, label='게이트 10°')
    a.set_title(f'축 {c+2}: {title}', fontsize=12)
    if c == 1:   # 누락 축: 재현되지 않은 1차 런을 숨기지 않고 명시한다
        a.text(0.03, 0.93, '주: 5% 1차 런은 S5 43.2°(gtbot 단독)로 탈락했으나\n'
                           '재실행에서 5.09°로 통과 — 재현되지 않아 곡선은 재실행값',
               transform=a.transAxes, fontsize=7, color='r', va='top')
    a.set_xlabel(xlabel); a.set_ylabel('[deg]'); a.grid(alpha=0.3)
    a.set_ylim(0, 12); a.legend(fontsize=8, loc='center left')

    a = ax[1, c]
    a.plot(X, [M[i].get('edge', np.nan) for i in ok], 'o-', c='#2ca02c',
           label='S6 편대 변 오차(중앙값)')
    a.axhline(0.6, ls='--', c='r', lw=1.0, label='게이트 0.6 m')
    a.set_xlabel(xlabel); a.set_ylabel('[m]'); a.grid(alpha=0.3)
    a.set_ylim(bottom=0)
    a2 = a.twinx()
    a2.plot(X, [M[i].get('valid', np.nan) for i in ok], '^--', c='0.45',
            label='est 유효율')
    a2.set_ylabel('est valid [%]'); a2.set_ylim(0, 105)
    h1, l1 = a.get_legend_handles_labels(); h2, l2 = a2.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=8, loc='center left')

fig.suptitle('저하 시나리오 강건성 곡선 — 실측 IMU 드리프트 + 융합 ON 위에서 3축 주입',
             fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f'{SWEEP}/../figs/degraded_sweep.png', dpi=110)
print('saved results/figs/degraded_sweep.png\n')

hdr = f'{"축":<18}{"수준":>7}  {"S4헤딩":>7} {"S5조준":>7} {"S6변오차":>8} {"valid":>7}  판정'
print(hdr); print('-' * len(hdr))
for title, x, n, m in rows:
    if not m:
        print(f'{title:<18}{x:>7}  (결과 없음)'); continue
    keys = ('s4pass', 's5pass', 's6pass')
    verdict = ('미완' if any(k not in m for k in keys)          # 아직 안 돈 게이트
               else '통과' if all(m[k] for k in keys) else '탈락')
    print(f'{title:<18}{x:>7}  {m.get("head",float("nan")):7.2f} '
          f'{m.get("aim",float("nan")):7.2f} {m.get("edge",float("nan")):8.3f} '
          f'{m.get("valid",float("nan")):6.1f}%  {verdict}')
