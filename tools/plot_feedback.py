"""되먹임 시험 비교 — 교란 후 복귀하는가?

대조군(uniform, 입사각 무관)과 시험군(incidence, 입사각 의존)을 나란히 놓는다.
판정: 교란 해제 후 gtbot의 조준이탈 δ와 스테이션 오차가 기준선으로 돌아오는가.
돌아오지 않으면 '판이 눕음 -> 검출 사망 -> 복귀 불가'의 양의 되먹임이 실재한다.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'UnDotum'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

FB = '/root/home/gtbot_ws/results/feedback'
TAG = sys.argv[1] if len(sys.argv) > 1 else 'r1'
# 세 조건: 대조군 / 순한 곡선(φz=50, 가정) / 실측 맞춤 곡선(φz=65.4, 이동2 bag 앵커)
CONDS = [('대조군 uniform', f'{FB}/fb_uniform_{TAG}.csv', '#1f77b4'),
         ('incidence 순한 φz50', f'{FB}/fb_incidence_{TAG}.csv', '#ff7f0e'),
         ('incidence 실측 φz65', f'{FB}/fb_incidence_real.csv', '#d62728')]
R = 'gtbot'          # 교란을 받은 로봇


def load(p):
    return np.genfromtxt(p, delimiter=',', names=True) if os.path.exists(p) else None


fig, ax = plt.subplots(2, 2, figsize=(15, 8))
summary = []
for name, path, col in CONDS:
    d = load(path)
    if d is None:
        print(f'{name}: 파일 없음 ({path})')
        continue
    t = d['t']
    push = d['push'] > 0.5
    t_end = t[push].max() if push.any() else 38.0
    base = (t > 5) & (t < 28)                       # 교란 전 기준선
    tail = t > t.max() - 30                          # 관찰 말미 30 s

    for a, key, ylab, ttl in [
            (ax[0, 0], f'{R}_delta_deg', '조준 이탈 δ [deg]', 'gtbot 조준 이탈 — 판이 얼마나 누웠나'),
            (ax[0, 1], f'{R}_station_err', '스테이션 오차 [m]', 'gtbot 스테이션 오차 — 복귀했나'),
            (ax[1, 0], f'{R}_valid', 'est 유효(0/1)', 'gtbot 검출 유효 — 지각이 살아있나')]:
        y = d[key]
        if key.endswith('valid'):                    # 유효는 2 s 이동평균으로 읽기 쉽게
            w = 20
            y = np.convolve(y, np.ones(w) / w, mode='same')
        a.plot(t, y, c=col, lw=1.1, label=name)
        a.set_ylabel(ylab); a.set_title(ttl, fontsize=11); a.grid(alpha=0.3)

    s = dict(name=name,
             d_base=np.median(d[f'{R}_delta_deg'][base]),
             d_tail=np.median(d[f'{R}_delta_deg'][tail]),
             s_base=np.median(d[f'{R}_station_err'][base]),
             s_tail=np.median(d[f'{R}_station_err'][tail]),
             v_base=np.mean(d[f'{R}_valid'][base]) * 100,
             v_post=np.mean(d[f'{R}_valid'][t > t_end]) * 100,
             v_tail=np.mean(d[f'{R}_valid'][tail]) * 100)
    summary.append(s)

    # 위험 노출 지표 — 말미 30 s만 보면 둘 다 '복귀함'이라 차이가 통째로 가려진다.
    post = t > t_end
    ok = (d[f'{R}_delta_deg'] < 15.0) & post
    rec = np.nan
    for i in np.where(ok)[0]:
        w = (t >= t[i]) & (t < t[i] + 5.0)
        if w.sum() > 5 and (d[f'{R}_delta_deg'][w] < 15.0).all():
            rec = t[i] - t_end
            break
    s['rec'] = rec
    s['lost_s'] = 0.1 * (post & (d[f'{R}_valid'] < 0.5)).sum()
    s['smax'] = d[f'{R}_station_err'][post].max()
    # '첫 회복'만 보면 재발을 놓친다 — 말미 30 s가 회복 상태인지 따로 판정한다.
    s['sustained'] = bool(s['d_tail'] < 15.0 and s['v_tail'] > 80.0)

for a in ax.ravel()[:3]:
    a.axvspan(30, 38, color='k', alpha=.12)
    a.set_xlabel('t [s]'); a.legend(fontsize=8)
ax[0, 0].text(31, ax[0, 0].get_ylim()[1] * .92, '교란(스핀)', fontsize=8)
a = ax[1, 1]
names = [s['name'].replace(' ', '\n', 1) for s in summary]
xs = np.arange(len(summary))
w = 0.26
a.bar(xs - w, [s['rec'] for s in summary], w, color='#1f77b4', label='회복까지 [s]')
a.bar(xs, [s['lost_s'] for s in summary], w, color='#d62728', label='검출 실패 [s]')
a.bar(xs + w, [s['smax'] * 100 for s in summary], w, color='#2ca02c', label='최대 station [cm]')
for i, s in enumerate(summary):
    a.text(i - w, s['rec'] + 2, f"{s['rec']:.0f}", ha='center', fontsize=8)
    a.text(i, s['lost_s'] + 2, f"{s['lost_s']:.0f}", ha='center', fontsize=8)
    a.text(i + w, s['smax'] * 100 + 2, f"{s['smax']*100:.0f}", ha='center', fontsize=8)
a.set_xticks(xs); a.set_xticklabels(names)
a.set_title('위험 노출 — 말미만 보면 안 보이는 차이', fontsize=11)
a.legend(fontsize=8); a.grid(alpha=.3, axis='y')

fig.suptitle('되먹임 고리 시험 — 판이 누우면 검출이 죽어 복귀가 막히는가', fontsize=13)
fig.tight_layout(rect=[0, 0, 1, .95])
fig.savefig('/root/home/gtbot_ws/results/figs/feedback_probe.png', dpi=110)
print('saved results/figs/feedback_probe.png\n')

hdr = f'{"조건":<18}{"δ 기준선":>10}{"δ 말미":>9}{"station 기준":>13}{"station 말미":>13}{"valid 기준":>11}{"valid 교란후":>12}{"valid 말미":>11}'
print(hdr); print('-' * len(hdr))
for s in summary:
    print(f'{s["name"]:<18}{s["d_base"]:9.2f}°{s["d_tail"]:8.2f}°{s["s_base"]:12.3f}m'
          f'{s["s_tail"]:12.3f}m{s["v_base"]:10.1f}%{s["v_post"]:11.1f}%{s["v_tail"]:10.1f}%')
print()
for s in summary:
    print(f'{s["name"]}: 첫 회복 +{s["rec"]:.1f} s, 검출 실패 {s["lost_s"]:.1f} s, '
          f'최대 station {s["smax"]:.3f} m, '
          f'말미 상태 {"회복 유지" if s["sustained"] else "**재발(갇힘)**"}')
