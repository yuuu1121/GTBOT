"""되먹임 시험 조건당 다중 런 집계 — 산포를 드러내 정성/정량 결론을 가른다.

조건당 1런으로는 회복 시간·검출 실패 시간의 스윙(실측: 순한 곡선에서 62.3 -> 4.3 s)을
런 간 산포와 분리할 수 없다. 이 도구는 조건별 여러 런을 모아 중앙값과 전 범위를 함께
찍어, 어떤 지표가 반복에 견디는지 보여준다.

판정 지표는 plot_feedback.py와 같은 정의:
  d_tail   말미 30 s 조준 이탈 중앙(판이 누웠는가)
  v_tail   말미 30 s est 유효율(지각이 살아있는가)
  lost_s   교란 해제 후 검출 실패 누적 시간
  rec      교란 해제 후 5 s 연속 δ<15°가 처음 성립하기까지
  sustained  말미가 회복 상태인가(d_tail<15° 이고 v_tail>80%)

사용: feedback_agg.py <조건라벨>=<glob> [<조건라벨>=<glob> ...]
"""
import glob
import sys

import numpy as np

R = 'gtbot'          # 교란을 받은 로봇


def one(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['t']
    push = d['push'] > 0.5
    t_end = t[push].max() if push.any() else 38.0
    tail = t > t.max() - 30
    post = t > t_end
    dd, vv = d[f'{R}_delta_deg'], d[f'{R}_valid']
    rec = np.nan
    for i in np.where((dd < 15.0) & post)[0]:
        w = (t >= t[i]) & (t < t[i] + 5.0)
        if w.sum() > 5 and (dd[w] < 15.0).all():
            rec = t[i] - t_end
            break
    d_tail, v_tail = np.median(dd[tail]), np.mean(vv[tail]) * 100
    return dict(d_tail=d_tail, v_tail=v_tail,
                lost_s=0.1 * (post & (vv < 0.5)).sum(),
                rec=rec, smax=d[f'{R}_station_err'][post].max(),
                sustained=bool(d_tail < 15.0 and v_tail > 80.0))


def rng(v):
    v = np.array([x for x in v if not np.isnan(x)])
    if not len(v):
        return '     — '
    return f'{np.median(v):6.1f} [{v.min():.1f}~{v.max():.1f}]'


for arg in sys.argv[1:]:
    label, pat = arg.split('=', 1)
    paths = sorted(glob.glob(pat))
    if not paths:
        print(f'{label}: 파일 없음 ({pat})')
        continue
    rs = [one(p) for p in paths]
    ns = sum(r['sustained'] for r in rs)
    print(f'--- {label}  ({len(rs)}런)')
    print(f'    조준δ 말미   {rng([r["d_tail"] for r in rs])} °')
    print(f'    유효율 말미  {rng([r["v_tail"] for r in rs])} %')
    print(f'    검출 실패    {rng([r["lost_s"] for r in rs])} s')
    print(f'    첫 회복      {rng([r["rec"] for r in rs])} s')
    print(f'    최대 station {rng([r["smax"] * 100 for r in rs])} cm')
    print(f'    말미 회복 유지  {ns}/{len(rs)}')
