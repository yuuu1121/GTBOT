"""입사각 φ 세 값 대조 — 기준점이 로봇 중심인가 판 중심인가.

배경: platform_perception은 한때 검출 생존 판정의 φ를 `origin`(= 판 중심에서
marker_offset을 뺀 **로봇 중심**) 기준 베어링으로 계산했다. 판은 옆으로 0.1386 m
물러나 있어 판 자신의 입사각은 시차 γ = atan(0.1386 / r)만큼(1 m에서 7.8°) 다르고,
그만큼 검출이 늦게 죽었다. 이 도구가 그 인과를 확정했고 기준점은 판 중심으로
정정됐다(2026-08-15). 이후로는 정정이 유지되는지 보는 회귀 도구다. 세 값:

  phi        파이프라인이 쓰는 값 — 2026-08-15 정정 후 **판 중심** 기준
  phi_robot  정정 전 기준(로봇 중심). 진단용으로만 로깅한다.
  rec        녹화 도구가 점군 장축과 판까지의 시선으로 잰 값(기하 실측)

정정 전에는 rec − phi가 −8.2°였고 rec − phi_plate는 −0.8°였다 — 그래서 기준점
시차가 원인으로 확정됐다. 정정 후에는 rec − phi가 −0.8°로 붙는 것이 정상이다.

짝짓기는 판 중심(px, py)과 녹화 도구의 FLU 중심을 yaw_p로 같은 프레임에 놓고
같은 시각(±0.15 s)·같은 위치(±0.08 m)인 것만 인정한다 — 허용오차를 오프셋
0.1386 m보다 작게 잡아야 로봇 중심과 판 중심을 혼동하지 않는다.

사용: compare_phi.py <pp_phi.csv> <rec_phi.csv>
"""
import sys

import numpy as np

pp = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
rc = np.genfromtxt(sys.argv[2], delimiter=',', names=True)
# 기준점 정정(2026-08-15) 후 로그의 phi는 판 중심 값이고, phi_robot이 옛 로봇 중심 값이다.
has_old = 'phi_robot' in (pp.dtype.names or ())
print(f'platform_perception {len(pp)}행, 녹화도구 {len(rc)}행'
      f'{"" if has_old else "  (구 포맷: phi_robot 없음)"}')

rows = []
for r in rc:
    w = np.abs(pp['t'] - r['t']) < 0.15
    if not w.any():
        continue
    for c in pp[w]:
        ca, sa = np.cos(c['yaw_p']), np.sin(c['yaw_p'])
        wx = ca * r['cx'] - sa * (-r['cy'])
        wy = sa * r['cx'] + ca * (-r['cy'])
        # 기준점: 판 중심이 있으면 그것으로, 없으면(구 포맷) 로봇 중심으로 느슨하게
        tx, ty, tol = ((c['px'], c['py'], 0.08) if has_old
                       else (c['ox'], c['oy'], 0.15))
        if np.hypot(wx - tx, wy - ty) < tol:
            rows.append((c['phi'], c['phi_robot'] if has_old else np.nan,
                         r['phi'], np.hypot(wx, wy), c['kept']))
            break

if not rows:
    print('짝지어진 검출이 없다 — 시각·좌표 허용오차 또는 프레임 규약을 다시 봐야 한다.')
    raise SystemExit(1)

a = np.array(rows)                    # [phi(판중심), phi_robot, rec, r_plate, kept]
print(f'짝 {len(a)}개, 판까지 거리 중앙 {np.median(a[:, 3]):.2f} m')
for name, col in [('파이프라인 φ (판 중심, 현행)', 0),
                  ('φ_robot  (로봇 중심, 정정 전)', 1), ('녹화 도구 φ (기하 실측)', 2)]:
    v = a[:, col]
    if np.all(np.isnan(v)):
        continue
    print(f'  {name:<24} 중앙 {np.nanmedian(v):6.2f}°  '
          f'5~95% {np.nanpercentile(v,5):.1f}~{np.nanpercentile(v,95):.1f}°')

for name, col in [('녹화 − 파이프라인', 0), ('녹화 − φ_robot', 1)]:
    if np.all(np.isnan(a[:, col])):
        continue
    d = a[:, 2] - a[:, col]
    print(f'  차이 {name:<18} 중앙 {np.nanmedian(d):6.2f}°  '
          f'표준편차 {np.nanstd(d):.2f}°  '
          f'상관 {np.corrcoef(a[~np.isnan(a[:,col])][:,col], a[~np.isnan(a[:,col])][:,2])[0,1]:.3f}')

# 시차 예측: γ = atan(0.1386 / r). 파이프라인과 판 기준의 차이가 이만큼이어야 한다.
if has_old:
    g = np.degrees(np.arctan2(0.1386, a[:, 3]))
    dd = np.abs(a[:, 0] - a[:, 1])
    print(f'  시차 예측 γ 중앙 {np.median(g):.2f}°  '
          f'vs 실제 |판중심 − 로봇중심| 중앙 {np.median(dd):.2f}°')
