"""입사각 φ 두 경로 대조 — 파이프라인이 쓴 값 vs 점군에서 다시 잰 값.

왜 필요한가: 녹화 도구(record_plate_cluster.py)가 화면에 쓰는 φ는 자체 클러스터링·
장축 적합에서 나오고, 실제로 검출 생존을 결정하는 φ는 platform_perception이
ouster_cluster의 검출 헤딩으로 계산한다. 두 값이 어긋나면 화면 숫자가 파이프라인을
대변하지 못한다. 같은 런에서 둘 다 로깅해 짝지어 비교한다.

짝짓기: platform_perception의 origin은 월드축 정렬(R(+yaw_p)·(x,−y))이고 녹화 도구의
중심은 LiDAR FLU 원본이라, 로그에 같이 남긴 yaw_p로 FLU를 월드축으로 옮긴 뒤
같은 시각(±0.15 s)·같은 위치(±0.15 m)인 것만 짝으로 본다.

사용: compare_phi.py <pp_phi.csv> <rec_phi.csv>
"""
import sys

import numpy as np

pp = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
rc = np.genfromtxt(sys.argv[2], delimiter=',', names=True)
print(f'platform_perception {len(pp)}행, 녹화도구 {len(rc)}행')

pairs = []
for r in rc:
    w = np.abs(pp['t'] - r['t']) < 0.15
    if not w.any():
        continue
    cand = pp[w]
    # FLU -> 월드축: (x, -y) 회전 R(+yaw_p). yaw_p는 같은 행에서 읽는다.
    for c in cand:
        ca, sa = np.cos(c['yaw_p']), np.sin(c['yaw_p'])
        wx = ca * r['cx'] - sa * (-r['cy'])
        wy = sa * r['cx'] + ca * (-r['cy'])
        if np.hypot(wx - c['ox'], wy - c['oy']) < 0.15:
            pairs.append((c['phi'], r['phi'], c['kept']))
            break

if not pairs:
    print('짝지어진 검출이 없다 — 시각·좌표 허용오차 또는 프레임 규약을 다시 봐야 한다.')
    raise SystemExit(1)

a = np.array(pairs)                       # [pp_phi, rec_phi, kept]
d = a[:, 1] - a[:, 0]
print(f'짝 {len(a)}개')
print(f'  platform_perception φ  중앙 {np.median(a[:,0]):6.2f}°  '
      f'5~95% {np.percentile(a[:,0],5):.1f}~{np.percentile(a[:,0],95):.1f}°')
print(f'  녹화도구 φ             중앙 {np.median(a[:,1]):6.2f}°  '
      f'5~95% {np.percentile(a[:,1],5):.1f}~{np.percentile(a[:,1],95):.1f}°')
print(f'  차이(녹화−파이프라인)  중앙 {np.median(d):6.2f}°  '
      f'평균 {d.mean():6.2f}°  표준편차 {d.std():.2f}°')
print(f'  상관계수 {np.corrcoef(a[:,0], a[:,1])[0,1]:.3f}')
print(f'  검출 생존율 {100*a[:,2].mean():.1f}%')
