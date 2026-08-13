"""가속도계 y축이 죽은 채널인지 판정 — 기울임 시험 로그 1개면 끝난다.

배경(imu_run1, 정지 55분): 16만 샘플 내내 accel y가 **정확히 0.0** 하나뿐이었다.
x·z는 고유값이 3~4개(심한 양자화)였으니 y도 양자화 레벨 0에 머문 것일 수 있고,
채널이 아예 죽은 것일 수도 있다. 정지 로그로는 이 둘을 구분할 수 없다 —
참값이 0에 가까우면 두 경우가 같은 데이터를 낸다.

판정 원리: **센서를 옆으로 기울이면 중력이 y축에 실린다.** 45° 기울이면
y ≈ ±6.9 m/s²가 나와야 한다. 살아 있으면 반응하고, 죽었으면 0에 머문다 —
참값을 몰라도 되고 기울기 각도를 정확히 맞출 필요도 없다.

시험 절차(2분):
  1. ros2 bag record /imu/ddpm /imu/raw   (또는 리맵 후 /<robot>/imu)
  2. 센서를 평평하게 20초 정지
  3. 왼쪽으로 크게(45° 이상) 기울여 20초 정지
  4. 오른쪽으로 크게 기울여 20초 정지
  5. python3 tools/imu_accel_y_check.py <bag>/<bag>_0.db3

사용: python3 tools/imu_accel_y_check.py imu_tilt/imu_tilt_0.db3
"""
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

BAG = sys.argv[1]
LIVE_MIN = 1.0        # m/s^2 — 이 이상 변동하면 채널이 살아 있다고 본다(45° 기대치의 1/7)

con = sqlite3.connect(BAG)
topics = [r[0] for r in con.execute("SELECT name FROM topics WHERE type LIKE '%Imu%'")]
if not topics:
    raise SystemExit('bag에 sensor_msgs/Imu 토픽이 없다')

print(f'{"토픽":<16}{"축":>4}{"평균":>10}{"표준편차":>10}{"최소":>10}{"최대":>10}{"고유값":>8}  판정')
verdicts = {}
tilted = {}      # 토픽별: 이 로그에 실제로 기울임이 있었나(x 또는 z가 크게 변동)
for t in topics:
    tid = con.execute('SELECT id FROM topics WHERE name=?', (t,)).fetchone()[0]
    rows = con.execute('SELECT data FROM messages WHERE topic_id=?', (tid,)).fetchall()
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
                  for m in (deserialize_message(r[0], Imu) for r in rows)])
    if len(a) == 0:
        continue
    for i, ax in enumerate('xyz'):
        v = a[:, i]
        span = float(np.ptp(v))
        live = span >= LIVE_MIN
        if ax == 'y':
            verdicts[t] = live
        else:
            tilted[t] = tilted.get(t, False) or live
        print(f'{t if i == 0 else "":<16}{ax:>4}{v.mean():10.4f}{v.std():10.4f}'
              f'{v.min():10.4f}{v.max():10.4f}{len(np.unique(v)):8d}  '
              f'{"살아있음" if live else "무반응"}')

print()
for t, live in verdicts.items():
    # 기울임이 실제로 있었는지 먼저 확인 — 정지 로그를 넣으면 y는 당연히 무반응이고,
    # 그걸 '죽은 채널'로 읽으면 오판이다. x·z 중 하나라도 크게 움직여야 유효한 시험이다.
    if not tilted.get(t, False):
        print(f'{t}: **판정 불가 — 이 로그에는 기울임이 없다**(x·z 변동 < {LIVE_MIN} m/s²). '
              f'독스트링의 절차대로 좌·우로 45° 이상 기울인 로그가 필요하다.')
        continue
    if live:
        print(f'{t}: y축 정상 — 정지 로그의 0.0은 완벽 정렬 + 양자화였다. '
              f'scn linear_acceleration y를 실측 표준편차로 갱신할 것.')
    else:
        print(f'{t}: **y축 무반응 — 죽은 채널이다.** 기울였는데도 안 변한다면 드라이버가 '
              f'채널을 안 채우거나 센서 고장이다. roll 추정이 중력 기준을 하나 잃으므로 '
              f'실기 헤딩·자세 결론에 영향이 있다 — 논문에 한계로 명시하거나 교체할 것.')
if not verdicts:
    print('판정 불가 — Imu 토픽에 표본이 없다')
