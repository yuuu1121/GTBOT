"""실물 LiDAR bag(ouster_cluster 출력)의 판 검출을 BEV 영상으로 렌더링한다.

시뮬 영상(tools/record_plate_cluster.py)과 같은 판형으로 만들어 나란히 볼 수 있게 했다.
다른 점 하나: 실물은 intensity가 이진(255)이 아니라 연속값(실측 7~45)이라 점 색을
강도로 칠한다 — 시뮬에서 못 보던 구조다.

입력은 /ouster_cluster/box_points(검출기가 이미 판으로 판정한 점). 원 점군이 아니므로
'배경과의 분리'는 보이지 않고 '검출된 판이 어떻게 생겼나'만 보인다.

사용: python3 tools/render_real_bag.py <bag.db3> <out.mp4> [반경 m]
"""
import sqlite3
import sys

import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from scipy.cluster.hierarchy import fcluster, linkage
from sensor_msgs.msg import PointCloud2

DB = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else 'real_bag.mp4'
HALF = float(sys.argv[3]) if len(sys.argv) > 3 else 1.6
SIZE, FPS = 700, 25
ZHALF = 0.22                    # 우측 확대 패널 반경 m — 판(14 cm)이 화면을 채우는 크기
PPM = SIZE / (2 * HALF)
ZPPM = SIZE / (2 * ZHALF)
I_LO, I_HI = 5.0, 45.0          # 실측 강도 범위 — 색으로 매핑
# TURBO 컬러맵 LUT: 강도 구조를 눈에 보이게 한다(시뮬은 이진 255라 볼 것이 없었다)
_LUT = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1),
                         cv2.COLORMAP_TURBO).reshape(-1, 3)


def to_px(x, y, ppm=None, ctr=(0.0, 0.0)):
    ppm = PPM if ppm is None else ppm
    return (int(SIZE / 2 - (y - ctr[1]) * ppm), int(SIZE / 2 - (x - ctr[0]) * ppm))


def icol(v):
    t = float(np.clip((v - I_LO) / (I_HI - I_LO), 0, 1))
    return tuple(int(c) for c in _LUT[int(t * 255)])


con = sqlite3.connect(DB)
tid = con.execute("SELECT id FROM topics WHERE name='/ouster_cluster/box_points'").fetchone()[0]
rows = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                   (tid,)).fetchall()
t0 = rows[0][0] * 1e-9
vw = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (SIZE * 2, SIZE))
n_empty = 0
trail = []

for ts, blob in rows:
    m = deserialize_message(blob, PointCloud2)
    a = np.array([list(p) for p in pc2.read_points(
        m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)])
    img = np.full((SIZE, SIZE, 3), 22, np.uint8)
    zoom = np.full((SIZE, SIZE, 3), 16, np.uint8)
    zctr = None
    for g in np.arange(-int(HALF), int(HALF) + 1):
        p = int(SIZE / 2 - g * PPM)
        cv2.line(img, (0, p), (SIZE, p), (45, 45, 45), 1)
        cv2.line(img, (p, 0), (p, SIZE), (45, 45, 45), 1)
    cv2.circle(img, (SIZE // 2, SIZE // 2), 9, (40, 140, 255), -1)
    cv2.putText(img, 'LiDAR', (SIZE // 2 + 14, SIZE // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 140, 255), 1, cv2.LINE_AA)
    for p in trail[-400:]:
        cv2.circle(img, to_px(*p), 1, (70, 70, 90), -1)

    if len(a) == 0:
        n_empty += 1
        cv2.putText(img, 'NO DETECTION', (SIZE // 2 - 120, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 235), 2, cv2.LINE_AA)
    else:
        lab = (fcluster(linkage(a[:, :2], 'single'), 0.25, 'distance')
               if len(a) >= 2 else np.array([1]))
        for k in np.unique(lab):
            q = a[lab == k]
            for x, y, _, iv in q:
                cv2.circle(img, to_px(x, y), 2, icol(iv), -1)
            if len(q) < 5:
                continue
            c = q[:, :2].mean(0)
            d = q[:, :2] - c
            _, s, vt = np.linalg.svd(d, full_matrices=False)
            ext = np.ptp(d @ vt[0]) / 2
            cv2.line(img, to_px(*(c + vt[0] * ext)), to_px(*(c - vt[0] * ext)),
                     (255, 255, 255), 2, cv2.LINE_AA)
            cv2.line(img, (SIZE // 2, SIZE // 2), to_px(*c), (70, 70, 70), 1, cv2.LINE_AA)
            u, v = to_px(*c)
            cv2.putText(img, f'{len(q)}pt  {2*ext*100:.0f}cm  r={np.linalg.norm(c):.2f}m',
                        (u + 14, v - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (230, 230, 230), 1, cv2.LINE_AA)
            trail.append(tuple(c))
            if zctr is None:                       # 첫(=최대) 클러스터를 확대 패널에
                zctr = c
                for x, y, _, iv in q:
                    cv2.circle(zoom, to_px(x, y, ZPPM, zctr), 5, icol(iv), -1)
                cv2.line(zoom, to_px(*(c + vt[0] * ext), ZPPM, zctr),
                         to_px(*(c - vt[0] * ext), ZPPM, zctr), (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(zoom, f'x{ZPPM/PPM:.0f} zoom   {len(q)}pt   {2*ext*100:.1f}cm',
                            (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (230, 230, 230), 1, cv2.LINE_AA)

    cv2.putText(img, 'REAL LiDAR (Ouster OS0) - /ouster_cluster/box_points',
                (18, SIZE - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, f't = {ts*1e-9 - t0:5.1f} s', (SIZE - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    if zctr is None:
        cv2.putText(zoom, 'no cluster', (SIZE // 2 - 80, SIZE // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (90, 90, 90), 1, cv2.LINE_AA)
    for i in range(140):
        cv2.line(zoom, (16 + i, SIZE - 40), (16 + i, SIZE - 24),
                 icol(I_LO + (I_HI - I_LO) * i / 139), 1)
    cv2.putText(zoom, f'intensity {I_LO:.0f}', (14, SIZE - 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(zoom, f'{I_HI:.0f}', (162, SIZE - 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 170, 170), 1, cv2.LINE_AA)
    vw.write(np.hstack([img, zoom]))

vw.release()
print(f'{len(rows)} 프레임 -> {OUT} ({len(rows)/FPS:.0f} s @ {FPS} fps)')
print(f'미검출 {n_empty} / {len(rows)} = {100*n_empty/len(rows):.1f}%')
