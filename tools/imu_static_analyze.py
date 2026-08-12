"""정지 IMU 로그(rosbag2) -> Stonefish scn 잡음 파라미터.

Stonefish IMU 오차 모델은 노브가 둘뿐이다(IMU.cpp 확인):
  angle      샘플마다 더해지는 가우시안 백색잡음 stddev [rad]
  yaw_drift  결정론적 선형 램프 [rad/s]  (accumulatedYawDrift += rate*dt)
따라서 정지 로그에서 뽑을 통계량도 둘 — 추세 기울기와 추세 제거 잔차 표준편차.
"""
import sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

BAG = sys.argv[1] if len(sys.argv) > 1 else 'imu_run1/imu_run1_0.db3'
OUT = sys.argv[2] if len(sys.argv) > 2 else 'imu_run1_analysis.npz'


def quat_to_rpy(x, y, z, w):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load(con, topic):
    tid = con.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()[0]
    rows = con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid,)).fetchall()
    t = np.empty(len(rows)); q = np.empty((len(rows), 4))
    g = np.empty((len(rows), 3)); a = np.empty((len(rows), 3))
    for i, (ts, blob) in enumerate(rows):
        m = deserialize_message(blob, Imu)
        t[i] = ts * 1e-9
        q[i] = (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        g[i] = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        a[i] = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
    return t - t[0], q, g, a


def white_std(t, sig, win_s=1.0):
    """1초 이동평균을 뺀 성분의 표준편차 = 백색잡음 몫.

    잔차를 그냥 쓰면 1/f(bias instability)까지 백색으로 과대계상된다."""
    fs = len(t) / (t[-1] - t[0])
    n = max(3, int(round(win_s * fs)) | 1)
    k = np.ones(n) / n
    sm = np.convolve(sig, k, mode='same')
    r = (sig - sm)[n:-n]
    # 이동평균 자체가 잡음의 1/n을 먹으므로 분산 보정 sqrt(n/(n-1))
    return float(np.std(r) * np.sqrt(n / (n - 1.0)))


def allan_dev(t, sig, taus):
    fs = len(t) / (t[-1] - t[0])
    out = []
    for tau in taus:
        m = int(round(tau * fs))
        if m < 1 or len(sig) // m < 3:
            out.append(np.nan); continue
        k = len(sig) // m
        blocks = sig[:k * m].reshape(k, m).mean(axis=1)
        out.append(float(np.sqrt(0.5 * np.mean(np.diff(blocks) ** 2))))
    return np.array(out)


con = sqlite3.connect(BAG)
names = [r[0] for r in con.execute('SELECT name FROM topics').fetchall()]
res = {}
for topic in [n for n in names if n.endswith(('gp', 'ddpm', 'raw'))]:
    t, q, g, a = load(con, topic)
    fs = len(t) / (t[-1] - t[0])
    roll, pitch, yaw = quat_to_rpy(*q.T)
    yaw_u = np.unwrap(yaw)
    # 추세(드리프트) — 선형회귀
    A = np.c_[t, np.ones_like(t)]
    slope, icpt = np.linalg.lstsq(A, yaw_u, rcond=None)[0]
    resid = yaw_u - (A @ [slope, icpt])
    print(f'\n===== {topic}  ({len(t)} 샘플, {t[-1]:.0f} s, {fs:.1f} Hz) =====')
    print(f'  yaw   드리프트 {np.degrees(slope)*3600:+.3f} °/h  ({slope:+.3e} rad/s)')
    print(f'        추세제거 잔차 std {np.degrees(np.std(resid)):.4f}°   '
          f'백색성분 std {np.degrees(white_std(t, yaw_u)):.4f}°')
    print(f'        전체 변동폭 {np.degrees(np.ptp(yaw_u)):.3f}°')
    for nm, sig in (('roll', roll), ('pitch', pitch)):
        sl = np.linalg.lstsq(A, np.unwrap(sig), rcond=None)[0][0]
        print(f'  {nm:5s} 드리프트 {np.degrees(sl)*3600:+.3f} °/h  '
              f'백색성분 std {np.degrees(white_std(t, np.unwrap(sig))):.4f}°')
    for i, ax in enumerate('xyz'):
        print(f'  gyro {ax} 평균 {g[:,i].mean():+.3e} rad/s  std {np.std(g[:,i]):.3e}')
    for i, ax in enumerate('xyz'):
        print(f'  acc  {ax} 평균 {a[:,i].mean():+.4f} m/s2   std {np.std(a[:,i]):.4e}')
    taus = np.logspace(-1.5, 2.5, 25)
    res[topic] = dict(t=t, yaw=yaw_u, roll=roll, pitch=pitch, gyro=g, acc=a,
                      taus=taus, allan=allan_dev(t, yaw_u, taus), slope=slope, fs=fs)

np.savez_compressed(OUT, **{f'{k}|{kk}': vv for k, v in res.items() for kk, vv in v.items()})
print(f'\nsaved {OUT}')
