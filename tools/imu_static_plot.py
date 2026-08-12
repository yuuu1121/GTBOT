"""정지 IMU 로그 요약 그림 — 드리프트가 지배항임을 보이기 위한 4면."""
import numpy as np, sqlite3
import matplotlib; matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'UnDotum'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Temperature

SC = '/tmp/claude-0/-root-home-gtbot-ws/b8adc266-2d62-49c6-bf70-1cbe16a16cea/scratchpad'
d = np.load(f'{SC}/imu_run1_analysis.npz')
TOP = ['/imu/raw', '/imu/gp', '/imu/ddpm']
COL = {'/imu/raw': '#d62728', '/imu/gp': '#ff7f0e', '/imu/ddpm': '#1f77b4'}

fig, ax = plt.subplots(2, 2, figsize=(15, 9))

a = ax[0, 0]
for t in TOP:
    tt, y = d[f'{t}|t'], np.degrees(d[f'{t}|yaw'])
    a.plot(tt / 60, y - y[0], c=COL[t], lw=1.0,
           label=f'{t}  ({np.polyfit(tt, y, 1)[0]*3600:+.0f} °/h)')
a.axhline(0, c='k', lw=0.8); a.grid(alpha=0.3); a.legend(fontsize=9)
a.set_xlabel('t [min]'); a.set_ylabel('yaw - yaw(0) [deg]')
a.set_title('정지 상태 yaw 드리프트 — 참값은 상수, 전부 오차', fontsize=11)

a = ax[0, 1]
tt, y = d['/imu/ddpm|t'], np.degrees(d['/imu/ddpm|yaw'])
a.plot(tt / 60, y - y[0], c=COL['/imu/ddpm'], lw=1.0, label='/imu/ddpm yaw')
a.set_xlabel('t [min]'); a.set_ylabel('yaw - yaw(0) [deg]'); a.grid(alpha=0.3)
con = sqlite3.connect('/root/home/gtbot_ws/imu_run1/imu_run1_0.db3')
tid = con.execute("SELECT id FROM topics WHERE name='/imu/temperature'").fetchone()[0]
rows = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                   (tid,)).fetchall()
ttm = np.array([r[0] for r in rows]) * 1e-9; ttm -= ttm[0]
temp = np.array([deserialize_message(r[1], Temperature).temperature for r in rows])
a2 = a.twinx(); a2.plot(ttm / 60, temp, c='0.4', lw=1.4, ls='--', label='temperature')
a2.set_ylabel('temperature [°C]')
a.set_title(f'ddpm yaw vs 온도 (상관 {np.corrcoef(temp, np.interp(ttm, tt, y))[0,1]:.3f})',
            fontsize=11)
h1, l1 = a.get_legend_handles_labels(); h2, l2 = a2.get_legend_handles_labels()
a.legend(h1 + h2, l1 + l2, fontsize=9, loc='upper left')

a = ax[1, 0]
for t in TOP:
    a.loglog(d[f'{t}|taus'], np.degrees(d[f'{t}|allan']), 'o-', c=COL[t], ms=3, lw=1.2, label=t)
a.grid(alpha=0.3, which='both'); a.legend(fontsize=9)
a.set_xlabel('평균화 구간 τ [s]'); a.set_ylabel('Allan 편차 [deg]')
a.set_title('Allan 편차 — τ가 커질수록 오차가 커지면 드리프트 지배', fontsize=11)

a = ax[1, 1]
tt = d['/imu/ddpm|t']
w = int(50 * 1.0) | 1
for t in TOP:
    y = np.unwrap(d[f'{t}|yaw']); ttt = d[f'{t}|t']
    r = np.degrees(y - np.convolve(y, np.ones(w) / w, mode='same'))[w:-w]
    a.hist(r, bins=200, histtype='step', color=COL[t], lw=1.2,
           label=f'{t}  (std {np.std(r):.4f}°)')
a.set_yscale('log'); a.grid(alpha=0.3); a.legend(fontsize=9)
a.set_xlabel('1 s 이동평균 대비 잔차 [deg]'); a.set_ylabel('빈도')
a.set_title('백색잡음 성분 — 순간 잡음은 0.001°대로 매우 조용함', fontsize=11)

plt.rcParams['axes.unicode_minus'] = False
fig.suptitle('정지 IMU 로그 55분 (imu_run1) — 순간 잡음은 무시할 수준, 드리프트가 전부', fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(f'{SC}/imu_run1.png', dpi=110)
print('saved imu_run1.png')
