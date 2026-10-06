"""Stonefish 런 CSV 여러 편을 같은 시각 축으로 재생하는 제어 팔 비교 영상(상대좌표 탑뷰 + 변 오차 시계열).

koopman.csv 의 GT 열 g0..g11(odometry 기준 리더 상대 상태)만 쓴다 — 리더 월드 궤적은 로그에 없어
리더를 원점에 고정한 상대좌표로 그린다(편대 유지 성능이 보고 싶은 것이라 이 쪽이 더 읽기 쉽다).
t=0 은 제어 진입(첫 발행 + 워밍업 200틱). 각 런의 리더 출발은 정착 후 10 s 라 ±2 s 안에서 맞는다.

    python3 tools/render_arm_compare.py out.mp4 라벨1=a.csv 라벨2=b.csv ... [--speed 10] [--dur 377]
"""
import sys, os
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt, matplotlib.animation as anim
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'gtbot_formation'))
from gtbot_formation.relative_state import OFFSETS, FORMATION
plt.rcParams.update({'font.family': ['Noto Sans CJK HK', 'Baekmuk Dotum', 'DejaVu Sans'], 'axes.unicode_minus': False})
COLS = ['#E45756', '#4C78A8', '#54A24B', '#B279A2', '#F58518']


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    g = np.stack([d[f'g{i}'] for i in range(12)], 1); u = np.stack([d[f'u{i}'] for i in range(6)], 1)
    act = np.where((d['pub'] > 0) & (np.abs(g[:, :6]).sum(1) > 0))[0]; k0 = act[0] + 200
    t = d['t'][k0:] - d['t'][k0]; pos = g[k0:, :6].reshape(-1, 3, 2); u = u[k0:]
    ed = np.stack([np.abs(np.linalg.norm(pos[:, i] - pos[:, j], axis=1) - dd) for (i, j), dd in FORMATION.items()], 1)
    return dict(t=t, pos=pos, u=u, ed=ed, edmax=ed.max(1))


def at(run, tq):
    return min(int(np.searchsorted(run['t'], tq)), len(run['t']) - 1)


def main():
    args = sys.argv[1:]; out = args[0]
    speed = float(args[args.index('--speed') + 1]) if '--speed' in args else 10.0
    dur = float(args[args.index('--dur') + 1]) if '--dur' in args else None
    runs = [(a.split('=', 1)[0], load(a.split('=', 1)[1])) for a in args[1:] if '=' in a]
    T = dur or min(r['t'][-1] for _, r in runs); fps = 20; frames = int(T / speed * fps)
    n = len(runs); fig = plt.figure(figsize=(4.0 * n, 7.0))
    gs = fig.add_gridspec(2, n, height_ratios=[3.0, 1.5], hspace=0.08, wspace=0.22, top=0.88, bottom=0.08, left=0.05, right=0.98)
    axs = [fig.add_subplot(gs[0, i]) for i in range(n)]; axe = fig.add_subplot(gs[1, :])
    tg = np.asarray(OFFSETS)

    def draw(f):
        tq = f / fps * speed
        for ax, (name, r), c in zip(axs, runs, COLS):
            k = at(r, tq); ax.cla(); ax.set_aspect('equal'); ax.set_xlim(-1.4, 1.6); ax.set_ylim(-1.5, 1.5)
            ax.grid(color='#EEEEEE', lw=0.6); ax.tick_params(labelsize=7)
            ax.plot(0, 0, 's', color='#F58518', ms=9); ax.plot(tg[:, 0], tg[:, 1], 'x', color='#999999', ms=7, mew=1.5)
            k_tr = at(r, tq - 10)
            for i in range(3):
                ax.plot(r['pos'][k_tr:k + 1, i, 0], r['pos'][k_tr:k + 1, i, 1], color=c, lw=0.8, alpha=0.45)
                ax.plot(r['pos'][k, i, 0], r['pos'][k, i, 1], 'o', color=c, ms=7)
            for (i, j), dd in FORMATION.items():
                e = abs(np.linalg.norm(r['pos'][k, i] - r['pos'][k, j]) - dd)
                ax.plot([r['pos'][k, i, 0], r['pos'][k, j, 0]], [r['pos'][k, i, 1], r['pos'][k, j, 1]],
                        color=plt.cm.Reds(min(1.0, 0.25 + e / 0.4)), lw=1.4)
            k60 = at(r, 60.0); so_far = r['edmax'][k60:k + 1]; uu = np.abs(r['u'][k60:k + 1])     # 정착 후(60 s~) 누적 통계
            stat = (f'정착 후 최대 변 오차 중앙 {np.median(so_far):.3f} · p90 {np.percentile(so_far, 90):.3f} m\n|U| 중앙 {np.median(uu):.2f} · 포화 {np.mean(uu >= 0.299) * 100:.0f} %'
                    if tq > 60 else '정착 중 (통계는 60 s 부터)\n')
            ax.set_title(f'{name}   (지금 {r["edmax"][k]:.2f} m)\n{stat}', fontsize=8)
        axe.cla()
        for (name, r), c in zip(runs, COLS):
            k = at(r, tq); axe.plot(r['t'][:k + 1], r['edmax'][:k + 1], color=c, lw=1.0, label=name)
        axe.axvline(tq, color='#444444', lw=0.8); axe.set_xlim(0, T); axe.set_ylim(0, 0.8); axe.grid(color='#EEEEEE', lw=0.6)
        axe.set_ylabel('변 오차 최대 [m]', fontsize=8.5); axe.set_xlabel('제어 진입 후 시간 [s]', fontsize=8.5); axe.tick_params(labelsize=7.5)
        axe.legend(fontsize=8, ncol=n, loc='upper right', framealpha=0.9)
        fig.suptitle(f'Stonefish 리더 4×4 m 사각(1.7바퀴) — 리더 고정 상대좌표, {speed:.0f}배속, t = {tq:5.1f} s   '
                     f'(주황 ■ 리더, × 목표 자리, 선 = 편대 변·붉을수록 오차 큼)', fontsize=10)

    a = anim.FuncAnimation(fig, draw, frames=frames, interval=1000 / fps)
    a.save(out, writer=anim.FFMpegWriter(fps=fps, bitrate=3000, extra_args=['-pix_fmt', 'yuv420p']))
    print('saved', out, frames, 'frames')


if __name__ == '__main__':
    main()
