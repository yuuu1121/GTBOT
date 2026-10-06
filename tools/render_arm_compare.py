"""Stonefish 런 CSV 여러 편을 같은 시각 축으로 재생하는 제어 팔 비교 영상(상대좌표 탑뷰 + 변 오차 시계열).

koopman.csv 의 GT 열 g0..g11(odometry 기준 리더 상대 상태)만 쓴다 — 리더 월드 궤적은 로그에 없어
리더를 원점에 고정한 상대좌표로 그린다(편대 유지 성능이 보고 싶은 것이라 이 쪽이 더 읽기 쉽다).
t=0 은 제어 진입(첫 발행 + 워밍업 200틱). 각 런의 리더 출발은 정착 후 10 s 라 ±2 s 안에서 맞는다.

    python3 tools/render_arm_compare.py out.mp4 라벨1=a.csv 라벨2=b.csv ... [--speed 10] [--dur 377]
        [--top 라벨1=a_topview.mp4 ...] [--top-offset 7]
--top: 같은 런의 월드 좌표 탑뷰 녹화(record_square_run / rec_top4, 10배속 30 fps)를 윗줄에 같이 돌린다 —
사각 경로를 실제로 도는지 보이게. 탑뷰 녹화는 정착 직후 시작하고 제어 진입은 그보다 --top-offset 초(정착 ~7 s)
앞이라 그만큼 당겨 맞춘다(±1 s).
"""
import cv2
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
    runs = [(a.split('=', 1)[0], load(a.split('=', 1)[1])) for a in args[1:] if '=' in a and not a.startswith('--') and not (args.index(a) > 0 and args[args.index(a) - 1] == '--top')]
    tops = {a.split('=', 1)[0]: a.split('=', 1)[1] for i, a in enumerate(args) if i > 0 and args[i - 1] == '--top'}
    top_off = float(args[args.index('--top-offset') + 1]) if '--top-offset' in args else 7.0
    caps = {k: [cv2.VideoCapture(v), -1, None] for k, v in tops.items()}          # [reader, 마지막 프레임 번호, 프레임]
    def top_frame(k, tq):
        cap, idx, fr = caps[k]; want = max(0, int(round((tq - top_off) / 10.0 * cap.get(cv2.CAP_PROP_FPS))))
        while idx < want:
            ok, f = cap.read()
            if not ok: break
            idx += 1; fr = f
        caps[k][1], caps[k][2] = idx, fr
        return None if fr is None else cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)[200:620, 250:690]      # 사각 경로 둘레 crop
    T = dur or min(r['t'][-1] for _, r in runs); fps = 20; frames = int(T / speed * fps)
    n = len(runs); has_top = bool(tops)
    fig = plt.figure(figsize=(4.0 * n, 10.4 if has_top else 7.0))
    gs = fig.add_gridspec(3 if has_top else 2, n, height_ratios=([3.0, 3.0, 1.5] if has_top else [3.0, 1.5]), hspace=0.28, wspace=0.22,
                          top=0.92 if has_top else 0.88, bottom=0.06, left=0.05, right=0.98)
    axt = [fig.add_subplot(gs[0, i]) for i in range(n)] if has_top else []
    axs = [fig.add_subplot(gs[1 if has_top else 0, i]) for i in range(n)]; axe = fig.add_subplot(gs[-1, :])
    tg = np.asarray(OFFSETS)

    def draw(f):
        tq = f / fps * speed
        for ax, (name, r) in zip(axt, runs):
            ax.cla(); ax.set_xticks([]); ax.set_yticks([]); fr = top_frame(name, tq) if name in tops else None
            if fr is not None: ax.imshow(fr)
            ax.set_title(f'{name} — 월드 탑뷰', fontsize=8.5)
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
        fig.suptitle(f'Stonefish 리더 4×4 m 사각(1.7바퀴), {speed:.0f}배속, t = {tq:5.1f} s (제어 진입 기준)\n'
                     + ('윗줄 월드 탑뷰: 회색 사각 = 지령 경로, 주황 = 리더 자취(리더 속도는 런마다 조금 달라 위치가 어긋난다)   ' if has_top else '')
                     + '상대좌표: 주황 ■ 리더, × 목표 자리, 선 = 편대 변(붉을수록 오차 큼)', fontsize=9.5)

    a = anim.FuncAnimation(fig, draw, frames=frames, interval=1000 / fps)
    a.save(out, writer=anim.FFMpegWriter(fps=fps, bitrate=3000, extra_args=['-pix_fmt', 'yuv420p']))
    print('saved', out, frames, 'frames')


if __name__ == '__main__':
    main()
