"""LiDAR 지각 순수함수 — ROS 무관, 합성 클라우드로 테스트."""
import numpy as np

def cluster_2d(xy, linkage=0.3, min_pts=5):
    """그리디 유니온 클러스터링(O(N²) — 마커 대역 필터 후 수백 점 규모 전제)."""
    n = len(xy)
    label = -np.ones(n, dtype=int)
    cur = 0
    for i in range(n):
        if label[i] >= 0:
            continue
        stack = [i]
        label[i] = cur
        while stack:
            j = stack.pop()
            d = np.linalg.norm(xy - xy[j], axis=1)
            for k in np.nonzero((d < linkage) & (label < 0))[0]:
                label[k] = cur
                stack.append(k)
        cur += 1
    return [np.nonzero(label == c)[0] for c in range(cur)
            if (label == c).sum() >= min_pts]

def marker_split(xyz, h, main_band=(0.30, 0.40), shared_band=(0.25, 0.30), min_sep=0.08):
    """기둥 2개 분리: main_band(중앙 기둥 단독 높이)로 중앙 xy, shared_band에서
    중앙으로부터 min_sep 이상 떨어진 점 평균 = 오프셋 기둥 xy."""
    top = xyz[(h >= main_band[0]) & (h <= main_band[1])]
    if len(top) < 3:
        return None
    center_xy = top[:, :2].mean(axis=0)
    low = xyz[(h >= shared_band[0]) & (h <= shared_band[1])]
    if len(low) < 3:
        return None
    d = np.linalg.norm(low[:, :2] - center_xy, axis=1)
    aux = low[d > min_sep]
    if len(aux) < 3:
        return None
    return center_xy, aux[:, :2].mean(axis=0)

def marker_heading(center_xy, aux_xy):
    v = np.asarray(aux_xy) - np.asarray(center_xy)
    return float(np.arctan2(v[1], v[0]))
