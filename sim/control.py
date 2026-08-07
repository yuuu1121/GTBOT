import numpy as np
from scipy.optimize import linprog
from .koopman import build_zeta

def input_objective(theta, z1d, z2d, w_full, n_inputs, reduced=False):
    """ẑ(2)(k+1)=Θ̂ᵀζ 가 U-affine이므로 평가로 c 추출 — 블록 순서 버그 원천 차단 (스펙 LP 계약).
    reduced: Task 8b 폴백(g1·g3 제거 ζ)로 식별된 theta 소비 시 True."""
    base = theta.T @ build_zeta(z1d, z2d, np.zeros(n_inputs), reduced=reduced)
    eye = np.eye(n_inputs)
    c = np.array([w_full @ (theta.T @ build_zeta(z1d, z2d, eye[l], reduced=reduced) - base)
                  for l in range(n_inputs)])
    return c, float(w_full @ base)

def solve_input(c, u_min, u_max, tol=1e-12, reg=0.0):
    """reg=0(기본): 기존 LP — max c·u s.t. box → 항상 ±u_max 꼭짓점(릴레이).

    reg>0: QP 정규화 — max c·u - (reg/2)||u||^2 s.t. box. 박스 제약 + 대각 이차항이라
    해가 성분별로 분리돼 **폐형식**이다: u_i = clip(c_i/reg, u_min, u_max). solver 불요.
    릴레이는 참 상태(S3)에서는 유효했으나 추정 기반 출력피드백에서 한계사이클을 낳는다
    (round 6 판별: 지각을 천장까지 올려도 편대오차 2.7~10.8 m 발산). 정규화는 제어량을
    연속으로 만들어 노이즈로 뒤집힌 부호가 전 진폭 명령이 되지 않게 한다.
    """
    if not np.isfinite(c).all():
        return np.zeros(len(c)), "nan_guard"           # 스펙: 오염 → U=0 + 로그
    if reg > 0.0:
        return np.clip(np.asarray(c, dtype=float) / reg, u_min, u_max), "ok"
    res = linprog(-c, bounds=[(u_min, u_max)] * len(c), method="highs")  # scipy 최소화 → -c
    U = np.asarray(res.x)
    U[np.abs(c) < tol] = 0.0                           # 퇴화 → 0 (U0=0 일관)
    return U, "ok"
