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

def solve_input(c, u_min, u_max, tol=1e-12):
    if not np.isfinite(c).all():
        return np.zeros(len(c)), "nan_guard"           # 스펙: 오염 → U=0 + 로그
    res = linprog(-c, bounds=[(u_min, u_max)] * len(c), method="highs")  # scipy 최소화 → -c
    U = np.asarray(res.x)
    U[np.abs(c) < tol] = 0.0                           # 퇴화 → 0 (U0=0 일관)
    return U, "ok"
