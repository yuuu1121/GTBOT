import numpy as np

class RLS:
    """정규화 최소자승 (식 3.3~3.6). ε=사전오차(게이트 지표), εa=사후오차(원논문 Figure 대조용)."""

    def __init__(self, dim, out_dim, rho=1e-4, p0=100.0):
        self.theta = np.zeros((dim, out_dim))      # Θ0=0 (:1272)
        self.P = p0 * np.eye(dim)                  # P(-1)=P0=100I
        self.rho, self.p0, self.dim = rho, p0, dim

    def update(self, zeta, y):
        eps = self.theta.T @ zeta - y              # 식 3.3 (a priori)
        if not (np.isfinite(zeta).all() and np.isfinite(eps).all()):
            return eps, None                        # 스펙: 갱신 스킵 + 호출측 로그
        Pz = self.P @ zeta
        m2 = self.rho + zeta @ Pz                  # 식 3.5
        self.theta = self.theta - np.outer(Pz, eps) / m2   # 식 3.4
        self.P = self.P - np.outer(Pz, Pz) / m2            # 식 3.6
        self.P = 0.5 * (self.P + self.P.T)                 # 라운드오프 PSD 보전 (스펙)
        return eps, self.theta.T @ zeta - y

    def reset_P(self):
        self.P = self.p0 * np.eye(self.dim)
