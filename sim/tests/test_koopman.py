import numpy as np
from sim.koopman import zeta_dim, build_zeta, build_zeta_linear

def test_dims_match_paper():
    assert zeta_dim(3, 18) == 901        # 원논문 식 2.68 (E1 하드코딩 게이트)
    assert zeta_dim(3, 21) == 1075       # E2
    assert zeta_dim(5, 35) == 2941       # phi7-design §7

def test_build_zeta_shape_and_affinity():
    rng = np.random.default_rng(0)
    z1d, z2d = rng.normal(size=12), rng.normal(size=18)
    assert build_zeta(z1d, z2d, np.zeros(6)).shape == (901,)
    theta = rng.normal(size=(901, 18))
    U1, U2 = rng.normal(size=6), rng.normal(size=6)
    f = lambda U: theta.T @ build_zeta(z1d, z2d, U)
    lhs = f(U1 + U2) - f(np.zeros(6))
    rhs = (f(U1) - f(np.zeros(6))) + (f(U2) - f(np.zeros(6)))
    assert np.allclose(lhs, rhs)          # U-affine (LP 변환 전제)

def test_linear_zeta():
    assert build_zeta_linear(np.ones(12), np.ones(18), np.ones(6)).shape == (36,)  # 식 2.40
