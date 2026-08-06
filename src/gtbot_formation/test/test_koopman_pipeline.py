# test/test_koopman_pipeline.py
"""Stonefish 없이: 합성 이중적분기에서 워밍업 로직(이중 RLS + frozen 평가)이
게이트 1 성질(bilinear frozen 1-step < linear)을 재현하는지 — 워밍업 로직의 사양이자 회귀 가드."""
import numpy as np
from gtbot_formation.relative_state import make_scenario
from gtbot_formation.simpath import ensure
ensure()
from sim.dynamics import ab_matrices
from sim.experiment import make_rls, _zeta, operating_point, frozen_1step_eval
from sim.scenario import table1_input
from sim.utility import z2_vector

def test_warmup_identifies_bilinear_better():
    sc = make_scenario()
    A, B = ab_matrices(sc.n_robots, sc.dt)
    z10, z20 = operating_point(sc)
    rls = {m: make_rls(sc, m) for m in ('linear', 'bilinear')}
    X = np.zeros(12)
    X[:6] = np.array([(-1.0, 0.5), (-1.0, -0.5), (-2.0, 0.0)]).ravel()
    ks = 600
    X_log = np.zeros((ks + 1, 12)); X_log[0] = X
    U_log = np.zeros((ks, 6))
    for k in range(1, ks + 1):
        U = table1_input(k) / 8.0
        z2 = z2_vector(X, sc)
        X_next = A @ X + B @ U
        z2n = z2_vector(X_next, sc)
        for m in ('linear', 'bilinear'):
            rls[m].update(_zeta(sc, m, X, z2, U, z10, z20), z2n)
        X_log[k] = X_next; U_log[k - 1] = U
        X = X_next
    rmse = {m: float(np.sqrt(np.mean(frozen_1step_eval(sc, m, rls[m], X_log, U_log) ** 2)))
            for m in ('linear', 'bilinear')}
    assert rmse['bilinear'] < rmse['linear']


def test_publish_u_masks_expired_robots():
    """로봇별 발행: 만료된 로봇에만 침묵(발행 생략) + 지연보상 전파값도 0.

    구 동작(전원 침묵)은 한 대의 일시 실명이 세 대 전부를 velocity_loop 두절로 정지시켜
    영구 미복구가 됐다(S6 실측, task-5-report fix round 4)."""
    from gtbot_formation.koopman_node import KoopmanFormation

    class FakePub:
        def __init__(self):
            self.sent = []

        def publish(self, msg):
            self.sent.append(list(msg.data))

    class Stub:
        pubs = [FakePub(), FakePub(), FakePub()]

    s = Stub()
    KoopmanFormation.publish_u(s, np.arange(1.0, 7.0), (True, False, True))
    assert s.pubs[0].sent == [[1.0, 2.0]]
    assert s.pubs[1].sent == []                  # 침묵
    assert s.pubs[2].sent == [[5.0, 6.0]]
    assert list(s.u_prev) == [1.0, 2.0, 0.0, 0.0, 5.0, 6.0]
