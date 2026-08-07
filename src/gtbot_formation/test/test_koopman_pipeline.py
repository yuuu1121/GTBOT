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


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


def _tick_stub(state_source, odoms, ok, t, est_hold=1.0, est_stale_stop=5.0):
    """실제 tick()을 그대로 돌리기 위한 최소 스텁 — 두절/stale 조기 반환 경로만 탄다.

    log=None이라 logrow는 즉시 반환하고, 그 앞의 가드에서 return되므로 제어 계산에는
    도달하지 않는다(스텁에 sc/rls가 없어도 된다 = 경로가 실제로 조기 반환함의 증거).
    """
    from gtbot_formation.koopman_node import KoopmanFormation

    class Stub:
        pass

    s = Stub()
    s.now = lambda: t
    s.odoms, s.ok, s.prev, s.log = odoms, ok, 'sentinel', None
    s.state_source, s.est_hold, s.est_stale_stop = state_source, est_hold, est_stale_stop
    s.ests = {}
    s.pubs = [_FakePub(), _FakePub(), _FakePub()]
    s.logrow = KoopmanFormation.logrow.__get__(s)   # 실제 메서드(log=None이라 즉시 반환)
    KoopmanFormation.tick(s)
    return s


def _odoms(t, names=('platform', 'gtbot', 'gtbot2', 'gtbot3')):
    return {n: (np.zeros(2), np.zeros(2), t) for n in names}


def test_odometry_dropout_is_silent_not_zeros():
    """odometry 모드 두절 → **발행 자체를 생략**(zeros 발행 금지) 회귀 가드.

    zeros를 매 틱 발행하면 velocity_loop의 a_cmd 신선도가 계속 갱신돼 0.5 s dropout 타이머가
    영원히 발화하지 않는다 → 'a_cmd=0'으로 해석돼 v_ref가 고정되고 로봇이 직전 속도로 영구
    항해한다(round 4 실측 버그). 침묵해야 velocity_loop의 검증된 두절 경로가 정지시킨다."""
    s = _tick_stub('odometry', _odoms(t=0.0), ok={}, t=10.0)      # 전 odom이 10 s stale
    assert all(p.sent == [] for p in s.pubs)                      # zeros조차 나가지 않는다
    assert s.prev is None                                         # 두절 갱을 전이로 오인하지 않음


def test_odometry_fresh_does_not_take_dropout_path():
    """대조군: odom이 신선하면 위 조기 반환에 걸리지 않는다(가드가 항상 참이 아님을 확인)."""
    import pytest
    with pytest.raises(AttributeError):        # 조기 반환하지 않고 제어 계산으로 진행 -> 스텁에 sc 없음
        _tick_stub('odometry', _odoms(t=9.9), ok={}, t=10.0)


def test_permanent_loss_silences_all_channels():
    """한 로봇이 est_stale_stop을 넘겨 stale이면 **전 채널 침묵**.

    mask는 발행만 막고 X 조립에는 마지막 값이 그대로 쓰이므로, 영구 손실을 방치하면 건강한
    두 대가 유령 위치 기준으로 계속 최적화된다(리뷰 Important 1)."""
    ok = {0: (np.zeros(2), np.zeros(2), 9.9),       # 신선
          1: (np.zeros(2), np.zeros(2), 4.0),       # 6 s stale -> 임계(5 s) 초과
          2: (np.zeros(2), np.zeros(2), 9.9)}
    s = _tick_stub('lidar', _odoms(t=9.9), ok=ok, t=10.0)
    assert all(p.sent == [] for p in s.pubs)
    assert s.prev is None


def test_recovery_after_permanent_loss_resumes():
    """재획득되면(모든 트랙이 임계 안) 다시 제어 경로로 진행한다 — 침묵이 영구 고착되지 않는다."""
    import pytest
    ok = {k: (np.zeros(2), np.zeros(2), 9.9) for k in range(3)}
    with pytest.raises(AttributeError):        # 침묵 반환이 아니라 제어 계산으로 진행
        _tick_stub('lidar', _odoms(t=9.9), ok=ok, t=10.0)
