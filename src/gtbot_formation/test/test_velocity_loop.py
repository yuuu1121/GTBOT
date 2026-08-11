# test/test_velocity_loop.py
"""on_est의 yaw 오프셋 추정기(IMU 드리프트·프레임 정렬)와 rel_vel EMA 회귀 가드."""
import numpy as np
from gtbot_formation.velocity_loop import VelocityLoop


class _Msg:
    def __init__(self, rel, vel, yaw_meas, valid=1.0):
        self.data = [*rel, *vel, yaw_meas, valid]


def _stub(t=10.0):
    class S:
        pass
    s = S()
    s.now = lambda: t
    s.t_est, s.valid_since = None, None
    s.rel, s.rel_vel, s.yaw_meas = None, None, 0.0
    s.yaw_hat, s.meas_valid = None, False
    s.imu_yaw, s.t_imu = 0.5, t - 0.1          # 신선한 IMU, yaw=0.5 rad
    s.k_yaw_off, s.est_vel_alpha = 0.02, 0.35
    s.yaw_off = None
    s.gyro_z = 0.0
    s.on_est = VelocityLoop.on_est.__get__(s)
    return s


def _settled(s):
    """스트릭 3 s 이상 성립 상태로 만든다(오프셋 게이트 통과 조건)."""
    s.t_est = s.now() - 0.1
    s.valid_since = s.now() - 5.0


def test_yaw_offset_gated_until_streak_and_quasistatic():
    s = _stub()
    s.on_est(_Msg([1, 0], [0, 0], 0.6))            # 부트 첫 표본: 스트릭 0 -> 갱신 금지
    assert s.yaw_off is None                        # 자기잠금 방지(불량 초기화 차단)
    _settled(s)
    s.gyro_z = 0.2                                  # 회전 중: 준정지 게이트가 차단
    s.on_est(_Msg([1, 0], [0, 0], 0.6))
    assert s.yaw_off is None
    s.gyro_z = 0.0
    _settled(s)
    s.on_est(_Msg([1, 0], [0, 0], 0.6))            # 게이트 통과: err=0.1 로 초기화
    assert abs(s.yaw_off - 0.1) < 1e-9


def test_yaw_offset_tracks_drift_and_rejects_flips():
    s = _stub()
    _settled(s)
    s.on_est(_Msg([1, 0], [0, 0], 0.6))
    _settled(s)
    s.on_est(_Msg([1, 0], [0, 0], 0.6 + np.pi))    # π-접힘 이상치: 45° 게이트가 기각
    assert abs(s.yaw_off - 0.1) < 1e-9
    for _ in range(600):                            # 드리프트: err=0.2 를 느리게 추종
        _settled(s)
        s.on_est(_Msg([1, 0], [0, 0], 0.7))
    assert abs(s.yaw_off - 0.2) < 0.01


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


def _loop_stub(vref_tau, v_ref0, v_meas, a_cmd):
    """실제 tick()을 도는 최소 스텁 — est 모드 정상 추적 경로만 탄다."""
    s = type('S', (), {})()
    s.t = 100.0
    s.now = lambda: s.t
    s.odom_source, s.yaw_source, s.heading_mode = 'est', 'imu', 'bearing'
    s.rel = np.array([1.0, 0.0])
    s.rel_vel = np.array(v_meas, dtype=float)
    s.t_est = s.t - 0.05
    s.valid_since = s.t - 10.0
    s.imu_yaw, s.t_imu = 0.0, s.t - 0.05
    s.t_acc, s.a_cmd = s.t - 0.05, np.array(a_cmd, dtype=float)
    s.dt, s.v_max, s.vref_tau = 0.05, 0.2, vref_tau
    s.v_ref = np.array(v_ref0, dtype=float)
    s.ei = np.zeros(2)
    s.ki, s.i_max, s.kv, s.kpsi, s.e_db = 1.0, 0.6, 1.0, 0.18, 0.03
    s.yaw_ref_track, s.pos_hold, s.log = None, None, None
    s.pub = _Pub()
    s._syaw_track = VelocityLoop._syaw_track.__get__(s)
    return s


def test_vref_anchor_makes_braking_command_actually_brake():
    """감속 지령(a_cmd ∥ -v)은 v_ref를 **줄여야** 한다 — 공전의 마지막 고리 회귀 가드.

    실측 실패 모드: 순수 적분 v_ref가 실측 속도와 100~107° 벌어진 채 돌면, 감속 지령이
    v_ref의 크기를 못 줄이고 방향만 회전시킨다(v_ref 52~67°/s 공전 → 로봇이 원을 그림).
    앵커가 있으면 정상상태가 v_ref = v + a_cmd·tau라 지령이 곧 속도오차가 된다.
    """
    v_meas = (0.12, 0.0)
    a_cmd = (-0.2, 0.0)                       # 진행 반대 = 감속
    perp = (0.0, 0.16)                        # v와 90° 어긋난 기준(실측 실패 상태 재현)

    free = _loop_stub(0.0, perp, v_meas, a_cmd)      # 앵커 없음(기존)
    anch = _loop_stub(0.5, perp, v_meas, a_cmd)      # 앵커 0.5 s
    for _ in range(40):                              # 2 s
        for s in (free, anch):
            VelocityLoop.tick(s)
            s.t += s.dt
    # 앵커: v_ref가 실측 속도 근방 + a_cmd·tau 로 수렴 -> 진행 방향 성분이 감속을 지시
    assert anch.v_ref[0] < v_meas[0], anch.v_ref     # 진행 성분이 현재 속도보다 작다 = 제동
    assert abs(anch.v_ref[1]) < abs(perp[1]), anch.v_ref   # 수직 잔재 소멸
    # 앵커 없음: 제동 지령이 수직 성분을 못 지운다(회전으로 소비)
    assert abs(free.v_ref[1]) > abs(anch.v_ref[1]), (free.v_ref, anch.v_ref)


def test_rel_vel_ema_smooths_and_resets_on_gap():
    s = _stub()
    s.on_est(_Msg([1, 0], [1.0, 0], 0.6))
    s.t_est = s.now() - 0.1                         # 신선 — EMA 경로
    s.on_est(_Msg([1, 0], [0.0, 0], 0.6))
    assert abs(s.rel_vel[0] - 0.65) < 1e-9          # 1.0 + 0.35*(0-1.0)
    s.t_est = s.now() - 1.0                         # 두절(>0.5 s) — raw 재초기화
    s.on_est(_Msg([1, 0], [0.2, 0], 0.6))
    assert abs(s.rel_vel[0] - 0.2) < 1e-9
