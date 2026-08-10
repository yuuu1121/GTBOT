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


def test_rel_vel_ema_smooths_and_resets_on_gap():
    s = _stub()
    s.on_est(_Msg([1, 0], [1.0, 0], 0.6))
    s.t_est = s.now() - 0.1                         # 신선 — EMA 경로
    s.on_est(_Msg([1, 0], [0.0, 0], 0.6))
    assert abs(s.rel_vel[0] - 0.65) < 1e-9          # 1.0 + 0.35*(0-1.0)
    s.t_est = s.now() - 1.0                         # 두절(>0.5 s) — raw 재초기화
    s.on_est(_Msg([1, 0], [0.2, 0], 0.6))
    assert abs(s.rel_vel[0] - 0.2) < 1e-9
