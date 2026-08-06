import numpy as np
from gtbot_formation.mixer import setpoints, world_to_body, yaw_of, wrap


def test_forward_x():                      # +X 병진 = [0,0,-a,+a]
    s = setpoints(np.array([1.0, 0.0]), 0.0, kv=0.5)
    assert s[0] == 0 and s[1] == 0 and s[2] == -0.5 and s[3] == +0.5


def test_lateral_y():                      # +Y 병진 = [-a,+a,0,0]
    s = setpoints(np.array([0.0, 1.0]), 0.0, kv=0.5)
    assert s[2] == 0 and s[3] == 0 and s[0] == -0.5 and s[1] == +0.5


def test_yaw_added_and_clipped():
    s = setpoints(np.array([10.0, 0.0]), 0.3, kv=1.0)
    # sx=clip(10,-0.7,0.7)=0.7, sw=clip(0.3,-0.3,0.3)=0.3 → s[2]=-0.7+0.3=-0.4, s[3]=0.7+0.3=1.0
    assert np.isclose(s[2], -0.4) and s[3] == 1.0


def test_yaw_authority_reserved_under_saturation():
    # sx는 ±0.7에서 먼저 클립되므로, 병진 오차가 10배 더 커져 더 심하게
    # 포화해도(kv|e|=10 vs 1000) sw가 s[2]에 기여하는 양(-0.4)은 그대로다 —
    # yaw 권한이 병진 포화 정도에 따라 잠식되지 않음을 보장한다.
    s_mod = setpoints(np.array([10.0, 0.0]), 0.3, kv=1.0)
    s_extreme = setpoints(np.array([1000.0, 0.0]), 0.3, kv=1.0)
    assert np.isclose(s_mod[2], -0.4) and np.isclose(s_extreme[2], -0.4)


def test_world_to_body_quarter_turn():
    vb = world_to_body(0.0, 1.0, np.pi / 2)   # 월드 +Y = body +X (yaw 90°)
    assert np.allclose(vb, [1.0, 0.0], atol=1e-12)


def test_yaw_of_identity():
    assert abs(yaw_of(0, 0, 0, 1)) < 1e-12


def test_wrap():
    assert abs(abs(wrap(3 * np.pi)) - np.pi) < 1e-12
