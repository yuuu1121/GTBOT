# test/test_phi2_station.py
"""φ² 지시속도의 편대 문맥 재해석(v_des_r0·v_des_width_floor) 회귀 가드.

상대상태 정식화에서 nv는 **리더 대비 상대속도**다. 스테이션(nr→0)에서 v_des가 남으면
'위치는 붙잡히고 속도는 요구되는' 모순을 공전으로 푸는 평형이 생긴다(실측: nr 0.19 m,
|v_rel| 0.14 m/s, 주기 12 s). 여기서 지키는 것은 두 가지 — 기본값은 원 하드코딩과
비트 동일할 것, 편대 시나리오에서는 스테이션 지시속도가 죽고 원거리 만속은 남을 것.
"""
import numpy as np
from gtbot_formation.relative_state import make_scenario, OFFSETS
from gtbot_formation.simpath import ensure
ensure()
from sim.scenario import Scenario
from sim.utility import phi_robot


def _phi2(sc, nr_vec, nv_vec):
    """로봇 0을 (target + nr_vec) 위치, 상대속도 nv_vec로 두고 φ² 값을 뽑는다."""
    n = sc.n_robots
    X = np.zeros(4 * n)
    pos = np.array(sc.targets, dtype=float)
    pos[0] = np.asarray(sc.targets[0], dtype=float) + nr_vec
    X[:2 * n] = pos.ravel()
    X[2 * n:][:2] = nv_vec
    return float(phi_robot(X, 0, sc)[list(sc.phi_terms).index(2)])


def test_defaults_are_bit_identical_to_original():
    """기본 Scenario는 원 하드코딩(r0=0.2, 폭=0.8·v_des)과 완전히 같은 값을 낸다."""
    sc = Scenario(phi_terms=(2,), targets=np.zeros((3, 2)))
    for nr in (0.05, 0.2, 0.9, 3.0):
        for nv in (0.0, 0.5, 4.0):
            v_des = sc.v_cruise / (1.0 + np.exp(-10.0 * (nr - 0.2)))
            want = 1.0 - np.exp(-(((nv - v_des) / (0.8 * v_des)) ** 2))
            got = _phi2(sc, np.array([nr, 0.0]), np.array([nv, 0.0]))
            assert abs(got - want) < 1e-12, (nr, nv, got, want)


def test_station_speed_command_dies_but_far_field_keeps_cruise():
    sc = make_scenario()
    v_des = lambda nr: sc.v_cruise / (1.0 + np.exp(-10.0 * (nr - sc.v_des_r0)))
    assert v_des(0.2) < 0.02                    # 관측 공전 반경대 -> 사실상 정지 지시
    assert v_des(0.1) < 0.01
    assert v_des(1.5) > 0.29                    # 원거리 복귀 권한은 만속 유지
    assert v_des(2 * sc.v_des_r0) > 0.5 * sc.v_cruise   # 시그모이드가 뒤집히지 않았다


def test_station_term_damps_across_operating_speeds():
    """스테이션 근방 φ²는 운용 속도대(0~0.3 m/s) **전 구간에서** |v_rel|에 단조 증가한다.

    가중치 -1이므로 효용은 감소 = 감쇠다. 핵심은 포화 금지 — 폭이 좁으면 φ²가 0.1 m/s
    부근에서 이미 1로 붙어 그래디언트가 사라진다(실측 기여 0.8%, 공전 방치). 상단 구간의
    증가분까지 요구해 그 붕괴를 잡는다.
    """
    sc = make_scenario()
    r = np.array([0.15, 0.0])
    speeds = (0.0, 0.05, 0.10, 0.15, 0.20, 0.30)
    vals = [_phi2(sc, r, np.array([v, 0.0])) for v in speeds]
    assert all(b > a for a, b in zip(vals, vals[1:])), vals
    assert vals[-1] - vals[-2] > 0.02, vals      # 0.2->0.3 구간에도 살아있는 그래디언트
    assert vals[0] < 0.02                        # 정지가 최적


def test_phi1_alignment_is_gated_off_near_station():
    """φ¹(정렬 코사인)은 스테이션 근방에서 꺼지고 원거리에서는 원값을 회복한다.

    φ¹ 그래디언트는 속도에 수직이라(정규화 코사인) 크기를 못 줄이고 방향만 돌린다 =
    구심력. 스테이션에서 이게 지배하면 감속 없는 공전이 된다(실측 89%).
    """
    sc = make_scenario()
    n = sc.n_robots
    i1 = list(sc.phi_terms).index(1)

    def phi1(nr, v):
        X = np.zeros(4 * n)
        pos = np.array(sc.targets, dtype=float)
        pos[0] = np.asarray(sc.targets[0], dtype=float) + np.array([nr, 0.0])
        X[:2 * n] = pos.ravel()
        X[2 * n:][:2] = v
        return float(phi_robot(X, 0, sc)[i1])

    v = np.array([-0.15, 0.0])                   # 스테이션을 향해 이동(코사인 = -1)
    assert abs(phi1(0.15, v)) < 0.05             # 근방: 사실상 꺼짐
    assert phi1(2.0, v) < -0.95                  # 원거리: 정렬항 원값 회복
    assert abs(phi1(0.15, v)) < abs(phi1(0.6, v)) < abs(phi1(2.0, v))   # 단조 복귀


def test_offsets_unchanged():
    """편대 기하는 손대지 않았다(변경 범위 봉인)."""
    assert OFFSETS[0] == (0.866, 0.0)
    assert make_scenario().v_cruise == 0.3
