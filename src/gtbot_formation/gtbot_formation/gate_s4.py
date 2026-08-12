"""S4: 지각 품질. ① 정지 60 s — 위치 RMSE(GT 대비) <0.1 m ② gtbot을 전채널 setpoint로
~90°씩 4방위 회전시키며 각 방위 정지 후 마커 헤딩 vs GT yaw 오차 <10°. GT = 시뮬 odometry(평가 전용).

plate 마커 재정의(2026-08-09, 사용자 승인 "판으로 교체해서 진행"): 반사판 1장은 π-대칭
(mod-180)이고 edge-on에서 소실되므로, 마스트용이던 "4방위 회전" 검사를 **운용영역 검사**로
대체한다 — bearing 루프(플랫폼 지향 유지)가 도는 상태에서 120 s 연속 수집, 로봇 3대 전원의
위치 RMSE < 0.1 m + 헤딩 오차 median(mod-180) < 10°. raw 오차 병기(지향 상태에선 접기가
옳아 mod-180과 일치해야 정상). 실행 구성: formation.launch(start_leader:=false) + 시뮬.

정착 대기 추가(2026-08-12): 수집 전에 편대가 정착할 때까지 기다린다 — 종전에는 부팅
직후부터 수집해 스폰→스테이션 과도가 섞였고, 그것이 '지각 품질'로 오독됐다(근거는
wait_settled 독스트링). 결과 JSON에 settled_before_collect로 정착 여부를 병기한다.
**이 변경 이전 S4 수치는 과도 포함이라 직접 비교 불가**다."""
import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from gtbot_formation.mixer import yaw_of, wrap
from gtbot_formation.relative_state import OFFSETS

ROBOTS = ['gtbot', 'gtbot2', 'gtbot3']

class GateS4(Node):
    def __init__(self):
        super().__init__('gate_s4')
        self.gt = {}
        self.est = {}
        for n in ['platform'] + ROBOTS:
            self.create_subscription(Odometry, f'/{n}/odometry',
                                     lambda m, k=n: self.on_gt(k, m), 10)
        for n in ROBOTS:
            self.create_subscription(Float64MultiArray, f'/{n}/state_est',
                                     lambda m, k=n: self.on_est(k, m), 10)
        self.thr = self.create_publisher(Float64MultiArray, '/gtbot/thrusters', 10)

    def on_gt(self, k, m):
        q = m.pose.pose.orientation
        self.gt[k] = (np.array([m.pose.pose.position.x, m.pose.pose.position.y]),
                      yaw_of(q.x, q.y, q.z, q.w))

    def on_est(self, k, m):
        d = m.data
        self.est[k] = (np.array(d[0:2]), d[4], d[5],
                       self.get_clock().now().nanoseconds * 1e-9)

def wait_settled(g, err_max=0.5, hold=5.0, timeout=120.0):
    """수집 전 정착 대기 — 이 게이트가 '지각 품질'을 재려면 과도구간을 빼야 한다.

    2026-08-12 판별 시험: gtbot만 위치 RMSE가 10배(0.09 vs 0.009)로 나와 31런 중 3건이
    탈락했는데, 정착 후 재측정하면 세 대가 동일했다(0.0114/0.0118/0.0120 m). 검출 품질도
    동일(점수 186/176/176). 즉 그 차이는 지각이 아니라 **스폰->스테이션 이동·회전 과도**
    였다 — 스폰 자세가 셋 다 rpy 0인데 스테이션 지향 베어링은 180/-60/60이라 gtbot만
    180° 회전이 필요하고, 스폰 기하는 로봇 고정이라 편대를 회전시켜도 따라오지 않는다.
    판정 기준은 settle_wait와 동일(최대 편대오차 < err_max가 hold초 연속). 미정착으로
    timeout하면 그대로 진행하되 결과에 표시해 조용한 오염을 막는다.
    """
    t0 = time.time()
    since = None
    while time.time() - t0 < timeout:
        rclpy.spin_once(g, timeout_sec=0.1)
        if 'platform' not in g.gt or any(r not in g.gt for r in ROBOTS):
            continue
        pP, _ = g.gt['platform']
        err = max(float(np.linalg.norm((g.gt[r][0] - pP) - o))
                  for r, o in zip(ROBOTS, OFFSETS))
        if err < err_max:
            since = since if since is not None else time.time()
            if time.time() - since >= hold:
                print(f'settled for S4 after {time.time() - t0:.1f}s (max_err={err:.3f})')
                return True
        else:
            since = None
    print(f'S4: {timeout:.0f}s 안에 미정착 — 과도 포함 상태로 수집(결과에 표시)')
    return False


def spin_collect(g, dur):
    t0 = time.time()
    pos_err = {r: [] for r in ROBOTS}
    yaw_err = {r: [] for r in ROBOTS}
    n_valid = n_total = 0
    while time.time() - t0 < dur:
        rclpy.spin_once(g, timeout_sec=0.05)
        if 'platform' not in g.gt:
            continue
        pP, _ = g.gt['platform']
        for r in ROBOTS:
            if r not in g.gt or r not in g.est:
                continue
            (pG, yG), (rel, yE, valid, tE) = g.gt[r], g.est[r]
            n_total += 1
            if not valid or time.time() - tE > 1.0:
                continue
            n_valid += 1
            pos_err[r].append(np.linalg.norm((pG - pP) - rel))
            e = wrap(yE - yG)
            e180 = e - np.sign(e) * np.pi if abs(e) > np.pi / 2 else e   # mod-180 접기
            yaw_err[r].append((abs(e180), abs(e)))
    return pos_err, yaw_err, n_valid, max(n_total, 1)

def main():
    rclpy.init()
    g = GateS4()
    settled = wait_settled(g)                                    # 과도구간 배제(2026-08-12)
    pe, ye, nv, nt = spin_collect(g, 120.0)                      # 운용영역 연속 수집
    rmse = {r: float(np.sqrt(np.mean(np.square(v)))) if v else None for r, v in pe.items()}
    hdg = {r: {'median_deg': float(np.degrees(np.median([x[0] for x in v]))),
               'p95_deg': float(np.degrees(np.percentile([x[0] for x in v], 95))),
               'median_deg_raw': float(np.degrees(np.median([x[1] for x in v])))}
           for r, v in ye.items() if v}
    out = {'pos_rmse': rmse, 'valid_ratio': nv / nt,
           'heading_err': hdg, 'settled_before_collect': bool(settled),
           'gate_s4_pass': bool(all(v is not None and v < 0.1 for v in rmse.values())
                                and len(hdg) == 3
                                and all(hdg[r]['median_deg'] < 10.0 for r in hdg))}
    with open('results/s4_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
