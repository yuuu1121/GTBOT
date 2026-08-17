import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import setpoints, world_to_body, yaw_of, wrap

class LeaderPilot(Node):
    def __init__(self):
        super().__init__('leader_pilot')
        defaults = [('waypoints', [8.0, 0.0, 8.0, 8.0, 0.0, 8.0, 0.0, 0.0]),
                    ('v_lead', 0.2), ('kp', 0.5), ('kv', 1.5), ('kpsi', 0.1), ('arrive_r', 0.5),
                    # yaw0_deg(2026-08-16 진단): 유지할 헤딩을 강제한다. NaN(기본)이면
                    # 종전대로 첫 odometry의 yaw를 쓴다. 사각 주행 실속도가 런마다
                    # 0.058/0.106 m/s 두 값으로 갈리는데, syaw가 yaw0을 유지하고
                    # 추력기가 몸체 고정이라 몸체축-경로축 정렬이 합력을 바꾼다는 가설을
                    # 통제 실험으로 검증하려면 이 값을 고정할 수 있어야 한다.
                    ('yaw0_deg', float('nan')),
                    # log_csv(2026-08-17): 10 Hz 시계열. 60초 요약으로는 정지 구간도
                    # 후반 변화도 안 보여 v 분기(0.058/0.106 m/s)를 못 가른다.
                    ('log_csv', '')]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.wps = np.array(p('waypoints'), dtype=float).reshape(-1, 2)
        self.v_lead, self.kp, self.kv = p('v_lead'), p('kp'), p('kv')
        self.kpsi, self.arrive_r = p('kpsi'), p('arrive_r')
        self.yaw0_forced = float(p('yaw0_deg'))
        path = str(p('log_csv'))
        self.log = open(path, 'w', buffering=1) if path else None
        if self.log:
            # dead = 데드밴드(웨이포인트 0.3 m 안 추력 차단) 활성 여부. '평균은 낮은데
            # 순간 속력은 높다'를 이 열 하나로 판별한다.
            self.log.write('t,x,y,vx,vy,v,yaw,wp,dead,s0,s1,s2,s3\n')
        self.t_start = None
        self.i = 0
        self.odom = None
        self.yaw0 = None
        self.t_odom = None
        self.create_subscription(Odometry, '/platform/odometry', self.on_odom, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/platform/thrusters', 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist.linear
        c, s = np.cos(yaw), np.sin(yaw)
        self.odom = (np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]),
                     np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y]), yaw)
        if self.yaw0 is None:
            self.yaw0 = (yaw if np.isnan(self.yaw0_forced)
                         else float(np.radians(self.yaw0_forced)))
            # 계측(2026-08-16): 사각 주행 실속도가 런마다 0.058/0.106 m/s 두 값으로
            # 갈리는 것을 추적한다. syaw가 yaw0을 유지하므로 몸체축과 경로축의 정렬이
            # 런마다 달라지고, 추력기가 몸체 고정이라 같은 지령에 합력이 달라진다는
            # 가설. yaw0은 파일럿이 받은 첫 odometry에서 정해지므로 정착 시점의
            # 우연에 좌우된다 — 그 값을 남겨야 가설을 검증할 수 있다.
            self.get_logger().info(
                f'yaw0 = {np.degrees(self.yaw0):+.2f} deg '
                f'({"강제" if not np.isnan(self.yaw0_forced) else "첫 odom"}, '
                f'실제 yaw {np.degrees(yaw):+.2f}) '
                f'pos {self.odom[0][0]:+.2f}, {self.odom[0][1]:+.2f}')
            self.t_log = self.now()
            self.p_log = self.odom[0].copy()
        self.t_odom = self.now()

    def tick(self):
        if self.odom is None or self.now() - self.t_odom > 0.5:
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return
        pos, v, yaw = self.odom
        wp = self.wps[self.i]
        if np.linalg.norm(wp - pos) < self.arrive_r:
            self.i = (self.i + 1) % len(self.wps)
            wp = self.wps[self.i]
        v_cmd = self.kp * (wp - pos)
        n = np.linalg.norm(v_cmd)
        if n > self.v_lead:
            v_cmd *= self.v_lead / n
        # 스테이션 키핑 데드밴드(2026-08-10): 목표 부근에서 연속 추력 보정이 hull을
        # 요동시켜(롤·피치 ±2~3°) 라이다 z-크롭 창을 거리×기울기만큼 휩쓸고 반사판
        # 검출을 간헐화시킨다(리플레이 실측). 0.3 m 안에서는 추력을 끊고 0.5 m를
        # 벗어나면 재개 — 웨이포인트 주행(단일 wp 유지 용법 외)에는 arrive_r 로직이
        # 먼저 인덱스를 넘겨 영향 없다.
        d_wp = float(np.linalg.norm(wp - pos))
        if not hasattr(self, 'dead'):
            self.dead = False
        if self.dead and d_wp > 0.5:
            self.dead = False
        elif not self.dead and d_wp < 0.3:
            self.dead = True
        if self.dead:
            v_cmd[:] = 0.0
        e_body = world_to_body(*(v_cmd - v), yaw)
        # platform 벤치 실측(2026-08-06, yaw_bench.py): sw=+0.1 -> yaw -157.3 deg/4s,
        # sw=-0.1 -> yaw +140.8 deg/4s. gtbot과 동일하게 양의 setpoint가 yaw를 감소시킨다
        # (거울상 배치를 scn에서 N/S 배선으로 상쇄했기 때문 — 우연 아님). velocity_loop.py와
        # 같은 부호: syaw = kpsi*wrap(yaw - yaw0). kpsi=0.1은 동일 플랜트 실측(G≈12 rad/s/unit,
        # tau≈0.5s)에 맞춘 zeta≈0.7 값.
        syaw = self.kpsi * wrap(yaw - self.yaw0)
        s = setpoints(e_body, syaw, self.kv)
        self.pub.publish(Float64MultiArray(data=list(s)))
        if self.log:
            if self.t_start is None:
                self.t_start = self.now()
            self.log.write(
                f'{self.now()-self.t_start:.3f},{pos[0]:.4f},{pos[1]:.4f},'
                f'{v[0]:.4f},{v[1]:.4f},{np.linalg.norm(v):.4f},{yaw:.5f},'
                f'{self.i},{int(self.dead)},'
                f'{s[0]:.4f},{s[1]:.4f},{s[2]:.4f},{s[3]:.4f}\n')
        # 60 s마다 진단 한 줄 — 실속도가 지령(v_lead=0.2)에 얼마나 못 미치는지, 그리고
        # 그 미달이 헤딩과 상관되는지 본다. 지령 v_cmd와 실제 v를 같이 남겨야
        # '지령이 작다'와 '지령은 큰데 안 나간다'를 구분할 수 있다.
        if self.now() - self.t_log >= 60.0:
            d = float(np.linalg.norm(pos - self.p_log))
            self.get_logger().info(
                f'yaw {np.degrees(yaw):+7.2f} (yaw0 {np.degrees(self.yaw0):+7.2f})  '
                f'v_cmd {np.linalg.norm(v_cmd):.3f}  v {np.linalg.norm(v):.3f}  '
                f'60s 이동 {d:.2f} m -> {d/(self.now()-self.t_log):.3f} m/s  '
                f'wp{self.i}  s[{s[0]:+.2f} {s[1]:+.2f} {s[2]:+.2f} {s[3]:+.2f}]')
            self.t_log = self.now()
            self.p_log = pos.copy()

def main():
    rclpy.init()
    rclpy.spin(LeaderPilot())
