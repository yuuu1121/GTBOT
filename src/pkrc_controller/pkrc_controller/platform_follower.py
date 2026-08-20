#!/usr/bin/env python3
"""플랫폼 추종기 — /pkrc/world_position 을 이동 목표로 삼는 leader_pilot 변형.

제어 법칙·부호·데드밴드는 gtbot_formation.leader_pilot 과 동일하다(P 속도 지령 +
yaw0 유지 + 0.3/0.5 m 스테이션 키핑 데드밴드 — 데드밴드는 hull 요동으로 LiDAR
반사판 검출이 간헐화되는 것을 막는 실측 기반 값이라 그대로 둔다). 다른 점은
웨이포인트 목록 대신 PoseStamped 목표를 구독하고, 목표가 stale(기본 5 s)이면
정지한다는 것뿐이다. gtbot 편대는 플랫폼을 따라오므로 이 노드만으로
'플랫폼+편대가 PKRC를 따라다니는' 동작이 된다.

vlm_ws pathfollower/blueboat_path_follower.py 의 역할에 대응하지만, 구동이
차동(VESC 2모터)이 아니라 플랫폼 4추력기 믹서라 제어부는 leader_pilot 쪽을 쓴다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from gtbot_formation.mixer import setpoints, world_to_body, yaw_of, wrap


class PlatformFollower(Node):
    def __init__(self):
        super().__init__('platform_follower')
        defaults = [('odom_topic', '/platform/calibrated_odom'),
                    ('target_topic', '/pkrc/world_position'),
                    ('v_max', 0.2), ('kp', 0.5), ('kv', 1.5), ('kpsi', 0.1),
                    ('target_stale_s', 5.0), ('yaw0_deg', float('nan')),
                    # 안전 클램프(2026-08-20, 8차에서 상대 반경으로 수정): 목표를
                    # 자기 위치 기준 반경 bound_rel 이내로 자른다. 절대좌표(±6) 클램프는
                    # 위치원(FAST-LIO) 프레임이 드리프트하면 자기위치와 어긋나 추종을
                    # 부수는 것이 8차에서 실측됐다 — 상대 반경은 프레임 드리프트에 불변이고
                    # 잘못된 목표로의 폭주(벽 충돌)도 같은 정도로 막는다.
                    ('bound_rel', 5.0),
                    # yaw 소스 분리(2026-08-20): LIO yaw 드리프트를 kpsi가 쫓으면
                    # 플랫폼이 실제로 회전해 반사판 관측각이 바뀌고 편대 est가 붕괴
                    # (6차 반복 실측 — 위치 필터로도 안 잡힘). 자세는 깨끗한 IMU로.
                    # 실기도 헤딩은 IMU(hwt9053/Ouster 내장)가 정석.
                    ('imu_yaw_topic', '/platform/imu_true')]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.v_max, self.kp, self.kv = p('v_max'), p('kp'), p('kv')
        self.kpsi, self.stale_s = p('kpsi'), p('target_stale_s')
        self.yaw0_forced = float(p('yaw0_deg'))
        self.odom = None
        self.t_odom = None
        self.yaw0 = None
        self.target = None
        self.t_target = None
        self.dead = False
        self.imu_yaw = None
        imu_t = str(p('imu_yaw_topic'))
        if imu_t:
            from sensor_msgs.msg import Imu
            self.create_subscription(Imu, imu_t, self.on_imu, 50)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 10)
        self.create_subscription(PoseStamped, p('target_topic'), self.on_target, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/platform/thrusters', 10)
        self.create_timer(0.05, self.tick)
        self.t_log = 0.0
        self.get_logger().info(
            f"platform follower: target={p('target_topic')} v_max={self.v_max}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = yaw_of(q.x, q.y, q.z, q.w)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        if self.imu_yaw is not None:
            yaw = self.imu_yaw
        tw = msg.twist.twist.linear
        c, s = np.cos(yaw), np.sin(yaw)
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        v_world = np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y])
        # 속도 추정(2026-08-20): FAST-LIO 계열 odometry는 twist를 채우지 않는다 —
        # v=0으로 읽히면 kv 감쇠항이 죽어 비감쇠 제어의 지속 요동이 반사판 검출을
        # 부순다(이분법 실측). twist가 0이면 위치 미분(저역 tau≈0.5 s)으로 대체한다.
        # 미분 기준선 1 s(2026-08-20 재조정): 메시지 간격(0.1 s)으로 미분하면 LIO
        # 위치 지터 ±5 cm가 0.5~1 m/s 속도 노이즈가 되고(v 0.909 실측), 감쇠항이
        # 그 노이즈를 추력에 주입해 hull 요동 → 팔로워 est 10% 붕괴. 1 s 창이면 10배 감쇠.
        now = self.now()
        if np.linalg.norm(v_world) < 1e-6:
            if not hasattr(self, '_vp_hist'):
                from collections import deque
                self._vp_hist = deque()
            self._vp_hist.append((now, pos.copy()))
            while len(self._vp_hist) > 2 and now - self._vp_hist[0][0] > 1.2:
                self._vp_hist.popleft()
            t0, p0 = self._vp_hist[0]
            dt = now - t0
            if dt > 0.5:
                raw = (pos - p0) / dt
                if not hasattr(self, '_v_filt'):
                    self._v_filt = np.zeros(2)
                self._v_filt = 0.8 * self._v_filt + 0.2 * raw
                # 물리 클램프: 플랫폼 실제 속도는 0.2 m/s급 — 위치원 발산 시 감쇠항이
                # 폭주 추력을 만드는 되먹임(요동→LIO 열화→v 노이즈→요동)을 끊는다.
                n_v = np.linalg.norm(self._v_filt)
                if n_v > 0.3:
                    self._v_filt *= 0.3 / n_v
                v_world = self._v_filt
        # 위치 저역필터(2026-08-20): FAST-LIO 위치는 스캔 지터 ±5~10 cm로 GT보다
        # 50~100배 시끄럽다 — P항이 이 지터를 추력 노이즈로 바꿔 hull 요동 → 팔로워
        # est 붕괴(구간 분석: 정지 유지 100 s est 99%, 연속 이동 시작 직후 붕괴).
        # tau≈1 s, 0.15 m/s 주행에서 지연 0.15 m는 데드밴드(0.25) 안이라 무해.
        if not hasattr(self, '_p_filt'):
            self._p_filt = pos.copy()
        self._p_filt = 0.9 * self._p_filt + 0.1 * pos
        self.odom = (self._p_filt, v_world, yaw)
        if self.yaw0 is None:
            self.yaw0 = (yaw if np.isnan(self.yaw0_forced)
                         else float(np.radians(self.yaw0_forced)))
            self.get_logger().info(f'yaw0 = {np.degrees(self.yaw0):+.2f} deg')
        self.t_odom = self.now()

    def on_target(self, msg):
        t = np.array([msg.pose.position.x, msg.pose.position.y])
        now = self.now()
        # 슬루 제한(2026-08-20): 위치원 글리치로 목표가 순간 점프해도 초당 0.5 m
        # 이상 끌려가지 않는다 — LIO 글리치 2 s 동안 플랫폼 이탈을 1 m로 제한.
        if self.target is not None and self.t_target is not None:
            dt = max(1e-3, min(1.0, now - self.t_target))
            d = t - self.target
            n = np.linalg.norm(d)
            lim = 0.5 * dt
            if n > lim:
                t = self.target + d * (lim / n)
        if self.odom is not None:
            b = float(self.get_parameter('bound_rel').value)
            d = t - self.odom[0]
            n = np.linalg.norm(d)
            if n > b:
                t = self.odom[0] + d * (b / n)
        self.target = t
        self.t_target = now

    def tick(self):
        stale = (self.odom is None or self.now() - self.t_odom > 0.5 or
                 self.target is None or self.now() - self.t_target > self.stale_s)
        if stale:
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return
        pos, v, yaw = self.odom
        v_cmd = self.kp * (self.target - pos)
        n = np.linalg.norm(v_cmd)
        if n > self.v_max:
            v_cmd *= self.v_max / n
        # 데드밴드 0.25/0.4 (2026-08-20 재조정): 0.1/0.2로 좁혔더니 잔추력 보정이
        # 끊이지 않아 hull 요동으로 팔로워 est가 0%까지 붕괴했다(실측 — 원래 0.3/0.5를
        # 실측으로 잡은 이유 그대로). 대신 추력 지령 저역필터(아래)로 스위칭을 완화해
        # 연속 추종과 편대 지각을 양립시킨다.
        d = float(np.linalg.norm(self.target - pos))
        if self.dead and d > 0.4:
            self.dead = False
        elif not self.dead and d < 0.25:
            self.dead = True
        if self.dead:
            v_cmd[:] = 0.0
        e_body = world_to_body(*(v_cmd - v), yaw)
        syaw = self.kpsi * wrap(yaw - self.yaw0)
        s = np.asarray(setpoints(e_body, syaw, self.kv), dtype=float)
        # 추력 지령 저역필터(2026-08-20): 급격한 추력 스위칭이 hull을 흔들어
        # LiDAR 반사판 검출(팔로워 est)을 부수는 것이 실측 확인 — 1차 필터
        # (alpha 0.1 @20 Hz, tau≈0.5 s)로 보정을 부드럽게 만든다.
        if not hasattr(self, 's_filt'):
            self.s_filt = np.zeros_like(s)
        self.s_filt = 0.9 * self.s_filt + 0.1 * s
        self.pub.publish(Float64MultiArray(data=list(self.s_filt)))
        if self.now() - self.t_log >= 10.0:
            self.get_logger().info(
                f'target ({self.target[0]:+.2f}, {self.target[1]:+.2f})  '
                f'dist {d:.2f} m  dead={int(self.dead)}  v {np.linalg.norm(v):.3f}')
            self.t_log = self.now()


def main(args=None):
    rclpy.init(args=args)
    node = PlatformFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Float64MultiArray(data=[0.0] * 4))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
