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
                    ('bound_rel', 5.0)]
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
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 10)
        self.create_subscription(PoseStamped, p('target_topic'), self.on_target, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/platform/thrusters', 10)
        self.create_timer(0.05, self.tick)
        self.t_log = 0.0
        self.get_logger().info(
            f"platform follower: target={p('target_topic')} v_max={self.v_max}")

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
            self.get_logger().info(f'yaw0 = {np.degrees(self.yaw0):+.2f} deg')
        self.t_odom = self.now()

    def on_target(self, msg):
        t = np.array([msg.pose.position.x, msg.pose.position.y])
        if self.odom is not None:
            b = float(self.get_parameter('bound_rel').value)
            d = t - self.odom[0]
            n = np.linalg.norm(d)
            if n > b:
                t = self.odom[0] + d * (b / n)
        self.target = t
        self.t_target = self.now()

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
        # 스테이션 키핑 데드밴드 — leader_pilot과 동일(근거는 그쪽 주석)
        d = float(np.linalg.norm(self.target - pos))
        if self.dead and d > 0.5:
            self.dead = False
        elif not self.dead and d < 0.3:
            self.dead = True
        if self.dead:
            v_cmd[:] = 0.0
        e_body = world_to_body(*(v_cmd - v), yaw)
        syaw = self.kpsi * wrap(yaw - self.yaw0)
        s = setpoints(e_body, syaw, self.kv)
        self.pub.publish(Float64MultiArray(data=list(s)))
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
