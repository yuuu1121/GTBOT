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
                    ('v_lead', 0.2), ('kp', 0.5), ('kv', 1.5), ('kpsi', 0.1), ('arrive_r', 0.5)]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.wps = np.array(p('waypoints'), dtype=float).reshape(-1, 2)
        self.v_lead, self.kp, self.kv = p('v_lead'), p('kp'), p('kv')
        self.kpsi, self.arrive_r = p('kpsi'), p('arrive_r')
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
            self.yaw0 = yaw
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
        e_body = world_to_body(*(v_cmd - v), yaw)
        # platform 벤치 실측(2026-08-06, yaw_bench.py): sw=+0.1 -> yaw -157.3 deg/4s,
        # sw=-0.1 -> yaw +140.8 deg/4s. gtbot과 동일하게 양의 setpoint가 yaw를 감소시킨다
        # (거울상 배치를 scn에서 N/S 배선으로 상쇄했기 때문 — 우연 아님). velocity_loop.py와
        # 같은 부호: syaw = kpsi*wrap(yaw - yaw0). kpsi=0.1은 동일 플랜트 실측(G≈12 rad/s/unit,
        # tau≈0.5s)에 맞춘 zeta≈0.7 값.
        syaw = self.kpsi * wrap(yaw - self.yaw0)
        self.pub.publish(Float64MultiArray(data=list(setpoints(e_body, syaw, self.kv))))

def main():
    rclpy.init()
    rclpy.spin(LeaderPilot())
