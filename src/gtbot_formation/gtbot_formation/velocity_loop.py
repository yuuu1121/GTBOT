import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import setpoints, world_to_body, yaw_of, wrap

class VelocityLoop(Node):
    def __init__(self):
        super().__init__('velocity_loop')
        for name, default in [('robot', 'gtbot'), ('kv', 1.5), ('kpsi', 0.8),
                              ('v_max', 0.5), ('rate', 20.0)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.kv, self.kpsi, self.v_max = p('kv'), p('kpsi'), p('v_max')
        self.dt = 1.0 / p('rate')
        robot = p('robot')
        self.v_ref = np.zeros(2)
        self.a_cmd = np.zeros(2)
        self.odom = None            # (pos2, v_world2, yaw)
        self.yaw0 = None
        self.t_odom = self.t_acc = None
        self.create_subscription(Odometry, f'/{robot}/odometry', self.on_odom, 10)
        self.create_subscription(Float64MultiArray, f'/{robot}/accel_cmd', self.on_acc, 10)
        self.pub = self.create_publisher(Float64MultiArray, f'/{robot}/thrusters', 10)
        self.create_timer(self.dt, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_acc(self, msg):
        self.a_cmd = np.array(msg.data[:2])
        self.t_acc = self.now()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist.linear           # body(child) frame 가정 — S1이 실측 검증
        c, s = np.cos(yaw), np.sin(yaw)
        v_world = np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y])
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.odom = (pos, v_world, yaw)
        if self.yaw0 is None:
            self.yaw0 = yaw
        self.t_odom = self.now()

    def tick(self):
        t = self.now()
        if self.odom is None or t - self.t_odom > 0.5:
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return
        pos, v_world, yaw = self.odom
        if self.t_acc is None or t - self.t_acc > 0.5:
            self.a_cmd = np.zeros(2)
            self.v_ref = v_world.copy()       # 명령 두절 시 참조 리셋(윈드업 방지)
        self.v_ref = self.v_ref + self.a_cmd * self.dt
        n = np.linalg.norm(self.v_ref)
        if n > self.v_max:
            self.v_ref *= self.v_max / n
        e_body = world_to_body(*(self.v_ref - v_world), yaw)
        syaw = self.kpsi * wrap(self.yaw0 - yaw)
        self.pub.publish(Float64MultiArray(data=list(setpoints(e_body, syaw, self.kv))))

def main():
    rclpy.init()
    rclpy.spin(VelocityLoop())
