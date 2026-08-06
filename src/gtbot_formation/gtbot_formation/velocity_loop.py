import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import setpoints, world_to_body, yaw_of, wrap

class VelocityLoop(Node):
    def __init__(self):
        super().__init__('velocity_loop')
        # kpsi: yaw rate 플랜트 G≈12 rad/s/unit, 지연 τ≈0.5 s 실측 -> ζ≈0.7 되는 값이 0.085.
        for name, default in [('robot', 'gtbot'), ('kv', 1.5), ('kpsi', 0.1),
                              ('v_max', 0.5), ('rate', 20.0), ('ki', 1.0),
                              ('i_max', 0.6), ('log_csv', '')]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.kv, self.kpsi, self.v_max = p('kv'), p('kpsi'), p('v_max')
        self.ki, self.i_max = p('ki'), p('i_max')
        self.dt = 1.0 / p('rate')
        robot = p('robot')
        self.v_ref = np.zeros(2)
        self.a_cmd = np.zeros(2)
        self.ei = np.zeros(2)
        self.odom = None            # (pos2, v_world2, yaw)
        self.yaw0 = None
        self.t_odom = self.t_acc = None
        path = p('log_csv')
        self.log = open(path, 'w', buffering=1) if path else None
        if self.log:
            self.log.write('t,vx,vy,yaw,vrefx,vrefy,ax,ay,eix,eiy,s0,s1,s2,s3\n')
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
            self.ei[:] = 0.0
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return
        pos, v_world, yaw = self.odom
        if self.t_acc is None or t - self.t_acc > 0.5:
            self.a_cmd = np.zeros(2)
            self.v_ref = v_world.copy()       # 명령 두절 시 참조 리셋(윈드업 방지)
            self.ei[:] = 0.0
        self.v_ref = self.v_ref + self.a_cmd * self.dt
        n = np.linalg.norm(self.v_ref)
        if n > self.v_max:
            self.v_ref *= self.v_max / n
        e_world = self.v_ref - v_world
        # 적분 기여를 ±i_max로 클램프(안티윈드업). i_max는 정상상태 항력을 이길 setpoint
        # 여유를 정하는 보정 노브 — mixer 병진 캡(±0.7) 아래에 둔다.
        lim = self.i_max / self.ki if self.ki else 0.0
        self.ei = np.clip(self.ei + e_world * self.dt, -lim, lim)
        e_body = world_to_body(*(e_world + (self.ki / self.kv) * self.ei), yaw)  # mixer 시그니처 불변 트릭: P+I 합성 오차
        # 실측(2026-08-06): [a,a,a,a] 양의 setpoint -> yaw 감소(sw=+0.1에서 -215deg/4s,
        # 정상 rate -12 rad/s per unit). 따라서 yaw>yaw0일 때 sw>0이어야 되돌린다.
        syaw = self.kpsi * wrap(yaw - self.yaw0)
        s = setpoints(e_body, syaw, self.kv)
        self.pub.publish(Float64MultiArray(data=list(s)))
        if self.log:
            row = [t, v_world[0], v_world[1], yaw, self.v_ref[0], self.v_ref[1],
                   self.a_cmd[0], self.a_cmd[1], self.ei[0], self.ei[1], *s]
            self.log.write(','.join(f'{x:.5f}' for x in row) + '\n')

def main():
    rclpy.init()
    rclpy.spin(VelocityLoop())
