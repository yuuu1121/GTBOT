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
                              ('i_max', 0.6), ('log_csv', ''),
                              ('heading_mode', 'hold'), ('k_cf', 0.1)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.kv, self.kpsi, self.v_max = p('kv'), p('kpsi'), p('v_max')
        self.ki, self.i_max = p('ki'), p('i_max')
        self.dt = 1.0 / p('rate')
        robot = p('robot')
        self.heading_mode, self.k_cf = p('heading_mode'), p('k_cf')
        if self.heading_mode == 'bearing':
            from sensor_msgs.msg import Imu
            from .mixer import cf_update  # noqa: 사용은 on_imu에서
            self.rel = None
            self.yaw_meas = 0.0
            self.meas_valid = False
            self.yaw_hat = None
            self.t_cf = None
            self.t_est = None
            self.create_subscription(Float64MultiArray, f'/{robot}/state_est', self.on_est, 10)
            self.create_subscription(Imu, f'/{robot}/imu', self.on_imu, 50)
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

    def on_est(self, msg):
        # valid==False(마커 가림 등)일 때 rel=(0,0) 더미값이 들어온다 — 이를 그대로 받으면
        # yaw_ref가 순간적으로 튀어 제어 발진의 원인이 된다(회전 중 실측 확인, task-4-report 참조).
        # t_est도 valid에서만 갱신해야 0.5s 게이트가 "마커 가림 → 자연 홀드 폴백"으로 동작한다.
        d = msg.data
        self.meas_valid = bool(d[5])
        if self.meas_valid:
            self.rel = np.array(d[0:2])
            self.yaw_meas = d[4]
            self.t_est = self.now()

    def on_imu(self, msg):
        from .mixer import cf_update
        t = self.now()
        if self.yaw_hat is None:
            if self.meas_valid:
                self.yaw_hat = self.yaw_meas
                self.t_cf = t
            return
        dt = max(0.0, min(t - self.t_cf, 0.1))
        self.t_cf = t
        self.yaw_hat = cf_update(self.yaw_hat, msg.angular_velocity.z, dt,
                                 self.yaw_meas, self.meas_valid, self.k_cf)

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
        if (self.heading_mode == 'bearing' and self.rel is not None
                and self.yaw_hat is not None and self.t_est is not None
                and t - self.t_est < 0.5):
            yaw_ref = np.arctan2(-self.rel[1], -self.rel[0])   # platform을 바라보는 방위
            syaw = self.kpsi * wrap(self.yaw_hat - yaw_ref)    # 검증된 플랜트 부호 유지
        else:
            syaw = self.kpsi * wrap(yaw - self.yaw0)           # 기존 hold (불변, 두절 폴백 겸용)
        s = setpoints(e_body, syaw, self.kv)
        self.pub.publish(Float64MultiArray(data=list(s)))
        if self.log:
            row = [t, v_world[0], v_world[1], yaw, self.v_ref[0], self.v_ref[1],
                   self.a_cmd[0], self.a_cmd[1], self.ei[0], self.ei[1], *s]
            self.log.write(','.join(f'{x:.5f}' for x in row) + '\n')

def main():
    rclpy.init()
    rclpy.spin(VelocityLoop())
