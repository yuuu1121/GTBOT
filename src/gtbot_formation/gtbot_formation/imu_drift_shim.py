"""시뮬 IMU에 실측 드리프트를 주입하는 심 — <robot>/imu_true -> <robot>/imu.

왜 심이 필요한가: Stonefish의 IMU 오차 모델은 백색잡음(`angle`)과 **결정론적 선형
램프**(`yaw_drift`, IMU.cpp:63 `accumulatedYawDrift += rate*dt`)뿐이다. 실측
(`imu_run1`, 정지 55분)의 드리프트는 램프가 아니라 배회한다 — 10구간 기울기가
31.7 -> 45.1 -> -6.7 °/h로 흔들린다(std 17.2). 램프만 넣으면 실제보다 순한
시나리오가 되므로 랜덤워크 성분을 여기서 더한다.

모델 분해(실측 근거):
  백색잡음  scn `angle`이 담당(센서 자체 잡음, 0.0013° = 2.3e-5 rad)
  램프      전체 선형 추세 32 °/h = 1.565e-4 rad/s  <- 온도 상관 0.875의 주성분
  랜덤워크  추세 제거 잔차 2.03°/3296 s -> 6.2e-4 rad/sqrt(s)
roll·pitch는 중력이 붙들어 주므로(실측 드리프트도 yaw의 1/4 이하) 주입하지 않는다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .mixer import yaw_of, wrap


def yaw_rotate(x, y, z, w, d):
    """자세 쿼터니언을 월드 z축으로 d rad 돌린다 — roll·pitch는 보존된다.

    q_new = q_z(d) (x) q,  q_z(d) = (0, 0, sin(d/2), cos(d/2))  (해밀턴 곱)"""
    c, s = np.cos(0.5 * d), np.sin(0.5 * d)
    return (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)


class ImuDriftShim(Node):
    def __init__(self):
        super().__init__('imu_drift_shim')
        for n, d in [('robot', 'gtbot'), ('yaw_drift_rate', 1.565e-4),
                     ('yaw_rw_sigma', 6.2e-4), ('seed', 0)]:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        robot = p('robot')
        self.rate, self.rw = float(p('yaw_drift_rate')), float(p('yaw_rw_sigma'))
        self.rng = np.random.default_rng(int(p('seed')))
        self.err = 0.0            # 누적 헤딩 오차(램프 + 랜덤워크)
        self.t_prev = None
        self.pub = self.create_publisher(Imu, f'/{robot}/imu', 50)
        self.create_subscription(Imu, f'/{robot}/imu_true', self.on_imu, 50)
        self.get_logger().info(
            f'{robot}: 램프 {np.degrees(self.rate)*3600:.1f} °/h + '
            f'랜덤워크 {np.degrees(self.rw):.4f} °/sqrt(s)')

    def on_imu(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.t_prev is None else min(t - self.t_prev, 0.1)
        self.t_prev = t
        self.err += self.rate * dt + self.rw * np.sqrt(dt) * self.rng.standard_normal()
        q = msg.orientation
        out = Imu()
        out.header = msg.header
        (out.orientation.x, out.orientation.y,
         out.orientation.z, out.orientation.w) = yaw_rotate(
            q.x, q.y, q.z, q.w, self.err)
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ImuDriftShim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()


if __name__ == '__main__':
    main()
