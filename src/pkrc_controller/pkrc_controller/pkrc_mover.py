#!/usr/bin/env python3
"""PKRC 시뮬용 이동 패턴 — 추종 데모에서 '따라갈 대상'을 만든다.

/pkrc/teleop/pwm 으로 병진만 낸다. **yaw 회전 금지**: ukfm의 aruco 보정이
`robot = marker_pos + tvec`로 카메라 프레임 회전을 무시하므로(vlm 원본의
단순화), PKRC가 yaw를 돌리면 보정값이 회전 오염돼 추정이 발산한다(2026-08-19
1·2차 데모 실측). 그래서 surge/sway 조합의 개루프 사각 배회로 yaw를 고정한다.
heave(4·5ch)는 depth_controller_sim 이 병합해 수심을 유지한다. 실기에는 이
노드가 없다(PKRC는 자체 미션/텔레옵으로 움직인다).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PkrcMover(Node):
    def __init__(self):
        super().__init__('pkrc_mover')
        # amp 0.1(2026-08-19 5차): PKRC 실속도가 플랫폼 추종 상한(v_max 0.2 m/s)보다
        # 확실히 느려야 '한 번 잡으면 계속 시야에 둔다'가 성립한다.
        defaults = [('amp', 0.1), ('leg_s', 30.0),
                    # 폐루프 사각(2026-08-20): 개루프 배회는 비대칭 항력으로 -y 표류
                    # (500 s에 6 m 실측)해 궤적이 사각형으로 안 보인다. 시뮬 전용
                    # GT(/pkrc/odometry) 웨이포인트 P제어로 닫는다 — 추종 사슬
                    # (FAST-LIO·카메라)은 건드리지 않는 데모 대상 생성기일 뿐이다.
                    ('closed_loop', False), ('side_m', 2.0), ('kp', 0.6)]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.amp, self.leg_s = float(p('amp')), float(p('leg_s'))
        self.closed = bool(p('closed_loop'))
        self.side, self.kp = float(p('side_m')), float(p('kp'))
        self.pub = self.create_publisher(Float64MultiArray, '/pkrc/teleop/pwm', 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        if self.closed:
            import numpy as np
            from nav_msgs.msg import Odometry
            from scipy.spatial.transform import Rotation as R
            self.np, self.R = np, R
            self.gt = None
            self.origin = None
            self.wp_i = 0
            self.create_subscription(Odometry, '/pkrc/odometry', self.on_gt, 10)
            self.create_timer(0.1, self.tick_closed)
            self.get_logger().info(f'PKRC mover: 폐루프 사각 {self.side} m @ amp {self.amp}')
        else:
            self.create_timer(0.1, self.tick)
            self.get_logger().info(
                f'PKRC mover: yaw 고정 사각 배회, 변 {self.leg_s:.0f}s @ {self.amp}')

    def on_gt(self, msg):
        q = msg.pose.pose.orientation
        yaw = self.R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        self.gt = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self.origin is None:
            self.origin = self.np.array([self.gt[0], self.gt[1]])
            s = self.side
            self.wps = [self.origin + d for d in
                        (self.np.array([s, 0.0]), self.np.array([s, s]),
                         self.np.array([0.0, s]), self.np.array([0.0, 0.0]))]

    def tick_closed(self):
        if self.gt is None:
            return
        np = self.np
        x, y, yaw = self.gt
        tgt = self.wps[self.wp_i]
        err = tgt - np.array([x, y])
        if np.linalg.norm(err) < 0.25:
            self.wp_i = (self.wp_i + 1) % 4
            return
        # 월드 오차 -> 바디(surge, sway): PKRC yaw는 자유 표류라 GT yaw로 회전
        c, s = np.cos(yaw), np.sin(yaw)
        ex, ey = c * err[0] + s * err[1], -s * err[0] + c * err[1]
        # 부호 실측(2026-08-20): 양의 surge pwm은 -x_body 추진(첫 시도에서 +x
        # 웨이포인트를 두고 -x로 7.5 m 이탈). sway 부호도 같은 켤레로 반전.
        surge = float(np.clip(-self.kp * ex, -self.amp, self.amp))
        sway = float(np.clip(-self.kp * ey, -self.amp, self.amp))
        self.pub.publish(Float64MultiArray(
            data=[surge, surge, sway, sway, 0.0, 0.0]))

    def tick(self):
        t = (self.get_clock().now().nanoseconds * 1e-9 - self.t0) % (4 * self.leg_s)
        leg = int(t // self.leg_s)
        a = self.amp
        # 전진 -> 우측 sway -> 후진 -> 좌측 sway (yaw 불변 사각형)
        cmd = [[a, a, 0.0, 0.0], [0.0, 0.0, a, a],
               [-a, -a, 0.0, 0.0], [0.0, 0.0, -a, -a]][leg]
        self.pub.publish(Float64MultiArray(data=cmd + [0.0, 0.0]))


def main(args=None):
    rclpy.init(args=args)
    node = PkrcMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Float64MultiArray(data=[0.0] * 6))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
