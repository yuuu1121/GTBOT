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
        defaults = [('amp', 0.1), ('leg_s', 30.0)]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: float(self.get_parameter(n).value)
        self.amp, self.leg_s = p('amp'), p('leg_s')
        self.pub = self.create_publisher(Float64MultiArray, '/pkrc/teleop/pwm', 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'PKRC mover: yaw 고정 사각 배회, 변 {self.leg_s:.0f}s @ {self.amp}')

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
