"""rpm_to_sim — 실기 하드웨어 인터페이스(thruster_rpm, 8ch RPM)를 stonefish 4ch
setpoint로 되돌리는 시뮬 전용 최종 어댑터.

실기·시뮬 대칭 구조의 시뮬 쪽 끝단:
  velocity_loop → thruster_bridge → /<robot>/thruster_rpm ─┬ 실기: thruster_can_node → CAN
                                                           └ 시뮬: 이 노드 → /<robot>/sim_thrusters
변환은 thruster_bridge의 정확한 역산(수평 4채널에서 sx·sy·sw 복원 → 시뮬 믹서
재합성)이라 왕복이 항등 — 제어 관점의 시뮬 거동은 어댑터 도입 전과 동일하고,
RPM 인터페이스 구간만 실기와 바이트 동일하게 실행된다.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray


class RpmToSim(Node):

    def __init__(self):
        super().__init__('rpm_to_sim')
        p = lambda n, d: self.declare_parameter(n, d).value
        robot = p('robot', 'gtbot')
        self.max_rpm = float(p('max_rpm', 2000))
        # thruster_bridge와 동일 배열이어야 역산이 성립한다(파라미터 한 쌍으로 관리)
        self.strafe_mix = np.array(p('strafe_mix', [-1.0, 1.0, 1.0, -1.0]))
        self.yaw_mix = np.array(p('yaw_mix', [-1.0, 1.0, -1.0, 1.0]))

        self.pub = self.create_publisher(Float64MultiArray,
                                         f'/{robot}/sim_thrusters', 10)
        self.create_subscription(Int32MultiArray, f'/{robot}/thruster_rpm',
                                 self.on_rpm, 10)

    def on_rpm(self, msg):
        h = np.array(msg.data[:4], dtype=float) / self.max_rpm
        sx = float(np.mean(h))
        sy = float(np.dot(h, self.strafe_mix)) / 4.0
        sw = float(np.dot(h, self.yaw_mix)) / 4.0
        s = np.array([-sy + sw, sy + sw, -sx + sw, sx + sw])  # mixer.py 규약
        out = Float64MultiArray()
        out.data = [float(v) for v in s]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RpmToSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
