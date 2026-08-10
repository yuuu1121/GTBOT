#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
thruster_bridge_node
편대 제어 스택(velocity_loop)의 시뮬 규약 출력 /<robot>/thrusters
(Float64MultiArray 4ch [E,W,N,S] setpoint, -1..1)을 실물 8채널 RPM 명령
(Int32MultiArray, thruster_rpm)으로 변환 — 제어 코드는 시뮬·실기 동일하게 두고
이 노드만 하드웨어 쪽에 얹는다.

변환: 시뮬 믹서(mixer.py)의 역산으로 (sx, sy, sw)를 복원한 뒤
  sx = (s[3]-s[2])/2   # +X 병진 성분  (믹서: +X = [0,0,-a,+a])
  sy = (s[1]-s[0])/2   # +Y 병진 성분  (믹서: +Y = [-a,+a,0,0])
  sw = mean(s)         # yaw 성분      (믹서: yaw = [a,a,a,a])
실물 수평 4채널(X-배치, keyboard_node의 실증 매핑과 동일 규약)로 재믹스:
  T[0..3] = sx*[1,1,1,1] + sy*strafe_mix + sw*yaw_mix
수직 4채널(4..7)은 수상 주행에서 0.

캘리브레이션 노브(실기 필수 — 시뮬 완결로 보이더라도 물에서 재보정):
  max_rpm      : setpoint 1.0에 대응하는 RPM 스케일
  strafe_mix   : 횡이동 부호 배열 (기본 keyboard_node 'a' 키와 동일)
  yaw_mix      : 요 부호 배열   (기본 keyboard_node 'q' 키와 동일)
  invert       : 채널별 극성 반전 (배선/프로펠러 방향 실측 후 조정)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray


class ThrusterBridgeNode(Node):

    def __init__(self):
        super().__init__('thruster_bridge')
        p = lambda n, d: self.declare_parameter(n, d).value
        robot = p('robot', 'gtbot')
        self.max_rpm = p('max_rpm', 2000)
        self.strafe_mix = np.array(p('strafe_mix', [-1.0, 1.0, 1.0, -1.0]))
        self.yaw_mix = np.array(p('yaw_mix', [-1.0, 1.0, -1.0, 1.0]))
        self.invert = np.array(p('invert', [1.0] * 8))
        cmd_topic = p('cmd_topic', f'/{robot}/thruster_rpm')

        self.pub = self.create_publisher(Int32MultiArray, cmd_topic, 10)
        self.create_subscription(Float64MultiArray, f'/{robot}/thrusters',
                                 self.on_setpoints, 10)
        self.get_logger().info(
            f'bridge 시작: /{robot}/thrusters(4ch setpoint) -> {cmd_topic}(8ch RPM, '
            f'max_rpm={self.max_rpm})')

    def on_setpoints(self, msg):
        s = np.array(msg.data[:4])
        sx = (s[3] - s[2]) / 2.0
        sy = (s[1] - s[0]) / 2.0
        sw = float(np.mean(s))
        horiz = sx * np.ones(4) + sy * self.strafe_mix + sw * self.yaw_mix
        out = np.concatenate([horiz, np.zeros(4)]) * self.invert
        rpm = np.clip(out * self.max_rpm, -self.max_rpm, self.max_rpm)
        m = Int32MultiArray()
        m.data = [int(v) for v in rpm]
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
