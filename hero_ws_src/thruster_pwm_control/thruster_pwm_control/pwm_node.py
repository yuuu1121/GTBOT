#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pwm_node
Float32MultiArray(thruster_pwm_cmd) 명령(-1.0~+1.0)을 받아
PCA9685로 각 ESC에 서보 펄스를 출력.

매핑: cmd -1.0 ~ 0.0 ~ +1.0  ->  us_min ~ us_mid ~ us_max (기본 1100~1500~1900µs)
시작/종료 시 전 채널 중립(아밍/안전 정지).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from thruster_pwm_control.pca9685 import PCA9685


class PwmNode(Node):

    def __init__(self):
        super().__init__('pwm_node')

        # 파라미터
        self.i2c_bus = self.declare_parameter('i2c_bus', 1).value
        self.i2c_addr = self.declare_parameter('i2c_addr', 0x40).value
        self.pwm_freq = self.declare_parameter('pwm_freq', 200).value
        self.channels = list(self.declare_parameter('channels', [1, 2, 3, 4]).value)
        self.us_min = self.declare_parameter('us_min', 1100.0).value
        self.us_mid = self.declare_parameter('us_mid', 1500.0).value
        self.us_max = self.declare_parameter('us_max', 1900.0).value
        self.max_cmd = self.declare_parameter('max_cmd', 0.5).value
        cmd_topic = self.declare_parameter('cmd_topic', 'thruster_pwm_cmd').value

        try:
            self.pca = PCA9685(bus=self.i2c_bus, addr=self.i2c_addr, freq=self.pwm_freq)
        except Exception as e:
            self.get_logger().fatal(
                f'PCA9685(bus={self.i2c_bus}, addr=0x{self.i2c_addr:02X}) 열기 실패: {e} '
                f'(i2cdetect -y {self.i2c_bus} 로 배선 확인)')
            raise

        # 아밍: 전 채널 중립
        for ch in self.channels:
            self.pca.set_us(ch, self.us_mid)
        self.get_logger().info(
            f'PWM Node 시작 (freq={self.pwm_freq}Hz, channels={self.channels}, '
            f'중립 {self.us_mid}µs 출력 중 — ESC 아밍음 확인) '
            f'출력 하드 리밋 max_cmd={self.max_cmd} -> '
            f'{self.cmd_to_us(-self.max_cmd):.0f}~{self.cmd_to_us(self.max_cmd):.0f}µs')

        self.subscription = self.create_subscription(
            Float32MultiArray, cmd_topic, self.cmd_callback, 10)

    def cmd_to_us(self, c):
        c = max(-self.max_cmd, min(self.max_cmd, float(c)))
        if c >= 0.0:
            return self.us_mid + c * (self.us_max - self.us_mid)
        return self.us_mid + c * (self.us_mid - self.us_min)

    def cmd_callback(self, msg):
        cmds = msg.data
        if len(cmds) != len(self.channels):
            self.get_logger().error(
                f'명령 {len(self.channels)}개가 필요합니다 (받음: {len(cmds)})')
            return
        us_list = [self.cmd_to_us(c) for c in cmds]
        for ch, us in zip(self.channels, us_list):
            self.pca.set_us(ch, us)
        self.get_logger().info(
            '  '.join(f'ch{ch}:{us:.0f}µs' for ch, us in zip(self.channels, us_list)),
            throttle_duration_sec=1.0)

    def shutdown(self):
        """안전 정지: 중립 잠깐 유지 후 출력 끔."""
        try:
            for ch in self.channels:
                self.pca.set_us(ch, self.us_mid)
            self.pca.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
