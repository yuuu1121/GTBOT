#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
thruster_can_node
Int32MultiArray(thruster_rpm) 명령을 받아 각 스러스터로 SocketCAN 프레임을 전송.

CAN 프레임: 스러스터 i(0-based) -> arbitration_id = can_base_id + (i + 1),
            data = int32 RPM(little-endian, signed, 4바이트), 확장 ID.
"""

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class ThrusterCanNode(Node):

    def __init__(self):
        super().__init__('thruster_can_node')

        # 파라미터
        self.can_channel = self.declare_parameter('can_channel', 'can0').value
        self.can_interface = self.declare_parameter('can_interface', 'socketcan').value
        self.can_base_id = self.declare_parameter('can_base_id', 0x300).value
        self.num_thrusters = self.declare_parameter('num_thrusters', 8).value
        cmd_topic = self.declare_parameter('cmd_topic', 'thruster_rpm').value

        self.subscription = self.create_subscription(
            Int32MultiArray, cmd_topic, self.cmd_callback, 10)

        try:
            self.bus = can.interface.Bus(
                channel=self.can_channel, interface=self.can_interface)
        except Exception as e:
            self.get_logger().fatal(
                f"CAN 버스 '{self.can_channel}' 열기 실패: {e} "
                f"(예: sudo ip link set {self.can_channel} up type can bitrate 500000)")
            raise

        self.get_logger().info(
            f"Thruster CAN Node 시작 (channel={self.can_channel}, "
            f"base_id=0x{self.can_base_id:X}, thrusters={self.num_thrusters})")

    def cmd_callback(self, msg):
        rpms = msg.data

        if len(rpms) != self.num_thrusters:
            self.get_logger().error(
                f"스러스터 값 {self.num_thrusters}개가 필요합니다 (받음: {len(rpms)})")
            return

        for i in range(self.num_thrusters):
            data = list(int(rpms[i]).to_bytes(4, byteorder='little', signed=True))
            can_msg = can.Message(
                arbitration_id=self.can_base_id + (i + 1),
                data=data,
                is_extended_id=True)
            self.bus.send(can_msg)

        self.get_logger().info(f"Sent {list(rpms)}")


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
