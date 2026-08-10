#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyboard_node
키보드 입력을 받아 8채널 스러스터 RPM 명령(Int32MultiArray)을 발행하는 텔레옵 노드.
대화형 stdin(input())을 쓰므로 전용 터미널에서 `ros2 run` 으로 실행할 것.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self.rpm = self.declare_parameter('rpm', 2000).value
        self.num_thrusters = self.declare_parameter('num_thrusters', 8).value
        cmd_topic = self.declare_parameter('cmd_topic', 'thruster_rpm').value

        self.publisher = self.create_publisher(Int32MultiArray, cmd_topic, 10)

    def _mixes(self):
        """키 -> 스러스터 8채널 RPM 벡터 매핑 (수평 4채널 + 수직 4채널)."""
        r = self.rpm
        return {
            'w': [r, r, r, r, 0, 0, 0, 0],          # forward
            'x': [-r, -r, -r, -r, 0, 0, 0, 0],      # reverse
            'a': [-r, r, r, -r, 0, 0, 0, 0],        # strafe left
            'd': [r, -r, -r, r, 0, 0, 0, 0],        # strafe right
            'q': [-r, r, -r, r, 0, 0, 0, 0],        # yaw left
            'e': [r, -r, r, -r, 0, 0, 0, 0],        # yaw right
            'r': [0, 0, 0, 0, r, r, r, r],          # up
            'f': [0, 0, 0, 0, -r, -r, -r, -r],      # down
            's': [0, 0, 0, 0, 0, 0, 0, 0],          # stop
        }

    def publish_thrusters(self, values):
        msg = Int32MultiArray()
        msg.data = values
        self.publisher.publish(msg)
        self.get_logger().info(str(values))

    def run(self):
        print("\n===== ROV CONTROL =====")
        print("w : forward     x : reverse")
        print("a : strafe left d : strafe right")
        print("q : yaw left    e : yaw right")
        print("r : up          f : down")
        print("s : stop        z : quit")
        print("=======================\n")

        mixes = self._mixes()

        while True:
            key = input("> ").strip()

            if key == 'z':
                break

            cmd = mixes.get(key)
            if cmd is None:
                continue

            self.publish_thrusters(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        node.run()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
