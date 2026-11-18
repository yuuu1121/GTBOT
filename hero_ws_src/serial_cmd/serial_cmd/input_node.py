#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
input_node
터미널에 입력한 줄("1001", "x" 등)을 String(serial_cmd) 토픽으로 발행.
serial_cmd_node 가 받아서 시리얼로 전송한다.

입력은 /dev/tty 에서 직접 읽으므로 `ros2 launch` 안에서도 동작한다
(launch 는 stdin 을 자식에게 넘기지 않음).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InputNode(Node):

    def __init__(self):
        super().__init__('serial_cmd_input')
        cmd_topic = self.declare_parameter('cmd_topic', 'serial_cmd').value
        self.pub = self.create_publisher(String, cmd_topic, 10)
        self.tty = open('/dev/tty', 'r')
        self.get_logger().info(f'명령 입력 후 Enter -> {cmd_topic} 발행 (q: 종료)')


def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    try:
        while rclpy.ok():
            line = node.tty.readline()
            if not line:
                break
            line = line.strip()
            if line == 'q':
                break
            if line:
                node.pub.publish(String(data=line))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
