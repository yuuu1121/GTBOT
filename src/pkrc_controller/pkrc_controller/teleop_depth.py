#!/usr/bin/env python3
"""
PKRC Teleop Depth - 키보드로 깊이 목표 조절

W: 깊이 증가 (더 깊게)
S: 깊이 감소 (더 얕게)
Q: 종료

Usage:
    ros2 run pkrc_controller teleop_depth
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import sys
import termios
import tty
import select


class TeleopDepth(Node):
    def __init__(self):
        super().__init__('teleop_depth')

        self.declare_parameter('depth_step', 0.5)  # 한번에 변경할 깊이 (m)
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('initial_depth', 1.0)

        self.depth_step = self.get_parameter('depth_step').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.target_depth = self.get_parameter('initial_depth').value

        # Publisher for depth target
        self.depth_pub = self.create_publisher(
            Float64,
            '/pkrc/depth/target',
            10
        )

        # Subscribe to current depth for display
        self.current_depth = 0.0
        self.depth_sub = self.create_subscription(
            Float64,
            '/pkrc/depth/current',
            self.depth_callback,
            10
        )

        # Keyboard input timer
        self.timer = self.create_timer(0.1, self.check_keyboard)

        # Store original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info('=== PKRC Teleop Depth ===')
        self.get_logger().info('W: Deeper | S: Shallower | Q: Quit')
        self.get_logger().info(f'Depth step: {self.depth_step:.2f} m')
        self.get_logger().info(f'Range: {self.min_depth:.1f} ~ {self.max_depth:.1f} m')
        self.publish_depth()

    def depth_callback(self, msg: Float64):
        self.current_depth = msg.data

    def publish_depth(self):
        msg = Float64()
        msg.data = self.target_depth
        self.depth_pub.publish(msg)
        self.get_logger().info(
            f'Target: {self.target_depth:.2f} m | Current: {self.current_depth:.2f} m'
        )

    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()

            if key == 'w':
                self.target_depth = min(self.max_depth, self.target_depth + self.depth_step)
                self.publish_depth()
            elif key == 's':
                self.target_depth = max(self.min_depth, self.target_depth - self.depth_step)
                self.publish_depth()
            elif key == 'q':
                self.get_logger().info('Exiting teleop depth...')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                raise KeyboardInterrupt

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
