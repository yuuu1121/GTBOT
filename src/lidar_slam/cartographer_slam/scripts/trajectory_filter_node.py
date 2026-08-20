#!/usr/bin/env python3
"""
Trajectory Filter Node

Filters out frozen trajectories (trajectory_id=0) from cartographer's
trajectory_node_list and republishes only active trajectories.

Use case: In localization mode, hide the old mapping path and show only
the current localization path.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class TrajectoryFilterNode(Node):
    def __init__(self):
        super().__init__('trajectory_filter_node')

        # Parameter: frozen trajectory IDs to filter out (default: [0])
        self.declare_parameter('frozen_trajectory_ids', [0])
        frozen_param = self.get_parameter('frozen_trajectory_ids').value
        self.frozen_ids: set[int] = set(frozen_param) if frozen_param else {0}

        self.sub = self.create_subscription(
            MarkerArray,
            'trajectory_node_list',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            'trajectory_node_list_filtered',
            10
        )

        self.get_logger().info(
            f'Filtering frozen trajectory IDs: {self.frozen_ids}'
        )

    def callback(self, msg: MarkerArray):
        filtered_markers = []

        for marker in msg.markers:
            # Cartographer uses namespace format: "Trajectory X"
            # Extract trajectory ID from namespace
            ns = marker.ns
            if ns.startswith('Trajectory '):
                try:
                    traj_id = int(ns.split(' ')[1])
                    if traj_id in self.frozen_ids:
                        continue  # Skip frozen trajectories
                except (IndexError, ValueError):
                    pass

            filtered_markers.append(marker)

        filtered = MarkerArray()
        filtered.markers = filtered_markers
        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
