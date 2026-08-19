#!/usr/bin/env python3
"""
PKRC Wall Following Controller
teleop 코드와 동일한 구조로 작성
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dvl_msgs.msg import DVL
import can
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class DistanceFilter:
    def __init__(self, window_size=5, max_jump=0.5):
        self.window_size = window_size
        self.max_jump = max_jump
        self.history = deque(maxlen=window_size)
        self.last_valid = 0.0

    def update(self, raw_value):
        if raw_value < 0.1 or raw_value > 10.0:
            return self.last_valid if self.last_valid > 0 else 0.0
        if self.last_valid > 0 and abs(raw_value - self.last_valid) > self.max_jump:
            return self.last_valid
        self.history.append(raw_value)
        if len(self.history) > 0:
            self.last_valid = sum(self.history) / len(self.history)
        return self.last_valid


class WallFollowingController(Node):
    STATE_APPROACH = 0
    STATE_FOLLOW = 1

    def __init__(self):
        super().__init__('wall_following_controller')

        # CAN bus
        self.bus = None
        try:
            self.bus = can.interface.Bus(channel='can0', interface='socketcan')
            self.get_logger().info('CAN bus initialized on can0')
        except Exception as e:
            self.get_logger().error(f'CAN init failed: {e}')

        # VESC CAN IDs (teleop과 동일)
        self.vesc_can_ids = {
            "surge_left": 0x151,
            "surge_right": 0x152,
            "sway_left": 0x153,
            "sway_right": 0x154,
            "heave_up": 0x155,
            "heave_down": 0x156
        }

        # Current values (teleop과 동일)
        self.target_current = {name: 0.0 for name in self.vesc_can_ids}
        self.actual_current = {name: 0.0 for name in self.vesc_can_ids}
        self.max_ramp_step = 0.5

        # Control parameters (teleop과 동일한 전류값)
        self.surge_current = 2.0
        self.sway_current = 2.0
        self.yaw_current = 2.0

        # Trim (teleop과 동일)
        self.surge_trim = 0.95
        self.sway_trim = 1.05

        # Wall following parameters
        self.target_distance = 1.0
        self.sway_speed = 0.5  # 0~1

        # PID gains
        self.dist_kp = 1.0
        self.dist_ki = 0.05
        self.dist_kd = 0.2
        self.head_kp = 0.3
        self.head_kd = 0.1

        # PID state
        self.dist_integral = 0.0
        self.dist_prev_error = 0.0
        self.head_integral = 0.0
        self.head_prev_error = 0.0

        # Filter
        self.dist_filter = DistanceFilter(window_size=5, max_jump=0.5)
        self.min_beams = 2

        # State
        self.state = self.STATE_APPROACH
        self.wall_distance = 0.0
        self.wall_distance_raw = 0.0
        self.heading_error = 0.0
        self.valid_beam_count = 0
        self.last_time = None

        # Subscriber
        self.dvl_sub = self.create_subscription(
            DVL, '/dvl/data', self.dvl_callback, sensor_qos
        )

        # Publishers
        self.dist_pub = self.create_publisher(Float64, '/pkrc/wall/distance', 10)

        # Control timer (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=== Wall Following Controller ===')
        self.get_logger().info(f'Target distance: {self.target_distance:.2f}m')
        self.get_logger().info(f'Surge current: {self.surge_current}A, Sway current: {self.sway_current}A')

    def dvl_callback(self, msg: DVL):
        """DVL 데이터 처리"""
        valid_beams = [(i, b) for i, b in enumerate(msg.beams) if b.valid and b.distance > 0]
        self.valid_beam_count = len(valid_beams)

        if self.valid_beam_count < self.min_beams:
            return

        valid_dists = [b.distance for i, b in valid_beams]
        self.wall_distance_raw = sum(valid_dists) / len(valid_dists)
        self.wall_distance = self.dist_filter.update(self.wall_distance_raw)

        # Heading error (좌우 빔 차이)
        left_beams = [(i, b) for i, b in valid_beams if i in [0, 3]]
        right_beams = [(i, b) for i, b in valid_beams if i in [1, 2]]

        if left_beams and right_beams:
            left_dist = sum(b.distance for i, b in left_beams) / len(left_beams)
            right_dist = sum(b.distance for i, b in right_beams) / len(right_beams)
            self.heading_error = left_dist - right_dist
        else:
            self.heading_error = 0.0

        msg_dist = Float64()
        msg_dist.data = self.wall_distance
        self.dist_pub.publish(msg_dist)

    def send_current(self, can_id: int, current: float):
        """teleop과 동일"""
        if self.bus is None:
            return

        current = max(-5.0, min(5.0, current))
        scaled_current = int(current * 1000)

        if scaled_current < 0:
            scaled_current = scaled_current & 0xFFFFFFFF

        data = [
            (scaled_current >> 24) & 0xFF,
            (scaled_current >> 16) & 0xFF,
            (scaled_current >> 8) & 0xFF,
            scaled_current & 0xFF
        ]

        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True, dlc=4)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f'CAN error: {e}', throttle_duration_sec=1.0)

    def apply_ramping(self):
        """teleop과 동일"""
        for name in self.vesc_can_ids:
            diff = self.target_current[name] - self.actual_current[name]
            if abs(diff) > self.max_ramp_step:
                self.actual_current[name] += self.max_ramp_step if diff > 0 else -self.max_ramp_step
            else:
                self.actual_current[name] = self.target_current[name]

    def stop_all(self):
        for name in self.vesc_can_ids:
            self.target_current[name] = 0.0
            self.actual_current[name] = 0.0
        for name, can_id in self.vesc_can_ids.items():
            self.send_current(can_id, 0.0)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Reset targets
        for name in self.vesc_can_ids:
            self.target_current[name] = 0.0

        if self.last_time is None or self.wall_distance <= 0:
            self.last_time = now
            self.apply_ramping()
            for name, can_id in self.vesc_can_ids.items():
                self.send_current(can_id, self.actual_current[name])
            return

        dt = now - self.last_time
        if dt <= 0:
            return

        distance_error = self.wall_distance - self.target_distance

        # ============================================
        # APPROACH: surge로 벽면 1m까지 접근
        # ============================================
        if self.state == self.STATE_APPROACH:
            if distance_error > 0.1:
                # Distance PID
                self.dist_integral += distance_error * dt
                self.dist_integral = max(-1.0, min(1.0, self.dist_integral))
                dist_derivative = (distance_error - self.dist_prev_error) / dt
                self.dist_prev_error = distance_error

                surge = self.dist_kp * distance_error + self.dist_ki * self.dist_integral + self.dist_kd * dist_derivative
                surge = max(-1.0, min(1.0, surge))

                # === teleop과 동일한 방식 ===
                self.target_current['surge_left'] = surge * self.surge_current
                self.target_current['surge_right'] = surge * self.surge_current * self.surge_trim

                self.get_logger().info(
                    f'[APPROACH] Dist: {self.wall_distance:.2f}m (beams:{self.valid_beam_count}), '
                    f'Error: {distance_error:.2f}m, Surge: {surge:.2f}',
                    throttle_duration_sec=1.0
                )
            else:
                self.state = self.STATE_FOLLOW
                self.dist_integral = 0.0
                self.dist_prev_error = 0.0
                self.head_integral = 0.0
                self.head_prev_error = 0.0
                self.get_logger().info('=== FOLLOW MODE: Sway RIGHT ===')

        # ============================================
        # FOLLOW: 거리 유지 + 오른쪽 이동 + heading 보정
        # ============================================
        elif self.state == self.STATE_FOLLOW:
            # 1. Distance PID → surge
            self.dist_integral += distance_error * dt
            self.dist_integral = max(-1.0, min(1.0, self.dist_integral))
            dist_derivative = (distance_error - self.dist_prev_error) / dt
            self.dist_prev_error = distance_error

            surge = self.dist_kp * distance_error + self.dist_ki * self.dist_integral + self.dist_kd * dist_derivative
            surge = max(-1.0, min(1.0, surge))

            # 2. Heading PID → yaw
            self.head_integral += self.heading_error * dt
            self.head_integral = max(-1.0, min(1.0, self.head_integral))
            head_derivative = (self.heading_error - self.head_prev_error) / dt
            self.head_prev_error = self.heading_error

            yaw = self.head_kp * self.heading_error + self.head_kd * head_derivative
            yaw = max(-0.3, min(0.3, yaw))  # yaw 제한

            # 3. Sway (오른쪽)
            sway = -self.sway_speed  # teleop에서 RIGHT = sway=-1

            # === teleop과 완전히 동일한 적용 ===

            # Surge (전/후진)
            self.target_current['surge_left'] = surge * self.surge_current
            self.target_current['surge_right'] = surge * self.surge_current * self.surge_trim

            # Yaw (회전) - 차동 구동
            self.target_current['surge_left'] += yaw * self.yaw_current
            self.target_current['surge_right'] += -yaw * self.yaw_current

            # Sway (좌/우 이동) - teleop과 동일
            self.target_current['sway_left'] = sway * self.sway_current * self.sway_trim
            self.target_current['sway_right'] = -sway * self.sway_current

            self.get_logger().info(
                f'[FOLLOW] Dist: {self.wall_distance:.2f}m, Err: {distance_error:.2f}m, '
                f'Head: {self.heading_error:.2f}m, Surge: {surge:.2f}, Yaw: {yaw:.2f}, Sway: {sway:.2f}',
                throttle_duration_sec=1.0
            )

        self.last_time = now

        # Apply ramping and send (teleop과 동일)
        self.apply_ramping()
        for name, can_id in self.vesc_can_ids.items():
            self.send_current(can_id, self.actual_current[name])

    def destroy_node(self):
        self.stop_all()
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.get_logger().info('Controller stopped.')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()