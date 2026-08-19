#!/usr/bin/env python3
"""
PKRC Depth Controller - Real Hardware Version
- MS5837 압력센서로 깊이 측정 (/pressure)
- VESC CAN 통신으로 heave 스러스터 제어
- teleop과 함께 동작 (heave만 오버라이드)

Usage:
    ros2 run pkrc_controller depth_controller

Prerequisites:
    - 압력센서 노드 실행: ros2 run pressure_sensor pressure_sensor_node
    - CAN 인터페이스 활성화: sudo ip link set can0 up type can bitrate 500000
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

import can


class DepthPID:
    def __init__(self, kp=0.8, ki=0.35, kd=1.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_integral = 1.0
        self.max_output = 1.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class DepthControllerReal(Node):
    # VESC CAN IDs for heave control
    VESC_HEAVE_UP = 0x155    # vesc_5
    VESC_HEAVE_DOWN = 0x156  # vesc_6

    # Teleop axes mapping
    VESC_IDS = {
        'surge_left': 0x151,
        'surge_right': 0x152,
        'sway_left': 0x153,
        'sway_right': 0x154,
        'heave_up': 0x155,
        'heave_down': 0x156,
    }

    def __init__(self):
        super().__init__('depth_controller_real')

        # Parameters
        self.declare_parameter('target_depth', 0.0)  # meters (positive = down)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.35)
        self.declare_parameter('kd', 1.2)
        self.declare_parameter('enabled', True)
        self.declare_parameter('depth_offset', 0.0)
        self.declare_parameter('max_current', 2.0)
        self.declare_parameter('current_scale', 3.0)
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('standalone_mode', False)  # True: heave만 제어, False: teleop과 함께

        self.target_depth = self.get_parameter('target_depth').value
        self.enabled = self.get_parameter('enabled').value
        self.depth_offset = self.get_parameter('depth_offset').value
        self.max_current = self.get_parameter('max_current').value
        self.current_scale = self.get_parameter('current_scale').value
        self.can_channel = self.get_parameter('can_channel').value
        self.standalone_mode = self.get_parameter('standalone_mode').value

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid = DepthPID(kp, ki, kd)

        self.current_depth = 0.0
        self.last_time = None

        # Pressure to depth conversion
        self.surface_pressure = 1013.25  # mbar

        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(channel=self.can_channel, interface='socketcan')
            self.get_logger().info(f'CAN bus initialized on {self.can_channel}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.bus = None

        # Current ramping
        self.target_currents = {name: 0.0 for name in self.VESC_IDS}
        self.actual_currents = {name: 0.0 for name in self.VESC_IDS}
        self.max_ramp_step = 0.5

        # Store teleop commands (for non-standalone mode)
        self.teleop_surge = 0.0
        self.teleop_sway = 0.0

        # Subscribe to pressure sensor
        self.pressure_sub = self.create_subscription(
            Float64,
            '/pressure',
            self.pressure_callback,
            10
        )

        # Subscribe to target depth commands
        self.target_sub = self.create_subscription(
            Float64,
            '/pkrc/depth/target',
            self.target_callback,
            10
        )

        # Subscribe to joystick for teleop (non-standalone mode)
        if not self.standalone_mode:
            self.joy_sub = self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                10
            )

        # Publish current depth
        self.depth_pub = self.create_publisher(
            Float64,
            '/pkrc/depth/current',
            10
        )

        # Control timer (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # Parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info('=== PKRC Depth Controller (Real Hardware) ===')
        self.get_logger().info(f'Target depth: {self.target_depth:.2f} m')
        self.get_logger().info(f'Mode: {"Standalone (heave only)" if self.standalone_mode else "With teleop"}')
        self.get_logger().info(f'Set depth: ros2 topic pub /pkrc/depth/target std_msgs/Float64 "data: 2.0"')

    def param_callback(self, params):
        for param in params:
            if param.name == 'target_depth':
                self.target_depth = param.value
                self.pid.reset()
                self.get_logger().info(f'Target depth set to: {self.target_depth:.2f} m')
            elif param.name == 'enabled':
                self.enabled = param.value
                if not self.enabled:
                    self.pid.reset()
                    self.stop_heave()
                self.get_logger().info(f'Depth hold {"enabled" if self.enabled else "disabled"}')
            elif param.name == 'kp':
                self.pid.kp = param.value
            elif param.name == 'ki':
                self.pid.ki = param.value
            elif param.name == 'kd':
                self.pid.kd = param.value
            elif param.name == 'depth_offset':
                self.depth_offset = param.value
                self.get_logger().info(f'Depth offset: {self.depth_offset:.2f} m')
            elif param.name == 'max_current':
                self.max_current = param.value
            elif param.name == 'current_scale':
                self.current_scale = param.value
        return SetParametersResult(successful=True)

    def pressure_callback(self, msg: Float64):
        """MS5837 압력센서에서 깊이 계산"""
        pressure_mbar = msg.data
        self.current_depth = (pressure_mbar - self.surface_pressure) * 0.01019716 + self.depth_offset

        # Publish current depth
        depth_msg = Float64()
        depth_msg.data = self.current_depth
        self.depth_pub.publish(depth_msg)

    def target_callback(self, msg: Float64):
        self.target_depth = msg.data
        self.pid.reset()
        self.get_logger().info(f'Target depth set to: {self.target_depth:.2f} m')

    def joy_callback(self, msg: Joy):
        """조이스틱 입력 처리 (teleop 모드)"""
        if len(msg.axes) >= 2:
            # axes[0]: left/right (sway)
            # axes[1]: forward/backward (surge)
            self.teleop_sway = msg.axes[0]
            self.teleop_surge = msg.axes[1]

    def send_current(self, can_id: int, current: float):
        """VESC에 전류 명령 전송"""
        if self.bus is None:
            return

        current = max(-self.max_current, min(self.max_current, current))
        scaled_current = int(current * 1000)

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
            self.get_logger().error(f'CAN send error: {e}', throttle_duration_sec=1.0)

    def apply_ramping(self):
        """전류 램핑 적용"""
        for name in self.VESC_IDS:
            diff = self.target_currents[name] - self.actual_currents[name]
            if abs(diff) > self.max_ramp_step:
                if diff > 0:
                    self.actual_currents[name] += self.max_ramp_step
                else:
                    self.actual_currents[name] -= self.max_ramp_step
            else:
                self.actual_currents[name] = self.target_currents[name]

    def send_all_currents(self):
        """모든 VESC에 전류 명령 전송"""
        self.apply_ramping()
        for name, can_id in self.VESC_IDS.items():
            self.send_current(can_id, self.actual_currents[name])

    def stop_heave(self):
        """Heave 스러스터만 정지"""
        self.target_currents['heave_up'] = 0.0
        self.target_currents['heave_down'] = 0.0

    def stop_all(self):
        """모든 스러스터 정지"""
        for name in self.VESC_IDS:
            self.target_currents[name] = 0.0
        self.send_all_currents()

    def pwm_to_current(self, pwm_value: float) -> float:
        """PWM 값을 전류로 변환"""
        return pwm_value * self.current_scale

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Reset target currents
        for name in self.VESC_IDS:
            self.target_currents[name] = 0.0

        # Teleop control (non-standalone mode)
        if not self.standalone_mode:
            # Surge (forward/backward)
            surge_current = self.pwm_to_current(self.teleop_surge)
            self.target_currents['surge_left'] = -surge_current
            self.target_currents['surge_right'] = surge_current

            # Sway (left/right)
            sway_current = self.pwm_to_current(self.teleop_sway)
            self.target_currents['sway_left'] = sway_current
            self.target_currents['sway_right'] = -sway_current

        # Depth control (always active when enabled)
        if self.enabled and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # Depth error: positive means we need to go deeper
                depth_error = self.target_depth - self.current_depth

                # Compute heave correction
                heave_pwm = self.pid.compute(depth_error, dt)
                heave_current = self.pwm_to_current(heave_pwm)

                # Apply to heave thrusters
                self.target_currents['heave_up'] = heave_current
                self.target_currents['heave_down'] = -heave_current

                self.get_logger().debug(
                    f'Depth: {self.current_depth:.2f}m, Target: {self.target_depth:.2f}m, '
                    f'Error: {depth_error:.2f}m, Heave: {heave_current:.2f}A'
                )

        self.last_time = current_time

        # Send all commands
        self.send_all_currents()

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.stop_all()
        if self.bus is not None:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthControllerReal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.get_logger().info('Depth controller stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
