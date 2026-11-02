#!/usr/bin/env python3
"""
PKRC Depth Controller - maintains target depth using pressure sensor
Works alongside teleop by controlling only heave thrusters
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import FluidPressure
from rcl_interfaces.msg import SetParametersResult


class DepthPID:
    def __init__(self, kp=0.8, ki=0.35, kd=1.2, max_integral=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_integral = max_integral  # 적분 windup 방지
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


class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')    

        # Parameters
        self.declare_parameter('vehicle_name', 'pkrc')
        self.declare_parameter('target_depth', 0.0)  # meters (positive = down)
        self.declare_parameter('kp', 0.8)   # 비례항
        self.declare_parameter('ki', 0.35)   # 적분항 (정상상태 오차 제거)
        self.declare_parameter('kd', 1.2)   # 미분항 (안정성)
        self.declare_parameter('enabled', True)
        self.declare_parameter('depth_offset', 0.0)  # 깊이 보정값 (m)
        # 차량별 스러스터 배치(2026-11-01). pkrc: 6기, heave 4·5번, +PWM이 상승이라 둘 다 -1.
        # bluerov2: 8기, Dive 4~7번인데 프로펠러 handedness가 번갈아 있어 같은 PWM이면 추력이
        # 상쇄되고 반동 토크만 더해져 제자리 yaw 회전한다(실측 1 rad/s). TAM.yaml heave 행
        # [-1,+1,+1,-1]대로 부호를 줘야 추력이 모이고 토크가 상쇄된다. heave_signs[i]*PWM이 '하강'.
        self.declare_parameter('num_thrusters', 6)
        self.declare_parameter('heave_indices', [4, 5])
        self.declare_parameter('heave_signs', [-1.0, -1.0])
        # 적분 한계 × ki = 부력을 상쇄하는 정상 PWM의 상한(2026-11-01). pkrc는 0.344로 한계 1.0×0.35
        # 바로 아래, bluerov2는 0.49가 필요해 1.0이면 0.2 m 못 내려간다(실측) → bluerov2는 4.0(런치 max_integral).
        self.declare_parameter('max_integral', 1.0)

        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.target_depth = self.get_parameter('target_depth').value
        self.enabled = self.get_parameter('enabled').value
        self.depth_offset = self.get_parameter('depth_offset').value

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid = DepthPID(kp, ki, kd, self.get_parameter('max_integral').value)

        self.current_depth = 0.0
        self.last_time = None

        # Pressure to depth: depth = pressure / (rho * g)
        # Stonefish uses gauge pressure (relative to atmospheric)
        self.water_density = 1000.0  # kg/m3
        self.gravity = 9.81  # m/s2

        # Subscribe to pressure sensor
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            f'/{self.vehicle_name}/pressure',
            self.pressure_callback,
            10
        )

        # Subscribe to target depth commands
        self.target_sub = self.create_subscription(
            Float64,
            f'/{self.vehicle_name}/depth/target',
            self.target_callback,
            10
        )

        # Subscribe to teleop PWM
        self.teleop_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.vehicle_name}/teleop/pwm',
            self.teleop_callback,
            10
        )

        # Publisher for merged PWM
        self.pwm_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.vehicle_name}/setpoint/pwm',
            10
        )

        # Store teleop commands
        self.heave_indices = list(self.get_parameter('heave_indices').value)
        self.heave_signs = [float(v) for v in self.get_parameter('heave_signs').value]
        assert len(self.heave_signs) == len(self.heave_indices), 'heave_signs와 heave_indices 길이가 다름'
        self.teleop_pwm = [0.0] * int(self.get_parameter('num_thrusters').value)
        self.heave_pwm = 0.0
        self.last_meas_time = None

        # Also publish depth info
        self.depth_pub = self.create_publisher(
            Float64,
            f'/{self.vehicle_name}/depth/current',
            10
        )

        # ukfm_localization의 pressure_callback은 mbar 절대압(Float64)을 기대한다
        # (내부에서 1013.25 mbar를 빼고 깊이로 환산). Stonefish는 FluidPressure(게이지
        # Pa)라 여기서 절대 mbar로 바꿔 별도 토픽으로 중계한다 — 깊이(m)를 그대로
        # 물리면 (4−1013)mbar 환산으로 z가 −10 m에 박힌다(실측 −10.037).
        self.pressure_mbar_pub = self.create_publisher(
            Float64,
            f'/{self.vehicle_name}/pressure_mbar',
            10
        )

        # Control timer (higher rate to override teleop heave)
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz

        # Parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info(f'=== PKRC Depth Controller ===')
        self.get_logger().info(f'Target depth: {self.target_depth:.2f} m')
        self.get_logger().info(f'Set depth: ros2 topic pub /{self.vehicle_name}/depth/target std_msgs/Float64 "data: 2.0"')

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
        # rclpy.parameter.SetParametersResult는 존재하지 않는 API — 원본(vlm_ws)
        # 코드가 파라미터를 바꾸는 순간 AttributeError로 죽는 버그. 올바른 타입으로 교체.
        return SetParametersResult(successful=True)

    def pressure_callback(self, msg):
        # Convert gauge pressure to depth
        # depth = P / (rho * g) + offset
        pressure = msg.fluid_pressure
        self.current_depth = pressure / (self.water_density * self.gravity) + self.depth_offset

        # Publish current depth
        depth_msg = Float64()
        depth_msg.data = self.current_depth
        self.depth_pub.publish(depth_msg)

        mbar_msg = Float64()
        mbar_msg.data = pressure / 100.0 + 1013.25   # 게이지 Pa -> 절대 mbar
        self.pressure_mbar_pub.publish(mbar_msg)

        # PID는 측정이 들어올 때 실제 측정 간격 dt로 돈다(2026-11-01). 종전엔 50 Hz 루프에서
        # dt=0.02 s로 미분했는데 압력은 5~10 Hz라 갱신 순간마다 Δe/0.02로 튀어 PWM이 포화됐고,
        # bluerov2(압력 5 Hz)는 5 m 주위 ±0.3 m 한계 사이클이 이득을 줄여도 안 사라졌다(실측).
        now = self.get_clock().now().nanoseconds / 1e9
        if self.enabled and self.last_meas_time is not None:
            dt = now - self.last_meas_time
            if dt > 0:
                # Depth error: positive means we need to go deeper
                self.heave_pwm = self.pid.compute(self.target_depth - self.current_depth, dt)
        self.last_meas_time = now

    def target_callback(self, msg):
        self.target_depth = msg.data
        self.pid.reset()
        self.get_logger().info(f'Target depth set to: {self.target_depth:.2f} m')

    def teleop_callback(self, msg):
        # Store teleop commands
        if len(msg.data) >= 6:
            self.teleop_pwm = list(msg.data)

    def control_loop(self):
        # Start with teleop commands
        pwm_cmd = self.teleop_pwm.copy()

        if self.enabled:
            # Override heave with depth controller.
            # gtbot_world.scn의 pkrc heave 스러스터는 +PWM이 상승 방향(실측:
            # +1 인가 시 0.4 m 부상, -1 인가 시 25 s에 9 m 하강) — 깊이 오차
            # 양수(더 깊이)에 음수 PWM이 필요해 부호를 뒤집는다.
            for i, sign in zip(self.heave_indices, self.heave_signs):
                pwm_cmd[i] = sign * self.heave_pwm

        # Publish merged PWM
        msg = Float64MultiArray()
        msg.data = pwm_cmd
        self.pwm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop thrusters
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0] * len(node.teleop_pwm)
        node.pwm_pub.publish(stop_msg)
        node.get_logger().info('Depth controller stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()