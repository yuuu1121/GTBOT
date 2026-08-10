#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053-485 ROS2 노드
sensor_msgs/Imu 메시지를 퍼블리시합니다.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from hwt9053_driver.hwt9053_interface import HWT9053Interface


class HWT9053Node(Node):
    """HWT9053-485 ROS2 노드"""

    def __init__(self):
        super().__init__('hwt9053_node')

        # 파라미터 선언 및 읽기
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baudrate = self.declare_parameter('baudrate', 9600).value
        self.device_address = self.declare_parameter('device_address', 0x50).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        self.publish_rate = self.declare_parameter('publish_rate', 50).value
        self.publish_tf = self.declare_parameter('publish_tf', False).value

        # 공분산 값 (센서 스펙에 맞게 조정)
        self.orientation_covariance = self.declare_parameter(
            'orientation_covariance',
            [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.01]).value
        self.angular_velocity_covariance = self.declare_parameter(
            'angular_velocity_covariance',
            [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]).value
        self.linear_acceleration_covariance = self.declare_parameter(
            'linear_acceleration_covariance',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]).value

        # 퍼블리셔 생성
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, 'imu/rpy', 10)

        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)

        # 캘리브레이션 서비스 (기본 executor 는 단일 스레드라 타이머 read 와
        # 직렬화되어 시리얼 버스 충돌 없음. 노드를 끄지 않고 호출 가능)
        self.create_service(Trigger, '~/calibrate_acc', self.srv_calibrate_acc)
        self.create_service(Trigger, '~/calibrate_mag_start', self.srv_mag_start)
        self.create_service(Trigger, '~/calibrate_mag_stop', self.srv_mag_stop)
        self.create_service(Trigger, '~/reset_yaw', self.srv_reset_yaw)

        # 센서 인터페이스 생성
        self.sensor = HWT9053Interface(
            port=self.port,
            baudrate=self.baudrate,
            device_address=self.device_address
        )

        self.get_logger().info("HWT9053 노드 초기화 완료")
        self.get_logger().info(f"  포트: {self.port}")
        self.get_logger().info(f"  보드레이트: {self.baudrate}")
        self.get_logger().info(f"  프레임 ID: {self.frame_id}")
        self.get_logger().info(f"  퍼블리시 속도: {self.publish_rate} Hz")

        # 에러 관리
        self.error_count = 0
        self.max_errors = 10

        # 타이머 (퍼블리시 루프)
        self.timer = None

    def connect(self):
        """센서 연결"""
        if not self.sensor.connect():
            self.get_logger().error("센서 연결 실패!")
            return False
        self.get_logger().info("센서 연결 성공!")
        return True

    def start(self):
        """퍼블리시 타이머 시작"""
        period = 1.0 / float(self.publish_rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def publish_imu_data(self, data):
        """
        IMU 데이터 퍼블리시

        Args:
            data: 센서 데이터 딕셔너리
        """
        if data is None:
            return

        stamp = self.get_clock().now().to_msg()

        # IMU 메시지 (with orientation)
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        # orientation 쿼터니언 (오일러각에서 계산, yaw 포함)
        q = data['quaternion']
        imu_msg.orientation.w = q[0]
        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]
        imu_msg.orientation_covariance = list(self.orientation_covariance)

        # 각속도
        imu_msg.angular_velocity.x = data['gyroscope'][0]
        imu_msg.angular_velocity.y = data['gyroscope'][1]
        imu_msg.angular_velocity.z = data['gyroscope'][2]
        imu_msg.angular_velocity_covariance = list(self.angular_velocity_covariance)

        # 선형 가속도
        imu_msg.linear_acceleration.x = data['acceleration'][0]
        imu_msg.linear_acceleration.y = data['acceleration'][1]
        imu_msg.linear_acceleration.z = data['acceleration'][2]
        imu_msg.linear_acceleration_covariance = list(self.linear_acceleration_covariance)

        self.imu_pub.publish(imu_msg)

        # Raw IMU 메시지 (without orientation)
        imu_raw_msg = Imu()
        imu_raw_msg.header.stamp = stamp
        imu_raw_msg.header.frame_id = self.frame_id
        imu_raw_msg.angular_velocity = imu_msg.angular_velocity
        imu_raw_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        imu_raw_msg.linear_acceleration = imu_msg.linear_acceleration
        imu_raw_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        # orientation은 설정하지 않음 (0으로 남김)
        imu_raw_msg.orientation_covariance[0] = -1.0  # 방향 없음 표시

        self.imu_raw_pub.publish(imu_raw_msg)

        # 자기장 메시지
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = data['magnetometer'][0] * 1e-6  # μT to T
        mag_msg.magnetic_field.y = data['magnetometer'][1] * 1e-6
        mag_msg.magnetic_field.z = data['magnetometer'][2] * 1e-6
        mag_msg.magnetic_field_covariance = [0.0] * 9  # Unknown

        self.mag_pub.publish(mag_msg)

        # Roll-Pitch-Yaw 메시지
        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = stamp
        rpy_msg.header.frame_id = self.frame_id
        rpy_msg.vector.x = data['angles'][0]  # Roll
        rpy_msg.vector.y = data['angles'][1]  # Pitch
        rpy_msg.vector.z = data['angles'][2]  # Yaw

        self.rpy_pub.publish(rpy_msg)

        # TF broadcast (옵션)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "base_link"
            t.child_frame_id = self.frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = imu_msg.orientation
            self.tf_br.sendTransform(t)

    # ------------------------------------------------------------------ #
    # 캘리브레이션 서비스 핸들러 (std_srvs/Trigger)
    # ------------------------------------------------------------------ #
    def srv_calibrate_acc(self, request, response):
        self.get_logger().info("가속도/자이로 캘리브레이션 시작 (수평·정지 유지, 약 3초)")
        ok = self.sensor.calibrate_acc_gyro()
        response.success = ok
        response.message = "가속도 캘리브레이션 완료·저장" if ok else "가속도 캘리브레이션 실패"
        self.get_logger().info(response.message)
        return response

    def srv_mag_start(self, request, response):
        ok = self.sensor.start_mag_cali()
        response.success = ok
        response.message = ("자기장 캘리브레이션 시작 — 센서를 X/Y/Z 전 축으로 천천히 "
                            "360° 회전 후 calibrate_mag_stop 호출") if ok else "시작 실패"
        self.get_logger().info(response.message)
        return response

    def srv_mag_stop(self, request, response):
        ok = self.sensor.stop_cali_and_save()
        response.success = ok
        response.message = "자기장 캘리브레이션 종료·저장" if ok else "종료 실패"
        self.get_logger().info(response.message)
        return response

    def srv_reset_yaw(self, request, response):
        ok = self.sensor.reset_yaw()
        response.success = ok
        response.message = "Yaw(Z) 리셋·저장" if ok else "Yaw 리셋 실패"
        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        """퍼블리시 루프 (타이머 콜백)"""
        # 종료(SIGTERM/SIGINT) 중 컨텍스트가 무효화된 뒤 퍼블리시 시도 방지
        if not rclpy.ok():
            return
        data = self.sensor.read_all()
        if data:
            self.publish_imu_data(data)
            self.error_count = 0  # 성공 시 에러 카운터 리셋
        else:
            self.error_count += 1
            self.get_logger().warn("센서 데이터 읽기 실패",
                                   throttle_duration_sec=1.0)

            # 에러가 많으면 재연결 시도 (타이머 블로킹을 피하려고 짧게 처리)
            if self.error_count >= self.max_errors:
                self.get_logger().warn("센서 재연결 시도...")
                self.sensor.disconnect()
                if self.connect():
                    self.error_count = 0
                else:
                    self.get_logger().error("재연결 실패. 계속 재시도합니다...")
                    # 다음 콜백들에서 계속 시도되도록 카운터만 살짝 낮춤
                    self.error_count = self.max_errors // 2

    def shutdown(self):
        """종료 처리"""
        self.get_logger().info("노드 종료 중...")
        self.sensor.disconnect()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    node = HWT9053Node()

    if not node.connect():
        node.destroy_node()
        rclpy.shutdown()
        return

    node.start()

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
