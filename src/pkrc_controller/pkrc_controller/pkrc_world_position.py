#!/usr/bin/env python3
"""PKRC 월드좌표 계산 — vlm_ws pathfollower/rov_world_position.py 이식.

UKF-M이 낸 PKRC 상대좌표(마커 맵 = 플랫폼 정박 프레임)를 플랫폼의 월드좌표와
합쳐 PKRC의 월드좌표를 만든다. 원본과 같은 월드 잠금(lock) 방식: 한 번 계산한
좌표를 고정하고, 재계산값이 threshold 이상 벗어날 때만 갱신 — 플랫폼이 움직여도
PKRC 좌표가 흔들리지 않아 추종이 수렴할 수 있다.

시뮬/실기 차이는 ned_convert 하나로 처리한다:
  시뮬: /platform/odometry 가 이미 월드 NED → 변환 없음 (기본)
  실기: FAST-LIO odom(/Odometry, LiDAR FLU) → NED 변환 켬 (ned_convert:=true)
어느 쪽이든 추종기는 이 노드가 재발행하는 /platform/calibrated_odom 만 보면 된다.

주의(원본과 동일한 근사): ukfm 상대좌표를 플랫폼 yaw로 회전하지 않고 더한다.
마커 맵이 편대 오프셋(월드 정렬) 기준이라 플랫폼 yaw≈0 전제 — yaw가 크게
돌아가는 운용이면 여기에 회전을 넣어야 한다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R


class PkrcWorldPosition(Node):
    def __init__(self):
        super().__init__('pkrc_world_position')
        defaults = [('platform_odom_topic', '/platform/odometry'),
                    ('ukfm_odom_topic', '/ukfm/odom_validated'),
                    ('output_topic', '/pkrc/world_position'),
                    ('calibrated_odom_topic', '/platform/calibrated_odom'),
                    ('ned_convert', False),     # 실기 FAST-LIO(FLU)일 때만 True
                    ('max_position_jump', 2.0),
                    ('lock_threshold', 1.0),
                    # aruco 신선도 게이트(2026-08-19 3차 데모): DVL 없는 ukfm은 마커가
                    # 안 보이는 동안 IMU 적분 편류(~0.05 m/s, 일정 방향)로 est를 끌고
                    # 가고, 플랫폼이 그 허상을 쫓아 발산했다. 마지막 검출이 이 시간
                    # 이내일 때만 목표를 갱신 — 관측 없는 동안은 마지막 확인 위치 유지.
                    ('aruco_topic', '/aruco/pose_array'),
                    ('aruco_fresh_s', 2.0),
                    # yaw 합성(2026-08-20): 정사각 수조의 90° 대칭 때문에 FAST-LIO
                    # yaw는 동등한 4방향 사이를 널뛴다(정지 실측 —170→+82→−134°).
                    # 위치는 ±0.25 m로 건강하므로, yaw만 IMU에서 가져와 합성한다.
                    # imu_yaw_topic이 비어 있으면 종전대로 odom의 yaw를 쓴다.
                    ('imu_yaw_topic', ''),
                    ('imu_yaw_sign', 1.0),
                    ('imu_yaw_offset_deg', 0.0)]
        for n, d in defaults:
            self.declare_parameter(n, d)
        p = lambda n: self.get_parameter(n).value
        self.ned_convert = bool(p('ned_convert'))
        self.max_jump = float(p('max_position_jump'))
        self.lock_th = float(p('lock_threshold'))

        self.fresh_s = float(p('aruco_fresh_s'))
        self.plat_pos = None
        self.plat_yaw = 0.0
        self.last_plat = None
        self.locked = None          # 잠금된 PKRC 월드좌표
        self.ukfm_pos = None
        self.t_aruco = None
        # 점프 복구(2026-08-20): 기각만 하면 odom이 동결돼 추종기가 낡은 자기위치로
        # 오염된 목표를 쫓는다(실관찰 — 플랫폼이 벽까지 감). 기각이 연속되면 LIO가
        # 재정위한 것으로 보고 새 위치에 재앵커하고, 옛 프레임에서 계산된 잠금
        # 목표도 무효화한다.
        self.jump_count = 0
        self.jump_reanchor_n = 20   # 10 Hz 기준 2 s

        self.imu_yaw = None
        self.imu_sign = float(p('imu_yaw_sign'))
        self.imu_off = float(np.radians(p('imu_yaw_offset_deg')))
        imu_topic = str(p('imu_yaw_topic'))
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.world_pub = self.create_publisher(PoseStamped, p('output_topic'), 10)
        self.calib_pub = self.create_publisher(Odometry, p('calibrated_odom_topic'), 10)
        self.create_subscription(Odometry, p('platform_odom_topic'), self.on_platform, 10)
        self.create_subscription(Odometry, p('ukfm_odom_topic'), self.on_ukfm, 10)
        self.create_subscription(PoseArray, p('aruco_topic'), self.on_aruco, 10)
        self.get_logger().info(
            f"PKRC world position: plat={p('platform_odom_topic')} "
            f"ukfm={p('ukfm_odom_topic')} ned_convert={self.ned_convert} "
            f"lock={self.lock_th} m")

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]

    def on_platform(self, msg):
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        q = msg.pose.pose.orientation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        if self.ned_convert:
            # FAST-LIO FLU -> NED (원본 bb_body_to_world_ned와 동일)
            pos = np.array([pos[0], -pos[1], -pos[2]])
            yaw = -yaw
        if self.imu_yaw is not None:
            yaw = self.imu_sign * self.imu_yaw + self.imu_off
        if self.last_plat is not None and \
                np.linalg.norm(pos[:2] - self.last_plat[:2]) > self.max_jump:
            self.jump_count += 1
            if self.jump_count < self.jump_reanchor_n:
                self.get_logger().warn('플랫폼 odom 점프 — 무시', throttle_duration_sec=1.0)
                return
            # 연속 기각 → LIO 재정위로 판단, 재앵커 + 잠금 목표 무효화
            self.get_logger().warn(
                f'플랫폼 odom 재앵커: ({pos[0]:.2f}, {pos[1]:.2f}) — 잠금 목표 리셋')
            self.locked = None
        self.jump_count = 0
        self.plat_pos, self.plat_yaw, self.last_plat = pos, yaw, pos.copy()

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.child_frame_id = 'platform_calibrated'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pos
        quat = R.from_euler('xyz', [0.0, 0.0, yaw]).as_quat()
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = quat
        out.twist = msg.twist
        self.calib_pub.publish(out)

    def on_aruco(self, msg):
        if msg.poses:
            self.t_aruco = self.get_clock().now().nanoseconds * 1e-9

    def on_ukfm(self, msg):
        rel = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        self.ukfm_pos = rel
        if self.plat_pos is None:
            self.get_logger().warn('플랫폼 odom 미수신', throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_aruco is None or now - self.t_aruco > self.fresh_s:
            # 관측이 신선하지 않으면 갱신하지 않고 마지막 확인 위치만 재발행
            self.publish_locked()
            return
        cand = self.plat_pos + rel
        if self.locked is None:
            self.locked = cand.copy()
            self.get_logger().info(
                f'[LOCK] PKRC 월드좌표 잠금: ({cand[0]:.2f}, {cand[1]:.2f}, {cand[2]:.2f})')
        elif np.linalg.norm(cand[:2] - self.locked[:2]) > self.lock_th:
            self.locked = cand.copy()
            self.get_logger().info(
                f'[UNLOCK] PKRC 이동 감지 → 갱신: ({cand[0]:.2f}, {cand[1]:.2f})')
        self.publish_locked()

    def publish_locked(self):
        if self.locked is None:
            return
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose.position.x, out.pose.position.y, out.pose.position.z = self.locked
        out.pose.orientation.w = 1.0
        self.world_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PkrcWorldPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
