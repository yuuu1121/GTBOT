#!/usr/bin/env python3
"""
EKF-Based Multi-Sensor Fusion Localization for Real PKRC Robot

Based on: IEEE Sensors Journal 2021 - "A Multi-Sensor Fusion Self-Localization
System of the Underwater Robot Using the Improved EKF Algorithm" (Xing et al.)

State Vector: [x, y, z, theta, v]  (5-state, Paper Eq. 10)
  - x, y: horizontal position (world frame)
  - z: depth (from pressure sensor)
  - theta: yaw heading (from IMU, after 180 deg x-axis correction)
  - v: surge speed (scalar, DVL vx_body after R_dvl_to_body transform)

Prediction (Paper Eq. 11):
  x += v * dt * cos(theta)
  y += v * dt * sin(theta)
  z, theta, v: constant (updated by measurement)

Jacobian (Paper Eq. 18): Analytical 5x5 F matrix

Measurement (Paper Eq. 19): Two modes
  - ArUco detected: H = I(5x5), z = [x, y, z, theta, v]  (full update)
  - No ArUco:       H = 3x5,    z = [z, theta, v]         (partial update)

Sensors:
    - IMU: /imu/data (GV7-INS) - yaw only (after 180 deg x-axis correction)
    - DVL: /dvl/data (DVL A50) - surge speed vx_body (scalar)
    - Depth: /pressure (MS5837) - depth from pressure
    - ArUco: /aruco/pose_6dof, /aruco/pose_array - absolute position

Publishes:
    - /ekf/odom (Odometry)
    - /ekf/path (Path)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation
import threading

# DVL A50 message
try:
    from dvl_msgs.msg import DVL
    DVL_AVAILABLE = True
except ImportError:
    DVL_AVAILABLE = False
    print("Warning: dvl_msgs not found, DVL fusion disabled")


class EKFLocalizationReal(Node):
    def __init__(self):
        super().__init__('ekf_localization_real')

        # Parameters
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('imu_inverted', True)
        self.declare_parameter('imu_rotation_axis', 'x')
        self.declare_parameter('water_density', 1025.0)
        self.declare_parameter('atm_pressure', 101325.0)
        self.declare_parameter('gravity', 9.81)

        # Process noise Q (5x5 diagonal) - Paper Eq. 12
        self.declare_parameter('process_noise_Q', [0.01, 0.01, 0.001, 0.001, 0.01])

        # Measurement noise R - Paper Eq. 19
        # Full update: [x, y, z, theta, v]
        self.declare_parameter('measurement_noise_R_full', [3.0, 3.0, 0.02, 0.01, 0.01])
        # Partial update (no ArUco): [z, theta, v]
        self.declare_parameter('measurement_noise_R_partial', [0.02, 0.01, 0.01])

        # Marker map (single marker at origin by default)
        self.declare_parameter('marker_map_ids', [0])
        self.declare_parameter('marker_map_x', [0.0])
        self.declare_parameter('marker_map_y', [0.0])
        self.declare_parameter('marker_map_z', [0.0])

        # ArUco correction
        self.declare_parameter('use_aruco_correction', True)
        self.declare_parameter('aruco_innovation_limit', 0.5)

        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.water_density = self.get_parameter('water_density').value
        self.atm_pressure = self.get_parameter('atm_pressure').value
        self.gravity = self.get_parameter('gravity').value
        self.aruco_innovation_limit = self.get_parameter('aruco_innovation_limit').value
        self.use_aruco_correction = self.get_parameter('use_aruco_correction').value

        # IMU correction
        imu_inverted = self.get_parameter('imu_inverted').value
        imu_rotation_axis = self.get_parameter('imu_rotation_axis').value

        # DVL to Robot body frame transformation
        # DVL mounting: LED -> Robot -Y (right), Transducers -> Robot +X (forward)
        self.R_dvl_to_body = np.array([
            [0, 0, 1],   # robot_vx (surge) = dvl_vz
            [-1, 0, 0],  # robot_vy (sway) = -dvl_vx
            [0, 1, 0],   # robot_vz (heave) = dvl_vy
        ])

        # IMU transformation matrix
        if imu_inverted:
            if imu_rotation_axis == 'x':
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
            elif imu_rotation_axis == 'y':
                self.R_imu_correction = Rotation.from_euler('y', 180, degrees=True)
            elif imu_rotation_axis == 'z':
                self.R_imu_correction = Rotation.from_euler('z', 180, degrees=True)
            else:
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
        else:
            self.R_imu_correction = Rotation.from_euler('x', 0, degrees=True)

        # Process noise Q (5x5)
        self.Q = np.diag(self.get_parameter('process_noise_Q').value)

        # Measurement noise R
        self.R_full = np.diag(self.get_parameter('measurement_noise_R_full').value)
        self.R_partial = np.diag(self.get_parameter('measurement_noise_R_partial').value)

        # State vector: [x, y, z, theta, v]
        self.state = np.zeros(5)
        self.P = np.diag([0.1, 0.1, 0.05, 0.01, 0.01])

        # Lock for thread-safe state access
        self.lock = threading.Lock()

        # Sensor data storage
        self.yaw = 0.0          # IMU yaw (after correction)
        self.depth = 0.0        # Pressure -> depth
        self.velocity = 0.0     # DVL surge speed (scalar)
        self.dvl_received = False
        self.imu_received = False
        self.aruco_data = None
        self.aruco_last_time = None
        self.marker_timeout = 5.0  # continuous re-application timeout
        self.aruco_update_count = 0
        self.last_dvl_nonzero_time = None  # track prolonged DVL dropout

        self.last_update_time = self.get_clock().now()

        # Build marker map
        map_ids = self.get_parameter('marker_map_ids').value
        map_x = self.get_parameter('marker_map_x').value
        map_y = self.get_parameter('marker_map_y').value
        map_z = self.get_parameter('marker_map_z').value
        self.marker_map = {}
        for i, mid in enumerate(map_ids):
            self.marker_map[int(mid)] = np.array([
                float(map_x[i]) if i < len(map_x) else 0.0,
                float(map_y[i]) if i < len(map_y) else 0.0,
                float(map_z[i]) if i < len(map_z) else 0.0
            ])

        # QoS profiles
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        dvl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, imu_qos)
        self.pressure_sub = self.create_subscription(
            Float64, '/pressure', self.pressure_callback, 10)
        self.aruco_sub = self.create_subscription(
            PoseArray, '/aruco/pose_array', self.aruco_callback, 10)
        self.aruco_6dof_sub = self.create_subscription(
            PoseStamped, '/aruco/pose_6dof', self.aruco_6dof_callback, 10)
        if DVL_AVAILABLE:
            self.dvl_sub = self.create_subscription(
                DVL, '/dvl/data', self.dvl_callback, dvl_qos)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        self.path_pub = self.create_publisher(Path, '/ekf/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        # Timer for EKF update loop
        self.timer = self.create_timer(1.0 / self.frequency, self.ekf_update_loop)

        q_diag = self.get_parameter('process_noise_Q').value
        self.get_logger().info('EKF Localization (5-state, Xing et al. 2021)')
        self.get_logger().info(f'  State: [x, y, z, theta, v]')
        self.get_logger().info(f'  Frequency: {self.frequency} Hz')
        self.get_logger().info(f'  Q diag: {q_diag}')
        self.get_logger().info(f'  DVL available: {DVL_AVAILABLE}')

    def imu_callback(self, msg):
        """IMU callback: extract yaw after 180 deg x-axis correction."""
        q = np.array([msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w])
        R_imu = Rotation.from_quat(q)
        R_corrected = self.R_imu_correction * R_imu
        _, _, yaw = R_corrected.as_euler('xyz', degrees=False)

        self.yaw = yaw
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info(f'IMU initialized: yaw={np.degrees(yaw):.1f}deg')

    def dvl_callback(self, msg):
        """DVL callback: extract surge speed (vx_body) after R_dvl_to_body transform."""
        if not msg.velocity_valid:
            return

        dvl_vel_raw = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        dvl_vel_body = self.R_dvl_to_body @ dvl_vel_raw

        # Surge speed = vx in body frame (scalar)
        self.velocity = dvl_vel_body[0]

        if abs(self.velocity) > 0.01:
            self.last_dvl_nonzero_time = self.get_clock().now()

        if not self.dvl_received:
            self.dvl_received = True
            self.get_logger().info('DVL first data received')

    def pressure_callback(self, msg):
        """Pressure sensor callback: mbar -> depth conversion."""
        pressure_pa = msg.data * 100.0
        self.depth = max(0.0, (pressure_pa - self.atm_pressure) /
                         (self.water_density * self.gravity))

    def aruco_callback(self, msg):
        """ArUco PoseArray callback with distance-weighted averaging."""
        if len(msg.poses) == 0:
            return

        # Parse marker IDs from frame_id
        try:
            parts = msg.header.frame_id.split(',')
            marker_ids = []
            for part in parts:
                if ':' in part:
                    mid = int(part.split(':')[0])
                else:
                    mid = int(part)
                marker_ids.append(mid)
        except ValueError:
            return

        if len(marker_ids) != len(msg.poses):
            return

        # Process markers
        positions = []
        weights = []
        orientations = []

        for i, pose in enumerate(msg.poses):
            marker_id = marker_ids[i]
            if marker_id not in self.marker_map:
                continue

            marker_pos = self.marker_map[marker_id]
            tvec = np.array([pose.position.x, pose.position.y, pose.position.z])

            robot_x = marker_pos[0] + tvec[0]
            robot_y = marker_pos[1] + tvec[1]

            positions.append([robot_x, robot_y])
            weight = 1.0 / (np.linalg.norm(tvec) + 0.1)
            weights.append(weight)
            orientations.append((pose.orientation, weight))

        if len(positions) == 0:
            return

        # Distance-weighted average
        positions = np.array(positions)
        weights = np.array(weights)
        weights /= np.sum(weights)

        robot_x = np.sum(weights * positions[:, 0])
        robot_y = np.sum(weights * positions[:, 1])

        # Use orientation from the closest (highest weight) marker
        best_orientation = max(orientations, key=lambda x: x[1])[0]

        self.aruco_data = {
            'x': robot_x,
            'y': robot_y,
            'z': self.depth,
            'orientation': best_orientation,
            'stamp': msg.header.stamp
        }
        self.aruco_last_time = self.get_clock().now()

    def aruco_6dof_callback(self, msg):
        """Process single ArUco 6DOF pose."""
        self.aruco_data = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': self.depth,
            'orientation': msg.pose.orientation,
            'stamp': msg.header.stamp
        }
        self.aruco_last_time = self.get_clock().now()

    def predict(self, dt):
        """EKF Prediction step (Paper Eq. 11 + Jacobian Eq. 18).

        State: [x, y, z, theta, v]
          x += v * dt * cos(theta)
          y += v * dt * sin(theta)
          z, theta, v: constant (updated by measurement step)
        """
        if dt <= 0 or dt > 1.0:
            return

        if not self.imu_received:
            return

        with self.lock:
            # Set theta and v directly from sensors (not integrated)
            self.state[3] = self.yaw
            if self.dvl_received:
                # Reset velocity if DVL hasn't sent valid data for >1s
                if self.last_dvl_nonzero_time is not None:
                    dvl_gap = (self.get_clock().now() - self.last_dvl_nonzero_time).nanoseconds / 1e9
                    if dvl_gap > 0.3:
                        self.state[4] = 0.0
                    else:
                        self.state[4] = self.velocity
                else:
                    self.state[4] = self.velocity

            theta = self.state[3]
            v = self.state[4]

            # State prediction (Paper Eq. 11)
            self.state[0] += v * dt * np.cos(theta)
            self.state[1] += v * dt * np.sin(theta)
            # z, theta, v: constant

            # Analytical Jacobian F (Paper Eq. 18)
            F = np.array([
                [1, 0, 0, -v * dt * np.sin(theta), dt * np.cos(theta)],
                [0, 1, 0,  v * dt * np.cos(theta), dt * np.sin(theta)],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]
            ])

            # Covariance prediction
            self.P = F @ self.P @ F.T + self.Q * dt

    def update_full(self, z_measured):
        """Full state update when ArUco is detected (Paper Eq. 19).

        H = I(5x5), z = [x, y, z, theta, v]
        """
        with self.lock:
            H = np.eye(5)
            y = z_measured - self.state

            # Normalize angle innovation
            y[3] = np.arctan2(np.sin(y[3]), np.cos(y[3]))

            S = H @ self.P @ H.T + self.R_full
            K = self.P @ H.T @ np.linalg.inv(S)

            self.state += K @ y
            self.state[3] = np.arctan2(np.sin(self.state[3]), np.cos(self.state[3]))
            self.P = (np.eye(5) - K @ H) @ self.P

    def update_partial(self, z_measured):
        """Partial update without ArUco (Paper Eq. 19).

        H = 3x5, z = [z, theta, v]
        """
        with self.lock:
            H = np.array([
                [0, 0, 1, 0, 0],  # z
                [0, 0, 0, 1, 0],  # theta
                [0, 0, 0, 0, 1]   # v
            ])

            z_pred = H @ self.state
            y = z_measured - z_pred

            # Normalize angle innovation
            y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))

            S = H @ self.P @ H.T + self.R_partial
            K = self.P @ H.T @ np.linalg.inv(S)

            self.state += K @ y
            self.state[3] = np.arctan2(np.sin(self.state[3]), np.cos(self.state[3]))
            self.P = (np.eye(5) - K @ H) @ self.P

    def ekf_update_loop(self):
        """Main EKF update loop."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9

        if not self.imu_received or not self.dvl_received:
            self.last_update_time = current_time
            return

        self.last_update_time = current_time

        # Prediction step
        self.predict(dt)

        # Check ArUco availability (continuous re-application within timeout)
        aruco_available = False
        aruco_xy = None

        if self.aruco_data is not None and self.use_aruco_correction and self.aruco_last_time is not None:
            time_since_aruco = (current_time - self.aruco_last_time).nanoseconds / 1e9
            if time_since_aruco < self.marker_timeout:
                aruco_xy = np.array([self.aruco_data['x'], self.aruco_data['y']])
                aruco_available = True

        # Measurement update (Paper Eq. 19)
        if aruco_available:
            z_full = np.array([
                aruco_xy[0],     # x from ArUco
                aruco_xy[1],     # y from ArUco
                self.depth,       # z from pressure
                self.yaw,         # theta from IMU
                self.velocity     # v from DVL
            ])
            self.update_full(z_full)
        else:
            z_partial = np.array([
                self.depth,       # z from pressure
                self.yaw,         # theta from IMU
                self.velocity     # v from DVL
            ])
            self.update_partial(z_partial)

        self.publish_odometry(current_time)

    def publish_odometry(self, stamp):
        """Publish odometry message."""
        with self.lock:
            x, y, z, theta, v = self.state

            odom_msg = Odometry()
            odom_msg.header.stamp = stamp.to_msg()
            odom_msg.header.frame_id = 'world'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = float(x)
            odom_msg.pose.pose.position.y = float(y)
            odom_msg.pose.pose.position.z = float(z)

            rot = Rotation.from_euler('xyz', [0, 0, theta])
            q = rot.as_quat()
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Velocity in world frame
            odom_msg.twist.twist.linear.x = float(v * np.cos(theta))
            odom_msg.twist.twist.linear.y = float(v * np.sin(theta))
            odom_msg.twist.twist.linear.z = 0.0

            # Covariance (map 5x5 to 6x6 ROS format)
            odom_msg.pose.covariance[0] = self.P[0, 0]   # x
            odom_msg.pose.covariance[7] = self.P[1, 1]   # y
            odom_msg.pose.covariance[14] = self.P[2, 2]  # z
            odom_msg.pose.covariance[35] = self.P[3, 3]  # yaw

            self.odom_pub.publish(odom_msg)

            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            self.path_msg.poses.append(pose_stamped)

            if len(self.path_msg.poses) > 1000:
                self.path_msg.poses.pop(0)

            self.path_msg.header.stamp = stamp.to_msg()
            self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationReal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
