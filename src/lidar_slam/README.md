# LiDAR SLAM

ROS2 LiDAR SLAM packages for 3D mapping and localization with Livox MID-360.

## Packages

| Package | Description |
|---------|-------------|
| `fast_lio` | FAST-LIO2 LiDAR-Inertial Odometry |
| `cartographer_slam` | Google Cartographer 2D SLAM with Livox support |
| `boat_description` | Robot URDF description and TF tree |

## Build

```bash
cd /workspace/ros2_ws
colcon build --packages-select boat_description fast_lio cartographer_slam
source install/setup.bash
```

## Usage

### FAST-LIO

```bash
# Basic mapping
ros2 launch fast_lio mapping.launch.py

# With RViz
ros2 launch fast_lio mapping.launch.py rviz:=true

# Bag playback (simulation time + BEST_EFFORT QoS)
ros2 launch fast_lio mapping.launch.py use_sim_time:=true qos_reliability:=best_effort
```

### Cartographer SLAM

```bash
# Livox CustomMsg SLAM (default)
ros2 launch cartographer_slam slam.launch.py

# PointCloud2 SLAM
ros2 launch cartographer_slam slam.launch.py input_type:=pointcloud2

# Localization mode
ros2 launch cartographer_slam slam.launch.py localization:=true \
  load_state_filename:=/path/to/map.pbstream
```

## TF Tree

```
map
└── odom
    └── base_link
        ├── livox_frame (LiDAR)
        ├── imu_link
        └── sonar_link
```

## Topics

### FAST-LIO

| Topic | Type | Description |
|-------|------|-------------|
| `/slam/fast_lio/odometry` | Odometry | LiDAR-Inertial odometry |
| `/slam/fast_lio/points_body` | PointCloud2 | Registered point cloud (body frame) |
| `/slam/fast_lio/debug/path` | Path | Trajectory path (debug, opt-in) |

### Cartographer

| Topic | Type | Description |
|-------|------|-------------|
| `/slam/cartographer/map` | OccupancyGrid | 2D occupancy grid map |
| `/slam/cartographer/tracked_pose` | PoseStamped | Current robot pose |
| `/slam/cartographer/odom` | Odometry | Odometry output |

## Launch Arguments

### FAST-LIO

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | false | Use simulation time |
| `rviz` | false | Launch RViz |
| `foxglove` | true | Launch Foxglove bridge |
| `qos_reliability` | reliable | QoS: reliable or best_effort |
| `save_map_path` | '' | Path to save PCD map |

### Cartographer

| Argument | Default | Description |
|----------|---------|-------------|
| `input_type` | livox | Input: livox or pointcloud2 |
| `localization` | false | Pure localization mode |
| `resolution` | 0.05 | Occupancy grid resolution |
| `load_state_filename` | '' | Load .pbstream file |
| `save_state_filename` | '' | Save .pbstream on shutdown |
