# Changelog

## [Unreleased] — Pre-experiment Fix H-1 (fix)

### Changed
- `launch/robot_state_publisher.launch.py` — accepts `use_sim_time` argument and propagates to robot_state_publisher node parameters. Default `false` for live experiments. Without this, bag replay caused TF timestamps to use wall clock instead of sim clock, producing tf2 lookup mismatches against ROS-time-stamped sensor data.

### Notes
- Parents (`fast_lio/launch/mapping.launch.py`, `fast_lio/launch/localization.launch.py`) updated in same PR to forward `use_sim_time` via `launch_arguments`.

## [Unreleased] — Phase P3: TF naming standard (refactor)

### Added
- `livox_link` (alias of `livox_frame`, REP-105 + Autoware 표준)
- `oculus_link`, `ping360_link`, `ping1d_link`, `sonoptix_link` — per-sensor child links under `sonar_link`

### Notes
- Legacy aliases (`body`, `oculus`, `livox_frame`) preserved for old bag file compatibility
- All new sensor links use xyz="0 0 0" rpy="0 0 0" (same physical mount as parent — placeholder for future per-sensor offsets)

### Verification
- XML parse PASS (links 11, joints 10)
- colcon build PASS
- robot_state_publisher: 신규 5 frames 모두 /tf_static에 publish됨
