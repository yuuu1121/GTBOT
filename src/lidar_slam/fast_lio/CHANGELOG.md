# Changelog

## [1.1.1] — 2026-07-29 (fix)

### Fixed
- `localization_node`: `/initialpose` hints from RViz / pkrc_visualizer Pose Estimate are now honored immediately. The previous code wrote `loc_initialized_ = false` from the callback, but the main loop never read that flag — it just kept running single-pass `performLocalizationStep(false)` whose fitness gate (≥0.5) typically rejected the hint-seeded update, so the user click decayed silently. Click-to-snap now converges in seconds instead of ~60 s.

### Changed
- `localization_node.h`: new `std::atomic<bool> reinit_requested_{false}` member alongside `running_`.
- `localization_node.cpp` (`initialPoseCallback`): sets `reinit_requested_.store(true)` and logs the requested xyz; no longer touches `loc_initialized_` (the loop owns that state).
- `localization_engine.cpp` (`localizationLoop`): top of the main `while` body atomically exchanges the flag and, when set, replays `initializeLocalization()` plus a Kalman re-seed before resuming tracking. `initializeLocalization()` itself is unchanged — its existing `mat_initialpose_.norm() < 0.1` branch already routes user-supplied hints around global FPFH and into the multi-pass init ICP path.

### Verification
- colcon build PASS (fast_lio).
- Manual smoke: launch node with a missing map → clean shutdown; launch with a real map and bag → normal init + tracking unchanged.
- Regression A (no-hint): measured pre-rebase as v1.0.2 baseline vs v1.0.3 candidate on UCRC bag, last-100-pose mean Δposition ≤ 0.05 m, Δyaw ≤ 1°. (Manual smoke in PR test plan.)
- Regression B (hint fast-path, measured pre-rebase): bag replay + Pose Estimate click ~2 m off → confidence ≥ 0.5 within 5 s. (Manual smoke in PR test plan.)

### Notes
- No public API, topic, or QoS changes. ROS 2 graph identical to v1.1.0.
- The previous `// Re-initialize localization` comment in the callback was aspirational; this patch makes it true.

## [1.1.0] — 2026-05-07 (minor)

### Changed
- Topic prefix unified from `/localization/...` → `/slam/...` and debug topics moved under the same root. Workspace-wide convention: every SLAM-domain topic now lives under `/slam/<engine>/...`. Renames:
  - `/localization/fast_lio/odometry` → `/slam/fast_lio/odometry`
  - `/localization/fast_lio/points_body` → `/slam/fast_lio/points_body`
  - `/localization/fast_lio_loc/odometry` → `/slam/fast_lio_loc/odometry`
  - `/localization/fast_lio_loc/confidence` → `/slam/fast_lio_loc/confidence`
  - `/localization/fast_lio_loc/map` → `/slam/fast_lio_loc/map`
  - `/localization/fast_lio_loc/occupancy_grid` → `/slam/fast_lio_loc/occupancy_grid`
  - `/fast_lio/debug/points_world` → `/slam/fast_lio/debug/points_world`
  - `/fast_lio/debug/points_effected` → `/slam/fast_lio/debug/points_effected`
  - `/fast_lio/debug/map` → `/slam/fast_lio/debug/map`
  - `/fast_lio/debug/path` → `/slam/fast_lio/debug/path`

### Notes
- Coordinated breaking change with `cartographer_slam`, `sonar_3d_reconstruction`, `pkrc_visualizer`. All four packages must be upgraded together; mixed-version deployments will silently lose topic connectivity.
- TF tree (REP-105 `map → odom → base_link → ...`) unchanged. Frame names are independent of topic names.
- QoS unchanged (RELIABLE for odometry/confidence/initialpose, SENSOR for point clouds, LATCHED for map/occupancy_grid).

### Verification
- grep `/localization/` `/fast_lio/debug/` in `src/`, `launch/`, `config/`, `rviz/`, `scripts/` → 0 hits (CHANGELOG history entries excluded).
- colcon build PASS (fast_lio).
- Manual smoke: `ros2 topic list | grep ^/slam/fast_lio` after launch → 6 topics in mapping mode, 10 in localization mode.

## [1.0.2] — 2026-05-06 (fix)

### Fixed
- `launch/localization.launch.py`: force `publish.scan_publish_en: True` on the FAST-LIO node so `/fast_lio/debug/points_world` is published in localization mode. Without this, the publisher was never created (laserMapping.cpp:1001-1002 creates it only when `scan_pub_en || pcd_save_en`, and the same launch sets `pcd_save_en=False`). This silenced any visualizer or downstream consumer that subscribed to the world-frame cloud during localization runs.

### Changed
- `config/slam/mid360.yaml`: rewrote the `scan_publish_en: false` comment to explain the real reason (publisher stays alive in mapping mode via `pcd_save_en=true`; localization mode overrides via launch). The old comment "disable for localization mode" was misleading — the override now lives in the localization launch.

### Notes
- Underlying coupling in `laserMapping.cpp` (`scan_pub_en || pcd_save_en` shared guard for publisher lifetime) is left intact. Splitting the two responsibilities (debug-publish vs. pcd-save) would touch SLAM core code and require regression validation; out of scope for this patch.
- Algorithm impact: none (publish path is purely a side effect; ICP / IKD-tree / odometry unaffected). Cost is one additional PointCloud2 serialization per scan, only in localization mode.

### Verification
- colcon build PASS (fast_lio).
- Manual smoke: localization launch + bag replay → `ros2 topic hz /fast_lio/debug/points_world` returns non-zero rate.

## [Unreleased] — Post-Audit Fix PR-N (fix)

### Fixed
- `fast_lio/scripts/regression_test.sh`: `ODOM_TOPIC` default `/fast_lio/odometry` → `/localization/fast_lio/odometry` to track PR-J topic rename (7th audit High N-1).
- `fast_lio/scripts/regression_test_localization.sh`: `ODOM_TOPIC` default `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry`; comment cite of `/fast_lio/odometry` → `/localization/fast_lio/odometry` (7th audit High N-1).

### Changed
- `README.md` FAST-LIO topic catalog reflects 8-phase workspace convention rename: `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/cloud_registered` → `/localization/fast_lio/points_body`, `/fast_lio/path` → `/fast_lio/debug/path` (7th audit Medium N-3).

### Added
- `.gitignore` (repo root): exclude `build/`, `install/`, `log/`, `fast_lio/PCD/*.pcd`, `*.pyc`, `__pycache__/` to prevent accidental check-in of 164MB PCD outputs and colcon build artefacts (7th audit High N-2).

### Verification
- regression scripts, README, .gitignore are non-build assets; colcon build skipped (no source change).

## [Unreleased] — Post-Audit Fix PR-G (fix)

### Changed
- `rviz/fastlio.rviz`: `Fixed Frame: camera_init` → `odom`; TF Frames + Tree `camera_init` → `odom`. Topics `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body`, `/fast_lio/debug/cloud_effected` → `/fast_lio/debug/points_effected` (4th audit Critical: post-P3/P15 rename never propagated to rviz config).
- `rviz/fastlio.rviz`: Reliability Policy Reliable → Best Effort for `/fast_lio/debug/path`, `/localization/fast_lio/points_body`, `/fast_lio/debug/points_effected`, `/fast_lio/debug/map` (publishers use `pkrc_qos::sensor_qos()` BE; Reliable sub never received messages).
- `rviz/localization.rviz`: TF Frames + Tree `camera_init` → `odom`. Topics `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body`, `/fast_lio/localization/map` → `/localization/fast_lio_loc/map`, `/fast_lio/localization/occupancy_grid` → `/localization/fast_lio_loc/occupancy_grid`, `/fast_lio/odometry` → `/localization/fast_lio/odometry`, `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry`.
- `rviz/localization.rviz`: Path display Reliability → Best Effort; CurrentScan Reliability → Best Effort.
- `launch/mapping.launch.py`: rviz2 Node `parameters=[{'use_sim_time': use_sim_time == 'true'}]` (without this, rviz uses wall clock during bag replay; localization.launch.py already had this).
- `src/slam/laserMapping.cpp:publish_path` — `path.header.stamp = msg_body_pose.header.stamp` per call. Without this, Path msg header carried constructor-time wall clock for the entire run.

### Verification
- colcon build PASS

## [Unreleased] — Post-Audit Fix PR-E (fix)

### Changed
- `launch/localization.launch.py` docstring TOPICS Input — `/livox/lidar` → `/sensor/lidar/livox_mid360/points`, `/livox/imu` → `/sensor/ins/livox_mid360/imu` (H-4)
- `launch/mapping.launch.py` docstring TOPICS Input/Output — livox topic names corrected; `/fast_lio/path` → `/fast_lio/debug/path` (H-5)
- `src/slam/laserMapping.cpp` lidar/IMU subscriber QoS — `qos_reliability` yaml 파라미터 분기 제거, `rclcpp::QoS(10)` / `rclcpp::SensorDataQoS()` 로컬 패턴 → `pkrc_qos::sensor_qos()` (M-2). depth 5로 통일.
- `src/localization/localization_node.cpp:193` `sub_initialpose_` QoS — `10` → `pkrc_qos::reliable_qos()` (L-1, style only).

### Verification
- colcon build PASS (56.2s, fast_lio)

## [Unreleased] — Pre-experiment Fix H-1 (fix)

### Changed
- `launch/mapping.launch.py`, `launch/localization.launch.py` — pass `use_sim_time` via `launch_arguments` when including `boat_description/robot_state_publisher.launch.py`. Required so RSP runs on sim clock during bag replay.

## [Unreleased] — Post-Audit Fix B-1 + B-2 (fix)

### Fixed
- B-1: localization_node.cpp cloud subscription topic `/localization/fast_lio/cloud_body` → `/localization/fast_lio/points_body` (publisher uses `points_body`; PR-A introduced silent ICP data loss).
- B-2: localization_node.cpp subscriber QoS — sub_odom_ now `pkrc_qos::reliable_qos()`, sub_cloud_ now `pkrc_qos::sensor_qos()` (matches publisher QoS profiles).

### Verification
- colcon build PASS
- grep `/localization/fast_lio/cloud_body` runtime = 0 hits

### Notes
- Triggered by 2026-05-06 second-pass audit on PR-A1.

## [Unreleased] — Phase P9: Post-merge fixes (fix)

### Fixed
- `src/localization/localization_node.cpp`: subscriber 토픽명을 P4 컨벤션에 맞춰 수정
  - `/fast_lio/odometry` → `/localization/fast_lio/odometry` (C-1)
  - `/fast_lio/cloud_registered_body` → `/localization/fast_lio/cloud_body` (C-1)
- `src/slam/laserMapping.cpp`: `qos_reliability` 기본값 `best_effort`로 변경 (C-1b)
  - livox driver SensorDataQoS(BEST_EFFORT) publisher와 정합
- `src/slam/laserMapping.cpp`, `include/fast_lio/localization/localization_node.h`, `include/fast_lio/localization/tf_publisher.h`: 레거시 frame 기본값 REP-105/Autoware 기준으로 통일 (M-1)
  - `camera_init` → `odom`, `body` → `base_link`
- `launch/mapping.launch.py`, `launch/localization.launch.py`: TOPICS docstring을 실제 publish 토픽에 맞춰 정렬

### Verification
- colcon build PASS (fast_lio)
- grep 검증: `"camera_init"` / `"body"` src/include 0 hits
- grep 검증: `/fast_lio/odometry` / `/fast_lio/cloud_registered_body` runtime 코드 0 hits

### Notes
- 2026-05-06 post-merge audit에서 발견된 항목 (docs/superpowers/audits/)

## [Unreleased] — Phase P7: Map save UX (refactor)

### Changed
- `launch/mapping.launch.py`: `generate_launch_description()` node creation wrapped in `OpaqueFunction` (`_setup_nodes`) so `output_map_path` can be resolved at launch time (was static LaunchConfiguration substitution)
- `launch/mapping.launch.py`: `output_map_path` description updated: `Empty = auto-timestamp`
- `launch/mapping.launch.py`: `import os.path` → `import os` (needed for `os.environ`/`os.path.expanduser`)
- `launch/mapping.launch.py`: `IfCondition` removed (replaced by plain `if use_rviz:` in OpaqueFunction)
- Header docstring updated: default note for `output_map_path` + auto-timestamp example

### Added
- `launch/mapping.launch.py`: `_resolve_map_path(user_path, pkg, filename)` inline helper (spec §2.9):
  - Empty `output_map_path` → auto-timestamp dir `$PKRC_MAP_DIR/fast_lio/<YYYYMMDD_HHMMSS>/` (fallback `~/data/maps`)
  - Updates relative `latest -> <ts>` symlink
  - Non-empty → use as-is, parent dir auto-created

### Verification
- colcon build PASS (fast_lio 0.27s)
- `ros2 launch fast_lio mapping.launch.py --show-args` PASS
- Empty `output_map_path`: timestamp dir + `latest` symlink created under `$PKRC_MAP_DIR/fast_lio/`

### Notes
- `IfCondition` from `launch.conditions` is no longer imported (was only used for rviz conditional)

## [Unreleased] — Phase P6: Config structure (refactor)

### Changed
- `config/slam/mid360.yaml`: header docstring + category separators (TF/COMMON/PREPROCESS/MAPPING/PUBLISH/PCD_SAVE) + unit annotations ([m]/[s]/[Hz]/[deg]) + [Static]/[Dynamic] tags
- `config/localization/localization.yaml`: header docstring + category separators (MAP/REGISTRATION/INITIAL POSE/GLOBAL LOCALIZATION/KALMAN FILTER/SUBMAP UPDATE/DEBUG/TF FRAMES/OCCUPANCY GRID) + unit annotations + [Static] tags

### Added
- No new parameters — documentation only

### Verification
- colcon build PASS (fast_lio 0.26s)
- yaml.safe_load PASS (both yaml files)
- ros2 launch fast_lio mapping.launch.py startup: no parse errors

### Notes
- scan_publish_en working-tree edit (false→true) excluded from this commit — unrelated user edit
- Values unchanged — cosmetic / documentation only per spec §2.7

## [Unreleased] — Phase P5a: Launch arg standardization (refactor)

### Changed (BREAKING — external launch invocations)
- mapping.launch.py:
  - `rviz` → `use_rviz`
  - `save_map_path` → `output_map_path`
  - `config_path` 단일화 (`config_file` 통합; default: `<pkg>/config/slam/mid360.yaml`)
  - `rviz_cfg` → `rviz_config_path`
- localization.launch.py:
  - `rviz` → `use_rviz`
  - `config_path` 단일화 (`config_file`/`lio_config_path`/`lio_config_file` 통합)
- 헤더 docstring 표준 적용 (spec §2.5.3)

### Removed (BREAKING)
- mapping.launch.py: `qos_reliability` arg (QoS helper 적용 후 미사용 — P4)
- mapping.launch.py: `config_file` arg (config_path 통합)
- localization.launch.py: `lio_config_path`, `lio_config_file` args
- localization.launch.py: `config_file` arg

### Fixed
- mapping.launch.py: `decalre_config_file_cmd` typo (블록 제거로 해결)

### Migration
- `rviz:=true` → `use_rviz:=true`
- `save_map_path:=/path` → `output_map_path:=/path`
- `rviz_cfg:=/path` → `rviz_config_path:=/path`
- `config_file:=foo.yaml` → `config_path:=/full/path/to/foo.yaml`
- `lio_config_path:=foo lio_config_file:=bar.yaml` → `config_path:=/full/path/to/foo/bar.yaml`
- `qos_reliability:=...` → (제거; 영향 없음)

### Verification
- colcon build PASS
- ros2 launch --show-args: 신규 arg 표시

---

## [Unreleased] — Phase P4b: Topic naming + QoS (refactor)

### Changed
- `src/slam/laserMapping.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/odometry` → `/localization/fast_lio/odometry` (RELIABLE_QOS)
  - `/fast_lio/cloud_registered_body` → `/localization/fast_lio/points_body` (SENSOR_QOS)
  - `/fast_lio/debug/cloud_registered` → `/fast_lio/debug/points_world` (SENSOR_QOS)
  - `/fast_lio/debug/cloud_effected` → `/fast_lio/debug/points_effected` (SENSOR_QOS)
  - `/fast_lio/debug/map`: name unchanged, QoS → SENSOR_QOS helper
  - `/fast_lio/debug/path`: name unchanged, QoS → SENSOR_QOS helper
- `src/localization/localization_node.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry` (RELIABLE_QOS)
  - `/fast_lio/localization/confidence` → `/localization/fast_lio_loc/confidence` (RELIABLE_QOS)
- `src/localization/tf_publisher.cpp`: publisher topic renames + QoS helper adoption
  - `/fast_lio/localization/map` → `/localization/fast_lio_loc/map` (LATCHED_QOS)
  - `/fast_lio/localization/occupancy_grid` → `/localization/fast_lio_loc/occupancy_grid` (LATCHED_QOS)
- `scripts/regression_compare.py`: `--topic` default updated to `/localization/fast_lio/odometry`
- `scripts/regression_plot.py`: `read_xyz` default topic updated to `/localization/fast_lio_loc/odometry`

### Notes
- Subscribers in `localization_node.cpp` (`/fast_lio/odometry`, `/fast_lio/cloud_registered_body`) intentionally unchanged — updated in P4c (sonar_3d sync)

### Verification
- colcon build PASS (fast_lio, cartographer_slam, ~61s)
- static grep: 0 legacy publisher topic refs in source

---

## [Unreleased] — Phase P4a: QoS helper module (refactor)

### Added
- `include/fast_lio/qos.hpp` — workspace QoS 3-tier helper (SENSOR/RELIABLE/LATCHED) per spec §2.4

### Verification
- colcon build PASS

---

## [Unreleased] — Phase P3: TF naming standard (refactor)

### Changed
- `fast_lio/src/slam/preprocess.cpp`: hardcoded `frame_id = "livox"` → `"livox_link"`
- `fast_lio/src/localization/localization_node.cpp`: default parameter
  `odom_frame "camera_init"` → `"odom"`, `body_frame "body"` → `"base_link"`
- `fast_lio/config/slam/mid360.yaml`: `odom_frame: "map"` → `"odom"` (REP-105)
- `fast_lio/config/localization/localization.yaml`: `odom_frame: "odom"` already set (no change)

### Verification
- `colcon build --packages-select fast_lio --symlink-install`: PASS

---

## [Unreleased] — Phase P2: Foxglove removal (refactor)

### Removed
- `foxglove` launch arg, `foxglove_bridge` Node, IfCondition 블록 일괄 제거
  - `fast_lio/launch/mapping.launch.py`
  - `fast_lio/launch/localization.launch.py`

### Verification
- colcon build PASS
- `ros2 launch fast_lio {mapping,localization}.launch.py --show-args`에 foxglove 없음

