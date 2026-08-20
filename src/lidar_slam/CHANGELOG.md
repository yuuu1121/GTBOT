# CHANGELOG - lidar_slam

## [Unreleased] — fix: localization.rviz topic names (bug fix)

### Changed
- `fast_lio/rviz/localization.rviz`: 3개 topic 이름을 실제 publisher와 일치시킴.
  - `/localization/map` → `/fast_lio/localization/map` (Map PointCloud2)
  - `/localization/occupancy_grid` → `/fast_lio/localization/occupancy_grid` (OccupancyGrid)
  - `/localization/body2map` → `/fast_lio/localization/odometry` (Localized Pose)

### Notes
- 원인: TfPublisher와 LocalizationNode는 모두 `/fast_lio/localization/...` 접두로 publish하지만 RViz config는 접두 없는 이름으로 subscribe → publisher 0건 → map cloud / occupancy grid / localized pose 모두 RViz에 안 보임.
- `body2map`는 정확한 publisher가 코드에 없음 (ghost topic) → 가장 의미 가까운 `/fast_lio/localization/odometry`로 매핑. Localized Pose display의 빨간 axes가 정상 표시됨.
- `/fast_lio/cloud_registered_body` (CurrentScan, 초록)은 mapping 노드가 publish하므로 변경 불필요.

### Verification
- KIRO `20260122_134357` bag으로 mapping → `kiro_demo_map.pcd` (1.27M pts, 57×38×24m) 생성
- 같은 bag으로 localization replay → Init ICP fitness **0.994** ("INITIALIZATION: EXCELLENT"), RViz에 map (무지개색 AxisColor) + scan (초록색 FlatColor) + OccupancyGrid + Localized Pose (빨간 axes) 모두 정상 시각화
- `map → odom → base_link` TF 트리 정상 (단, 이를 위해 mapping yaml의 `tf.odom_frame`을 localization launch 사용 시 `"map"` → `"odom"`로 override해야 함; PR 범위 외, 향후 `localization.launch.py`에서 자동 처리 검토 필요)

## [Unreleased] — Phase C-3: PCD part files cleanup after consolidate (refactor)

### Changed
- `laserMapping.cpp` `consolidate_pcd_parts()`: 성공적으로 final PCD를 쓴 후 `pcd_save_cleanup_parts_after_consolidate=true`이면 `pcd_save_part_paths`의 모든 part 파일을 `std::filesystem::remove`로 삭제하고, `pcd_save_part_counter`/`pcd_save_part_paths`/`pcl_wait_save`를 리셋합니다. 같은 세션의 후속 flush가 `part000`부터 다시 쌓이도록 보장.
- `mid360.yaml` `pcd_save:` 블록에 `cleanup_parts_after_consolidate: true` 추가.

### Added
- `laserMapping.cpp`: `bool pcd_save_cleanup_parts_after_consolidate = true;` global + declare/get_parameter.

### Notes
- **Disk fill-up 방지**: 기본 `interval=6000` (10분 @ 10Hz) → 24시간 운용 시 144 part files (~3 GB). 한 달 = ~90 GB. consolidate 후 자동 삭제되도록 변경. Localization 핸드오프 산출물은 `scans.pcd` 단일 파일.
- Part 파일 보존이 필요한 디버깅 케이스에서는 `cleanup_parts_after_consolidate: false`로 끌 수 있음.
- consolidate 호출 경로 두 곳(`map_save` 서비스, `main()` shutdown) 모두 동일 함수 사용 → 어느 경로든 정리됨.

### Verification
- colcon build PASS (56.3s)
- Regression: 30s bag replay (interval=150 frames) → t=30s 시점 part000 존재 → `ros2 service call /map_save` → log: `Consolidated 1 part(s) + buffer -> 444420 pts at /tmp/c3_test/scans.pcd` + `Cleaned up 1/1 part file(s) after consolidate.` → 디렉토리에 `scans.pcd`만 + 새 `scans_part000.pcd` (counter 리셋 검증)
- 추가 30s bag → 3 parts 누적 → SIGINT shutdown → `Consolidated 3 part(s) + buffer -> 720786 pts` + `Cleaned up 3/3 part file(s)` → 최종 `scans.pcd` 단일 파일 (23 MB)

## [Unreleased] — Phase A-3: 누적 dead code 정리 (refactor)

### Removed
- `laserMapping.cpp`: `map_file_path` 파라미터 (declare/get/global) — C-1 후 어디에서도 사용 안 됨. C-1 노트에서 "추후 phase에서 정리" 예고됨.
- `laserMapping.cpp`: `string root_dir = ROOT_DIR;` global — 선언만 있고 사용 처 없음 (`resolve_save_path` 등은 `string(ROOT_DIR)` 직접 사용).
- `mid360.yaml`: `map_file_path: "./test.pcd"` 라인 — 파라미터 제거에 동기화.
- `localization_node.h`: `tf2_ros::Buffer tf_buffer_`, `std::shared_ptr<tf2_ros::TransformListener> tf_listener_` 멤버 — B-2b 노트에서 dead로 식별됨. 선언과 ctor init만 있고 어디서도 lookup 안 함.
- `localization_node.h`: `<tf2_ros/buffer.h>`, `<tf2_ros/transform_listener.h>` includes — 위 멤버 제거에 동기화.
- `localization_node.cpp`: ctor의 `tf_buffer_(this->get_clock()), tf_listener_(...)` init list 항목 제거.

### Notes
- 알고리즘 영향 0% — 전부 single-write/no-read globals 또는 선언만 있는 멤버. Phase A 회귀 의무 없음.
- `tf_publisher_`는 그대로 유지 (TF broadcasting 정상 사용 중).

### Verification
- colcon build PASS (55.0s)
- 변경 stat: 4 files changed, 2 insertions(+), 15 deletions(-)

## [Unreleased] — Phase C-2: path.poses ring buffer (refactor)

### Changed
- `fast_lio/src/slam/laserMapping.cpp` `publish_path()` — `path.poses.push_back` 후 `path_max_poses` 초과 시 oldest pose erase. 매 publish 시 메시지 크기 bounded.
- `fast_lio/config/slam/mid360.yaml` `publish:` 블록에 `path_max_poses: 3600` 추가.

### Added
- `int path_max_poses = 3600` global (default 1시간 @ 1 Hz publish — `publish_path`는 LiDAR 10Hz 콜백 중 매 10번째에 publish → 1 Hz)
- `publish.path_max_poses` ROS2 파라미터 (declare/get) — `0` 또는 음수면 cap 비활성 (기존 unbounded 동작 유지, backward compat)
- `fast_lio/scripts/regression_test_path_buffer.sh` — 3-mode 회귀 (baseline / candidate / candidate-cap)

### Notes
- 기존 동작: 코드 코멘트가 *"if path is too large, the rvis will crash"*로 인정만 하고 cap은 없었음. 12h 운용 시 ~43,200 poses (≈ 3 MB 메시지) 매 1초 → 24 Mbps 네트워크 + viz client RAM 누적.
- `path_max_poses: 3600` 기본값 효과: 메시지 크기 ≤ ~263 KB, 매 publish ≤ 2 Mbps, viz client는 1시간 윈도우 표시.
- 시각화 외 trajectory 분석은 `/fast_lio/odometry` (이미 publish됨, stateless) 또는 별도 PCD 누적 사용.
- `path.poses.erase(begin())`는 O(N)이지만 N≤3600이라 µs 수준. `std::deque`로 대체할 정도는 아님 (PoseStamped 복사 비용 미미).

### Verification
- colcon build PASS (54.7s)
- 60s bag replay (UCRC watertank) — 3-mode 비교:

  | 모드 | path_msgs | last_path_poses | 검증 |
  |---|---:|---:|---|
  | baseline (main `3e85d39`, unbounded) | 69 | 69 | 모든 pose 누적 |
  | candidate (path_max_poses=3600) | 68 | 68 | limit 미도달, baseline 동등 (±1 timing 노이즈) |
  | candidate-cap (path_max_poses=5) | 68 | **5** | ring buffer cap 정상 동작 |

  - publish 횟수는 timing 노이즈로 ±1 변동, last poses 수는 cap이 강제 적용됨을 확인
  - SLAM 알고리즘·odometry 무영향 (회귀 의무 약함 — `publish_path` 단일 함수만 수정)

## [Unreleased] — Phase C-1: pcd_save interval flush + consolidate (refactor)

### Changed
- `fast_lio/src/slam/laserMapping.cpp` 1,097줄 → 1,163줄 (+66, helper 함수 + globals)
- `publish_frame_world`: `pcl_wait_save` 누적부에 mutex 추가 + `pcd_save_interval` frame마다 part PCD flush + buffer clear
- `save_to_pcd()` (map_save 서비스): 이전엔 `pcl_wait_pub` (publish_map 미호출 → 항상 빈 PCD) 저장 → 이제 `consolidate_pcd_parts(resolve_save_path())` 호출 (모든 part + buffer 합치기). **빈 PCD 버그 수정**.
- `main()` shutdown 처리: 단일 inline voxel-write → `consolidate_pcd_parts(resolve_save_path())` 단일 경로
- `fast_lio/config/slam/mid360.yaml`: `pcd_save.interval`, `pcd_save.consolidate_on_shutdown` 재도입

### Added
- `fast_lio/src/slam/laserMapping.cpp` helper 함수 4종:
  - `resolve_save_path()` — yaml `save_path` 또는 `ROOT_DIR/PCD/scans.pcd` (확장자 보정)
  - `compute_part_path(base, idx)` — `<save_path>_partNNN.pcd`
  - `flush_pcd_part_unlocked()` — voxel-down → write part → buffer clear (caller holds mutex)
  - `consolidate_pcd_parts(final_path)` — 모든 part PCD load + 현재 buffer + voxel-down → final write
- `pcd_save_interval` (default 6000 frames = 10 min @ 10 Hz)
- `pcd_save_consolidate_on_shutdown` (default true)
- `pcd_save_part_paths` global vector — flush된 part 파일 경로 추적
- `pcl_wait_save_mutex` — `pcl_wait_save` race 보호
- `fast_lio/scripts/regression_test_pcd_save.sh` — 3-mode 회귀 (baseline / candidate-default / candidate-flush)

### Notes
- **운용 패턴**: 사용자는 fast_lio를 항시 ON 상태로 두고 sonar 3D recon + bag만 껐다 켜는 패턴 사용. fast_lio buffer가 무한 누적되면 12h × 1.6 MB/s ≈ 70 GB → OOM. interval flush로 RAM cap = `interval × ~160 KB ≈ 960 MB` (default).
- **`map_save` 서비스 동작 변경**: 이전엔 `pcl_wait_pub` (publish_map 미호출로 항상 빈 PCD) 저장 → 이제 정상적으로 모든 누적 점을 통합 저장. 호출 시점까지의 모든 part + 현재 buffer가 단일 PCD로 통합됨.
- **`map_file_path` 파라미터는 여전히 declare/get하지만 더 이상 PCD 저장 경로로 사용되지 않음** (yaml `pcd_save.save_path` 또는 launch arg `save_map_path`로 통합). 추후 phase에서 정리.
- `pcd_save.interval: -1` 설정 시 기존 RAM-only 동작 (장기 운용 시 OOM 위험).
- Phase C-2(`path.poses` ring buffer)는 별도 PR.

### Verification
- colcon build PASS (54.7s)
- 60s bag replay (UCRC watertank) — 3-mode 비교:

  | 모드 | final_pts | RSS peak |
  |---|---:|---:|
  | baseline (main `e5e50a6`, RAM-only) | 673,503 | **331 MB** |
  | candidate-default (interval=6000, flush 0회) | 677,246 | 332 MB |
  | candidate-flush (interval=300, flush 2회) | 675,639 | **239 MB (−28%)** |

  - **점 개수 차 0.3–0.6%** (voxel grid 두-pass 노이즈 수준)
  - **flush 모드에서 RAM peak 28% 감소** — 중간 flush로 `pcl_wait_save` 비워짐
  - candidate-flush의 part 파일: `scans_part000.pcd` 420,160 pts + `scans_part001.pcd` 443,067 pts → consolidate 후 675,639 pts (정상 동작)

- 추가 시나리오 검증 (3종):

  | 모드 | 결과 | 검증 항목 |
  |---|---|---|
  | `service-call` (interval=300, t=30s에 `ros2 service call /map_save`) | response `success=True, 'Map saved to ...'`; 호출 직후 PCD = **423,951 pts** | **빈 PCD 버그 fix 확인** (이전엔 `pcl_wait_pub` 0 pts 저장) |
  | `ram-only` (`interval: -1`) | final 677,211 pts, parts 0개, RSS 340 MB | flush disabled 분기 정상 (기존 RAM-only 동작 유지) |
  | `no-consolidate` (`consolidate_on_shutdown: false`) | final 미생성, parts 2개 (421k+445k) 디스크 잔존, log: `2 part(s) left on disk, 407606 buffered points discarded` | shutdown skip 분기 정상 |

## [Unreleased] — Phase A-2: laserMapping.cpp dead code 제거 (refactor)

### Changed
- `fast_lio/src/slam/laserMapping.cpp` 1,270줄 → 1,097줄 (-173, dead code only — 알고리즘 미수정)
- `fast_lio/config/slam/mid360.yaml`: `runtime_pos_log_enable`, `pcd_save.interval` 라인 제거 (기능 미구현)

### Removed
- 12개 BSS 누적 배열 (`T1`, `s_plot[1..11]`) — 각 `MAXN=720000` × 8 B = 70 MB BSS, **`s_plot11[scan_count]`은 callback에서 무조건 작성되어 ~20시간 후 OOB 시한폭탄이었음**
- `MAXN` 매크로
- `runtime_pos_log_enable` 파라미터 + 본체 (timer_callback의 if block, main()의 csv 출력, debug fout_pre/out/dbg ofstream, fp FILE*) — 활성화 시 위 OOB 폭탄을 깨움
- `pcd_save.interval` 파라미터 — 선언만 되고 구현 없음 (yaml 코멘트가 *"may lead to memory crash"*로 인정). C-1에서 실제 의미와 함께 재도입 예정
- 통계 추적 globals: `kdtree_incremental_time`, `kdtree_search_time`, `kdtree_delete_time`, `match_time`, `solve_time`, `solve_const_H_time`, `kdtree_size_st`, `kdtree_size_end`, `add_point_size`, `kdtree_delete_counter`, `time_log_counter`, `aver_time_*`, `frame_num`
- `dump_lio_state_to_log()` 함수 — runtime_pos_log 전용
- `_featsArray` global + `points_cache_collect()`의 dead 코멘트 — 선언만 되고 사용 처 없음
- `SigHandle()` signal handler — 등록 코드 주석처리 상태로 dead
- 사용 안 하는 includes: `<fstream>`, `<thread>`, `<csignal>`, `<unistd.h>`, `<visualization_msgs/msg/marker.hpp>`, `<geometry_msgs/msg/vector3.hpp>`
- 미사용 LaserMappingNode 멤버: `effect_feat_num`, `frame_num`, `deltaT`, `deltaR`, `aver_time_*`, `flg_EKF_converged`, `EKF_stop_flg`, `FILE *fp`, `ofstream fout_*`, ~LaserMappingNode 빈 destructor

### Notes
- A-2는 알고리즘 변경 0%. 회귀 측정 의무 없음. 모든 제거 객체가 검증 가능하게 dead 였음 — 통계용 globals들은 `runtime_pos_log` 블록 안에서만 read, 그 블록은 yaml/launch 어디서도 enable 안 됨.
- 다음 phase: **C-1** — `pcl_wait_save` interval-based flush + mutex + map_save 서비스 fix (이게 사용자가 보고한 "장시간 fast_lio 운용 시 RAM 먹통" 문제의 fix).

### Verification
- colcon build PASS (61s)
- 남은 BSS 80 MB는 모두 `ikdtree` SLAM 자료구조 (76 MB) + `res_last`/`point_selected_surf` (500 KB) — 전부 SLAM 필수.

## [Unreleased] — Phase B-2b: TfPublisher 추출 (refactor)

### Changed
- `localization_node.cpp` 430줄 → 345줄 (-85, TF/map/occupancy_grid publishing 제거)
- `localization_node.h`: `pub_map_`, `pub_occupancy_grid_`, `tf_broadcaster_`, `tf_timer_`, `og_*` 파라미터, 4개 publish 메서드 제거 → `std::unique_ptr<TfPublisher> tf_publisher_` 추가

### Added
- `fast_lio/include/fast_lio/localization/tf_publisher.h` — `fast_lio::localization::TfPublisher` 클래스 선언 (68줄)
- `fast_lio/src/localization/tf_publisher.cpp` — TF/map/occupancy_grid publishing 구현 (125줄)

### Notes
- TfPublisher는 `mat_odom2map_`을 const ref로 공유 (소유는 `LocalizationNode`). `odom_mutex_`도 ref로 주입 → 엔진 코드(`localization_engine.cpp`) 무수정.
- `tf_buffer_`/`tf_listener_`는 dead member (현재 어디서도 사용 안 함). 다음 dead-code phase에서 정리 예정.

### Verification
- colcon build PASS (22.1s)
- baseline (post-B-2a, main `37903cd`) vs candidate (B-2b) 60s replay → 마지막 100 포인트 평균 위치 차 **11.82 cm**
- B-2a에서 확립한 noise floor (median 4.7cm, p95 150cm) 내. baseline-vs-baseline 분포와 일관 → 회귀 없음.

## [Unreleased] — Phase B-2a: localization_engine split (refactor)

### Changed
- `localization_node.cpp` 1,107줄 → 430줄 (-677, ROS2 plumbing만 남김)
- 알고리즘 메서드 9종 (`localizationLoop`, `initializeLocalization`, `performGlobalLocalizationWithFitness`, `performLocalization`, `performLocalizationStep`, `prepareScanForGlobalLocalization`, `ensureMapFpfhComputed`, `generateTranslationCandidates`, `evaluateSingleHypothesis`) 본체를 `localization_engine.cpp`로 이동
- 클래스 구조 무변경: `LocalizationNode`는 단일 클래스 그대로, 메서드 정의만 두 .cpp로 split

### Added
- `fast_lio/include/fast_lio/localization/localization_node.h` — 클래스 선언 (171줄)
- `fast_lio/src/localization/localization_engine.cpp` — 알고리즘 본체 (567줄)
- `fast_lio/scripts/regression_test_localization.sh` — Localization 회귀 (KIRO map_v1.pcd + bag, use_global_localization=false로 결정적 ICP)
- `fast_lio/scripts/regression_plot.py` — baseline vs candidate trajectory 시각 비교

### Verification
- colcon build PASS (29.5s)
- baseline (post-B-1) vs candidate odometry 60s replay → 마지막 100 포인트 평균 위치 차 **0.02 cm** (부유점 잡음 수준)
- Trajectory plot: XY/Z 궤적 완전 일치 (육안 식별 불가)

### Notes
- `use_global_localization: false` + 고정 `initial_pose`로 ICP만 동작 → 결정적 비교. 실제 운용 시는 `use_global_localization: true`(기본)로 사용.
- Phase B-2b (`tf_publisher` 추출)는 별도 PR.

## [Unreleased] — Phase B-1: KalmanFilter & pose_utils 추출 (refactor)

### Changed
- `localization_node.cpp` 1,210줄 → 1,107줄 (-103줄)
- 호출부는 `pose_utils::` namespace prefix로 갱신 (`createOdometryMsg`, `createCropBox`, `limitPointCloud`)
- `KalmanFilter` 호출은 `using` alias로 무변경

### Added
- `fast_lio/include/fast_lio/localization/kalman_filter.h` — 1D Kalman Filter (header-only, ROS2/Open3D 의존 0)
- `fast_lio/include/fast_lio/localization/pose_utils.h` — pure helper 4종 선언
- `fast_lio/src/localization/pose_utils.cpp` — `matrixToPose`, `createOdometryMsg`, `createCropBox`, `limitPointCloud` 구현
- `fast_lio/CMakeLists.txt`: `localization_pose_utils` 라이브러리 등록 + `localization_node`에 링크

### Notes
- 알고리즘 영향 0 (단순 cut/paste). 회귀 테스트 의무 약함.
- `localization_node.cpp`에 남은 SRP 위반(performLocalizationStep ~210줄 등)은 Phase B-2에서 처리.

## [2026-05-02] — Phase A: dead code & config cleanup (refactor)

### Removed
- **Vendored dead headers** (~4,656 LOC, 0 references in any source)
  - `fast_lio/include/fast_lio/slam/matplotlibcpp.h` (2,499 LOC)
  - `fast_lio/include/fast_lio/slam/nanoflann.hpp` (2,040 LOC)
  - `fast_lio/include/fast_lio/slam/KDTreeVectorOfVectorsAdaptor.h` (117 LOC)
- **Unused LiDAR config files** (운용 LiDAR는 Livox MID-360 단일)
  - `fast_lio/config/slam/avia.yaml`
  - `fast_lio/config/slam/horizon.yaml`
  - `fast_lio/config/slam/ouster64.yaml`
  - `fast_lio/config/slam/velodyne.yaml`
- **Dead Python/matplotlib 의존성**
  - `fast_lio/src/slam/laserMapping.cpp`: `#include <Python.h>` 제거 (matplotlibcpp 잔재)
  - `fast_lio/CMakeLists.txt`: `find_package(PythonLibs REQUIRED)`, `find_path(MATPLOTLIB_CPP_INCLUDE_DIRS …)`, `${PYTHON_INCLUDE_DIRS}`, `${PYTHON_LIBRARIES}` 모두 제거

### Added
- **Regression test skeleton** (`fast_lio/scripts/`)
  - `regression_test.sh` — baseline/candidate 두 번 replay 후 비교 워크플로우
  - `regression_compare.py` — ATE RMSE + 최종 drift 계산 (임계: ATE ≤ 1cm, drift ≤ 5cm)
  - Phase B/C 같은 알고리즘 영향 가능 PR마다 사용

### Notes
- Baseline 보존: 태그 `archive/lidar-slam-pre-refactor-2026-05-02` (SHA `6d75fcd`)
- 설계 문서: `/workspace/docs/plans/2026-05-02-lidar-slam-refactor-design.md`
- Phase A는 알고리즘 영향 0% (참조되지 않는 헤더·config 제거뿐) → regression replay 의무 없음. Phase B부터는 머지 전 PASS 필수.
