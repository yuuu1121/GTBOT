#!/usr/bin/env bash
#
# fast_lio LOCALIZATION regression test (Phase B-2 onwards)
#
# localization_node 리팩토링 PR 머지 전 실행. KIRO bag + map_v1.pcd 기반.
#
# 사용:
#   bash scripts/regression_test_localization.sh baseline   # archive 태그 빌드 측정
#   bash scripts/regression_test_localization.sh candidate  # 현재 브랜치 빌드 측정
#   bash scripts/regression_test_localization.sh compare    # ATE/drift 비교
#
# 결과: ${OUT_DIR}/{baseline,candidate}/odom_bag/

set -eo pipefail
# Note: `set -u` removed — ROS2 setup.bash and ros2 CLI internally rely on
# unbound variables and sourcing them under strict mode aborts the script.

# ============================================================================
# Configuration (env로 override 가능)
# ============================================================================
BAG_PATH="${BAG_PATH:-/workspace/data/2_kiro_watertank/20260122_KIRO/10.90.m3000d.robotX.boat.fastlio.ver1/20260122_134357}"
MAP_PATH="${MAP_PATH:-/workspace/data/2_kiro_watertank/map/fast_lio/map_v1.pcd}"
# Custom config skipping global localization with initial_pose from bag's first /slam/fast_lio/odometry.
# Necessary because FPFH+RANSAC global init is non-deterministic and unreliable on this map.
CONFIG_FILE="${CONFIG_FILE:-/tmp/regression_config/localization_init.yaml}"
PLAY_DURATION="${PLAY_DURATION:-60}"   # 초; deterministic ICP, no global loc → 빠른 수렴
ODOM_TOPIC="${ODOM_TOPIC:-/slam/fast_lio_loc/odometry}"

OUT_DIR="${OUT_DIR:-/tmp/fast_lio_localization_regression}"

# 임계 (설계 문서 §5.2)
ATE_THRESHOLD_M=0.01
DRIFT_THRESHOLD_M=0.05

# ============================================================================

mode="${1:-}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${WS_DIR:-/workspace/ros2_ws}"

run_replay() {
    local label="$1"
    local out="${OUT_DIR}/${label}"
    rm -rf "${out}"
    mkdir -p "${out}"

    [[ -f "${MAP_PATH}" ]] || { echo "[regression] map not found: ${MAP_PATH}"; exit 1; }
    [[ -d "${BAG_PATH}" ]] || { echo "[regression] bag not found: ${BAG_PATH}"; exit 1; }

    echo "[regression] [$label] launching localization with map ${MAP_PATH}"
    # shellcheck disable=SC1090,SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1090,SC1091
    source "${ws_dir}/install/setup.bash"

    # Forcefully clear any stale ros2 daemon and orphan launch processes
    # — `ros2 daemon stop` is unreliable when the daemon's topic state is corrupted.
    pkill -9 -f "ros2-daemon"          > /dev/null 2>&1 || true
    pkill -9 -f "localization_node"    > /dev/null 2>&1 || true
    pkill -9 -f "fastlio_mapping"      > /dev/null 2>&1 || true
    pkill -9 -f "robot_state_publisher" > /dev/null 2>&1 || true
    sleep 2

    # 1) start localization in background (with deterministic initial_pose config)
    ros2 launch fast_lio localization.launch.py \
        map_path:="${MAP_PATH}" \
        config_file:="${CONFIG_FILE}" \
        use_sim_time:=true \
        rviz:=false \
        foxglove:=false \
        > "${out}/launch.log" 2>&1 &
    local launch_pid=$!

    # 2) wait for localization_node ready (publisher available)
    echo "[regression] [$label] waiting for ${ODOM_TOPIC} publisher..."
    local waited=0
    local ready=0
    while [[ $waited -lt 60 ]]; do
        if ros2 topic info "${ODOM_TOPIC}" 2>&1 | grep -qE "Publisher count: [1-9]"; then
            ready=1
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done
    if [[ $ready -eq 0 ]]; then
        echo "[regression] [$label] FAIL: ${ODOM_TOPIC} not ready after 60s"
        kill -INT "${launch_pid}" 2>/dev/null || true
        tail -50 "${out}/launch.log"
        exit 1
    fi
    echo "[regression] [$label] publisher ready (${waited}s)"

    # 3) record odometry to bag
    ros2 bag record -o "${out}/odom_bag" "${ODOM_TOPIC}" > "${out}/record.log" 2>&1 &
    local record_pid=$!
    sleep 2  # let recorder subscribe

    # 4) play bag (clock + sensors)
    echo "[regression] [$label] playing bag for ${PLAY_DURATION}s..."
    timeout "${PLAY_DURATION}s" ros2 bag play "${BAG_PATH}" \
        --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    # 5) graceful shutdown
    sleep 2
    kill -INT "${record_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true
    kill -INT "${launch_pid}" 2>/dev/null || true
    sleep 1
    kill -KILL "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true

    if [[ ! -d "${out}/odom_bag" ]] || [[ -z "$(ls -A "${out}/odom_bag" 2>/dev/null)" ]]; then
        echo "[regression] [$label] FAIL: odom_bag empty"
        exit 1
    fi
    echo "[regression] [$label] done → ${out}/odom_bag"
}

case "${mode}" in
    baseline|candidate)
        run_replay "${mode}"
        ;;
    compare)
        python3 "${script_dir}/regression_compare.py" \
            --baseline "${OUT_DIR}/baseline/odom_bag" \
            --candidate "${OUT_DIR}/candidate/odom_bag" \
            --topic "${ODOM_TOPIC}" \
            --ate-threshold "${ATE_THRESHOLD_M}" \
            --drift-threshold "${DRIFT_THRESHOLD_M}"
        ;;
    *)
        echo "usage: $0 {baseline|candidate|compare}" >&2
        exit 2
        ;;
esac
