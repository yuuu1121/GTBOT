#!/usr/bin/env bash
#
# fast_lio regression test
#
# 동일 baseline bag을 두 번 replay하여 trajectory가 허용 오차 내에 있는지 확인합니다.
# Phase B/C 같은 리팩토링 PR마다 머지 전 실행합니다.
#
# 사용:
#   bash scripts/regression_test.sh baseline   # archive 태그 빌드로 baseline 측정
#   bash scripts/regression_test.sh candidate  # 현재 브랜치 빌드로 candidate 측정
#   bash scripts/regression_test.sh compare    # 두 trajectory 비교 → ATE/drift 계산
#
# 결과는 /tmp/fast_lio_regression/{baseline,candidate}/odom.bag 에 저장됩니다.
# 비교는 evo (ATE)와 단순 drift 측정을 사용합니다.

set -euo pipefail

# ============================================================================
# Configuration
# ============================================================================
BAG_PATH="${BAG_PATH:-/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1}"
PLAY_DURATION="${PLAY_DURATION:-90}"   # seconds; first 90s only
ODOM_TOPIC="${ODOM_TOPIC:-/slam/fast_lio/odometry}"
LAUNCH_PKG="${LAUNCH_PKG:-fast_lio}"
LAUNCH_FILE="${LAUNCH_FILE:-mapping.launch.py}"

OUT_DIR="${OUT_DIR:-/tmp/fast_lio_regression}"

# ATE/drift 임계 (설계 문서 §5.2)
ATE_THRESHOLD_M=0.01
DRIFT_THRESHOLD_M=0.05

# ============================================================================

mode="${1:-}"

run_replay() {
    local label="$1"
    local out="${OUT_DIR}/${label}"

    rm -rf "${out}"
    mkdir -p "${out}"

    echo "[regression] launching fast_lio mapping ..."
    ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true \
        rviz:=false \
        foxglove:=false \
        > "${out}/launch.log" 2>&1 &
    LAUNCH_PID=$!
    sleep 5  # 노드 기동 대기

    echo "[regression] recording ${ODOM_TOPIC} ..."
    ros2 bag record -s sqlite3 -o "${out}/odom" "${ODOM_TOPIC}" \
        > "${out}/record.log" 2>&1 &
    RECORD_PID=$!
    sleep 1

    echo "[regression] playing baseline bag (first ${PLAY_DURATION}s) ..."
    timeout "$((PLAY_DURATION + 10))" \
        ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    sleep 5  # drain
    kill -INT "${RECORD_PID}" 2>/dev/null || true
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait 2>/dev/null || true

    echo "[regression] saved → ${out}/odom"
}

compare_results() {
    local b="${OUT_DIR}/baseline/odom"
    local c="${OUT_DIR}/candidate/odom"

    [[ -d "${b}" ]] || { echo "[regression] missing baseline at ${b}"; exit 1; }
    [[ -d "${c}" ]] || { echo "[regression] missing candidate at ${c}"; exit 1; }

    echo "[regression] computing ATE/drift ..."
    python3 "$(dirname "$0")/regression_compare.py" \
        --baseline "${b}" \
        --candidate "${c}" \
        --topic "${ODOM_TOPIC}" \
        --ate-threshold "${ATE_THRESHOLD_M}" \
        --drift-threshold "${DRIFT_THRESHOLD_M}"
}

case "${mode}" in
    baseline)  run_replay baseline ;;
    candidate) run_replay candidate ;;
    compare)   compare_results ;;
    *)
        echo "usage: $0 {baseline|candidate|compare}"
        echo ""
        echo "Typical flow:"
        echo "  git checkout archive/lidar-slam-pre-refactor-2026-05-02"
        echo "  colcon build --packages-select fast_lio"
        echo "  bash $0 baseline"
        echo ""
        echo "  git checkout refactor/phase-X-..."
        echo "  colcon build --packages-select fast_lio"
        echo "  bash $0 candidate"
        echo "  bash $0 compare"
        exit 1
        ;;
esac
