#!/usr/bin/env bash
#
# Phase C-2 regression: path.poses ring buffer
#
# 검증:
#   baseline      — main HEAD (no path_max_poses, unbounded). 60s replay.
#   candidate     — C-2 default (path_max_poses=3600, 60s replay에선 limit 미도달).
#                   baseline과 path message poses 수 동일해야 함.
#   candidate-cap — C-2 with path_max_poses=5. limit 도달 → poses=5에서 cap.
#   compare       — 모든 summary 출력
#
# 측정: bag 재생 중 마지막 path 메시지의 poses.size()를 기록.
#
# 사용:
#   bash scripts/regression_test_path_buffer.sh baseline      # main 빌드 상태에서
#   bash scripts/regression_test_path_buffer.sh candidate     # C-2 빌드 상태에서
#   bash scripts/regression_test_path_buffer.sh candidate-cap
#   bash scripts/regression_test_path_buffer.sh compare

set -euo pipefail

BAG_PATH="${BAG_PATH:-/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1}"
PLAY_DURATION="${PLAY_DURATION:-60}"
OUT_DIR="${OUT_DIR:-/tmp/fast_lio_c2_regression}"
LAUNCH_PKG=fast_lio
LAUNCH_FILE=mapping.launch.py
QOS="${QOS:-best_effort}"
PATH_TOPIC="${PATH_TOPIC:-/slam/fast_lio/debug/path}"
YAML="/workspace/ros2_ws/install/fast_lio/share/fast_lio/config/slam/mid360.yaml"
YAML_BACKUP=/tmp/mid360_install_c2_backup.yaml

mode="${1:-}"

ensure_yaml_backup() { [[ -f "$YAML_BACKUP" ]] || cp "$YAML" "$YAML_BACKUP"; }
restore_yaml() { [[ -f "$YAML_BACKUP" ]] && cp "$YAML_BACKUP" "$YAML"; }

set_yaml_path_max() {
    # $1: int or "remove" to delete the line entirely (for baseline on main)
    local val="$1"
    if [[ "$val" == "remove" ]]; then
        sed -i -E '/^\s*path_max_poses:\s/d; /Cap path\.poses/d; /risk of viz crash/d' "$YAML"
    else
        if grep -qE '^\s*path_max_poses:\s' "$YAML"; then
            sed -i -E "s/^(\s*path_max_poses:\s*)-?[0-9]+(.*)/\1${val}\2/" "$YAML"
        else
            sed -i -E "/^\s*path_en:\s/a\\
            path_max_poses: ${val}" "$YAML"
        fi
    fi
}

existing_pids() { pgrep -f '/install/fast_lio/lib/fast_lio/fastlio_mapping' 2>/dev/null | sort -u || true; }

# Bag-record path topic, then read final message size with python.
run_one() {
    local label="$1"
    local out="${OUT_DIR}/${label}"
    rm -rf "$out" && mkdir -p "$out"

    echo "[c2] [${label}] yaml publish block:"
    grep -A4 "publish:" "$YAML" || true

    local pre_pids
    pre_pids=$(existing_pids | tr '\n' ' ')

    echo "[c2] [${label}] launching ..."
    ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false foxglove:=false \
        qos_reliability:="${QOS}" \
        > "${out}/launch.log" 2>&1 &
    local launch_pid=$!
    sleep 6

    local node_pid=""
    for _ in 1 2 3 4 5 6 7 8; do
        local cur
        cur=$(existing_pids)
        for p in $cur; do
            if ! echo " ${pre_pids} " | grep -qw " ${p} "; then
                node_pid="$p"; break
            fi
        done
        [[ -n "$node_pid" ]] && break
        sleep 1
    done
    echo "$node_pid" > "${out}/node_pid.txt"

    echo "[c2] [${label}] recording ${PATH_TOPIC} ..."
    ros2 bag record -s sqlite3 -o "${out}/path_bag" "${PATH_TOPIC}" \
        > "${out}/record.log" 2>&1 &
    local rec_pid=$!
    sleep 1

    echo "[c2] [${label}] playing bag (${PLAY_DURATION}s) ..."
    timeout "$((PLAY_DURATION + 10))" \
        ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    sleep 3
    kill -INT "$rec_pid" 2>/dev/null || true
    sleep 2
    [[ -n "${node_pid:-}" ]] && kill -INT "$node_pid" 2>/dev/null || true
    sleep 4
    kill -INT "$launch_pid" 2>/dev/null || true
    sleep 2
    kill -KILL "$launch_pid" 2>/dev/null || true
    pkill -KILL -P "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    sleep 1

    # Read last path message poses count from the bag
    {
        echo "=== ${label} ==="
        if [[ -d "${out}/path_bag" ]]; then
            python3 << PYEOF
import sqlite3, glob, sys
try:
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Path
except Exception as e:
    print("import error:", e); sys.exit(0)

dbs = glob.glob("${out}/path_bag/*.db3")
if not dbs:
    print("path_msgs: 0  (no db3)")
    sys.exit(0)

con = sqlite3.connect(dbs[0])
cur = con.cursor()
cur.execute("SELECT COUNT(*) FROM messages")
n_msgs = cur.fetchone()[0]
cur.execute("SELECT data FROM messages ORDER BY timestamp DESC LIMIT 1")
last = cur.fetchone()
if last is None:
    print(f"path_msgs: {n_msgs}  last_msg: NONE")
    sys.exit(0)

msg = deserialize_message(last[0], Path)
print(f"path_msgs: {n_msgs}")
print(f"last_path_poses: {len(msg.poses)}")
PYEOF
        else
            echo "path_msgs: 0  (no bag)"
        fi
    } | tee "${out}/summary.txt"
}

case "${mode}" in
    baseline)
        ensure_yaml_backup
        set_yaml_path_max remove
        trap restore_yaml EXIT
        run_one baseline
        restore_yaml
        ;;
    candidate)
        ensure_yaml_backup
        set_yaml_path_max 3600
        trap restore_yaml EXIT
        run_one candidate
        restore_yaml
        ;;
    candidate-cap)
        ensure_yaml_backup
        set_yaml_path_max 5
        trap restore_yaml EXIT
        run_one candidate-cap
        restore_yaml
        ;;
    compare)
        for label in baseline candidate candidate-cap; do
            [[ -f "${OUT_DIR}/${label}/summary.txt" ]] && cat "${OUT_DIR}/${label}/summary.txt" && echo
        done | tee "${OUT_DIR}/comparison.txt"
        ;;
    *)
        echo "usage: $0 {baseline|candidate|candidate-cap|compare}"
        exit 1
        ;;
esac
