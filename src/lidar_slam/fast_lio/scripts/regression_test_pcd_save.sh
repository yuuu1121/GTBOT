#!/usr/bin/env bash
#
# Phase C-1 regression: pcd_save flush + consolidate
#
# Phase C-1은 odometry/SLAM 동작 변경 0% (PCD save 경로만 수정).
# 따라서 trajectory 비교가 아니라 PCD 저장·서비스 동작을 확인합니다.
#
# 모드:
#   baseline           — main HEAD 빌드 + interval 라인 제거. RAM-only baseline.
#   candidate-default  — interval=6000 (60s replay → flush 0회). baseline과 동치.
#   candidate-flush    — interval=300 (60s replay → flush 2회). RAM cap 효과 측정.
#   service-call       — interval=300 + bag 30s 후 ros2 service call /map_save.
#                         빈 PCD 버그(pcl_wait_pub 저장) fix 검증.
#   ram-only           — interval=-1 (flush disabled). 기존 RAM-only 동작 유지 검증.
#   no-consolidate     — interval=300 + consolidate_on_shutdown=false.
#                         shutdown 시 part 파일만 남고 final consolidate 미실행 검증.
#   compare            — 모든 summary 출력
#
# 사용 (C-1 빌드 상태에서):
#   bash scripts/regression_test_pcd_save.sh candidate-default
#   bash scripts/regression_test_pcd_save.sh candidate-flush
#   bash scripts/regression_test_pcd_save.sh service-call
#   bash scripts/regression_test_pcd_save.sh ram-only
#   bash scripts/regression_test_pcd_save.sh no-consolidate
#   bash scripts/regression_test_pcd_save.sh compare
#
# baseline은 main HEAD 빌드 후 별도 실행 (interval 라인 자동 제거).

set -euo pipefail

BAG_PATH="${BAG_PATH:-/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1}"
PLAY_DURATION="${PLAY_DURATION:-60}"
SERVICE_CALL_AT="${SERVICE_CALL_AT:-30}"   # seconds into bag play
OUT_DIR="${OUT_DIR:-/tmp/fast_lio_c1_regression}"
LAUNCH_PKG=fast_lio
LAUNCH_FILE=mapping.launch.py
QOS="${QOS:-best_effort}"
YAML="/workspace/ros2_ws/install/fast_lio/share/fast_lio/config/slam/mid360.yaml"
YAML_BACKUP=/tmp/mid360_install_backup.yaml

mode="${1:-}"

ensure_yaml_backup() { [[ -f "$YAML_BACKUP" ]] || cp "$YAML" "$YAML_BACKUP"; }
restore_yaml() { [[ -f "$YAML_BACKUP" ]] && cp "$YAML_BACKUP" "$YAML"; }

set_yaml_interval() {
    # $1: int (frames) or "remove" to delete the line entirely (for baseline on main)
    local val="$1"
    if [[ "$val" == "remove" ]]; then
        sed -i -E '/^\s*interval:\s/d; /^\s*consolidate_on_shutdown:\s/d; /-1 disables flush/d; /On shutdown, merge/d; /Frames per part flush/d; /risk of OOM/d' "$YAML"
    else
        if grep -qE '^\s*interval:\s' "$YAML"; then
            sed -i -E "s/^(\s*interval:\s*)-?[0-9]+(.*)/\1${val}\2/" "$YAML"
        else
            sed -i -E "/^\s*filter_size:\s/a\\
            interval: ${val}\\
            consolidate_on_shutdown: true" "$YAML"
        fi
    fi
}

set_yaml_consolidate() {
    # $1: true | false
    if grep -qE '^\s*consolidate_on_shutdown:\s' "$YAML"; then
        sed -i -E "s/^(\s*consolidate_on_shutdown:\s*)(true|false)(.*)/\1$1\3/" "$YAML"
    fi
}

set_save_path() {
    sed -i -E "s|(\s*save_path:\s*)\".*\"(.*)|\1\"$1\"\2|" "$YAML"
}

# Match only the install-built executable, never stale processes from other sessions
existing_pids() { pgrep -f '/install/fast_lio/lib/fast_lio/fastlio_mapping' 2>/dev/null | sort -u || true; }

# Run a single scenario.
# Args: label, [extra_action_fn]   extra_action_fn runs in background after launch
run_one() {
    local label="$1"
    local extra_action="${2:-}"
    local out="${OUT_DIR}/${label}"
    rm -rf "$out" && mkdir -p "$out"
    local save_pcd="${out}/scans.pcd"
    set_save_path "$save_pcd"

    echo "[c1] [${label}] yaml pcd_save block:"
    grep -A6 "pcd_save:" "$YAML" || true

    local pre_pids
    pre_pids=$(existing_pids | tr '\n' ' ')

    echo "[c1] [${label}] launching (qos=${QOS}) ..."
    ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false foxglove:=false \
        qos_reliability:="${QOS}" save_map_path:="${save_pcd}" \
        > "${out}/launch.log" 2>&1 &
    local launch_pid=$!
    sleep 6

    local node_pid=""
    for _ in 1 2 3 4 5 6 7 8 9 10; do
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
    echo "[c1] [${label}] node_pid=${node_pid:-NOT_FOUND}"

    if [[ -n "$node_pid" ]]; then
        ( while kill -0 "$node_pid" 2>/dev/null; do
              ps -o rss= -p "$node_pid" 2>/dev/null || break
              sleep 1
          done ) > "${out}/rss.log" 2>&1 &
        local rss_pid=$!
    fi

    echo "[c1] [${label}] playing bag (${PLAY_DURATION}s) ..."
    timeout "$((PLAY_DURATION + 10))" \
        ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 &
    local play_pid=$!

    if [[ -n "$extra_action" ]]; then
        ( "$extra_action" "$out" ) &
        local extra_pid=$!
    fi

    wait "$play_pid" 2>/dev/null || true
    [[ -n "${extra_pid:-}" ]] && wait "$extra_pid" 2>/dev/null || true

    sleep 4
    if [[ -n "${node_pid:-}" ]]; then
        kill -INT "$node_pid" 2>/dev/null || true
    fi
    sleep 5
    # ros2 launch sometimes ignores SIGINT; force-kill after a brief grace
    kill -INT "$launch_pid" 2>/dev/null || true
    sleep 3
    kill -KILL "$launch_pid" 2>/dev/null || true
    pkill -KILL -P "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    [[ -n "${rss_pid:-}" ]] && kill "$rss_pid" 2>/dev/null || true
    sleep 2

    {
        echo "=== ${label} ==="
        echo "save_pcd: ${save_pcd}"
        if [[ -f "$save_pcd" ]]; then
            local pts
            pts=$(python3 -c "import open3d as o3d; pc=o3d.io.read_point_cloud('${save_pcd}'); print(len(pc.points))" 2>/dev/null || echo "0")
            echo "final_pts: ${pts}"
        else
            echo "final_pts: MISSING"
        fi
        echo "part_files:"
        local found=0
        for f in "${out}"/scans_part*.pcd; do
            [[ -f "$f" ]] || continue
            found=1
            local pts
            pts=$(python3 -c "import open3d as o3d; pc=o3d.io.read_point_cloud('${f}'); print(len(pc.points))" 2>/dev/null || echo "0")
            echo "  ${f}: ${pts} pts"
        done
        [[ $found -eq 0 ]] && echo "  (none)"
        echo "rss_peak_kb:"
        if [[ -s "${out}/rss.log" ]]; then sort -n "${out}/rss.log" | tail -1; else echo "  0"; fi
        # Service-call mode: also dump call result
        if [[ -f "${out}/service_call_result.txt" ]]; then
            echo "service_call_result:"
            cat "${out}/service_call_result.txt"
        fi
    } | tee "${out}/summary.txt"
}

# Background action: wait SERVICE_CALL_AT seconds into the bag, then call /map_save
service_call_action() {
    local out="$1"
    sleep "${SERVICE_CALL_AT}"
    echo "[c1] [service-call] calling /map_save at t≈${SERVICE_CALL_AT}s ..."
    {
        ros2 service call /map_save std_srvs/srv/Trigger 2>&1
    } > "${out}/service_call_result.txt" || true
    # Snapshot the PCD state immediately after service call
    cp "${out}/scans.pcd" "${out}/scans_after_service.pcd" 2>/dev/null || true
}

case "${mode}" in
    baseline)
        ensure_yaml_backup
        set_yaml_interval remove
        trap restore_yaml EXIT
        run_one baseline
        restore_yaml
        ;;
    candidate-default)
        ensure_yaml_backup
        set_yaml_interval 6000
        set_yaml_consolidate true
        trap restore_yaml EXIT
        run_one candidate-default
        restore_yaml
        ;;
    candidate-flush)
        ensure_yaml_backup
        set_yaml_interval 300
        set_yaml_consolidate true
        trap restore_yaml EXIT
        run_one candidate-flush
        restore_yaml
        ;;
    service-call)
        ensure_yaml_backup
        set_yaml_interval 300
        set_yaml_consolidate true
        trap restore_yaml EXIT
        run_one service-call service_call_action
        restore_yaml
        ;;
    ram-only)
        ensure_yaml_backup
        set_yaml_interval -1
        set_yaml_consolidate true
        trap restore_yaml EXIT
        run_one ram-only
        restore_yaml
        ;;
    no-consolidate)
        ensure_yaml_backup
        set_yaml_interval 300
        set_yaml_consolidate false
        trap restore_yaml EXIT
        run_one no-consolidate
        restore_yaml
        ;;
    compare)
        for label in baseline candidate-default candidate-flush service-call ram-only no-consolidate; do
            [[ -f "${OUT_DIR}/${label}/summary.txt" ]] && cat "${OUT_DIR}/${label}/summary.txt" && echo
        done | tee "${OUT_DIR}/comparison.txt"
        ;;
    *)
        echo "usage: $0 {baseline|candidate-default|candidate-flush|service-call|ram-only|no-consolidate|compare}"
        exit 1
        ;;
esac
