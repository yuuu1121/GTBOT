#!/bin/bash
# 되먹임 고리 시험: 편대를 교란한 뒤 복귀하는가?
#   가설: 편대가 흐트러짐 -> 판이 눕음 -> 검출 사망 -> 복귀 불가(양의 되먹임)
#   대조군 dropout_mode=uniform(입사각 무관, 종전) / 시험군 incidence(입사각 의존)
# 교란은 gtbot에 공통모드 추력(=yaw 스핀)을 8초 주입해 판을 강제로 눕히는 것이다.
# 사용: bash tools/run_feedback_probe.sh <uniform|incidence> <출력접미> [phi_zero]
# set -u 금지: ROS setup.bash가 미정의 변수를 참조해 즉시 죽는다
MODE=${1:-uniform}
TAG=${2:-r1}
PZERO=${3:-50.0}   # incidence 곡선의 100% 지점(φ). 실측 맞춤값은 65.4
SC=/tmp/claude-0/-root-home-gtbot-ws/b8adc266-2d62-49c6-bf70-1cbe16a16cea/scratchpad
WS=/root/home/gtbot_ws
OUT=$WS/results/feedback

mkdir -p $OUT
for round in 1 2 3; do
  for pat in "stonefish_ros2/stonefish" "gtbot_world" "ouster_cluster_node" "platform_perception" \
             "velocity_loop" "koopman_formation" "formation.launch" "perception.launch" \
             "leader_pilot" "simulator_gpu" "thruster_bridge" "rpm_to_sim" "imu_drift_shim" "feedback_record"; do
    for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
  done
  sleep 2
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
source /opt/ros/humble/setup.bash
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1
source $WS/install/setup.bash

rm -f /tmp/gtbot_formation_logs/koopman.csv
setsid nohup ros2 launch gtbot_formation formation.launch.py start_leader:=false \
  "dropout_mode:=$MODE" "inc_phi_zero:=$PZERO" > $SC/fb_formation_${MODE}_${TAG}.log 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 run gtbot_formation leader_pilot --ros-args \
  -p "waypoints:=[0.0, 0.0]" > $SC/fb_leader_${MODE}_${TAG}.log 2>&1 < /dev/null &
export DISPLAY=:1 XDG_RUNTIME_DIR=/tmp/xdg
setsid nohup ros2 launch gtbot_description gtbot_world.launch.py \
  > $SC/fb_sim_${MODE}_${TAG}.log 2>&1 < /dev/null &

for i in $(seq 1 30); do sleep 10
  timeout -k 5 6 ros2 topic echo /gtbot3/odometry --once >/dev/null 2>&1 && break; done
echo "loaded $(date +%T)"
for i in $(seq 1 24); do sleep 10
  grep -q "bootstrap: est" $SC/fb_formation_${MODE}_${TAG}.log && break; done
echo "boot $(date +%T)"
cd $WS
timeout -k 5 240 ros2 run gtbot_formation settle_wait || { echo "SETTLE FAIL"; exit 2; }
echo "settled $(date +%T)"

python3 $WS/tools/feedback_record.py "$OUT/fb_${MODE}_${TAG}.csv"
echo "=== FEEDBACK PROBE DONE ($MODE $TAG) $(date +%T) ==="
