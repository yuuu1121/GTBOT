#!/bin/bash
# 복합 조건 + 구성당 3런. 새 정보가 먼저 나오도록 1단계(복합)를 앞에 둔다.
#   1단계  기준선·복합 3수준 × 3런 = 12런
#   2단계  단일축 반복 보충(1차 런은 이미 있음) = 11런
# 런마다 시드를 바꿔야 반복이 의미를 가진다(고정 시드면 같은 잡음 궤적 재생).
SC=/tmp/claude-0/-root-home-gtbot-ws/b8adc266-2d62-49c6-bf70-1cbe16a16cea/scratchpad
WS=/root/home/gtbot_ws
ENV_SCN=$WS/src/gtbot_description/data/common/environment.scn
OUT=$WS/results/sweep

mkdir -p $OUT
cp $ENV_SCN $SC/environment.scn.bak
restore_env() { cp $SC/environment.scn.bak $ENV_SCN; }
trap restore_env EXIT

kill_all() {
  for round in 1 2 3; do
    for pat in "stonefish_ros2/stonefish" "gtbot_world" "ouster_cluster_node" "platform_perception" \
               "velocity_loop" "koopman_formation" "formation.launch" "perception.launch" \
               "leader_pilot" "simulator_gpu" "thruster_bridge" "rpm_to_sim" "imu_drift_shim"; do
      for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
    done
    sleep 2
    LEFT=0
    for pat in "stonefish_ros2/stonefish" "ouster_cluster_node" "koopman_formation" "velocity_loop"; do
      C=$(pgrep -fc "$pat" 2>/dev/null); LEFT=$((LEFT + ${C:-0}))
    done
    [ "$LEFT" = "0" ] && break
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  source /opt/ros/humble/setup.bash
  ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1
}

set_waves() {
  python3 - "$1" "$ENV_SCN" <<'PYEOF'
import re, sys
h, p = float(sys.argv[1]), sys.argv[2]
s = open(p).read()
s = re.sub(r'\s*<waves height="[^"]*"/>', '', s)
if h > 0:
    s = s.replace('<ocean>', f'<ocean>\n\t\t\t<waves height="{h}"/>')
open(p, 'w').write(s)
PYEOF
}

# $1=이름 $2=bias(rad) $3=dropout $4=waves(m) $5=런번호
run_one() {
  local NAME=$1 BIAS=$2 DROP=$3 WAVE=$4 RUN=$5
  local TAG="${NAME}_r${RUN}"
  [ -f "$OUT/s6_$TAG.json" ] && { echo "[$TAG] 이미 있음 — 건너뜀"; return 0; }
  echo "===== [$TAG] bias=$BIAS drop=$DROP wave=$WAVE seed_base=$((100+10*RUN)) $(date +%T) ====="
  kill_all
  set_waves "$WAVE"
  source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
  rm -f /tmp/gtbot_formation_logs/koopman.csv
  setsid nohup ros2 launch gtbot_formation formation.launch.py start_leader:=false \
    "lidar_yaw_bias:=$BIAS" "det_dropout:=$DROP" "dropout_seed:=$RUN" \
    "seed_base:=$((100 + 10 * RUN))" > $SC/cs_formation_$TAG.log 2>&1 < /dev/null &
  sleep 3
  setsid nohup ros2 run gtbot_formation leader_pilot --ros-args \
    -p "waypoints:=[0.0, 0.0]" > $SC/cs_leader_$TAG.log 2>&1 < /dev/null &
  export DISPLAY=:1 XDG_RUNTIME_DIR=/tmp/xdg
  setsid nohup ros2 launch gtbot_description gtbot_world.launch.py \
    > $SC/cs_sim_$TAG.log 2>&1 < /dev/null &

  local ok=1
  for i in $(seq 1 30); do sleep 10
    timeout -k 5 6 ros2 topic echo /gtbot3/odometry --once >/dev/null 2>&1 && { ok=0; break; }; done
  [ $ok -ne 0 ] && { echo "[$TAG] LOAD FAIL"; return 1; }
  for i in $(seq 1 24); do sleep 10; grep -q "bootstrap: est" $SC/cs_formation_$TAG.log && break; done
  echo "[$TAG] boot $(date +%T)"

  cd $WS
  python3 -c "from gtbot_formation import gate_s4; gate_s4.main()" >/dev/null 2>&1
  cp $WS/results/s4_stonefish.json $OUT/s4_$TAG.json 2>/dev/null
  python3 -c "from gtbot_formation import gate_s5; gate_s5.main()" >/dev/null 2>&1
  cp $WS/results/s5_stonefish.json $OUT/s5_$TAG.json 2>/dev/null
  if ! timeout -k 5 240 ros2 run gtbot_formation settle_wait >/dev/null 2>&1; then
    echo "[$TAG] SETTLE FAIL — S6 생략"; return 0; fi
  python3 -c "from gtbot_formation import gate_s6; gate_s6.main()" >/dev/null 2>&1
  cp $WS/results/s6_stonefish.json $OUT/s6_$TAG.json 2>/dev/null
  echo "[$TAG] DONE $(date +%T)"
}

echo "########## 파괴점 2단계: 누락 50/60/70% x 3런 (플랫폼 드리프트 ON) ##########"
# 플랫폼 IMU가 로봇과 동일 기종으로 확정돼 심이 4개가 됐다 — 구성이 바뀌었으므로
# 1단계의 끝점(50/70%)도 이 조건에서 다시 잰다. 그래야 한 조건 안에서 경계가 닫힌다.
for R in 1 2 3; do
  run_one bp2_d50 0.0 0.50 0 $R
  run_one bp2_d60 0.0 0.60 0 $R
  run_one bp2_d70 0.0 0.70 0 $R
done
echo "########## STAGE2 DONE $(date +%T) ##########"
