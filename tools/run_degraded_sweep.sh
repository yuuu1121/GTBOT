#!/bin/bash
# 저하 시나리오 3축 스윕 — 축당 2수준, 기준선(전부 0)은 융합 검증 런 재사용.
#   축2 상수 편의   lidar_yaw_bias  0.5° / 1.0°
#   축3 검출 누락   det_dropout     5% / 15%
#   축4 파랑        <waves height>  0.05 / 0.10 m   (environment.scn 편집 후 원복)
# 각 구성마다 S4·S5·S6를 돌리고 결과 JSON을 구성 이름으로 보존한다.
SC=/tmp/claude-0/-root-home-gtbot-ws/b8adc266-2d62-49c6-bf70-1cbe16a16cea/scratchpad
WS=/root/home/gtbot_ws
ENV_SCN=$WS/src/gtbot_description/data/common/environment.scn
OUT=$WS/results/sweep

mkdir -p $OUT
cp $ENV_SCN $SC/environment.scn.bak     # 파랑 편집 원복용 — 스크립트 종료 시 반드시 복원

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

set_waves() {   # $1 = 높이(m). 0이면 waves 요소 제거.
  python3 - "$1" "$ENV_SCN" <<'PYEOF'
import re, sys
h, p = float(sys.argv[1]), sys.argv[2]
s = open(p).read()
s = re.sub(r'\s*<waves height="[^"]*"/>', '', s)      # 기존 것 제거(멱등)
if h > 0:
    s = s.replace('<ocean>', f'<ocean>\n\t\t\t<waves height="{h}"/>')
open(p, 'w').write(s)
print(f'waves={h}')
PYEOF
}

run_config() {   # $1=이름  $2=bias(rad)  $3=dropout  $4=waves(m)
  local NAME=$1 BIAS=$2 DROP=$3 WAVE=$4
  echo "===== [$NAME] bias=$BIAS dropout=$DROP waves=$WAVE  $(date +%T) ====="
  kill_all
  set_waves "$WAVE"
  source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
  rm -f /tmp/gtbot_formation_logs/koopman.csv
  setsid nohup ros2 launch gtbot_formation formation.launch.py start_leader:=false \
    "lidar_yaw_bias:=$BIAS" "det_dropout:=$DROP" > $SC/sweep_formation_$NAME.log 2>&1 < /dev/null &
  sleep 3
  setsid nohup ros2 run gtbot_formation leader_pilot --ros-args \
    -p "waypoints:=[0.0, 0.0]" > $SC/sweep_leader_$NAME.log 2>&1 < /dev/null &
  export DISPLAY=:1 XDG_RUNTIME_DIR=/tmp/xdg
  setsid nohup ros2 launch gtbot_description gtbot_world.launch.py \
    > $SC/sweep_sim_$NAME.log 2>&1 < /dev/null &

  local ok=1
  for i in $(seq 1 30); do sleep 10
    timeout -k 5 6 ros2 topic echo /gtbot3/odometry --once >/dev/null 2>&1 && { ok=0; break; }; done
  [ $ok -ne 0 ] && { echo "[$NAME] LOAD FAIL"; return 1; }
  echo "[$NAME] loaded $(date +%T)"
  for i in $(seq 1 24); do sleep 10; grep -q "bootstrap: est" $SC/sweep_formation_$NAME.log && break; done
  echo "[$NAME] boot $(date +%T)"

  cd $WS
  python3 -c "from gtbot_formation import gate_s4; gate_s4.main()" 2>&1 | tail -3
  cp $WS/results/s4_stonefish.json $OUT/s4_$NAME.json 2>/dev/null
  python3 -c "from gtbot_formation import gate_s5; gate_s5.main()" 2>&1 | tail -3
  cp $WS/results/s5_stonefish.json $OUT/s5_$NAME.json 2>/dev/null
  if ! timeout -k 5 240 ros2 run gtbot_formation settle_wait; then
    echo "[$NAME] SETTLE FAIL — S6 생략"; return 0; fi
  python3 -c "from gtbot_formation import gate_s6; gate_s6.main()" 2>&1 | tail -3
  cp $WS/results/s6_stonefish.json $OUT/s6_$NAME.json 2>/dev/null
  echo "[$NAME] DONE $(date +%T)"
}

run_config bias0.5  0.008727 0.0 0
run_config bias1.0  0.017453 0.0 0
run_config drop05   0.0      0.05 0
run_config drop15   0.0      0.15 0
run_config wave005  0.0      0.0  0.05
run_config wave010  0.0      0.0  0.10
echo "===== SWEEP DONE $(date +%T) ====="
