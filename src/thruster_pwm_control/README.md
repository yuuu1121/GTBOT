# thruster_pwm_control

T200 + Basic ESC(BESC30-R3) 4채널 PWM 스러스터 제어용 ROS2(Jazzy) 패키지.
라즈베리파이 5에서 PCA9685(I2C)로 서보 펄스를 만들어 ESC에 준다.
`thruster_control`(CAN/VESC 계열)의 PWM 대응판.

```
keyboard_node ──(std_msgs/Float32MultiArray "thruster_pwm_cmd")──▶ pwm_node ──(I2C)──▶ PCA9685 ──▶ Basic ESC ×4 ──▶ T200
```

## 노드

| 노드 | 역할 |
|------|------|
| `keyboard_node` | 키 입력 → 4채널 명령(-1.0~+1.0) 발행 (텔레옵, `/dev/tty` 직접 읽음 → launch 터미널에서 조작) |
| `pwm_node`      | `thruster_pwm_cmd` 구독 → PCA9685 채널별 펄스폭 출력 |

**명령 → 펄스폭**: `-1.0 ~ 0.0 ~ +1.0` → `us_min ~ us_mid ~ us_max` (기본 1100~1500~1900µs).
`max_cmd`(기본 **0.25**)로 클램프되어 실제 출력은 **1400~1600µs** 범위. 이 값이 모든 경로(텔레옵·PID)의 하드 리밋.
시작 시 전 채널 중립(아밍), 종료 시 중립 후 출력 off.

## 배선

- Pi 5: 핀1(3.3V)→VCC, 핀3→SDA, 핀5→SCL, 핀6→GND (I2C 버스 1, 주소 0x40)
- ESC 신호선 → PCA9685 채널 **1 / 2 / 3 / 4** (`channels` 파라미터, 신호 GND 공통 필수)
- V+ 단자는 사용 안 함 (ESC 전원은 배터리 직결)

## 파라미터 (`config/thruster_pwm_params.yaml`)

| 파라미터 | 노드 | 기본값 | 설명 |
|----------|------|--------|------|
| `i2c_bus` / `i2c_addr` | pwm | `1` / `64`(0x40) | PCA9685 위치 |
| `pwm_freq`  | pwm | `200` | 200Hz → 1틱 ≈ 1.2µs (50Hz는 4.9µs) |
| `channels`  | pwm | `[1,2,3,4]` | PCA9685 채널 (명령 벡터 순서와 동일) |
| `us_min/us_mid/us_max` | pwm | `1100/1500/1900` | 펄스폭 매핑 |
| `max_cmd`   | pwm | `0.25` | 출력 하드 리밋 (1400~1600µs) |
| `cmd`       | kbd | `0.10` | 키당 세기 (1500±40µs), `+`로 `cmd_max`(0.25)까지 |
| `cmd_step`  | kbd | `0.05` | +/- 키 증감량 |
| `directions` | kbd | `[1,-1,1,-1]` | 스러스터별 추력 부호 (쌍끼리 반대 방향 보정) |
| `release_timeout` | kbd | `0.6` | 키 입력이 이 시간(초) 끊기면 정지 (홀드 투 런) |

## 실행

```bash
cd ~/ros2_ws && source install/setup.bash

# PWM 노드 + 키보드 텔레옵 한 번에 — 시작하면 ESC 아밍음 확인, 같은 터미널에서 키 조작
ros2 launch thruster_pwm_control thruster_pwm.launch.py
```

## 키 매핑

```
w:전진 (스러스터 1·2)    s:후진
a:좌측 이동 (3·4)        d:우측 이동
1~4:스러스터 단독 (ch1/ch2/ch3/ch4, directions 미적용)
+/-:세기 증감   x:정지   z:종료 (launch 전체 종료)
※ 키를 누르고 있는 동안만 동작, 떼면 release_timeout 후 정지
```

## 참고 (캘리브레이션 실측, `~/dynamixel/CALIBRATION.md`)

- 데드존 ≈ 1475~1525µs — `cmd=0.15`(±60µs)는 시동점(정 ≈1529)을 겨우 넘는 저속
- 역추력 ≈ 정추력의 60% (같은 Δµs 기준)
- 정방향 모터 매칭은 항등(보정 불필요), 역방향만 −2~−4µs 편차
- ⚠️ T200은 수냉 설계 — 공기 중 구동은 저출력·단시간만. 첫 테스트는 프로펠러 제거.

---

## 헤딩 유지 캐스케이드 PID (`heading_hold.launch.py`)

IMU(hwt9053 + madgwick)의 yaw/자이로로 선수 방향을 붙잡는 2단 PID와
Thruster Allocation Matrix 믹서. 전후좌우 이동 중에도 헤딩이 유지된다.

```
heading_sp ─▶ [외부 헤딩 PID, 10Hz] ─▶ yaw_rate_sp ─▶ [내부 요레이트 PID, 50Hz] ─▶ N
                 yaw: imu/data_filtered                 gyro z: imu/data_raw
cmd_vel.linear.x ─▶ X   cmd_vel.linear.y ─▶ Y
[X Y N] ─▶ B(4×3) ─▶ 포화 축소 ─▶ directions 부호 ─▶ thruster_pwm_cmd ─▶ pwm_node
```

### 하드웨어 배치 가정 (접선형 +, FLU 좌표계)

```
            [ch3] 횡
              │
   ↑[ch1] ── + ── ↑[ch2]          X+ 전진, Y+ 좌측, N+ 반시계
              │
            [ch4] 횡
```

| | X(surge) | Y(sway) | N(yaw) |
|---|---|---|---|
| ch1 ↑ | +1 | 0 | +1 |
| ch2 ↑ | +1 | 0 | −1 |
| ch3 횡 | 0 | +1 | +1 |
| ch4 횡 | 0 | +1 | −1 |

(2026-10-27 실기: a/d 반대 → ch3·4 전기 + = 좌측 추력으로 수정. 정지 시 한 방향 연속 회전(양의 피드백) → yaw 열 전체 반전.
yaw 열은 'q' 키 실기 확인으로 최종 확정 필요 — 아래 "yaw 부호 검증")
행렬은 `config/heading_hold_params.yaml` 의 `surge_mix / sway_mix / yaw_mix` 열로 주어지며
부호가 실측과 다르면 yaml 만 고친다. `directions` 는 기존 텔레옵과 동일한 전기 부호.

### 노드 / 토픽 / 서비스

| 노드 | 구독 | 발행 | 서비스 |
|---|---|---|---|
| `cascade_controller_node` | `cmd_vel`(Twist), `imu/data_filtered`, `imu/data_raw` | `thruster_pwm_cmd`, `~/debug` | `~/enable_hold`(SetBool), `~/set_heading_here`, `~/calibrate_yaw_mix`(Trigger) |
| `heading_teleop_node` | — | `cmd_vel` | (위 서비스 호출) |

`~/debug` (Float32MultiArray) 순서: `yaw°, sp°, err°, rate_sp°/s, gyro°/s, N, X, Y`.
`angular.z` 는 헤딩 유지 ON 이면 목표 헤딩을 돌리는 속도, OFF 이면 개루프 yaw.

### 안전 동작

- `cmd_vel` 0.6 s 끊김 → X=Y=0, 헤딩 유지는 계속
- IMU 0.5 s 끊김 → 전 채널 0
- 발산 감지: 헤딩 오차 60° 이상이 2 s 지속되면 헤딩 유지 자동 OFF + 에러 로그 (yaw 부호 반대 보호). `t` 로 다시 ON
- 캘리브레이션 중에는 조종 입력 무시, 채널 하나만 저출력 구동
- 출력 한계 사슬: 조종 `xy_max` 0.15 / yaw `yaw_cmd_max` 0.15 → 믹서 포화 시 전 채널 동일 비율 축소 → 채널 `max_cmd` 0.25 → pwm_node `max_cmd` 0.25 (하드 리밋, 1400~1600µs)
- 시작 시 IMU 수신 후 `init_settle_sec`(3 s) 기다려 목표 헤딩 = 현재 yaw (madgwick 수렴 대기, 그 전엔 출력 0)

### 실행

```bash
# 1) 드라이런 — pwm_node 없이 명령만 확인 (모터 안 돎)
ros2 launch thruster_pwm_control heading_hold.launch.py use_pwm:=false
ros2 topic echo /thruster_pwm_cmd            # 다른 터미널
ros2 topic echo /cascade_controller/debug

# 2) 실기 — 첫 테스트는 프로펠러 제거 또는 수중에서 저출력
ros2 launch thruster_pwm_control heading_hold.launch.py
```

텔레옵 키: `w/s` 전후진, `a/d` 좌/우 이동, `q/e` 목표 헤딩 좌/우 회전, `h` 목표=현재 헤딩,
`t` 헤딩 유지 ON/OFF, `c` yaw 부호 자동 캘리브레이션, `+/-` 세기, `x`/space 정지, `z`/ESC 종료.

### yaw 부호 자동 캘리브레이션 (실기 최초 1회, 필수)

`yaw_mix` 는 모터 위치·배선·IMU 장착 방향에 다 걸려 있어 추론으로 정하지 않는다.
로봇을 **수중에 정지**시키고 텔레옵에서 `c` 를 누르면 (`~/calibrate_yaw_mix` 서비스). IMU 준비 전("초기 목표 헤딩" 로그 전)에 눌러도 준비되는 즉시 자동 시작:

1. 헤딩 유지 자동 OFF
2. ch1 → ch4 순서로 각 채널에 물리 +`yawcal_cmd`(0.06) 를 `yawcal_sec`(1.0 s) 주고, 그동안의 자이로 z 평균 부호를 기록. 채널 사이에는 |gyro| < 3°/s 가 1 s 이어질 때까지(최소 3 s, 최대 12 s) 기다려 앞 채널의 관성 회전이 사라진 뒤 측정하며, 잔류 회전이 남은 채 측정된 채널은 무효 처리(기존 부호 유지)
3. 약 20~40 s 후 `yaw_mix: [±1, ±1, ±1, ±1]` 를 로그로 출력하고 즉시 적용 → **이 줄을 yaml 에 저장**. 무효 채널이 있으면 로봇이 완전히 멈춘 뒤 `c` 재실행
4. `t` 로 헤딩 유지 ON (목표 = 현재 헤딩) 후 `q`(좌회전)/`e`(우회전) 확인

반응이 1°/s 미만인 채널은 기존 부호를 유지하고 에러 로그를 낸다 (`yawcal_cmd` 를 올리거나 배선 확인).
IMU 가 뒤집혀 달려 있어도 자이로와 yaw 가 같은 센서라 측정값이 그 부호까지 포함하므로 결과는 그대로 일관된다.

### 튠 순서

1. 외부 루프 끄고(`heading_kp: 0`) 내부만: `q/e` 로 요레이트 명령 → `gyro` 가 `rate_sp` 를 따라가게 `rate_kp` ↑, 떨리면 ↓. 정상상태 오차는 `rate_ki`.
   (10-27 실기: 0.12 cmd 1.5 s 에 100°/s 이상 도는 고응답 선체라 `rate_kp` 0.6 → 0.12, `rate_ki` 0.3 → 0.05 로 시작값을 낮춤)
2. 내부가 되면 `heading_kp` 를 올려 오버슈트 없이 복귀하게. `yaw_rate_max` 로 복귀 속도 제한.
3. `deadband: 0.0625`(±25µs) 는 기본 ON — 끄면 작은 N 이 데드존에 묻혀 헤딩이 수 초 주기로 느리게 진동한다(실기 확인). 좌우 비대칭이면 `reverse_gain: 1.6`.
