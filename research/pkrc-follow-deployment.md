# PKRC 추종 시스템 — 구조, 실기 명령, sim-to-real 판단

2026-08-19. 수중 PKRC가 upward 카메라로 로봇 하부 aruco LED를 보고 상대위치를
추정하면, 플랫폼이 이를 월드좌표로 변환해 따라가고 gtbot 편대는 플랫폼을 따라간다.
vlm_ws의 blueboat↔ROV 추종(`pathfollower/rov_world_position.py`,
`blueboat_path_follower.py`) 구조를 이 편대 시스템에 이식한 것.

## 1. 구조 (시뮬 기준, 검증된 배선)

```
[PKRC 탑재분 — pkrc_sim_stack.launch.py]
  depth_controller_sim : /pkrc/pressure(Pa) → PID → /pkrc/setpoint/pwm heave(4·5ch), 수심 4 m 유지
                         부가 발행: /pkrc/depth/current(m), /pkrc/pressure_mbar(ukfm용 절대압)
  aruco_detector_6dof  : /pkrc/up/image_color → 마커 8·17·58·59 검출 → /aruco/pose_array
  ukfm_localization    : IMU+깊이+aruco 융합 → /ukfm/odom_validated (마커맵=편대 오프셋 프레임)
  (pkrc_mover          : 시뮬 전용 배회 패턴 — 실기엔 없음, PKRC는 자체 미션으로 움직임)

[플랫폼 탑재분 — pkrc_follow.launch.py]
  pkrc_world_position  : PKRC월드 = 플랫폼월드 + ukfm상대 (1 m 데드밴드 월드잠금)
                         → /pkrc/world_position, /platform/calibrated_odom
  platform_follower    : /pkrc/world_position 을 이동 목표로 leader_pilot 제어 재사용
                         (P 속도지령 + yaw0 유지 + 0.3/0.5 m 스테이션 키핑 데드밴드)
                         ⚠ leader_pilot 과 /platform/thrusters 경합 — 동시 기동 금지

[gtbot 편대] 기존 스택 그대로 — 플랫폼 LiDAR est 하행, 플랫폼을 추종하므로 자동 동행
```

시뮬에서 잡은 이식 결함 3건(원본 코드 버그 포함):
- 시뮬 pkrc heave 스러스터는 +PWM=상승(실측) — 제어기 heave 부호 반전 필요
- `rclpy.parameter.SetParametersResult`는 없는 API — 파라미터 변경 시 노드 사망(원본 버그)
- ukfm pressure 입력은 **mbar 절대압**(내부에서 1013.25 감산) — 깊이(m)를 물리면 z가 −10 m에 박힘

## 2. 실기 명령 (기계별)

멀티캐스트 디스커버리로 토픽이 이어지는 LAN 전제. **시뮬용 SHM 프로파일
(`FASTRTPS_DEFAULT_PROFILES_FILE`)이 셸에 남아 있으면 기계 간 통신이 통째로
끊긴다** — 실기 런치가 스스로 unset하지만 수동 실행 시 주의.

### 플랫폼 PC (3개 터미널)
```bash
# 1) FAST-LIO SLAM (OS0 드라이버가 내는 /ouster/points·/ouster/imu 소비 → /Odometry)
ros2 launch fast_lio mapping.launch.py config_file:=ouster64.yaml   # OS0에 맞게 파라미터 검수 필요

# 2) 지각+편대 제어 (OS0 드라이버 + 반사판 검출 + koopman + hwt9053)
ros2 launch gtbot_formation hardware_platform.launch.py sensor_hostname:=<OS0 IP>

# 3) PKRC 추종 (leader_pilot 대신 이것만)
ros2 launch pkrc_controller pkrc_follow.launch.py \
    platform_odom_topic:=/Odometry ned_convert:=true
```

### gtbot 3대 (각 로봇 PC에서 1줄)
```bash
ros2 launch gtbot_formation hardware_robot.launch.py robot:=gtbot   # gtbot2 / gtbot3
```

### PKRC PC
```bash
# 1) 깊이 유지 (MS5837 + VESC heave, 실기용 제어기)
ros2 launch pkrc_controller depth_controller.launch.py target_depth:=4.0

# 2) 마커 검출 (stellarHD 직결 — /dev/video4, LED용 노출 설정 내장)
ros2 run pkrc_controller aruco_detector_6dof \
    --ros-args -p marker_ids:="[8, 17, 58, 59]"  # + 실기 마커맵 파라미터

# 3) 위치 추정 (DVL A50 있으면 dvl_msgs 설치돼 자동 융합)
ros2 run pkrc_controller ukfm_localization --ros-args -p imu_inverted:=true  # 실물 장착 방향 확인

# (선택) 깊이 목표 키보드 조절
ros2 run pkrc_controller teleop_depth --ros-args -p initial_depth:=4.0   # 기본 1.0이라 반드시 4.0 지정
```

**통신 전제 확인 필요**: /ukfm/odom_validated가 플랫폼 PC에 닿아야 한다. 수중에서
Wi-Fi는 불가 — PKRC가 테더(이더넷)로 플랫폼/지상국과 이어져 있어야 이 구조가
성립한다. vlm_ws의 blueboat↔ROV 구성과 동일한 전제.

## 3. sim-to-real gap 판단

당장 갭을 만드는 것 (영향 큰 순):

1. **마커 장착 깊이·광학**. 현재 scn의 aruco LED는 수면 위 ~0.5 m에 있다(사용자
   배치). 실물은 선체 하부 수중 장착이라 시뮬은 공기-물 굴절 경로, 실물은 수중
   직시 — 검출 기하가 다르다. **scn 마커를 실물 장착 깊이(수면 아래)로 옮기는
   것이 갭 축소 1순위.** 지금 시뮬 검출률 0.26 Hz의 유력 원인이기도 하다.
2. **카메라 자체**. 시뮬 640×360 무왜곡 vs 실물 stellarHD 1600×1200 + plumb_bob
   왜곡 + LED용 수동 노출(exposure 1, brightness −64). 검출기는 camera_info를
   쓰므로 코드 경로는 같지만, 검출률·포즈 정확도는 실기 백에서 회귀 확인해야
   한다. 실물 백 하나로 검출기 파라미터(CLAHE, clip)를 고정할 것.
3. **4 Hz 점멸 × 노출**. 30 Hz 카메라에서 절반의 프레임은 LED off. 실물은 저노출로
   LED만 남기는 설계라 off 프레임 검출 0 — 유효 검출률 상한이 절반이다. 점멸이
   ID 식별에 필수가 아니면 상시 점등이 추종 품질에 유리.
4. **플랜트 차이**. 시뮬 heave 부호 반전(+PWM=상승)은 시뮬 제어기에만 넣었다 —
   실기 제어기는 원래 부호 그대로. PID 이득(0.8/0.35/1.2)은 시뮬 재현이 잘 됐지만
   VESC 전류 제어 플랜트는 다르므로 실기 재튜닝 필요.
5. **좌표 변환 검증**. FAST-LIO FLU→NED 변환(`ned_convert`)과 'ukfm 상대좌표를
   플랫폼 yaw로 회전하지 않는 근사'(원본 vlm 코드와 동일)는 플랫폼 yaw≈0 전제.
   실기에서 플랫폼이 크게 선회하는 운용이면 pkrc_world_position에 yaw 회전을
   넣어야 한다.
6. **헤딩 기준**. hwt9053 yaw 32°/h 드리프트(imu_run1 실측) — 편대 est 경로 문제와
   동일. 추종 자체는 상대거리 기반이라 둔감하지만, yaw0 유지형 플랫폼 제어는
   드리프트를 그대로 먹는다.
7. **DVL**. 시뮬 ukfm은 IMU+깊이+aruco만(수평 속도 무보정 → 마커 놓치면 드리프트).
   실기는 DVL A50 융합으로 더 낫다. 갭을 줄이려면 stonefish /pkrc/dvl을
   dvl_msgs/DVL로 중계하는 브리지를 만들어 시뮬에서도 융합을 켜는 것.

정리하면: **코드 경로는 시뮬·실기 동일**(토픽·노드 구조 일치)하고, 갭은 센서
물리(마커 깊이·카메라·점멸)와 플랜트 이득에 있다. 1·2·3을 먼저 하면 시뮬 검증이
실기 예측력을 갖는다.

## 4. 추종 데모 5회 반복의 판정 (2026-08-19, seed 921 계열)

| 회차 | 조치 | 결과(플랫폼-PKRC 거리) |
|:---|:---|:---|
| 1차 | 초기 배선 그대로 | 발산 26 m — 마커 수면 위 + 회전 오염 + 편류 추종 |
| 2차 | (실험) 마커 수중 이동 | 발산 16 m — PKRC 회전이 보정 오염 |
| 3차 | yaw 고정 배회 | 발산 12 m, 단 t=181 s에 1.05 m 근접 성공 |
| 4차 | aruco 신선도 게이트 | 유계 9~14 m — 발산은 멈춤 |
| 5차 | 마커 사용자 원본 복원 + 허위검출 거리 게이트 | 유계, 중앙 7.9 m / 최대 12.3 m |

수심 유지는 5회 전부 4.00 m 고정(완벽). 추종 잔여 오차의 지배 요인은 **마커
관측 품질**로 확정:

- cone_angle 10° → 마커가 PKRC 정수리 ±10°(4 m 수심에서 반경 ~0.7 m)에서만 밝음.
  gtbot 마커 3개는 시선각 ~22°라 사실상 항상 소등 상태로 보임 (up 카메라 프레임 실측)
- illuminance 5000 → 흰 셀 블룸이 번져 패턴이 뭉개짐. ID 59가 17로 오독되는 사례
  다수, 방향(tvec) 정보 상실 → ukfm 보정이 "PKRC≈마커 지도 중심"으로 수렴
- 그 결과 est ≈ 플랫폼 자신 위치가 되어 **플랫폼이 자기 마커를 쫓는 자기참조
  루프**가 형성 — 보정 편향이 누적되며 한 방향으로 서서히 이동(5차에서 +y로 8 m)
- 수면 위 마커 배치는 물-공기 굴절 + 내부전반사 반사상(거리 7~14 m 허위 검출)을
  추가로 만든다. 거리 게이트(0.2~5.5 m)로 일부만 기각 가능

이 한계는 파이프라인이 아니라 관측 기하의 문제다. 개선 경로(효과 순):
① 마커를 실물처럼 수면 아래·광각(콘각 60°+)·적정 밝기로 — 2차 실험에서 검출
175회·1 m 근접이 재현됨 ② DVL 융합(stonefish /pkrc/dvl → dvl_msgs 브리지)으로
관측 공백의 편류 제거 ③ 상시 점등(4 Hz 점멸은 유효 프레임을 절반으로 깎음).

## 5. FAST-LIO 시뮬 통합 판정 (2026-08-20, 6~8차)

| 조치 | /Odometry 대 GT |
|:---|:---|
| 바다(구조물 없음) | 발산 75 m — 스캔 정합 수평 퇴화 |
| watertank 이식(사용자 지정 pkrc_world 환경) | 유계·yaw 표류 잔차 2.7 m |
| 코너 보존 튜닝 + imu_true(드리프트 shim 우회) | 잔차 1.7~2.3 m, yaw 여전히 배회 |
| **비대칭 랜드마크 2기 추가(사용자 승인)** | **잔차 5 mm, yaw σ 0.3°** — 합격 |

정사각 수조의 90° 회전 대칭이 yaw 해를 4개로 만들어 스캔 정합이 분지 사이를
널뛰는 것이 근본 원인이었다(정지 실험으로 확정). 랜드마크가 방향을 유일화하면
즉시 해결된다. 좌표 관계: LIO 프레임은 FLU(y 반전) — `ned_convert:=true` 정확.

**단일 마커 캘리브레이션**(ID 59만, 1,709쌍): 수직 z 스케일 1.35(굴절 이론 1.33과
일치), 수평은 2×2 변환 [[0.688,−0.840],[−0.476,0.165]] — 잔차 중앙 0.24 m.
검출기 `tvec_xy_A`·`tvec_scale` 파라미터로 적용(실기는 항등·1.0).

**8차 통합**(FAST-LIO 사슬 + 보정 + 신선도 게이트): 추종기 기준 목표 거리
0.1~1.5 m, GT 기준 플랫폼-PKRC 2~5 m 유계, 수심 4.00 m 고정, 벽 충돌 없음.
남은 문제: 전체 스택 부하에서 FAST-LIO가 장기 드리프트(런 중반 ~47 m 누적) —
상대 제어는 무사하지만 절대좌표는 어긋난다. 절대 클램프는 이 드리프트와 충돌해
자기 기준 상대 반경 클램프(bound_rel 5 m)로 교체했다. 후속 후보: PKRC 전방
카메라(우리 스택 미사용, 30 Hz×2 + 60 Hz)가 만드는 렌더 부하 절감, LIO
키프레임/맵 파라미터 재조정.

## 6. 실시간 추종 달성 (2026-08-20, 10~13차)

| 회차 | 조치 | 결과 |
|:---|:---|:---|
| 10차 | 추종 속도 0.2→0.4 | LIO 점프(고속 이동 × deskew 부재) → 목표 오염 |
| 11차 | 점프 재앵커 + 속도 0.3 | 방향 반전 — 고정 A에 캘리브레이션 시점 yaw가 구워진 것 |
| 12차 | **yaw 불변 보정**(yaw 기록 재캘리브레이션 1,712쌍, 잔차 0.044 m) | 첫 60 s 0.1~0.5 m 밀착, t≈120 +y 폭주 재발 |
| 13차 | **blind 0.5→4.0 복원** + 목표 슬루 제한(0.5 m/s) | **합격** — 280 s 밀착(0.4~0.7 m), 글리치 후 회복, 방향 코사인 0.84 |

핵심 교훈 둘. ① PKRC는 yaw 제어가 없어 자유 표류(캘리브레이션 240 s 동안 53°)
— 카메라 보정은 반드시 yaw-0 기준 M + 실시간 IMU yaw 회전으로 구성해야 한다
(`tvec_xy_A` [[-0.025,-1.357],[-1.160,0.024]] + `tvec_yaw_topic`). ② FAST-LIO
blind에 1~2 m의 편대 로봇(움직이는 반사체)이 들어가면 편대 이동이 지도를 끌어
LIO가 반대 방향으로 계통 드리프트한다 — blind 4.0으로 정적 특징(벽·랜드마크)만
지도에 넣는 것이 맞다. 잔여 한계: 드문 LIO 글리치(~수 분에 1회)로 수 m 일시
이탈 후 재앵커·슬루 제한으로 회복 — 근본 해소는 점군 타임스탬프(deskew) 추가.
영상: results/video/pkrc_follow_demo_h264.mp4

## 7. 최종 합격 (2026-08-20, 14~17차) — 연속 추종 + 편대 유지 양립

| 회차 | 조치 | 결과 |
|:---|:---|:---|
| 14차 | v 0.15 + 데드밴드 0.1/0.2 축소 | 편대 est 0% 붕괴 — 잔추력 스위칭이 hull 요동 유발 |
| 15차 | 데드밴드 0.25/0.4 + 추력 저역필터 | 여전히 est 붕괴 — 원인 오판 |
| 16차 | **이분법: 추종기 odom만 GT로** | **합격** — 500 s 거리 중앙 0.40/최대 0.77 m, est 100%, 변오차 0.93 m |

est 붕괴의 진짜 범인: **FAST-LIO odometry에 twist(속도)가 비어 있다.** 추종기
제어는 `kv·(v_cmd − v)` 감쇠항이 필수인데 v가 항상 0으로 읽혀 감쇠가 죽고,
비감쇠 제어의 지속 요동이 반사판 검출을 무너뜨렸다(추종기 로그의 `v 0.000`이
증거, GT odom 전환 즉시 완치). 데드밴드·저역필터는 원인이 아니었다.

**운용 확정 구성(시뮬)**: 추종 제어 루프는 `/platform/odometry`(GT, twist 포함),
FAST-LIO는 병행 가동(관측·기록·실기 대비). **실기 전환 시 필수 작업**: FAST-LIO
`/Odometry`의 twist가 비어 있으면 추종기가 같은 이유로 편대를 부순다 —
위치 미분으로 속도를 추정하는 경로를 추종기에 추가하거나 fastlio twist 채움을
확인할 것. 영상: results/video/pkrc_follow_demo_h264.mp4 (합격 런).
