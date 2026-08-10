# HWT9053-485 IMU — ROS2 드라이버 구현 정리

Jetson Orin(ROS2 Humble)에서 WitMotion **HWT9053-485** 9축 AHRS IMU를 구동한 기록.
기존 ROS1(catkin) 패키지를 ROS2로 포팅하고, 하드웨어/드라이버/센서 결함까지 해결한 전 과정.

---

## 1. 최종 구성 (TL;DR)

| 항목 | 내용 |
|---|---|
| 센서 | WitMotion HWT9053-485 (9축 AHRS, Modbus RTU) |
| 연결 | CH340 USB-RS485 어댑터 (`1a86:7523`) |
| 포트 | `/dev/hwt9053` (udev 심볼릭 링크 → `/dev/ttyUSB2`) |
| 통신 | 9600 baud, 8N1, Modbus RTU, 장치주소 `0x50` |
| 플랫폼 | Jetson Orin, L4T R36.5, 커널 5.15.185-tegra, Ubuntu 22.04 |
| ROS | ROS2 Humble, 워크스페이스 `~/ros2_ws` |
| 패키지 | `hwt9053_driver` (ament_python) |
| 자세 추정 | **imu_filter_madgwick** (센서 내장 자세는 결함으로 미사용) |
| yaw | 자이로 상대각 (`use_mag:=false`) — 자기장 간섭 심함 |

**실행:**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch hwt9053_driver hwt9053_madgwick.launch.py
```

---

## 2. 시스템 아키텍처

```
HWT9053-485 (RS485)
      │ Modbus RTU 9600 8N1, addr 0x50
      ▼
CH340 USB-RS485  ──USB──▶  Jetson  ( /dev/hwt9053 )
      ▼
hwt9053_node (rclpy)
  ├─ /imu/data       ← 센서 내장 자세 (결함, 미사용)
  ├─ /imu/data_raw   ← 가속도 + 자이로  ┐
  ├─ /imu/mag        ← 자기장            ├─▶ imu_filter_madgwick
  └─ /imu/rpy        ← 오일러각(디버그)  ┘         │
                                                  ▼
                                    /imu/data_filtered  (올바른 orientation)
                                    TF: base_link → imu_link
                                                  ▼
                                                RViz2
```

**핵심 설계 결정**: 센서 내장 자세(Euler/쿼터니언)는 **pitch를 못 잡는 결함**이 있어 버리고,
정상인 **원시 데이터(가속도+자이로)를 madgwick로 융합**해 자세를 얻는다.

---

## 3. 셋업 과정

### 3-1. ch341 커널 드라이버 빌드 (필수)

이 Jetson 커널에는 `ch341` 모듈이 **아예 빌드돼 있지 않아** CH340 어댑터가 tty로 안 잡혔다.
(`cp210x.ko`, `ftdi_sio.ko`는 있는데 ch341만 없음 → `CONFIG_USB_SERIAL_CH341 is not set`)

```bash
# 1) 같은 커널(5.15.y) 소스에서 ch341.c만 받기
mkdir -p ~/ch341_build && cd ~/ch341_build
curl -fsSL -o ch341.c \
  https://raw.githubusercontent.com/gregkh/linux/refs/heads/linux-5.15.y/drivers/usb/serial/ch341.c

# 2) 외부 모듈로 빌드 (커널 헤더 + Module.symvers 이미 존재)
printf 'obj-m := ch341.o\n' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules
#   → vermagic: 5.15.185-tegra  (정확히 일치)

# 3) 설치 + 자동로드 등록
sudo cp ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
sudo chown root:root /lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341.ko
sudo depmod -a && sudo modprobe ch341
```
`depmod` 후 modalias가 등록되어 **재부팅 후에도 CH340 꽂으면 자동 로드**된다.

### 3-2. brltty 제거 (필수)

modprobe 직후 `ttyUSB2`가 잠깐 생겼다가 사라짐. 원인은 **brltty**(점자 단말 데몬)가
CH340을 브라유 디스플레이로 오인해 usbfs로 가로채는 우분투 고질병.

```
dmesg: usb 1-2.3.1: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
       ch341-uart ttyUSB2: ch341-uart converter now disconnected
```

```bash
sudo apt-get purge -y brltty          # ⚠ brltty-udev 같이 지정하면 apt 통째로 실패
sudo udevadm control --reload-rules
# 강제 재열거 (물리적 재연결 없이)
echo 0 | sudo tee /sys/bus/usb/devices/1-2.3.1/authorized
echo 1 | sudo tee /sys/bus/usb/devices/1-2.3.1/authorized
```

### 3-3. udev 안정 포트 이름

FTDI 어댑터가 여러 개라 `ttyUSB*` 번호가 바뀌므로 고정 이름 부여.

`/etc/udev/rules.d/99-hwt9053.rules`:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="hwt9053", MODE="0666"
```
→ `/dev/hwt9053` 생성

### 3-4. 의존 패키지

```bash
pip3 install --user pymodbus            # 3.14
sudo apt install ros-humble-imu-filter-madgwick
```

---

## 4. ROS1 → ROS2 포팅

| 항목 | ROS1 | ROS2 |
|---|---|---|
| 노드 | `rospy` | `rclpy` (Node 서브클래스) |
| 파라미터 | `rospy.get_param` | `declare_parameter` |
| 루프 | `rospy.Rate` + while | `create_timer` |
| TF | `tf.TransformBroadcaster` | `tf2_ros` + `TransformStamped` |
| launch | XML | `*.launch.py` |
| 파라미터 파일 | rosparam yaml | `<node>: ros__parameters:` 형식 |
| 빌드 | catkin | ament_python (colcon) |

**함정**: launch에서 `baudrate`/`publish_rate` 같은 int 파라미터는
`ParameterValue(LaunchConfiguration('x'), value_type=int)`로 **타입 명시 필수**
(안 하면 문자열로 넘어가 타입 에러).

### 패키지 구조
```
hwt9053_driver/
├── package.xml            # rclpy, sensor_msgs, geometry_msgs, std_srvs, tf2_ros
├── setup.py
├── config/hwt9053_params.yaml
├── launch/
│   ├── hwt9053.launch.py            # 센서 노드만
│   ├── hwt9053_with_rviz.launch.py  # 센서 + RViz (센서 내장 자세)
│   └── hwt9053_madgwick.launch.py   # ⭐ 센서 + madgwick 융합 + RViz (권장)
├── rviz/imu_view.rviz     # ROS2용으로 재작성
└── hwt9053_driver/
    ├── hwt9053_interface.py   # Modbus 통신 + 캘리브레이션
    ├── hwt9053_node.py        # ROS2 노드
    └── calibrate.py           # 대화형 캘리브레이션 도구
```

---

## 5. 겪은 문제와 해결 (핵심)

### ① pymodbus API가 버전마다 3번 바뀜
슬레이브 주소 지정 키워드가 계속 변경됨:

| pymodbus | 키워드 | import 경로 |
|---|---|---|
| 2.x | `unit=` | `pymodbus.client.sync` |
| 3.0~3.13 | `slave=` | `pymodbus.client` |
| **3.14+** | **`device_id=`** | `pymodbus.client` |

→ `read_holding_registers` 시그니처를 **introspection해서 자동 선택** (`_detect_slave_kw`).
2.x/3.x 전부 호환. (`pymodbus.payload`, `constants.Endian`은 3.x에서 제거됨)

### ② 쿼터니언 레지스터 주소가 틀림
기존 코드는 `0x40~0x43`을 쿼터니언으로 읽었으나, **수평인데도 w=0 (≈150° 회전)** 이 나옴.
레지스터 덤프 결과:
- `0x40~0x43` → 쿼터니언 아님 (**`0x43`은 온도**)
- `0x44~0x50` → 전부 0 (미사용)
- `0x51~0x54` → 실제 쿼터니언 위치이나 오일러각과 불일치

→ 쿼터니언을 레지스터에서 읽지 않고 **오일러각에서 계산**(Tait-Bryan ZYX)하도록 변경.

### ③ ⭐ 센서 내장 자세(pitch)가 고장 — 최대 난관

**실측 증거** (센서를 +X 방향으로 54° 기울인 상태):
```
가속도:   ax=-7.92  ay=+0.44  az=+5.56   (크기 9.69 ≈ g, 유효)
  → 중력 역산: 실제 X축 기울기 = 55°  ← 진짜로 크게 피치됨

센서 각도: Roll=+25.2°   Pitch=+0.0°   Yaw=-67.3°
                         ^^^^^^^^^^^^ 붙박이 0!
```

- **가속도계는 완벽** (물리와 정확히 일치)
- **센서 Pitch 레지스터(0x3E)는 항상 0.0** — 어떤 자세에서도 안 변함
- 그 움직임이 엉뚱하게 Roll + Yaw로 샘
- Roll은 정상 동작(±180°)인데 **Pitch만 망가짐**
- **캘리브레이션으로도 안 고쳐짐** → 센서 펌웨어/융합 결함

**증상**: RViz에서 살짝 기울여도 90° 돌고, 회전축이 엉뚱하게 잡힘.

**해결**: 센서 내장 자세를 버리고, **정상인 원시 데이터를 `imu_filter_madgwick`로 융합**.
노드가 이미 `/imu/data_raw`(가속도+자이로)와 `/imu/mag`를 발행하므로 그대로 연결.
→ roll/pitch가 가속도+자이로 기반이라 **정확해짐**.

### ④ 자기장 간섭으로 yaw 불안정
```
자기장 크기 = 4255 µT   (지구 자기장 25~65 µT의 약 70배)
```
로봇의 모터·CAN 배선·금속이 만드는 국소 자기장이 지구 자기장을 압도.
→ 절대 방위(북쪽)를 못 찾고, **자기장 캘리브레이션할 때마다 초기 yaw가 달라짐**
(회전 경로마다 하드/소프트 아이언 피팅 결과가 달라지고, 간섭 때문에 수렴 안 됨)

**해결**: `use_mag:=false` (기본값)
- yaw를 **자이로로만** 계산 → 켤 때 **항상 0에서 시작**, 재현성 확보, 안 튐
- 단점: 절대 북쪽 없음, 느린 드리프트 → 실내 로봇은 보통 SLAM/오도메트리로 방위 보정
- 절대 방위가 꼭 필요하면 센서를 간섭원에서 멀리 떨어뜨려 장착 후 `use_mag:=true`

---

## 6. Modbus 레지스터 맵 (실측 확인)

| 주소 | 내용 | 스케일 | 신뢰도 |
|---|---|---|---|
| `0x34~0x36` | 가속도 X/Y/Z | `raw/32768*16*9.81` m/s² | ✅ 정확 |
| `0x37~0x39` | 각속도 X/Y/Z | `raw/32768*2000` °/s | ✅ 정확 |
| `0x3A~0x3C` | 자기장 X/Y/Z | raw | ⚠️ 간섭 심함 |
| `0x3D` | Roll | `raw/32768*180` ° | ✅ 정확 |
| `0x3E` | **Pitch** | `raw/32768*180` ° | ❌ **항상 0 (고장)** |
| `0x3F` | Yaw | `raw/32768*180` ° | ⚠️ 융합 결함 영향 |
| `0x43` | 온도 | — | (쿼터니언 아님) |
| `0x51~0x54` | 쿼터니언 | `raw/32768` | ❌ 오일러와 불일치 |

### 캘리브레이션 레지스터
출처: [WITMOTION 공식 SDK](https://github.com/WITMOTION/WitStandardModbus_WT901C485) (`REG.h`, `wit_c_sdk.c`)

| 단계 | 레지스터 | 값 |
|---|---|---|
| 언락 | `KEY` = `0x69` | `0xB588` |
| 캘리브레이션 모드 | `CALSW` = `0x01` | `0x00` NORMAL(종료)<br>`0x01` 가속도/자이로<br>`0x04` yaw 리셋<br>`0x07` 자기장 |
| 저장 | `SAVE` = `0x00` | `0x0000` |

순서: **언락 → CALSW 설정 → (대기/회전) → CALSW=NORMAL → 저장**

---

## 7. 사용법

### 실행
```bash
cd ~/ros2_ws && source install/setup.bash

# ⭐ 권장: madgwick 융합 + RViz (자기장 미사용, yaw 시작 0)
ros2 launch hwt9053_driver hwt9053_madgwick.launch.py

# 절대 방위가 필요할 때만
ros2 launch hwt9053_driver hwt9053_madgwick.launch.py use_mag:=true

# 센서 노드만
ros2 launch hwt9053_driver hwt9053.launch.py
```

### 토픽
| 토픽 | 타입 | 내용 |
|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | 가속도 + 자이로 (orientation 없음) |
| `/imu/mag` | `sensor_msgs/MagneticField` | 자기장 |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | 오일러각 (디버그용) |
| `/imu/data` | `sensor_msgs/Imu` | 센서 내장 자세 — **결함, 사용 금지** |
| **`/imu/data_filtered`** | `sensor_msgs/Imu` | **madgwick 융합 자세 — 이걸 사용** |
| `/tf` | — | `base_link` → `imu_link` (madgwick 발행) |

### 캘리브레이션
```bash
pkill -f hwt9053_node          # 포트 단독 점유 필요
ros2 run hwt9053_driver calibrate            # 대화형 메뉴
ros2 run hwt9053_driver calibrate acc        # 가속도만 (수평·정지)
ros2 run hwt9053_driver calibrate mag        # 자기장만 (전 축 360° 회전)
```

노드를 켠 채로 서비스 호출도 가능:
```bash
ros2 service call /hwt9053_node/calibrate_acc        std_srvs/srv/Trigger
ros2 service call /hwt9053_node/calibrate_mag_start  std_srvs/srv/Trigger
ros2 service call /hwt9053_node/calibrate_mag_stop   std_srvs/srv/Trigger
ros2 service call /hwt9053_node/reset_yaw            std_srvs/srv/Trigger
```

---

## 8. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `/dev/ttyUSB` 안 생김 | ch341 드라이버 없음 | §3-1 모듈 빌드 |
| ttyUSB 생겼다 바로 사라짐 | brltty가 가로챔 | `sudo apt purge brltty` |
| `Could not exclusively lock port` | 노드 중복 실행 | `pkill -f hwt9053_node` (한 번에 하나만) |
| `unexpected keyword 'slave'` | pymodbus 3.14는 `device_id=` | introspection 자동 선택 (해결됨) |
| RViz에서 살짝 기울여도 크게 돔 | 센서 내장 자세 결함 | madgwick 런치 사용 |
| 켤 때마다 초기 yaw 다름 | 자기장 간섭(4255µT) | `use_mag:=false` (기본) |
| RViz에 아무것도 안 나옴 | `.rviz`가 ROS1 형식 | ROS2 클래스(`rviz_default_plugins/*`)로 재작성 (해결됨) |

**RViz 축 색상**: 🔴 X, 🟢 Y, 🔵 Z (RGB=XYZ)

---

## 9. 성능 / 한계

- 퍼블리시 속도: **~17~20 Hz** (9600 baud에서 16레지스터 왕복의 한계. `publish_rate:=50`을 줘도 그 이하)
  → 더 빠르게 하려면 센서 보드레이트를 올려야 함
- 절대 방위(자기 북쪽) 사용 불가 — 자기장 간섭 환경
- 센서 내장 AHRS 미사용 (pitch 결함)
