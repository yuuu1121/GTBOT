# thruster_control

8채널 ROV 스러스터 제어용 ROS2(Humble) 패키지.
키보드로 방향을 입력하면 각 스러스터의 RPM 명령을 만들어 SocketCAN으로 전달한다.

```
keyboard_node ──(std_msgs/Int32MultiArray "thruster_rpm")──▶ thruster_can_node ──(SocketCAN)──▶ ESC×8
```

## 노드

| 노드 | 역할 |
|------|------|
| `keyboard_node`     | 키 입력 → 8채널 RPM 벡터 발행 (텔레옵, 전용 터미널 필요) |
| `thruster_can_node` | `thruster_rpm` 구독 → 스러스터별 CAN 프레임 송신 |

**CAN 프레이밍**: 스러스터 `i`(0-based) → `arbitration_id = can_base_id + (i + 1)`,
`data = int32 RPM(little-endian, signed, 4바이트)`, 확장 ID.

## 파라미터 (`config/thruster_params.yaml`)

| 파라미터 | 노드 | 기본값 | 설명 |
|----------|------|--------|------|
| `can_channel`   | can  | `can0`      | SocketCAN 인터페이스 |
| `can_interface` | can  | `socketcan` | python-can 인터페이스 |
| `can_base_id`   | can  | `768`(0x300)| CAN ID 베이스 |
| `num_thrusters` | 공통 | `8`         | 스러스터 수 |
| `rpm`           | kbd  | `2000`      | 방향키당 RPM 크기 |
| `cmd_topic`     | 공통 | `thruster_rpm` | 명령 토픽 |

## 실행

```bash
# 0) CAN 인터페이스 up (한 번)
sudo ip link set can0 up type can bitrate 500000

# 1) CAN 노드 (launch)
ros2 launch thruster_control thruster_control.launch.py

# 2) 키보드 텔레옵 (별도 터미널)
ros2 run thruster_control keyboard_node --ros-args \
    --params-file $(ros2 pkg prefix thruster_control)/share/thruster_control/config/thruster_params.yaml
```

## 의존성

- `rclpy`, `std_msgs`
- `python3-can` (SocketCAN) — `sudo apt install python3-can` 또는 `pip install python-can`

## 키 매핑

```
w:전진  x:후진   a:좌평행  d:우평행
q:좌회전 e:우회전  r:상승    f:하강
s:정지  z:종료
```
