# serial_cmd

CP2102 USB-UART 로 텍스트 명령(`1001`, `x` 등)을 보내는 패키지.
여러 라즈베리파이가 **같은 시각에** 명령을 실행하도록 "실행 시각 예약" 방식을 지원한다.

## 동작 원리

```
[명령 PC] broadcast_node ──ScheduledCmd(exec_time=now+1s, cmds)──▶ [Pi 1] serial_cmd_node ─▶ 시리얼
                        └───────────────────────────────────────▶ [Pi 2] serial_cmd_node ─▶ 시리얼
                        ◀──────────────── CmdAck(오차 ms) ────────┘
```

* 송신 측은 명령에 **실행 시각(epoch)** 을 찍어 보낸다 (`lead_time`, 기본 1초 뒤).
* 각 Pi 는 메시지를 받으면 자기 시스템 시계가 그 시각이 될 때까지 기다렸다가 시리얼로 쏜다.
* 모든 Pi 가 chrony 로 같은 NTP 서버에 동기화돼 있으면 네트워크 지연(WiFi 수십 ms)과 무관하게
  chrony 오차 수준(1 ms 이하)으로 동시에 실행된다.
* 실행 후 `CmdAck` 로 목표 시각 대비 오차(ms)와 **그 Pi 의 chrony 시계 오프셋**을 돌려준다.
* 송신 측은 명령마다 ack 를 모아 **"정말 동시에 실행됐는지" 를 PASS / FAIL / UNSURE 로 판정**해 찍는다
  (아래 "동시 실행 판정" 참고). 눈으로 보고 애매해 할 필요 없이 이 줄을 보면 된다.

## 패키지 구성

| 노드 | 역할 | 어디서 |
|------|------|--------|
| `serial_cmd_node` | `/serial_cmd`(String) 즉시 전송, `/serial_cmd_sched`(ScheduledCmd) 예약 전송, `/serial_cmd_ack` 발행, `/serial_rx` 수신 발행 | 각 Pi |
| `broadcast_node` | 터미널 입력을 `ScheduledCmd` 로 발행, ack 모아 동시성 판정, 시계 프로브, 1001 주기 재동기화 | 명령 PC |
| `input_node` | 터미널 입력을 `/serial_cmd`(String) 로 발행 (단일 Pi 로컬용, 예전 방식) | 단일 Pi |

메시지는 별도 패키지 [`serial_cmd_msgs`](../serial_cmd_msgs/) (`ScheduledCmd`, `CmdAck`, `ClockProbe`, `ClockProbeReply`).
**명령 PC 와 모든 Pi 에서 `serial_cmd_msgs` 와 `serial_cmd` 를 같은 버전으로 빌드해야 한다**
(메시지 정의가 다르면 ack 가 안 온다).

## 사전 조건

* 모든 컴퓨터: 같은 `ROS_DOMAIN_ID` (현재 90), `ROS_LOCALHOST_ONLY` 미설정, 같은 서브넷.
* 모든 Pi: chrony 로 같은 서버에 동기화. 확인: `chronyc tracking` 의 `System time` 이 1 ms 이내.
* 의존성: `sudo apt install python3-serial`, `dialout` 그룹.

## 실행

```bash
# 각 Pi (수신). robot_id 생략 시 hostname.
# SD 카드를 복제해서 hostname 이 모두 같으면(예: hero-desktop) 반드시 robot_id 를 다르게 준다
ros2 launch serial_cmd receiver.launch.py robot_id:=r1
ros2 launch serial_cmd receiver.launch.py           # hostname 이 Pi 마다 다를 때만

# 명령 PC (송신)
ros2 launch serial_cmd broadcast.launch.py
ros2 launch serial_cmd broadcast.launch.py lead_time:=0.3   # 유선이면 줄여도 됨

# 예전 방식: 한 Pi 에서 입력 + 전송
ros2 launch serial_cmd serial_cmd.launch.py
```

### broadcast_node 입력

| 입력 | 동작 |
|------|------|
| `1001` | 전체 Pi 에 1초 뒤 `1001` 전송. 이후 10초마다 `x` → (1초) → `1001` 재전송(resync) 자동 시작 |
| `500` | 전체 Pi 에 1초 뒤 `500` 전송 (한 번 깜빡임 등) |
| `x` | 전체 Pi 에 1초 뒤 `x` 전송, resync 자동 중지 |
| `@r1,r2 1001` | robot_id 가 r1, r2 인 Pi 만 |
| `blink` / `blink 0.5` / `@r1,r2 blink` | **1초(또는 지정 주기)마다 `100` 을 전 Pi 에 동시 예약 전송** (한 번 깜빡임 반복). `x` 또는 `blink off` 로 중지 |
| `resync on` / `resync off` | 재동기화 수동 시작/중지 (명령 전송 없음) |
| `probe` | 각 Pi 시계와 PC 시계의 차이를 왕복 프로브로 재서 표로 출력 |
| `tol 3` | 판정 허용 편차를 3 ms 로 변경 |
| `status` | 현재 설정/상태 (resync 횟수, 다음 resync 까지 남은 시간, 시계 표) |
| `q` | 종료 |

출력 예:
```
[hero-desktop-1] ['1001'] -> 전체 (수신 노드 2개), 1.00s 후 실행
  ack [hero-desktop-1] r1@192.168.10.30: OK 전송오차 +0.00ms, 시계 -0.10ms (mail.innotab.com s3)
  ack [hero-desktop-1] r2@192.168.10.23: OK 전송오차 +0.01ms, 시계 +0.32ms (192.168.10.30 s4)
=== 판정 [hero-desktop-1] ['1001']  PASS  편차 0.43ms (허용 5ms)  응답 2/2
    r1@192.168.10.30   전송 +0.00ms  시계 chrony -0.10ms (...), 왕복측정 -0.0±0.5ms  => 실제 +0.00ms
    r2@192.168.10.23   전송 +0.01ms  시계 chrony +0.32ms (...), 왕복측정 +0.4±5.1ms  => 실제 -0.31ms
    * 192.168.10.30 가 chrony 서버 (다른 Pi 는 이 시계를 따름)
=== 판정 resync #1 [rs-hero-desktop-2] ['x', '1001']  PASS  편차 0.41ms (허용 5ms)  응답 2/2
=== 판정 resync #2 [rs-hero-desktop-3] ['x', '1001']  PASS  편차 0.38ms (허용 5ms)  응답 2/2
```

### 동시 실행 판정 (PASS / FAIL / UNSURE)

Pi 가 보고하는 "전송오차 +0.00ms" 는 **그 Pi 자기 시계 기준**이라 Pi 끼리 정말 같은 순간인지는
말해주지 않는다. 그래서 ack 에 chrony 가 보는 시계 오프셋(이 Pi 시계 − 기준 서버 시계)을 함께 싣고,
송신 측에서 Pi 마다

    실제 실행 오차 = 전송오차 − 시계 오프셋

를 구해 **편차 = max − min** 을 낸다. 판정 규칙:

| 판정 | 조건 |
|------|------|
| `PASS` | 모든 수신 노드가 ack 했고, 모두 성공, 모두 chrony 동기화(같은 기준 서버), 편차 ≤ `sync_tolerance_ms`(5 ms) |
| `FAIL` | 응답 누락 / 실행 실패(늦게 도착 드롭) / 편차 초과 / 기준 서버가 서로 다름 / 시계차를 알 수 없음. `!` 줄에 이유 |
| `UNSURE` | chrony 값이 없어 왕복 프로브로만 판정했는데 그 불확실성(±왕복지연/2)이 허용치를 넘음 |

* `*` 줄은 경고. 특히 **`robot_id "…" 중복`** 이 뜨면 Pi 들의 hostname 이 같은 것이므로 `robot_id:=` 로 구분해 준다.
* 사람 눈은 20~30 ms 차이도 잘 못 느끼므로 5 ms PASS 면 "동시" 로 봐도 된다. 더 엄격히 보려면 `tol 2`.
* 왕복 프로브(`probe`)는 PC 가 ROS 메시지를 핑퐁해서 직접 재는 값이라 chrony 와 무관한 독립 검증이다.
  WiFi 에서는 ±5 ms 정도로 거칠지만, chrony 가 잘못됐을 때(수십 ms 이상 어긋남) 바로 드러난다.
  chrony 값과 프로브 값이 서로 안 맞으면 `*` 경고가 뜬다.
* `FAIL (늦게 도착 …, 드롭)` 이면 네트워크 지연이 `lead_time` 보다 큰 것이므로 `lead_time` 을 늘린다.
* 장치(펌웨어) 쪽 지연은 모든 장치가 같은 펌웨어라 동일하므로 판정에서 상쇄된다. 하드웨어 수준까지 확인하려면
  휴대폰 슬로모션(240 fps = 4 ms 분해능)으로 LED 들을 한 화면에 찍어 프레임을 세면 된다.

### blink (권장: Pi 시계로 매 깜빡임을 직접 쏘기)

`1001` 은 STM32 가 자체 타이머로 깜빡이므로 보드마다 주기가 미세하게 달라 시간이 지나면 어긋난다.
`blink` 는 그 대신 **매 깜빡임마다 `100`(한 번 깜빡임) 을 실행 시각을 찍어 전 Pi 에 보낸다.**
k 번째 깜빡임의 실행 시각은 `첫 시각 + k × period` 로 고정이라 누적 드리프트가 없고,
장치 간 편차는 Pi 시계 편차(판정의 "편차", 보통 1 ms 이하)가 그대로다.

```
blink          # 1초마다
blink 0.5      # 0.5초마다
@r1,r2 blink   # 일부만
x              # 중지
```

출력은 10회마다 요약 한 줄 (`=== blink #1~#10: PASS 10/10, 최대 편차 0.31ms`), FAIL 이 나면 그 즉시 상세.
`resync_verbose: true` 면 매회 출력. 주기 < `lead_time` 이어도 된다 (각 깜빡임을 `lead_time` 전에 미리 보냄).
`blink_cmd`(기본 `100`), `blink_period`, `blink_report_every` 파라미터로 바꾼다.

### resync (1001 위상 드리프트 보정)

장치 펌웨어가 자체 타이머로 깜빡이므로 시간이 지나면 장치 간 위상이 어긋난다.
`1001` 을 보내면 `resync_period`(10 s) 마다 `resync_cmds`(`x`, `1001`) 를 `resync_gap`(1 s) 간격으로
모든 Pi 에 동시 재전송해 다시 맞춘다. 즉 **10 초마다 전 장치가 동시에 1 초 꺼졌다가 동시에 다시 켜진다.**
이 꺼짐/켜짐이 눈에 같이 보이면 동기화가 되는 것이고, 어느 장치가 안 꺼지면 그 장치(또는 그 Pi)가 문제다.
(예전 기본값 20 ms 는 꺼짐이 안 보이고, 펌웨어가 줄을 느리게 읽으면 `x` 가 `1001` 과 붙어 무시될 수 있었다.) 재전송 시각은 최초 실행 시각 + 10 s × n 으로 고정된다.
동작 여부는 10 초마다 찍히는 `=== 판정 resync #n … PASS` 줄과 `status` 의 "지금까지 n회, 다음 …s 후" 로 확인한다
(`resync_verbose: true` 면 ack 까지 상세 출력). `x` 를 치면 resync 가 멈추고, 이미 Pi 에 가 있는 resync 가 있으면
그 뒤에 정지가 실행되도록 시각을 조정한다.

## 파라미터 (`config/serial_cmd_params.yaml`)

**serial_cmd_node**

| 이름 | 기본 | 설명 |
|------|------|------|
| `port` | CP2102 by-id 경로 | 시리얼 포트 |
| `baud` | 115200 | |
| `eol` | `lf` | 줄 끝 (`lf`/`crlf`/`cr`/`none`) |
| `cmd_topic` | `serial_cmd` | 즉시 실행 String 토픽 |
| `sched_topic` | `serial_cmd_sched` | 예약 실행 ScheduledCmd 토픽 |
| `ack_topic` | `serial_cmd_ack` | 실행 결과 CmdAck 토픽 |
| `rx_topic` | `serial_rx` | 장치 수신 줄 발행 |
| `robot_id` | `""` (hostname) | targets 매칭용 식별자 |
| `max_late_sec` | 0.5 | 실행 시각이 이만큼 이상 지나서 도착하면 드롭, 이내면 경고 후 즉시 실행 |
| `probe_topic` / `probe_reply_topic` | `serial_cmd_probe` / `…_reply` | 시계 프로브 토픽 |

**serial_cmd_broadcast**

| 이름 | 기본 | 설명 |
|------|------|------|
| `lead_time` | 1.0 | 입력 → 실행 여유(초) |
| `periodic_cmd` | `1001` | 이 명령 전송 시 resync 자동 시작 |
| `stop_cmd` | `x` | 이 명령 전송 시 resync 자동 중지 |
| `resync_enabled` | true | |
| `resync_period` | 10.0 | resync 주기(초) |
| `resync_cmds` | `[x, 1001]` | resync 때 순서대로 보낼 명령 |
| `resync_gap` | 1.0 | resync_cmds 사이 간격(초). x 뒤 이만큼 꺼졌다가 1001 |
| `resync_verbose` | false | resync/blink 의 ack/판정 상세 출력 (false: PASS 는 한 줄/요약) |
| `blink_cmd` / `blink_period` / `blink_report_every` | `100` / 1.0 / 10 | blink 모드 명령, 기본 주기(초), 요약 간격(회) |
| `sync_tolerance_ms` | 5.0 | 판정 허용 편차(ms) |
| `ack_wait` | 0.5 | 실행 시각 뒤 ack 를 기다리는 시간(초). 지나면 누락으로 판정 |
| `probe_count` / `probe_interval` / `probe_settle` | 8 / 0.05 / 0.3 | 왕복 프로브 발수, 간격, 응답 대기(초) |

## 수동 테스트 (ros2 CLI)

```bash
# 즉시 실행
ros2 topic pub --once /serial_cmd std_msgs/msg/String "{data: 'x'}"

# 예약 실행 (ros2 topic pub 은 기동에 ~1초 걸리므로 exec_time 을 3초 뒤로)
T=$(python3 -c "import time;t=time.time()+3;print(f'{{sec: {int(t)}, nanosec: {int(t%1*1e9)}}}')")
ros2 topic pub --once /serial_cmd_sched serial_cmd_msgs/msg/ScheduledCmd \
  "{exec_time: $T, cmds: ['x'], gap: 0.0, targets: [], seq_id: 'manual'}"
ros2 topic echo /serial_cmd_ack
```

ack 의 `clock_offset_ms` 는 그 Pi 에서 `chronyc tracking` 의 `System time` 값(slow 면 음수)이다.
chrony 가 없거나 Leap 이 Normal 이 아니면 `clock_synced: false` 가 되어 판정이 FAIL/UNSURE 로 떨어진다.
시계 동기화 설정 자체는 `~/GTGUI` 로 한다 (PC 가 서버, 로봇이 클라이언트).
