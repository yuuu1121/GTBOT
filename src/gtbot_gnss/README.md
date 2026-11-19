# GTBot-GNSS

SparkFun GPS-RTK2 (u-blox ZED-F9P) + Taoglas TS.125.0111W 안테나로
국토지리정보원 NTRIP 보정을 받아 **cm급 RTK 측위**를 하는 스크립트 모음입니다.
Ubuntu PC와 Jetson 모두에서 동일하게 동작합니다.

| 스크립트 | 역할 |
|---|---|
| `scripts/gnss_read.py` | 보드 연결 확인용. NMEA/UBX 파싱해서 fix 상태, 위성 수, 위경도 출력 |
| `scripts/rtk_run.py` | **메인.** NTRIP 보정 수신 → 보드에 주입, 위치 CSV 저장, 실시간 지도 웹서버 |
| `scripts/plot_track.py` | 저장된 CSV를 지도(HTML)와 KML(Google Earth)로 변환 |
| `scripts/live_map.html` | `rtk_run.py`가 띄우는 실시간 지도 페이지 (수정 불필요) |
| `setup.sh` | 패키지 설치 + 시리얼 권한 + 설정 파일 생성 |

---

## 0. ROS2 패키지로 사용 (GTBot 워크스페이스)

원본 스크립트를 `gtbot_gnss` 패키지로 감쌌습니다. 명령 하나로 측위 + NTRIP 보정 + 실시간 지도(브라우저 자동 오픈) + CSV 저장.

```bash
# 최초 1회: 의존성 (NTRIP 계정은 config/gnss_params.yaml 에 들어 있음)
python3 -m pip install --user --break-system-packages pyubx2

colcon build --packages-select gtbot_gnss && source install/setup.bash
ros2 launch gtbot_gnss gnss.launch.py                  # 데스크톱이면 지도 브라우저 자동 오픈
ros2 launch gtbot_gnss gnss.launch.py open_map:=false  # SSH 등 (지도는 http://<IP>:8000)
```

| 토픽 | 타입 | 내용 |
|---|---|---|
| `gnss/fix` | `sensor_msgs/NavSatFix` | 위경도, 고도(WGS84 타원체고), 공분산 hAcc²/vAcc². RTK 적용 시 status=GBAS_FIX(2) |
| `gnss/rtk_status` | `std_msgs/String` | `RTK-FIXED` / `RTK-FLOAT` / `NONE` / `NoFix` |

파라미터는 `config/gnss_params.yaml` (포트 자동 탐지, `ntrip_*` 계정/마운트포인트, `web_port`, `save_csv`, `csv_dir`=`~/gnss_logs`).
원본 스크립트도 그대로 실행 가능: `ros2 run gtbot_gnss gnss_read --pvt`, `ros2 run gtbot_gnss rtk_run`, `ros2 run gtbot_gnss plot_track`
(`rtk_run` 은 `~/.config/gtbot_gnss/ntrip.conf` 가 있으면 그 값, 없으면 코드 기본 계정 사용. 로그는 `~/gnss_logs`)

---

## 1. 하드웨어

### 부품
- SparkFun GPS-RTK2 Breakout (ZED-F9P) 또는 GPS-RTK-SMA
- Taoglas TS.125.0111W 액티브 멀티밴드 안테나 (SMA 수, L1/L2/L5/L-Band)
- U.FL → SMA 변환 케이블 (GPS-RTK2 보드인 경우. RTK-SMA 보드는 불필요)
- USB-C 케이블
- (권장) 안테나 밑에 깔 10cm 이상 금속판. 지면 반사파를 막아 RTK FIXED가 빨라집니다.

### 연결
1. 안테나를 보드에 연결합니다. GPS-RTK2는 U.FL 커넥터에 변환 케이블을 수직으로 눌러 끼우고, 케이블을 보드 구멍에 통과시켜 고정합니다.
2. 안테나는 **하늘이 사방으로 트인 곳**에 둡니다. 실내나 건물 바로 옆에서는 RTK FIXED가 되지 않습니다.
3. USB-C로 PC와 연결합니다. PWR LED가 켜지면 정상입니다.

안테나 전원은 보드가 자동 공급합니다(TS.125 요구 전압 2.0~5.5V, 보드 바이어스 출력 범위 내). 별도 전원은 필요 없습니다.

### 참고 문서
- [SparkFun GPS-RTK2 Hookup Guide](https://learn.sparkfun.com/tutorials/gps-rtk2-hookup-guide)
- [Taoglas TS.125.0111W Datasheet](https://www.taoglas.com/product/ts-125-accura-series-multiband-gnss-antenna/)
- [u-blox ZED-F9P Integration Manual](https://www.u-blox.com/en/product/zed-f9p-module#Documentation)

---

## 2. 소프트웨어 설치

```bash
git clone https://github.com/HERO-Lab-POSTECH/-GTBot-GNSS.git
cd -GTBot-GNSS
./setup.sh
```

`setup.sh`가 하는 일:
- `requirements.txt` 설치 (pyserial, pyubx2, pynmea2, pygnssutils)
- 사용자를 `dialout` 그룹에 추가 (시리얼 포트 권한). **로그아웃 후 재로그인해야 적용됩니다.**
- `scripts/ntrip.conf.example` → `scripts/ntrip.conf` 복사

재로그인이 귀찮으면 당장은 아래로 임시 권한을 줄 수 있습니다 (보드를 뺐다 꽂으면 다시 해야 함).
```bash
sudo chmod a+rw /dev/ttyACM0
```

보드가 인식됐는지 확인:
```bash
lsusb | grep -i u-blox        # "U-Blox AG u-blox GNSS receiver"
ls /dev/ttyACM*               # /dev/ttyACM0
```

---

## 3. 보드 수신 확인 (보정 없이)

```bash
python3 scripts/gnss_read.py --pvt
```

```
[open] /dev/ttyACM0 @ 38400
[NMEA] fix=NoFix sats=0 ...
[UBX] fix=3D sats=18 lat=36.0206749 lon=129.3217563 alt=39.05m hAcc=2.280m vAcc=4.187m utc=06:59:03
```

- 콜드 스타트는 1~2분, 이후에는 수 초 안에 3D fix가 잡힙니다.
- 이 단계의 정확도(hAcc)는 1~2m 수준입니다. 정상입니다.
- `--raw`를 붙이면 NMEA 문장을 그대로 볼 수 있습니다.

---

## 4. NTRIP 보정 계정 (국토지리정보원)

RTK는 기준국의 보정 데이터(RTCM)가 있어야 cm급이 됩니다. 국토지리정보원이 무료로 제공합니다.

1. https://www.gnssdata.or.kr 회원가입 (바로 승인됨)
2. `scripts/ntrip.conf`를 열어 `user`를 **가입한 로그인 이메일**로 수정

```ini
host = www.gnssdata.or.kr
port = 2101
mountpoint = POHN-RTCM32
user = your_id@postech.ac.kr
password = gnss
```

주의할 점:
- 비밀번호는 홈페이지 비밀번호가 아니라 **고정값 `gnss`** 입니다 (공식 안내: 데이터 서비스 > 실시간 데이터(RTCM)).
- `RTS1.ngii.go.kr`, `RTS2.ngii.go.kr`는 측량업체용 네트워크 RTK 서버라 별도 신청이 필요합니다. 여기서는 쓰지 않습니다.
- 아이디 철자가 한 글자라도 틀리면 `401 Unauthorized`가 납니다.
- `mountpoint`는 `관측소코드-RTCM32` 형식입니다. 포항 캠퍼스에서는 `POHN`(포항, 약 14km)이 가장 가깝고, `YCHG`(영천), `HOMI`(호미곶)도 20km 안입니다. 다른 지역이면 아래로 전체 목록을 받아 가까운 곳을 고르세요.

```bash
curl -s http://www.gnssdata.or.kr:2101/ | grep ^STR | cut -d';' -f2,4,10,11
```

`ntrip.conf`는 `.gitignore`에 있어 저장소에 올라가지 않습니다.

---

## 5. RTK 실행 (메인)

```bash
python3 scripts/rtk_run.py
```

```
[open] /dev/ttyACM0  NTRIP www.gnssdata.or.kr:2101/POHN-RTCM32 as ...
[csv] scripts/logs/20261004_162734.csv
[web] http://localhost:8000
07:18:41 3D RTK-FLOAT sats=31 lat=36.0206771 lon=129.3217252 alt=44.80 hAcc=0.367m vAcc=0.611m | rtcm=34042B age=2s | ntrip: connected
```

한 줄에 들어 있는 정보:

| 항목 | 의미 |
|---|---|
| `3D` / `NoFix` | 위성 fix 종류 |
| `RTK-FLOAT` / `RTK-FIXED` / 공백 | 보정 적용 상태. FIXED가 돼야 cm급 |
| `hAcc` / `vAcc` | 수신기 자체 추정 수평/수직 오차 (m) |
| `rtcm=...B age=2s` | 받은 보정 누적 바이트, 마지막 수신 후 경과 초 |
| `ntrip: connected` | 보정 서버 연결 상태. `AUTH FAIL`이면 계정 확인 |

Ctrl+C로 종료합니다. 옵션:

| 옵션 | 설명 |
|---|---|
| `--csv 파일명` | 위치 CSV 경로 지정 (기본: `scripts/logs/날짜_시각.csv`) |
| `--no-csv` | CSV 저장 끄기 |
| `--web-port 8000` | 실시간 지도 포트 (기본 8000), `--no-web`으로 끄기 |
| `--log 파일명` | 수신기 원본 NMEA/UBX 바이트 저장 (디버깅용) |
| `--port /dev/ttyACM1` | 포트 직접 지정 (기본: 자동 탐지) |
| `--conf 경로` | 다른 설정 파일 사용 |

### 실시간 지도
실행 중 브라우저에서 http://localhost:8000 을 열면 1초마다 현재 위치와 경로가 그려집니다.
- 초록 = RTK FIXED, 주황 = RTK FLOAT, 파랑 = 보정 없음
- 현재 위치 주변 원 = hAcc 크기
- 같은 와이파이/핫스팟의 폰에서는 `http://<PC IP>:8000`
- 경로는 스크립트를 켠 동안 누적되고, 새로고침해도 유지됩니다

### 저장 CSV 형식
```
local_time,utc,fix,rtk,sats,lat,lon,alt_m,hAcc_m,vAcc_m,rtcm_age_s
2026-10-04 16:22:55,07:22:55,3D,RTK-FLOAT,30,36.02066480,129.32174340,44.307,0.100,0.139,1
```
위경도는 십진수 도, 고도는 해발(m). 1Hz. `hAcc_m`로 신뢰도를 걸러 쓰면 됩니다.

---

## 6. 경로 그리기 (사후)

```bash
python3 scripts/plot_track.py                      # 가장 최근 CSV → 같은 이름 .html
python3 scripts/plot_track.py logs/xxx.csv --kml   # 특정 파일 + Google Earth용 KML
python3 scripts/plot_track.py --open               # 생성 후 브라우저로 열기
```
보정 상태별 색, 시작/끝 마커, 점 클릭 시 시각/정확도 표시, 총 거리와 FIXED 비율 요약이 포함됩니다.

---

## 7. 정확도와 주기

| 상태 | 수평 정확도 |
|---|---|
| 보정 없음 | 1~2 m |
| RTK FLOAT | 0.1~0.5 m |
| RTK FIXED | 1~3 cm (사양: 1cm + 기준국 거리 1km당 0.1cm) |

- 측정 위치는 보드가 아니라 **안테나 위상 중심**(바닥면 중앙 기준 1~3mm 위)입니다. 보드와 PC 위치는 무관합니다.
- 업데이트 주기는 기본 1Hz입니다. 빠른 이동체면 `CFG-RATE-MEAS`로 최대 10~20Hz까지 올릴 수 있습니다.

### RTK FIXED가 안 될 때
FLOAT에서 FIXED로 가려면 반송파 정수 모호성을 확정해야 하는데, 신호가 약하거나 반사파가 섞이면 확정을 못 합니다.
- 실내, 창가, 건물 옆, 가로수 밑 → FLOAT에 머뭅니다. 건물에서 10~20m 떨어진 트인 곳으로 가세요.
- 처음 1~2분은 정지 상태로 두세요. 한 번 FIXED가 되면 걸으면서도 대체로 유지됩니다.
- 안테나 밑에 10cm 금속판을 깔고, 몸이나 머리가 가리지 않게 위로 띄우세요.
- `age`가 계속 커지면 인터넷(핫스팟)이 끊긴 것입니다.

---

## 8. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `시리얼 포트를 못 찾음` | USB 연결 확인, `lsusb`에 u-blox가 보이는지 확인 |
| `Permission denied: /dev/ttyACM0` | `dialout` 적용 전. 재로그인하거나 `sudo chmod a+rw /dev/ttyACM0` |
| `ntrip: AUTH FAIL 401` | 아이디 철자(이메일 전체), 비밀번호 `gnss`, 서버가 `www.gnssdata.or.kr`인지 확인 |
| `rtcm=0B` 계속 | 마운트포인트 이름 확인 (`관측소-RTCM32`), 인터넷 연결 확인 |
| `sats=0` 계속 | 안테나 연결(U.FL 눌러 끼움) 확인, 하늘이 보이는 곳으로 이동 |
| 지도 타일이 안 뜸 | 지도 타일은 인터넷에서 받아옵니다. 데이터는 정상 저장되고 있음 |
| `[wait] 데이터 없음` | 보드 PWR LED 확인, 다른 프로그램이 포트를 잡고 있는지 확인 |

---

## 9. 내부 동작 요약
- 보드는 USB CDC 시리얼로 NMEA + UBX를 내보냅니다. 스크립트는 시작 시 `CFG-VALSET`으로 `NAV-PVT`를 USB에 켭니다 (RAM만, 재부팅 시 원복).
- `rtk_run.py`는 스레드 3개로 동작합니다: 시리얼 읽기, NTRIP 클라이언트(RTCM 수신 → 시리얼로 쓰기, 10초마다 GGA 전송), 웹서버.
- ZED-F9P는 USB로 들어오는 RTCM3을 자동 인식해 RTK에 사용합니다. 별도 포트 설정이 없습니다.
- 포항 관측소 `POHN-RTCM32`는 GPS/GLONASS/Galileo/BeiDou/QZSS MSM5 + 1006 + 1230을 1Hz로 보냅니다.
