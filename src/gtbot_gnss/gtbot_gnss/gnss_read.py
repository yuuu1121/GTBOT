#!/usr/bin/env python3
"""
ZED-F9P (SparkFun GPS-RTK2) + Taoglas TS.125 수신 테스트
사용법:
  python3 gnss_read.py                 # 자동 포트 탐지, NMEA+UBX 파싱해서 요약 출력
  python3 gnss_read.py --raw           # 받은 문장 그대로 출력
  python3 gnss_read.py --pvt           # UBX NAV-PVT 메시지 켜서 고정밀 정보(hAcc 등) 출력
  python3 gnss_read.py --log out.log   # 원본 바이트를 파일에 저장
  python3 gnss_read.py --port /dev/ttyACM0 --baud 38400
"""
import argparse, glob, sys, time
import serial
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL, SET

FIX = {0: "NoFix", 1: "DeadReck", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "TimeOnly"}
CARR = {0: "", 1: "RTK-FLOAT", 2: "RTK-FIXED"}
NMEA_Q = {0: "NoFix", 1: "GPS", 2: "DGPS", 4: "RTK-FIXED", 5: "RTK-FLOAT", 6: "DR"}

def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("시리얼 포트를 못 찾음. 보드가 USB-C로 연결됐는지, lsusb에 u-blox가 보이는지 확인.")
    return ports[0]

def enable_pvt(ser):
    # CFG-VALSET: USB 포트로 NAV-PVT 1Hz 출력 (RAM만, 재부팅시 원복)
    msg = UBXMessage.config_set(1, 0, [("CFG_MSGOUT_UBX_NAV_PVT_USB", 1)])
    ser.write(msg.serialize())

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--raw", action="store_true"); ap.add_argument("--pvt", action="store_true")
    ap.add_argument("--log")
    a = ap.parse_args()
    port = a.port or find_port()
    print(f"[open] {port} @ {a.baud}")
    try:
        ser = serial.Serial(port, a.baud, timeout=2)
    except (PermissionError, serial.SerialException) as e:
        if "Permission denied" not in str(e): sys.exit(f"포트 열기 실패: {e}")
        sys.exit(f"권한 없음: sudo usermod -aG dialout $USER 후 재로그인 (또는 sudo chmod a+rw {port})")
    if a.pvt:
        enable_pvt(ser); print("[cfg] NAV-PVT on USB enabled")
    logf = open(a.log, "ab") if a.log else None
    ubr = UBXReader(ser, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL, quitonerror=0)
    last = 0; st = {}
    while True:
        try:
            raw, msg = ubr.read()
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("[err]", e); continue
        if raw is None:
            print("[wait] 데이터 없음... (PWR LED 켜졌는지 확인)"); continue
        if logf: logf.write(raw)
        if a.raw:
            print(raw.decode("ascii", "replace").strip() if raw[:1] == b"$" else f"<UBX {msg.identity}>"); continue
        if msg is None: continue
        i = msg.identity
        if i.endswith("GGA"):
            st.update(src="NMEA", lat=msg.lat, lon=msg.lon, alt=msg.alt, sats=msg.numSV,
                      fix=NMEA_Q.get(int(msg.quality or 0), str(msg.quality)), hdop=msg.HDOP)
        elif i == "NAV-PVT":
            st.update(src="UBX", lat=msg.lat, lon=msg.lon, alt=msg.hMSL/1000, sats=msg.numSV,
                      fix=FIX.get(msg.fixType, msg.fixType) + " " + CARR.get(msg.carrSoln, ""),
                      hacc=msg.hAcc/1000, vacc=msg.vAcc/1000,
                      utc=f"{msg.hour:02d}:{msg.min:02d}:{msg.second:02d}")
        elif i.endswith("GSV"):
            st["gsv_total"] = msg.numSV
        now = time.time()
        if st and now - last >= 1:
            last = now
            line = (f"[{st.get('src')}] fix={st.get('fix')} sats={st.get('sats')}"
                    f" lat={st.get('lat')} lon={st.get('lon')} alt={st.get('alt')}m")
            if "hacc" in st: line += f" hAcc={st['hacc']:.3f}m vAcc={st['vacc']:.3f}m utc={st['utc']}"
            if "hdop" in st: line += f" hdop={st['hdop']}"
            if "gsv_total" in st: line += f" inview={st['gsv_total']}"
            print(line)
    ser.close()

if __name__ == "__main__":
    main()
