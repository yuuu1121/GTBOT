#!/usr/bin/env python3
"""
ZED-F9P + NGII NTRIP RTK 실행기
  python3 rtk_run.py                # ntrip.conf 읽어서 보정 수신 + 상태 출력
  python3 rtk_run.py --log rtk.log  # 수신기 원본 바이트(NMEA/UBX) 저장
  python3 rtk_run.py --csv out.csv  # 위치 CSV 저장 경로 지정 (기본: ~/gnss_logs/YYYYMMDD_HHMMSS.csv)
  python3 rtk_run.py --no-csv       # CSV 저장 끄기
  python3 rtk_run.py --web-port 8000  # 실시간 지도 포트 (기본 8000, --no-web 으로 끄기)
실행 중 브라우저에서 http://localhost:8000 열면 경로가 실시간으로 그려짐. Ctrl+C로 종료.
"""
import argparse, base64, glob, json, os, socket, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import serial
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL

FIX = {0: "NoFix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}
CARR = {0: "", 1: "RTK-FLOAT", 2: "RTK-FIXED"}
HERE = os.path.dirname(os.path.abspath(__file__))
CONF_DEFAULT = os.path.expanduser("~/.config/gtbot_gnss/ntrip.conf")
LOG_DIR = os.path.expanduser("~/gnss_logs")
# 기본 NTRIP 계정 (국토지리정보원). 설정 파일(--conf)이 있으면 그 값이 우선
NTRIP_DEFAULT = dict(host="www.gnssdata.or.kr", port="2101", mountpoint="POHN-RTCM32",
                     user="kwbnoa123@postech.ac.kr", password="gnss")

def load_conf(path):
    c = {}
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line: continue
        k, v = line.split("=", 1); c[k.strip()] = v.strip()
    return c

class State:
    def __init__(s):
        s.lock = threading.Lock(); s.gga = None; s.pvt = {}; s.rtcm_bytes = 0
        s.rtcm_last = 0; s.ntrip = "connecting"; s.stop = False; s.points = []

def serial_reader(ser, st, logf):
    ubr = UBXReader(ser, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL, quitonerror=0)
    while not st.stop:
        try:
            raw, msg = ubr.read()
        except Exception as e:
            continue
        if raw is None or msg is None: continue
        if logf: logf.write(raw)
        i = msg.identity
        if i.endswith("GGA") and msg.lat:
            with st.lock: st.gga = raw
        elif i == "NAV-PVT":
            with st.lock:
                st.pvt = dict(fix=FIX.get(msg.fixType, msg.fixType), carr=CARR.get(msg.carrSoln, ""),
                              sats=msg.numSV, lat=msg.lat, lon=msg.lon, alt=msg.hMSL / 1000,
                              hacc=msg.hAcc / 1000, vacc=msg.vAcc / 1000,
                              utc=f"{msg.hour:02d}:{msg.min:02d}:{msg.second:02d}")

def ntrip_client(conf, ser, st):
    auth = base64.b64encode(f"{conf['user']}:{conf['password']}".encode()).decode()
    while not st.stop:
        try:
            s = socket.create_connection((conf["host"], int(conf["port"])), timeout=10)
            s.sendall((f"GET /{conf['mountpoint']} HTTP/1.0\r\nUser-Agent: NTRIP pyubx2\r\n"
                       f"Authorization: Basic {auth}\r\n\r\n").encode())
            hdr = s.recv(1024)
            if b"200" not in hdr.split(b"\r\n")[0]:
                st.ntrip = "AUTH FAIL " + hdr.split(b"\r\n")[0].decode(errors="replace")
                s.close(); time.sleep(10); continue
            st.ntrip = "connected"
            last_gga = 0; s.settimeout(5)
            while not st.stop:
                now = time.time()
                if now - last_gga >= 10:            # VRS는 내 위치(GGA)를 주기적으로 보내야 함
                    with st.lock: g = st.gga
                    if g:
                        s.sendall(g if g.endswith(b"\r\n") else g + b"\r\n"); last_gga = now
                        st.ntrip = "connected"
                    elif last_gga == 0:
                        st.ntrip = "connected (GGA 대기: 수신기 fix 필요)"
                try:
                    data = s.recv(4096)
                except socket.timeout:
                    continue
                if not data: raise ConnectionError("caster closed")
                ser.write(data)
                with st.lock:
                    st.rtcm_bytes += len(data); st.rtcm_last = now
        except Exception as e:
            st.ntrip = f"reconnect ({e})"
            time.sleep(5)

def start_web(st, port):
    page = open(os.path.join(HERE, "live_map.html"), "rb").read()
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/points.json":
                frm = int(parse_qs(u.query).get("from", ["0"])[0])
                with st.lock:
                    pts = st.points[frm:]
                    age = round(time.time() - st.rtcm_last) if st.rtcm_last else None
                body = json.dumps(dict(points=pts, ntrip=st.ntrip, rtcm_age=age)).encode()
                ctype = "application/json"
            else:
                body, ctype = page, "text/html; charset=utf-8"
            self.send_response(200); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(body)
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[web] http://localhost:{port}  (같은 와이파이의 폰에서는 http://<이 PC IP>:{port})")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", default=CONF_DEFAULT)
    ap.add_argument("--port"); ap.add_argument("--log")
    ap.add_argument("--csv"); ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--web-port", type=int, default=8000); ap.add_argument("--no-web", action="store_true")
    a = ap.parse_args()
    conf = load_conf(a.conf) if os.path.exists(a.conf) else dict(NTRIP_DEFAULT)
    port = a.port or (sorted(glob.glob("/dev/ttyACM*")) or [None])[0]
    if not port: sys.exit("수신기 포트 없음 (USB 연결 확인)")
    try:
        ser = serial.Serial(port, 38400, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"포트 열기 실패: {e}\n  -> sudo chmod a+rw {port} 또는 dialout 그룹 적용 후 재로그인")
    ser.write(UBXMessage.config_set(1, 0, [("CFG_MSGOUT_UBX_NAV_PVT_USB", 1)]).serialize())
    print(f"[open] {port}  NTRIP {conf['host']}:{conf['port']}/{conf['mountpoint']} as {conf['user']}")
    st = State()
    logf = open(a.log, "ab") if a.log else None
    csvf = None
    if not a.no_csv:
        path = a.csv or os.path.join(LOG_DIR, time.strftime("%Y%m%d_%H%M%S") + ".csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        csvf = open(path, "w")
        csvf.write("local_time,utc,fix,rtk,sats,lat,lon,alt_m,hAcc_m,vAcc_m,rtcm_age_s\n")
        print(f"[csv] {path}")
    last_utc = None
    if not a.no_web: start_web(st, a.web_port)
    threading.Thread(target=serial_reader, args=(ser, st, logf), daemon=True).start()
    threading.Thread(target=ntrip_client, args=(conf, ser, st), daemon=True).start()
    try:
        while True:
            time.sleep(1)
            with st.lock:
                p = dict(st.pvt); rb = st.rtcm_bytes; age = time.time() - st.rtcm_last if st.rtcm_last else None
            rtcm = f"rtcm={rb}B" + (f" age={age:.0f}s" if age is not None else "")
            if p:
                if p['utc'] != last_utc:
                    last_utc = p['utc']
                    rtk = p['carr'] or 'NONE'
                    if p['fix'] not in ("NoFix", "Time") and p['lat']:
                        with st.lock:
                            st.points.append(dict(t=time.strftime('%H:%M:%S'), fix=p['fix'], rtk=rtk, sats=p['sats'],
                                                  lat=p['lat'], lon=p['lon'], alt=p['alt'], hacc=p['hacc']))
                    if csvf:
                        csvf.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{p['utc']},{p['fix']},{rtk},{p['sats']},"
                                   f"{p['lat']:.8f},{p['lon']:.8f},{p['alt']:.3f},{p['hacc']:.3f},{p['vacc']:.3f},"
                                   f"{'' if age is None else f'{age:.0f}'}\n")
                        csvf.flush()
                print(f"{p['utc']} {p['fix']} {p['carr']:9s} sats={p['sats']:2d} "
                      f"lat={p['lat']:.7f} lon={p['lon']:.7f} alt={p['alt']:.2f} "
                      f"hAcc={p['hacc']:.3f}m vAcc={p['vacc']:.3f}m | {rtcm} | ntrip: {st.ntrip}")
            else:
                print(f"[wait] 수신기 데이터 대기 | {rtcm} | ntrip: {st.ntrip}")
    except KeyboardInterrupt:
        st.stop = True; print("\n[exit]")
    finally:
        ser.close()
        if logf: logf.close()
        if csvf: csvf.close()

if __name__ == "__main__":
    main()
