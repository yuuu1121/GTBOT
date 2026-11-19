#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gnss_node
ZED-F9P(USB) 에서 UBX NAV-PVT 를 읽어 sensor_msgs/NavSatFix 로 발행하고,
국토지리정보원 NTRIP 보정(RTCM)을 받아 수신기에 주입해 RTK 측위를 한다.
NTRIP·실시간 지도·CSV 저장은 rtk_run.py 의 구현을 그대로 사용.

발행:
  gnss/fix         sensor_msgs/NavSatFix  (고도 = WGS84 타원체고, 공분산 = hAcc²/vAcc²)
  gnss/rtk_status  std_msgs/String        (RTK-FIXED | RTK-FLOAT | NONE | NoFix)
"""

import glob
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

import serial
from pyubx2 import NMEA_PROTOCOL, UBX_PROTOCOL, UBXMessage, UBXReader

from gtbot_gnss.rtk_run import (CARR, FIX, LOG_DIR, NTRIP_DEFAULT, State,
                                ntrip_client, start_web)

UBLOX_BY_ID = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'


class GnssNode(Node):

    def __init__(self):
        super().__init__('gnss_node')

        # 파라미터
        port = self.declare_parameter('port', '').value              # '' = 자동 탐지
        baud = self.declare_parameter('baud', 38400).value
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        use_ntrip = self.declare_parameter('use_ntrip', True).value
        conf = {k: str(self.declare_parameter(f'ntrip_{k}', v).value)
                for k, v in NTRIP_DEFAULT.items()}
        web_port = self.declare_parameter('web_port', 8000).value     # 0 = 지도 끔
        save_csv = self.declare_parameter('save_csv', True).value
        csv_dir = os.path.expanduser(self.declare_parameter('csv_dir', LOG_DIR).value)

        port = port or self._find_port()
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            self.get_logger().fatal(
                f'GNSS 포트 {port} 열기 실패: {e} (USB 연결, dialout 권한 확인)')
            raise
        # USB 로 NAV-PVT 1Hz 출력 (RAM 설정, 수신기 재부팅 시 원복)
        self.ser.write(UBXMessage.config_set(
            1, 0, [('CFG_MSGOUT_UBX_NAV_PVT_USB', 1)]).serialize())

        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.rtk_pub = self.create_publisher(String, 'gnss/rtk_status', 10)
        self.st = State()

        self.csvf = None
        if save_csv:
            os.makedirs(csv_dir, exist_ok=True)
            path = os.path.join(csv_dir, time.strftime('%Y%m%d_%H%M%S') + '.csv')
            self.csvf = open(path, 'w')
            self.csvf.write(
                'local_time,utc,fix,rtk,sats,lat,lon,alt_m,hAcc_m,vAcc_m,rtcm_age_s\n')
            self.get_logger().info(f'CSV 저장: {path}')

        if web_port:
            start_web(self.st, web_port)

        ntrip_desc = '끔'
        if use_ntrip:
            threading.Thread(target=ntrip_client, args=(conf, self.ser, self.st),
                             daemon=True).start()
            ntrip_desc = f"{conf['host']}:{conf['port']}/{conf['mountpoint']} ({conf['user']})"

        self.reader = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader.start()
        self.create_timer(1.0, self._status_log)
        self.get_logger().info(
            f'GNSS Node 시작 ({port} @ {baud}, frame={self.frame_id}, NTRIP: {ntrip_desc}, '
            f'지도: {"http://<IP>:%d" % web_port if web_port else "끔"})')

    @staticmethod
    def _find_port():
        if os.path.exists(UBLOX_BY_ID):
            return UBLOX_BY_ID
        ports = sorted(glob.glob('/dev/ttyACM*'))
        if not ports:
            raise RuntimeError('GNSS 수신기 포트 없음 (USB 연결, lsusb 에 u-blox 확인)')
        return ports[0]

    def _serial_reader(self):
        st = self.st
        ubr = UBXReader(self.ser, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL, quitonerror=0)
        while not st.stop:
            try:
                raw, msg = ubr.read()
            except Exception:
                continue
            if raw is None or msg is None:
                continue
            i = msg.identity
            if i.endswith('GGA') and msg.lat:
                with st.lock:
                    st.gga = raw           # NTRIP 캐스터로 보낼 내 위치
            elif i == 'NAV-PVT':
                if not rclpy.ok():         # Ctrl-C 로 ROS 컨텍스트가 먼저 닫힌 경우
                    break
                self._on_pvt(msg)

    def _on_pvt(self, m):
        st = self.st
        fix = FIX.get(m.fixType, m.fixType)
        carr = CARR.get(m.carrSoln, '')
        rtk = carr or 'NONE'
        hacc, vacc = m.hAcc / 1000, m.vAcc / 1000
        utc = f'{m.hour:02d}:{m.min:02d}:{m.second:02d}'
        with st.lock:
            st.pvt = dict(fix=fix, carr=carr, sats=m.numSV, lat=m.lat, lon=m.lon,
                          alt=m.hMSL / 1000, hacc=hacc, vacc=vacc, utc=utc)
            age = time.time() - st.rtcm_last if st.rtcm_last else None
        has_fix = m.gnssFixOk and fix not in ('NoFix', 'Time')

        # NavSatFix
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_GALILEO | NavSatStatus.SERVICE_COMPASS)
        if not has_fix:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif carr:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX   # RTK 보정 적용
        else:
            msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude, msg.longitude = m.lat, m.lon
        msg.altitude = m.height / 1000                         # 타원체고 (NavSatFix 규약)
        msg.position_covariance = [hacc ** 2, 0.0, 0.0,
                                   0.0, hacc ** 2, 0.0,
                                   0.0, 0.0, vacc ** 2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(msg)
        self.rtk_pub.publish(String(data=rtk if has_fix else 'NoFix'))

        # 실시간 지도 경로 + CSV (rtk_run.py 와 같은 형식)
        if has_fix:
            with st.lock:
                st.points.append(dict(t=time.strftime('%H:%M:%S'), fix=fix, rtk=rtk,
                                      sats=m.numSV, lat=m.lat, lon=m.lon,
                                      alt=m.hMSL / 1000, hacc=hacc))
        if self.csvf:
            self.csvf.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')},{utc},{fix},{rtk},{m.numSV},"
                f"{m.lat:.8f},{m.lon:.8f},{m.hMSL / 1000:.3f},{hacc:.3f},{vacc:.3f},"
                f"{'' if age is None else f'{age:.0f}'}\n")
            self.csvf.flush()

    def _status_log(self):
        st = self.st
        with st.lock:
            p = dict(st.pvt)
            rb = st.rtcm_bytes
            age = time.time() - st.rtcm_last if st.rtcm_last else None
        rtcm = f'rtcm={rb}B' + (f' age={age:.0f}s' if age is not None else '')
        if p:
            self.get_logger().info(
                f"{p['utc']} {p['fix']} {p['carr'] or '-':9s} sats={p['sats']:2d} "
                f"lat={p['lat']:.7f} lon={p['lon']:.7f} hAcc={p['hacc']:.3f}m | "
                f"{rtcm} | ntrip: {st.ntrip}")
        else:
            self.get_logger().info(f'수신기 데이터 대기 | {rtcm} | ntrip: {st.ntrip}')

    def shutdown(self):
        # 읽기 스레드가 발행 중이면 destroy_node 가 막히므로 먼저 멈춘다
        self.st.stop = True
        self.reader.join(timeout=2.0)
        try:
            self.ser.close()
        except Exception:
            pass
        if self.csvf:
            self.csvf.close()


def main(args=None):
    rclpy.init(args=args)
    node = GnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C 는 터미널과 launch 양쪽에서 SIGINT 가 오므로 정리 중 추가 SIGINT 무시
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
