#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053-485 AHRS 센서 인터페이스 클래스

pymodbus 2.x / 3.x 양쪽 API 를 모두 지원한다.
(ROS 와 무관한 순수 파이썬 모듈이라 단독 실행/테스트도 가능)
"""

import time
import math
import inspect

import pymodbus

# pymodbus 메이저 버전에 따라 import 경로와 호출 규약이 다르다.
_PYMODBUS_MAJOR = int(pymodbus.__version__.split('.')[0])

if _PYMODBUS_MAJOR >= 3:
    from pymodbus.client import ModbusSerialClient  # pymodbus 3.x
else:
    from pymodbus.client.sync import ModbusSerialClient  # pymodbus 2.x


def _detect_slave_kw():
    """
    pymodbus 버전마다 슬레이브(장치 주소) 지정 키워드가 다르다.
      2.x         -> unit
      3.0 ~ 3.13  -> slave
      3.14+       -> device_id
    설치된 버전의 read_holding_registers 시그니처를 보고 알맞은 키워드를 고른다.
    """
    if _PYMODBUS_MAJOR < 3:
        return 'unit'
    try:
        params = inspect.signature(
            ModbusSerialClient.read_holding_registers).parameters
    except (ValueError, TypeError):
        return 'slave'
    for name in ('device_id', 'slave', 'unit'):
        if name in params:
            return name
    return 'slave'


_SLAVE_KW = _detect_slave_kw()


class HWT9053Interface:
    """HWT9053-485 AHRS 센서 인터페이스"""

    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, device_address=0x50):
        """
        초기화

        Args:
            port: 시리얼 포트 경로
            baudrate: 통신 속도 (기본 9600)
            device_address: Modbus 장치 주소 (기본 0x50)
        """
        self.port = port
        self.baudrate = baudrate
        self.device_address = device_address

        client_kwargs = dict(
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8,
        )
        # pymodbus 2.x 는 RTU 프레이머를 method 로 명시해야 한다.
        # 3.x 는 시리얼 클라이언트 기본값이 RTU 이며 method 인자가 제거되었다.
        if _PYMODBUS_MAJOR < 3:
            client_kwargs['method'] = 'rtu'

        self.client = ModbusSerialClient(**client_kwargs)
        self.connected = False

    # ------------------------------------------------------------------ #
    # pymodbus 버전별 슬레이브(주소) 지정 키워드 (unit / slave / device_id)
    # ------------------------------------------------------------------ #
    def _slave_kwargs(self):
        return {_SLAVE_KW: self.device_address}

    def connect(self):
        """센서 연결"""
        self.connected = self.client.connect()
        return self.connected

    def disconnect(self):
        """센서 연결 해제"""
        self.client.close()
        self.connected = False

    def _read_registers(self, start_addr, count):
        """
        레지스터 읽기

        Args:
            start_addr: 시작 주소
            count: 읽을 레지스터 개수

        Returns:
            레지스터 값 리스트 또는 None
        """
        if not self.connected:
            return None

        try:
            result = self.client.read_holding_registers(
                start_addr, count=count, **self._slave_kwargs()
            )

            if result is None or result.isError():
                return None

            return result.registers
        except Exception as e:
            print(f"레지스터 읽기 오류: {e}")
            return None

    def _decode_int16(self, value):
        """16비트 부호있는 정수 디코딩"""
        if value > 32767:
            value -= 65536
        return value

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """
        오일러각(rad) -> 쿼터니언 (w, x, y, z).
        Tait-Bryan ZYX (yaw->pitch->roll) 순서.

        주의: HWT9053 의 쿼터니언 레지스터(0x40~0x43)는 오일러각(0x3D~0x3F)과
        일치하지 않는 값을 내보낸다(수평에서도 단위행렬이 아님). 실측 검증 결과
        오일러각은 정확하므로, orientation 은 오일러각에서 계산해서 쓴다.
        """
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (w, x, y, z)

    def read_acceleration(self):
        """
        가속도 읽기

        Returns:
            (ax, ay, az) in m/s² 또는 None
        """
        regs = self._read_registers(0x34, 3)
        if regs is None:
            return None

        ax = self._decode_int16(regs[0]) / 32768.0 * 16.0 * 9.81
        ay = self._decode_int16(regs[1]) / 32768.0 * 16.0 * 9.81
        az = self._decode_int16(regs[2]) / 32768.0 * 16.0 * 9.81

        return (ax, ay, az)

    def read_gyroscope(self):
        """
        각속도 읽기

        Returns:
            (gx, gy, gz) in rad/s 또는 None
        """
        regs = self._read_registers(0x37, 3)
        if regs is None:
            return None

        gx = self._decode_int16(regs[0]) / 32768.0 * 2000.0 * math.pi / 180.0
        gy = self._decode_int16(regs[1]) / 32768.0 * 2000.0 * math.pi / 180.0
        gz = self._decode_int16(regs[2]) / 32768.0 * 2000.0 * math.pi / 180.0

        return (gx, gy, gz)

    def read_magnetometer(self):
        """
        자기장 읽기

        Returns:
            (mx, my, mz) in μT 또는 None
        """
        regs = self._read_registers(0x3A, 3)
        if regs is None:
            return None

        mx = self._decode_int16(regs[0])
        my = self._decode_int16(regs[1])
        mz = self._decode_int16(regs[2])

        return (mx, my, mz)

    def read_angles(self):
        """
        오일러 각도 읽기

        Returns:
            (roll, pitch, yaw) in radians 또는 None
        """
        regs = self._read_registers(0x3D, 3)
        if regs is None:
            return None

        roll = self._decode_int16(regs[0]) / 32768.0 * math.pi
        pitch = self._decode_int16(regs[1]) / 32768.0 * math.pi
        yaw = self._decode_int16(regs[2]) / 32768.0 * math.pi

        return (roll, pitch, yaw)

    def read_quaternion(self):
        """
        쿼터니언 읽기 (오일러각에서 계산 — 레지스터 0x40 값은 신뢰 불가, _euler_to_quaternion 참고)

        Returns:
            (w, x, y, z) 또는 None
        """
        angles = self.read_angles()
        if angles is None:
            return None
        return self._euler_to_quaternion(*angles)

    def read_all(self):
        """
        모든 센서 데이터 읽기

        Returns:
            dict 형태의 센서 데이터 또는 None
        """
        # 한 번에 모든 데이터 읽기 (0x34부터 16개 레지스터)
        regs = self._read_registers(0x34, 16)
        if regs is None:
            return None

        # 가속도
        ax = self._decode_int16(regs[0]) / 32768.0 * 16.0 * 9.81
        ay = self._decode_int16(regs[1]) / 32768.0 * 16.0 * 9.81
        az = self._decode_int16(regs[2]) / 32768.0 * 16.0 * 9.81

        # 각속도
        gx = self._decode_int16(regs[3]) / 32768.0 * 2000.0 * math.pi / 180.0
        gy = self._decode_int16(regs[4]) / 32768.0 * 2000.0 * math.pi / 180.0
        gz = self._decode_int16(regs[5]) / 32768.0 * 2000.0 * math.pi / 180.0

        # 자기장
        mx = self._decode_int16(regs[6])
        my = self._decode_int16(regs[7])
        mz = self._decode_int16(regs[8])

        # 오일러 각도
        roll = self._decode_int16(regs[9]) / 32768.0 * math.pi
        pitch = self._decode_int16(regs[10]) / 32768.0 * math.pi
        yaw = self._decode_int16(regs[11]) / 32768.0 * math.pi

        # 쿼터니언: 레지스터 0x40~0x43(regs[12:16]) 값은 오일러각과 불일치하여
        # 신뢰할 수 없음(실측 검증). 검증된 오일러각에서 계산해 사용한다.
        q0, q1, q2, q3 = self._euler_to_quaternion(roll, pitch, yaw)

        data = {
            'acceleration': (ax, ay, az),
            'gyroscope': (gx, gy, gz),
            'magnetometer': (mx, my, mz),
            'angles': (roll, pitch, yaw),
            'quaternion': (q0, q1, q2, q3),
            'timestamp': time.time()
        }

        return data

    def set_output_rate(self, rate_hz):
        """
        출력 속도 설정

        Args:
            rate_hz: 출력 속도 (Hz) - 1, 5, 10, 20, 50, 100, 200

        Returns:
            성공 여부
        """
        rate_map = {
            1: 0x01, 5: 0x02, 10: 0x03, 20: 0x04,
            50: 0x05, 100: 0x06, 200: 0x07
        }

        if rate_hz not in rate_map:
            return False

        try:
            result = self.client.write_register(
                0x03, rate_map[rate_hz], **self._slave_kwargs()
            )
            return not result.isError()
        except Exception:
            return False

    def save_configuration(self):
        """
        설정 저장 (재부팅 후에도 유지)

        Returns:
            성공 여부
        """
        try:
            result = self.client.write_register(
                0x00, 0x00, **self._slave_kwargs()
            )
            return not result.isError()
        except Exception:
            return False

    def reset_to_factory(self):
        """
        공장 초기화

        Returns:
            성공 여부
        """
        try:
            result = self.client.write_register(
                0x00, 0x01, **self._slave_kwargs()
            )
            return not result.isError()
        except Exception:
            return False

    # ------------------------------------------------------------------ #
    # 캘리브레이션 (WitMotion 표준 Modbus, 공식 SDK REG.h/wit_c_sdk.c 기준)
    #   KEY(0x69)<-0xB588 언락 -> CALSW(0x01) 모드 -> SAVE(0x00)<-0x0000 저장
    #   CALSW: 0x00 NORMAL, 0x01 가속도/자이로, 0x04 yaw리셋, 0x07 자기장
    # ------------------------------------------------------------------ #
    REG_KEY = 0x69
    REG_CALSW = 0x01
    REG_SAVE = 0x00
    KEY_UNLOCK = 0xB588
    CAL_NORMAL = 0x00
    CAL_GYROACC = 0x01
    CAL_ANGLEZ = 0x04
    CAL_MAGMM = 0x07

    def _write(self, addr, value):
        """단일 레지스터 쓰기 (성공 여부)."""
        try:
            result = self.client.write_register(addr, value, **self._slave_kwargs())
            return not result.isError()
        except Exception as e:
            print(f"레지스터 쓰기 오류: {e}")
            return False

    def _unlock(self):
        ok = self._write(self.REG_KEY, self.KEY_UNLOCK)
        time.sleep(0.1)
        return ok

    def _set_calsw(self, mode):
        self._unlock()
        ok = self._write(self.REG_CALSW, mode)
        time.sleep(0.1)
        return ok

    def save_config(self):
        """파라미터 저장(재부팅 유지)."""
        self._unlock()
        ok = self._write(self.REG_SAVE, 0x0000)
        time.sleep(0.2)
        return ok

    def calibrate_acc_gyro(self, settle_sec=3.0):
        """가속도/자이로 캘리브레이션 — 센서를 수평·정지 상태로 두고 호출."""
        if not self._set_calsw(self.CAL_GYROACC):
            return False
        time.sleep(settle_sec)          # 캘리브레이션 수렴 대기(움직이면 안 됨)
        self._set_calsw(self.CAL_NORMAL)
        return self.save_config()

    def start_mag_cali(self):
        """자기장 캘리브레이션 시작 — 이후 센서를 전 축 360° 천천히 회전."""
        return self._set_calsw(self.CAL_MAGMM)

    def stop_cali_and_save(self):
        """캘리브레이션 종료(NORMAL) 후 저장 — 자기장 회전 완료 후 호출."""
        self._set_calsw(self.CAL_NORMAL)
        return self.save_config()

    def reset_yaw(self):
        """Yaw(Z) 각도를 0으로 리셋 후 저장."""
        if not self._set_calsw(self.CAL_ANGLEZ):
            return False
        return self.save_config()


def main():
    """테스트 메인 함수"""
    sensor = HWT9053Interface(port='/dev/ttyUSB0', baudrate=9600)

    if not sensor.connect():
        print("센서 연결 실패!")
        return

    print("센서 연결 성공!")
    print("데이터 읽기 시작... (Ctrl+C로 중지)\n")

    try:
        while True:
            data = sensor.read_all()
            if data:
                print(f"시간: {data['timestamp']:.3f}")
                print(f"  가속도 (m/s²): X={data['acceleration'][0]:7.3f}, "
                      f"Y={data['acceleration'][1]:7.3f}, "
                      f"Z={data['acceleration'][2]:7.3f}")
                print(f"  각속도 (rad/s): X={data['gyroscope'][0]:7.3f}, "
                      f"Y={data['gyroscope'][1]:7.3f}, "
                      f"Z={data['gyroscope'][2]:7.3f}")
                print(f"  자기장 (μT):    X={data['magnetometer'][0]:7.0f}, "
                      f"Y={data['magnetometer'][1]:7.0f}, "
                      f"Z={data['magnetometer'][2]:7.0f}")
                print(f"  각도 (deg):     Roll={math.degrees(data['angles'][0]):7.3f}, "
                      f"Pitch={math.degrees(data['angles'][1]):7.3f}, "
                      f"Yaw={math.degrees(data['angles'][2]):7.3f}")
                print(f"  쿼터니언:       W={data['quaternion'][0]:7.4f}, "
                      f"X={data['quaternion'][1]:7.4f}, "
                      f"Y={data['quaternion'][2]:7.4f}, "
                      f"Z={data['quaternion'][3]:7.4f}")
                print("-" * 80)
            else:
                print("데이터 읽기 실패!")

            time.sleep(0.1)  # 10Hz

    except KeyboardInterrupt:
        print("\n중지됨.")
    finally:
        sensor.disconnect()
        print("센서 연결 해제.")


if __name__ == '__main__':
    main()
