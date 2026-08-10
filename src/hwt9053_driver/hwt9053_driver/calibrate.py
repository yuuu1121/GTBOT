#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HWT9053-485 대화형 캘리브레이션 도구 (패키지 실행파일)

    ros2 run hwt9053_driver calibrate
    ros2 run hwt9053_driver calibrate --port /dev/hwt9053 --baud 9600 --addr 80

시리얼 포트를 단독 점유하므로 실행 전 드라이버 노드를 내려야 한다:
    pkill -f hwt9053_node   (또는 launch 터미널에서 Ctrl+C)

레지스터/값 출처: WITMOTION 공식 SDK (REG.h / wit_c_sdk.c)
"""

import sys
import time
import math
import argparse

from hwt9053_driver.hwt9053_interface import HWT9053Interface

MAG_ROTATE_SEC = 20


def show_angles(sensor):
    a = sensor.read_angles()
    if a:
        print("  현재 각도(deg): Roll=%+.1f  Pitch=%+.1f  Yaw=%+.1f"
              % (math.degrees(a[0]), math.degrees(a[1]), math.degrees(a[2])))


def calibrate_acc(sensor):
    print("\n=== 가속도/자이로 캘리브레이션 ===")
    print("센서를 '수평'하고 '완전히 정지' 상태로 두세요.")
    input("준비되면 Enter...")
    print("캘리브레이션 중... 3초간 움직이지 마세요.")
    ok = sensor.calibrate_acc_gyro(settle_sec=3.0)
    print("✓ 완료 및 저장." if ok else "✗ 실패.")
    show_angles(sensor)


def calibrate_mag(sensor):
    print("\n=== 자기장(지자기) 캘리브레이션 ===")
    print("시작하면 센서를 X, Y, Z 세 축 모두를 중심으로")
    print("천천히 각각 최소 한 바퀴(360°)씩 부드럽게 돌리세요.")
    print("⚠ 모터/CAN 배선/금속에서 최대한 떨어져서 하세요.")
    input("준비되면 Enter...")
    if not sensor.start_mag_cali():
        print("✗ 시작 실패."); return
    print(f"회전 시작! {MAG_ROTATE_SEC}초간 전 축을 천천히 돌리세요.")
    for i in range(MAG_ROTATE_SEC, 0, -1):
        print(f"  남은 시간 {i:2d}s", end='\r', flush=True)
        time.sleep(1)
    print()
    ok = sensor.stop_cali_and_save()
    print("✓ 자기장 캘리브레이션 완료 및 저장." if ok else "✗ 종료/저장 실패.")


def reset_yaw(sensor):
    print("\n=== Yaw(Z) 각도 0으로 리셋 ===")
    ok = sensor.reset_yaw()
    print("✓ 완료 및 저장." if ok else "✗ 실패.")
    show_angles(sensor)


def menu(sensor):
    actions = {'1': calibrate_acc, '2': calibrate_mag, '3': reset_yaw}
    while True:
        print("\n========== HWT9053 캘리브레이션 ==========")
        show_angles(sensor)
        print(" 1) 가속도/자이로 캘리브레이션 (수평·정지)")
        print(" 2) 자기장 캘리브레이션 (전 축 회전)   <-- yaw 튐 해결")
        print(" 3) Yaw(Z) 각도 리셋")
        print(" q) 종료")
        sel = input("> ").strip().lower()
        if sel == 'q':
            break
        act = actions.get(sel)
        if act:
            act(sensor)


def main():
    parser = argparse.ArgumentParser(description="HWT9053-485 캘리브레이션")
    parser.add_argument('--port', default='/dev/hwt9053')
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--addr', type=lambda x: int(x, 0), default=0x50,
                        help='Modbus 주소 (기본 0x50=80)')
    parser.add_argument('cmd', nargs='?', choices=['acc', 'mag', 'yaw'],
                        help='지정 시 해당 캘리브레이션만 수행(미지정 시 메뉴)')
    args = parser.parse_args()

    sensor = HWT9053Interface(port=args.port, baudrate=args.baud,
                              device_address=args.addr)
    if not sensor.connect():
        print(f"✗ 포트 {args.port} 열기 실패. "
              f"(드라이버 노드가 잡고 있으면: pkill -f hwt9053_node)")
        sys.exit(1)
    print(f"✓ 연결됨: {args.port} @ {args.baud}, addr 0x{args.addr:02X}")

    try:
        if args.cmd:
            {'acc': calibrate_acc, 'mag': calibrate_mag,
             'yaw': reset_yaw}[args.cmd](sensor)
        else:
            menu(sensor)
    except (KeyboardInterrupt, EOFError):
        print("\n중단됨.")
    finally:
        # 중간에 빠져나가도 캘리브레이션 모드에 갇히지 않도록 NORMAL 복귀
        try:
            sensor._set_calsw(sensor.CAL_NORMAL)
        except Exception:
            pass
        sensor.disconnect()


if __name__ == '__main__':
    main()
