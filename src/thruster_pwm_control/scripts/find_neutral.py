#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESC 실제 중립 펄스폭 찾기 (한 채널, 5µs 단위 수동 조정).

증상: 1500µs 를 줘도 한 번 돌기 시작한 스러스터가 안 멈춤
  -> PCA9685 오실레이터 보정값이 이 보드와 안 맞아 실제 펄스가 표시값과 다름.

방법:
  1) d 를 눌러 값을 올려 스러스터를 돌린다.
  2) a 로 5µs 씩 내리며 '멈추는 첫 값'(상단 경계)을 적는다.
  3) 계속 내려 '반대로 돌기 시작하는 값' 바로 위(하단 경계)를 적는다.
  4) 두 경계의 가운데가 이 보드의 실제 중립.

키: a = -5µs, d = +5µs, s = 1500 으로, q = 종료(출력 끔)
사용: python3 find_neutral.py [채널=6] [버스=7] [주소=0x40]
pwm_node 가 떠 있으면 먼저 종료할 것.
"""
import os
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from thruster_pwm_control.pca9685 import PCA9685  # noqa: E402

CH = int(sys.argv[1]) if len(sys.argv) > 1 else 6
BUS = int(sys.argv[2]) if len(sys.argv) > 2 else 7
ADDR = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0x40
US_MIN, US_MAX, STEP = 1400, 1600, 5

pca = PCA9685(bus=BUS, addr=ADDR, freq=200)
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
us = 1500
try:
    pca.set_us(CH, us)
    print(f'채널 {CH}: 1500µs 출력, 아밍 대기 5초...', flush=True)
    time.sleep(5.0)
    print('a = -5µs   d = +5µs   s = 1500   q = 종료')
    tty.setcbreak(fd)
    while True:
        sys.stdout.write(f'\r  현재 {us}µs   ')
        sys.stdout.flush()
        k = sys.stdin.read(1)
        if k == 'q':
            break
        if k == 'a':
            us = max(US_MIN, us - STEP)
        elif k == 'd':
            us = min(US_MAX, us + STEP)
        elif k == 's':
            us = 1500
        else:
            continue
        pca.set_us(CH, us)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    pca.set_us(CH, 0)
    pca.close()
    print('\n출력 끔')
