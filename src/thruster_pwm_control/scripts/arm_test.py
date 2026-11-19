#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESC 아밍 지점 찾기 (스러스터는 돌리지 않음).

중립 후보 펄스폭을 하나씩 내보내고, 각 단계 사이에는 신호를 완전히 끊는다.
ESC 는 신호가 끊기면 아밍이 풀리므로 어떤 단계에서도 모터가 돌지 않는다.

각 단계에서 들리는 소리:
  - 소리 없음            : ESC 가 신호를 못 받음 (배선/GND/핀 위치)
  - 낮은 음 1번만        : 신호는 받지만 중립으로 인식 못 함 (펄스폭 오차)
  - 낮은 음 + 높은 음    : 아밍 완료 -> 이 값이 이 보드의 실제 중립

사용: python3 arm_test.py [채널=8] [버스=7]
pwm_node 가 떠 있으면 먼저 종료할 것.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from thruster_pwm_control.pca9685 import PCA9685  # noqa: E402

CH = int(sys.argv[1]) if len(sys.argv) > 1 else 8
BUS = int(sys.argv[2]) if len(sys.argv) > 2 else 7
CANDIDATES = [1500, 1475, 1450, 1425, 1400, 1525, 1550, 1575]
HOLD_SEC = 5.0
OFF_SEC = 2.5

pca = PCA9685(bus=BUS, addr=0x40, freq=200)
try:
    print(f'채널 {CH} (버스 {BUS}) — 단계마다 Enter 를 누르면 다음으로, Ctrl+C 로 중단')
    for us in CANDIDATES:
        pca.set_us(CH, 0)                      # 신호 끊기 -> ESC 아밍 해제
        input(f'\n[Enter] 다음 후보: {us}µs ')
        time.sleep(OFF_SEC)
        pca.set_us(CH, us)
        print(f'  >>> {us}µs 출력 중 ({HOLD_SEC:.0f}초) — 소리를 들어보세요', flush=True)
        time.sleep(HOLD_SEC)
        ans = input('  들린 소리? (0=없음, 1=낮은 음만, 2=낮은+높은 음): ').strip()
        print(f'  기록: {us}µs -> {ans}')
except KeyboardInterrupt:
    print('\n중단')
finally:
    pca.set_us(CH, 0)
    pca.close()
    print('출력 끔')
