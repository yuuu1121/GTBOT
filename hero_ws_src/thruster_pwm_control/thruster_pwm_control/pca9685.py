#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCA9685 미니 드라이버 (smbus2) — 서보/ESC 펄스용.

~/pca9685.py 를 패키지로 가져온 것. 실측 오실레이터 보정 포함.
"""
import time

from smbus2 import SMBus


class PCA9685:
    # 실측 캘리브레이션: 이 개체의 실제 오실레이터 주파수 (2026-08 루프백 측정, 명목 25MHz 대비 +3.4%)
    OSC_HZ = 25_878_473

    def __init__(self, bus=1, addr=0x40, freq=50):
        self.bus = SMBus(bus)
        self.addr = addr
        self.freq = freq
        self._w(0x00, 0x00)            # MODE1: 리셋값
        time.sleep(0.005)
        prescale = round(self.OSC_HZ / (4096 * freq)) - 1
        old = self._r(0x00)
        self._w(0x00, (old & 0x7F) | 0x10)   # sleep
        self._w(0xFE, prescale)              # PRESCALE
        self._w(0x00, old & ~0x10)           # wake
        time.sleep(0.005)
        self._w(0x00, (old & ~0x10) | 0xA0)  # restart + auto-increment
        self.period_us = 1_000_000 / (self.OSC_HZ / (4096 * (prescale + 1)))

    def _w(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _r(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def set_us(self, ch, us):
        """채널 ch에 펄스폭 us 출력 (0이면 출력 정지)"""
        if us <= 0:
            self.bus.write_i2c_block_data(self.addr, 0x06 + 4*ch, [0, 0, 0, 0x10])  # full off
            return
        ticks = max(0, min(4095, round(us / self.period_us * 4096)))
        self.bus.write_i2c_block_data(self.addr, 0x06 + 4*ch, [0, 0, ticks & 0xFF, ticks >> 8])

    def all_off(self):
        self._w(0xFA, 0)
        self._w(0xFB, 0)
        self._w(0xFC, 0)
        self._w(0xFD, 0x10)

    def close(self):
        self.all_off()
        self.bus.close()
