#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyboard_node
키보드 입력을 받아 4채널 스러스터 명령(Float32MultiArray, -1.0~+1.0)을 발행하는 텔레옵 노드.
Enter 없이 키를 누르는 즉시 반응 (cbreak). 키 입력은 /dev/tty 에서 직접 읽으므로
`ros2 launch` 안에서도 동작한다 (launch 는 stdin 을 자식에게 넘기지 않음).

채널 순서는 pwm_node 의 channels 파라미터와 동일 (기본 [1, 2, 3, 4]).
스러스터 1·2(ch1·ch2) = 전진/후진, 3·4(ch3·ch4) = 좌우 평행이동.
키를 누르고 있는 동안만 동작 (홀드 투 런). 터미널은 키 떼기 이벤트가 없으므로
키 자동반복이 release_timeout 초 동안 끊기면 뗀 것으로 보고 정지한다.
쌍끼리 추력 방향이 반대라 directions 파라미터(기본 [1, 1, 1, 1])로 부호를 맞춘다.
"""

import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

ESC = '\x1b'


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self.cmd = self.declare_parameter('cmd', 0.15).value
        self.cmd_step = self.declare_parameter('cmd_step', 0.05).value
        self.cmd_max = self.declare_parameter('cmd_max', 0.25).value
        self.num_thrusters = self.declare_parameter('num_thrusters', 4).value
        self.directions = list(
            self.declare_parameter('directions', [1.0, 1.0, 1.0, 1.0]).value)
        self.release_timeout = self.declare_parameter('release_timeout', 0.6).value
        cmd_topic = self.declare_parameter('cmd_topic', 'thruster_pwm_cmd').value

        if len(self.directions) != self.num_thrusters:
            raise ValueError(
                f'directions 는 {self.num_thrusters}개여야 합니다 (받음: {self.directions})')
        self.tty = open('/dev/tty', 'r')
        self.tty_out = open('/dev/tty', 'w')

        self.publisher = self.create_publisher(Float32MultiArray, cmd_topic, 10)
        self.last_sent = None

    def _mixes(self):
        """키 -> 4채널(1,2,3,4) 명령 벡터. w/x/a/d 는 directions 부호 적용."""
        c = self.cmd
        d = self.directions
        return {
            'w': [c * d[0], c * d[1], 0.0, 0.0],      # 전진 (스러스터 1·2)
            's': [-c * d[0], -c * d[1], 0.0, 0.0],    # 후진
            'a': [0.0, 0.0, c * d[2], c * d[3]],      # 좌측 평행이동 (스러스터 3·4, 실측: + = 좌측)
            'd': [0.0, 0.0, -c * d[2], -c * d[3]],    # 우측 평행이동
            '1': [c, 0.0, 0.0, 0.0],    # ch1 단독 (테스트, directions 미적용)
            '2': [0.0, c, 0.0, 0.0],    # ch2 단독
            '3': [0.0, 0.0, c, 0.0],    # ch3 단독
            '4': [0.0, 0.0, 0.0, c],    # ch4 단독
            'x': [0.0] * self.num_thrusters,   # stop
            ' ': [0.0] * self.num_thrusters,   # space = stop
        }

    def publish_thrusters(self, values, label=''):
        values = [float(v) for v in values]
        # 키 자동반복으로 같은 명령이 연속 들어오면 다시 발행하지 않음 (pwm_node 로그 폭주 방지)
        if values != self.last_sent:
            msg = Float32MultiArray()
            msg.data = values
            self.publisher.publish(msg)
            self.last_sent = values
        vec = ' '.join(f'{v:+.2f}' for v in values)
        self._print(f'\r  [{vec}] {label}          ')

    def _print(self, text):
        self.tty_out.write(text)
        self.tty_out.flush()

    def run(self):
        self._print(
            '\n===== THRUSTER PWM TELEOP (ch 1/2/3/4) =====\n'
            'w : 전진          s : 후진\n'
            'a : 좌측 이동     d : 우측 이동\n'
            '1~4 : 스러스터 단독 (ch1/ch2/ch3/ch4)\n'
            '+ : 세기 증가     - : 세기 감소\n'
            'x 또는 space : stop    z 또는 ESC : quit (정지 후 종료)\n'
            '  * 키를 누르고 있는 동안만 동작, 떼면 정지\n'
            '=============================================\n\n')

        fd = self.tty.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        stop = [0.0] * self.num_thrusters
        last_key_t = 0.0
        try:
            while True:
                r, _, _ = select.select([self.tty], [], [], 0.05)
                if not r:
                    if (self.last_sent not in (None, stop)
                            and time.monotonic() - last_key_t > self.release_timeout):
                        self.publish_thrusters(stop, '(released)')
                    continue
                key = self.tty.read(1)

                if key in ('z', ESC):
                    self.publish_thrusters(
                        [0.0] * self.num_thrusters, 'STOP & QUIT')
                    break

                if key in ('+', '='):
                    self.cmd = min(self.cmd_max, self.cmd + self.cmd_step)
                    self._print(f'\r  세기: {self.cmd:.2f}          ')
                    continue
                if key == '-':
                    self.cmd = max(0.0, self.cmd - self.cmd_step)
                    self._print(f'\r  세기: {self.cmd:.2f}          ')
                    continue

                cmd = self._mixes().get(key)
                if cmd is None:
                    continue

                last_key_t = time.monotonic()
                self.publish_thrusters(cmd, f'({key}, cmd={self.cmd:.2f})')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self._print('\n')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        node.run()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
