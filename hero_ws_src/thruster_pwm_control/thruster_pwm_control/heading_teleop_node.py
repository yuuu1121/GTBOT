#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
heading_teleop_node
키보드 -> geometry_msgs/Twist(cmd_vel) 텔레옵. cascade_controller 와 짝.

  w/s : 전진/후진 (linear.x)        a/d : 좌/우 이동 (linear.y, a=좌=+)
  q/e : 목표 헤딩 좌/우 회전 (angular.z, rad/s)   h : 목표 헤딩 = 현재 헤딩
  t   : 헤딩 유지 ON/OFF 토글       +/- : 세기 증감
  x/space : 정지                    z/ESC : 종료

키를 누르고 있는 동안만 동작(홀드 투 런). /dev/tty 를 직접 읽어 launch 안에서도 조작 가능.
cmd_vel 은 키가 눌린 동안 publish_rate 로 계속 재발행된다 (컨트롤러 cmd_timeout 보다 빨라야 함).
"""

import math
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

ESC = '\x1b'


class HeadingTeleopNode(Node):

    def __init__(self):
        super().__init__('heading_teleop')
        self.speed = self.declare_parameter('speed', 0.15).value
        self.speed_step = self.declare_parameter('speed_step', 0.05).value
        self.speed_max = self.declare_parameter('speed_max', 0.15).value
        self.turn_rate = math.radians(self.declare_parameter('turn_rate_deg', 20.0).value)
        self.release_timeout = self.declare_parameter('release_timeout', 0.6).value
        self.publish_rate = self.declare_parameter('publish_rate', 10.0).value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel').value
        ctrl = self.declare_parameter('controller_name', 'cascade_controller').value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cli_hold = self.create_client(SetBool, f'/{ctrl}/enable_hold')
        self.cli_here = self.create_client(Trigger, f'/{ctrl}/set_heading_here')
        self.cli_cal = self.create_client(Trigger, f'/{ctrl}/calibrate_yaw_mix')
        self.hold_on = True

        self.tty = open('/dev/tty', 'r')
        self.tty_out = open('/dev/tty', 'w')

    def _print(self, text):
        self.tty_out.write(text)
        self.tty_out.flush()

    def _twist(self, key):
        s, r = self.speed, self.turn_rate
        table = {
            'w': (s, 0.0, 0.0), 's': (-s, 0.0, 0.0),
            'a': (0.0, s, 0.0), 'd': (0.0, -s, 0.0),
            'q': (0.0, 0.0, r), 'e': (0.0, 0.0, -r),
            'x': (0.0, 0.0, 0.0), ' ': (0.0, 0.0, 0.0),
        }
        if key not in table:
            return None
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = table[key]
        return t

    def _call_async(self, client, req, label):
        if not client.service_is_ready():
            self._print(f'\r  {label}: 서비스 없음 (컨트롤러 실행 중?)          ')
            return
        fut = client.call_async(req)

        def done(f):
            try:
                res = f.result()
                self._print(f'\r  {label}: {res.message}          ')
            except Exception as ex:  # noqa: BLE001
                self._print(f'\r  {label}: 실패 {ex}          ')
        fut.add_done_callback(done)

    def run(self):
        self._print(
            '\n===== HEADING-HOLD TELEOP (cmd_vel) =====\n'
            'w/s : 전진/후진      a/d : 좌/우 이동\n'
            'q/e : 목표 헤딩 좌/우 회전   h : 목표 헤딩 = 현재\n'
            't   : 헤딩 유지 ON/OFF       +/- : 세기 증감\n'
            'c   : yaw 부호 자동 캘리브레이션 (정지 상태, 약 15s, 끝나면 t)\n'
            'x/space : 정지              z/ESC : 종료\n'
            '  * 키를 누르고 있는 동안만 동작, 떼면 정지\n'
            '==========================================\n\n')
        fd = self.tty.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        stop = Twist()
        current = None          # 현재 눌린 명령 (None = 정지 상태)
        last_key_t = 0.0
        last_pub_t = 0.0
        period = 1.0 / self.publish_rate
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                r, _, _ = select.select([self.tty], [], [], 0.02)
                now = time.monotonic()
                if r:
                    key = self.tty.read(1)
                    if key in ('z', ESC):
                        self.pub.publish(stop)
                        self._print('\r  STOP & QUIT          \n')
                        break
                    if key in ('+', '='):
                        self.speed = min(self.speed_max, self.speed + self.speed_step)
                        self._print(f'\r  세기: {self.speed:.2f}          ')
                        continue
                    if key == '-':
                        self.speed = max(0.0, self.speed - self.speed_step)
                        self._print(f'\r  세기: {self.speed:.2f}          ')
                        continue
                    if key == 'h':
                        self._call_async(self.cli_here, Trigger.Request(), '헤딩 재설정')
                        continue
                    if key == 'c':
                        self.hold_on = False
                        self._call_async(self.cli_cal, Trigger.Request(), 'yaw 캘리브레이션')
                        continue
                    if key == 't':
                        self.hold_on = not self.hold_on
                        req = SetBool.Request()
                        req.data = self.hold_on
                        self._call_async(self.cli_hold, req, '헤딩 유지')
                        continue
                    tw = self._twist(key)
                    if tw is None:
                        continue
                    last_key_t = now
                    current = None if key in ('x', ' ') else (key, tw)
                    self.pub.publish(tw)
                    last_pub_t = now
                    self._print(
                        f'\r  [{key}] x={tw.linear.x:+.2f} y={tw.linear.y:+.2f} '
                        f'wz={math.degrees(tw.angular.z):+.0f}°/s  (세기 {self.speed:.2f})      ')
                    continue

                # 키 입력 없음: 떼면 정지, 누르고 있으면 재발행
                if current is not None:
                    if now - last_key_t > self.release_timeout:
                        current = None
                        self.pub.publish(stop)
                        self._print('\r  (released) 정지          ')
                    elif now - last_pub_t >= period:
                        self.pub.publish(current[1])
                        last_pub_t = now
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            try:
                self.pub.publish(stop)
            except Exception:
                pass
            self._print('\n')


def main(args=None):
    rclpy.init(args=args)
    node = HeadingTeleopNode()
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
