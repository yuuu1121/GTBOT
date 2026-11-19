#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_cmd_node
- String(serial_cmd) 토픽으로 받은 텍스트("1001", "x" 등)를 즉시 CP2102 시리얼로 전송
- ScheduledCmd(serial_cmd_sched) 토픽으로 받은 명령은 exec_time(시스템 시각)이 되면 전송
  -> 여러 Pi 가 chrony 로 시계를 맞춰두면 네트워크 지연과 무관하게 동시에 실행된다
- 실행 결과는 CmdAck(serial_cmd_ack) 로 발행 (목표 시각 대비 오차 ms + chrony 시계 오프셋 포함)
- ClockProbe(serial_cmd_probe) 를 받으면 즉시 ClockProbeReply 로 응답 (PC 가 Pi 시계차를 직접 재는 용도)
- 장치가 보내는 줄은 String(serial_rx) 로 발행

즉시 실행 예:
  ros2 topic pub --once /serial_cmd std_msgs/msg/String "{data: '1001'}"
"""

import re
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import String
from serial_cmd_msgs.msg import ScheduledCmd, CmdAck, ClockProbe, ClockProbeReply

import serial

EOLS = {'lf': '\n', 'crlf': '\r\n', 'cr': '\r', 'none': ''}

# 이 시간(초)만큼 남으면 sleep 을 멈추고 busy-wait 로 정밀하게 맞춘다
SPIN_MARGIN = 0.002


def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def float_to_stamp(t):
    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def local_ip():
    """기본 경로로 나가는 인터페이스의 IP. 실패하면 hostname."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))   # 실제 패킷은 안 나감
            return s.getsockname()[0]
    except OSError:
        return socket.gethostname()


_RE_SYSTIME = re.compile(r'System time\s*:\s*([0-9.]+) seconds (slow|fast) of NTP time')
_RE_REF = re.compile(r'Reference ID\s*:\s*\S+\s*\(([^)]*)\)')
_RE_STRATUM = re.compile(r'Stratum\s*:\s*(\d+)')
_RE_LEAP = re.compile(r'Leap status\s*:\s*(\S+)')


def read_chrony(timeout=1.0):
    """`chronyc tracking` 을 읽어 (synced, offset_ms, ref) 를 돌려준다.
    offset_ms = 이 시스템 시계 - 기준 시계 (+면 이 시계가 빠름).
    chrony 가 없거나 동기화 안 됐으면 (False, 0.0, 설명)."""
    try:
        out = subprocess.run(['chronyc', 'tracking'], capture_output=True, text=True,
                             timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError) as e:
        return False, 0.0, f'chronyc 실패: {e}'
    m = _RE_SYSTIME.search(out)
    leap = _RE_LEAP.search(out)
    if not m or not leap:
        return False, 0.0, 'chrony 응답 없음'
    val = float(m.group(1)) * 1000.0
    offset_ms = -val if m.group(2) == 'slow' else val
    ref = _RE_REF.search(out)
    stratum = _RE_STRATUM.search(out)
    ref_name = (ref.group(1) if ref else '').strip() or '?'
    ref_txt = f'{ref_name} s{stratum.group(1) if stratum else "?"}'
    # Stratum 0 / Reference ID 00000000 () 은 아직 아무 서버도 안 따르는 상태
    synced = leap.group(1) == 'Normal' and ref_name not in ('', '?') and \
        (stratum is None or int(stratum.group(1)) > 0)
    if not synced:
        return False, offset_ms, f'{ref_txt} leap={leap.group(1)}'
    return True, offset_ms, ref_txt


class SerialCmdNode(Node):

    def __init__(self):
        super().__init__('serial_cmd_node')

        # 파라미터
        self.port = self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ).value
        self.baud = self.declare_parameter('baud', 115200).value
        eol = self.declare_parameter('eol', 'lf').value
        cmd_topic = self.declare_parameter('cmd_topic', 'serial_cmd').value
        rx_topic = self.declare_parameter('rx_topic', 'serial_rx').value
        sched_topic = self.declare_parameter('sched_topic', 'serial_cmd_sched').value
        ack_topic = self.declare_parameter('ack_topic', 'serial_cmd_ack').value
        # 이 Pi 의 식별자. 비어 있으면 hostname
        self.robot_id = self.declare_parameter('robot_id', '').value or socket.gethostname()
        self.host_addr = local_ip()
        probe_topic = self.declare_parameter('probe_topic', 'serial_cmd_probe').value
        probe_reply_topic = self.declare_parameter(
            'probe_reply_topic', 'serial_cmd_probe_reply').value
        # exec_time 이 이미 지났을 때: 이 값(초) 이내로 늦었으면 경고 후 실행, 넘으면 드롭
        self.max_late_sec = float(self.declare_parameter('max_late_sec', 0.5).value)

        if eol not in EOLS:
            raise ValueError(f'eol 은 {list(EOLS)} 중 하나여야 합니다 (받음: {eol})')
        self.eol = EOLS[eol]

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except serial.SerialException as e:
            self.get_logger().fatal(f'시리얼 포트 {self.port} 열기 실패: {e}')
            raise

        self.write_lock = threading.Lock()
        self.rx_buf = b''
        self.rx_pub = self.create_publisher(String, rx_topic, 10)
        self.ack_pub = self.create_publisher(CmdAck, ack_topic, 10)
        self.subscription = self.create_subscription(
            String, cmd_topic, self.cmd_callback, 10)
        self.sched_sub = self.create_subscription(
            ScheduledCmd, sched_topic, self.sched_callback, 10)
        self.rx_timer = self.create_timer(0.02, self.rx_callback)
        # 시계 프로브: 받자마자 응답해야 하므로 콜백 안에서 바로 발행
        self.probe_pub = self.create_publisher(ClockProbeReply, probe_reply_topic, 50)
        self.probe_sub = self.create_subscription(
            ClockProbe, probe_topic, self.probe_callback, 50)

        synced, off, ref = read_chrony()
        self.get_logger().info(
            f'Serial Cmd Node 시작 ({self.port} @ {self.baud}, eol={eol!r}, '
            f'robot_id={self.robot_id}@{self.host_addr}, 즉시: {cmd_topic}, 예약: {sched_topic}, '
            f'ack: {ack_topic}, 수신 발행: {rx_topic})')
        if synced:
            self.get_logger().info(f'chrony: {ref}, 시계 오프셋 {off:+.2f}ms')
        else:
            self.get_logger().warn(f'chrony 동기화 안 됨 ({ref}) - 동시 실행 정확도 보장 불가')

    # ---------- 시리얼 쓰기 ----------

    def write_cmd(self, text):
        """한 줄 전송. 실패 시 예외."""
        with self.write_lock:
            self.ser.write((text + self.eol).encode('ascii'))
            self.ser.flush()

    # ---------- 즉시 실행 ----------

    def cmd_callback(self, msg):
        try:
            self.write_cmd(msg.data)
        except (serial.SerialException, UnicodeEncodeError) as e:
            self.get_logger().error(f'전송 실패 {msg.data!r}: {e}')
            return
        self.get_logger().info(f'>> {msg.data!r}')

    # ---------- 예약 실행 ----------

    def sched_callback(self, msg):
        if msg.targets and self.robot_id not in msg.targets:
            return  # 내 대상이 아님
        if not msg.cmds:
            return

        exec_t = stamp_to_float(msg.exec_time)
        remain = exec_t - time.time()
        if remain < -self.max_late_sec:
            reason = f'늦게 도착 ({-remain*1000:.0f}ms), 드롭'
            self.get_logger().warn(f'[{msg.seq_id}] {list(msg.cmds)} {reason}')
            self.publish_ack(msg, ok=False, error_ms=-remain * 1000, message=reason)
            return
        if remain < 0:
            self.get_logger().warn(
                f'[{msg.seq_id}] {list(msg.cmds)} {-remain*1000:.0f}ms 늦음, 즉시 실행')
        else:
            self.get_logger().info(
                f'[{msg.seq_id}] {list(msg.cmds)} 예약, {remain*1000:.0f}ms 후 실행')

        # 실행기 스레드를 막지 않도록 명령마다 전용 스레드에서 대기
        threading.Thread(
            target=self.run_scheduled, args=(msg, exec_t), daemon=True).start()

    def run_scheduled(self, msg, exec_t):
        self.wait_until(exec_t)
        fired = time.time()
        error_ms = (fired - exec_t) * 1000
        ok, message = True, ''
        try:
            for i, cmd in enumerate(msg.cmds):
                if i > 0 and msg.gap > 0:
                    self.wait_until(fired + i * msg.gap)
                self.write_cmd(cmd)
        except (serial.SerialException, UnicodeEncodeError) as e:
            ok, message = False, f'전송 실패: {e}'
            self.get_logger().error(f'[{msg.seq_id}] {message}')
        else:
            self.get_logger().info(
                f'>> [{msg.seq_id}] {list(msg.cmds)} 실행 (오차 {error_ms:+.2f}ms)')
        self.publish_ack(msg, ok=ok, error_ms=error_ms, message=message, chrony=read_chrony())

    @staticmethod
    def wait_until(t):
        """시스템 시각 t 까지 대기. 마지막 SPIN_MARGIN 은 busy-wait."""
        while True:
            remain = t - time.time()
            if remain <= 0:
                return
            if remain > SPIN_MARGIN:
                time.sleep(remain - SPIN_MARGIN)
            # else: spin

    def publish_ack(self, msg, ok, error_ms, message, chrony=None):
        ack = CmdAck()
        ack.robot_id = self.robot_id
        ack.host_addr = self.host_addr
        ack.seq_id = msg.seq_id
        ack.exec_time = msg.exec_time
        ack.error_ms = float(error_ms)
        ack.cmds = list(msg.cmds)
        ack.ok = ok
        ack.message = message
        synced, offset_ms, ref = chrony if chrony else (False, 0.0, '')
        ack.clock_synced = bool(synced)
        ack.clock_offset_ms = float(offset_ms)
        ack.clock_ref = ref
        self.ack_pub.publish(ack)

    # ---------- 시계 프로브 ----------

    def probe_callback(self, msg):
        t2 = time.time()
        reply = ClockProbeReply()
        reply.robot_id = self.robot_id
        reply.host_addr = self.host_addr
        reply.seq_id = msg.seq_id
        reply.t1 = msg.t1
        reply.t2 = float_to_stamp(t2)
        reply.t3 = float_to_stamp(time.time())
        self.probe_pub.publish(reply)

    # ---------- 시리얼 수신 ----------

    def rx_callback(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 0)
        except serial.SerialException as e:
            self.get_logger().error(f'수신 실패: {e}', throttle_duration_sec=5.0)
            return
        if not data:
            return
        self.rx_buf += data
        *lines, self.rx_buf = self.rx_buf.replace(b'\r', b'\n').split(b'\n')
        for line in lines:
            if line:
                text = line.decode(errors='replace')
                self.get_logger().info(f'<< {text!r}')
                self.rx_pub.publish(String(data=text))

    def shutdown(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
