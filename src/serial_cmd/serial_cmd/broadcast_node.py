#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
broadcast_node
터미널에 입력한 명령을 "실행 시각이 찍힌" ScheduledCmd 로 발행한다.
각 Pi 의 serial_cmd_node 는 자기 시계가 exec_time 이 되는 순간 시리얼로 쏘므로,
chrony 로 시계가 맞춰진 모든 Pi 가 네트워크 지연과 무관하게 동시에 동작한다.

명령마다 ack 를 모아 "정말 동시에 실행됐는지" 를 수치로 판정한다 (PASS / FAIL / UNSURE).
  실제 실행 시각 오차 = (Pi 가 보고한 전송 오차) - (그 Pi 시계의 기준 시계 대비 오프셋, chrony)
  편차 = max - min 이 sync_tolerance_ms 이내이고, 모든 Pi 가 응답했고, 시계가 동기화돼 있으면 PASS.
chrony 값이 없으면 PC 가 직접 잰 왕복 프로브(ClockProbe) 값으로 대략 판정한다 (UNSURE 가능).

입력 형식
  1001              -> 전체 Pi 에 lead_time 초 뒤 실행. (periodic_cmd 이면 resync 자동 시작)
  x                 -> 전체 Pi 에 lead_time 초 뒤 실행. (stop_cmd 이면 resync 자동 중지)
  @r1,r2 1001       -> robot_id 가 r1, r2 인 Pi 만
  blink [period]    -> blink_period(1s) 마다 blink_cmd("100") 를 전 Pi 에 동시 예약 전송 (한 번 깜빡임 반복).
                       STM32 자체 타이머 대신 Pi 시계로 매 깜빡임을 맞추므로 장치 간 주기 차이가 없다.
                       @r1,r2 blink 0.5 처럼 대상/주기 지정 가능. x 또는 blink off 로 중지
  resync on | off   -> 주기 재동기화 수동 시작/중지 (명령 전송 없음)
  probe             -> 각 Pi 시계와 PC 시계 차이를 왕복 프로브로 측정해 표로 출력
  tol <ms>          -> 판정 허용 편차 변경
  status            -> 현재 상태 (resync 횟수/다음 시각, 시계 표)
  q                 -> 종료

resync: periodic_cmd("1001") 를 보낸 뒤 resync_period(10s) 마다 resync_cmds(["x","1001"])
        를 resync_gap(1s) 간격으로 다시 보내 장치 간 깜빡임 위상 드리프트를 잡는다.
        즉 10초마다 모든 장치가 동시에 1초 꺼졌다가 동시에 다시 켜진다 (눈으로 확인 가능).
        매 resync 마다 "resync #n ... PASS/FAIL" 한 줄이 찍히므로 동작 여부를 바로 알 수 있다.
blink:  blink_report_every(10) 회마다 "blink #11~#20: PASS 10/10, 최대 편차 …" 요약 한 줄, FAIL 은 즉시 상세 출력.
"""

import socket
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from serial_cmd_msgs.msg import ScheduledCmd, CmdAck, ClockProbe, ClockProbeReply


def float_to_stamp(t):
    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def host_key(robot_id, addr):
    return f'{robot_id}@{addr}' if addr else robot_id


def ref_name(clock_ref):
    """'192.168.10.30 s4' -> '192.168.10.30'"""
    return clock_ref.split(' ')[0] if clock_ref else ''


class BroadcastNode(Node):

    def __init__(self):
        super().__init__('serial_cmd_broadcast')
        sched_topic = self.declare_parameter('sched_topic', 'serial_cmd_sched').value
        ack_topic = self.declare_parameter('ack_topic', 'serial_cmd_ack').value
        probe_topic = self.declare_parameter('probe_topic', 'serial_cmd_probe').value
        probe_reply_topic = self.declare_parameter(
            'probe_reply_topic', 'serial_cmd_probe_reply').value
        # 입력 후 실제 실행까지 여유 시간(초). 모든 Pi 에 메시지가 도착할 시간
        self.lead_time = float(self.declare_parameter('lead_time', 1.0).value)
        # 주기 깜빡임 명령 / 정지 명령
        self.periodic_cmd = self.declare_parameter('periodic_cmd', '1001').value
        self.stop_cmd = self.declare_parameter('stop_cmd', 'x').value
        # 재동기화
        self.resync_period = float(self.declare_parameter('resync_period', 10.0).value)
        self.resync_cmds = list(self.declare_parameter('resync_cmds', ['x', '1001']).value)
        self.resync_gap = float(self.declare_parameter('resync_gap', 1.0).value)
        self.resync_enabled = bool(self.declare_parameter('resync_enabled', True).value)
        # resync 판정을 매번 상세히 찍을지 (false 면 PASS 는 한 줄, 아니면 상세)
        self.resync_verbose = bool(self.declare_parameter('resync_verbose', False).value)
        # blink: Pi 시계로 매 깜빡임을 직접 쏘는 모드
        self.blink_cmd = self.declare_parameter('blink_cmd', '100').value
        self.blink_period = float(self.declare_parameter('blink_period', 1.0).value)
        self.blink_report_every = int(self.declare_parameter('blink_report_every', 10).value)
        # 판정
        self.sync_tolerance_ms = float(self.declare_parameter('sync_tolerance_ms', 5.0).value)
        self.ack_wait = float(self.declare_parameter('ack_wait', 0.5).value)
        # 시계 프로브
        self.probe_count = int(self.declare_parameter('probe_count', 8).value)
        self.probe_interval = float(self.declare_parameter('probe_interval', 0.05).value)
        self.probe_settle = float(self.declare_parameter('probe_settle', 0.3).value)

        self.pub = self.create_publisher(ScheduledCmd, sched_topic, 10)
        self.ack_sub = self.create_subscription(CmdAck, ack_topic, self.ack_callback, 50)
        self.probe_pub = self.create_publisher(ClockProbe, probe_topic, 50)
        self.probe_sub = self.create_subscription(
            ClockProbeReply, probe_reply_topic, self.probe_callback, 50)

        self.host = socket.gethostname()
        self.seq = 0
        self.seq_lock = threading.Lock()

        # 주기 전송 상태 (resync, blink 공용)
        self.periodic = {
            kind: {'thread': None, 'stop': threading.Event(), 'targets': [], 'count': 0,
                   'next_t': None, 'last_exec_t': 0.0, 'period': 0.0, 'cmds': [],
                   'stats': {'n': 0, 'pass': 0, 'max_spread': 0.0, 'first_k': 0}}
            for kind in ('resync', 'blink')}

        # 판정 대기 중인 명령: seq_id -> dict
        self.pending = {}
        self.pending_lock = threading.Lock()

        # 시계 프로브 결과: host_key -> dict(offset_ms, delay_ms, n, t)
        self.clock = {}
        self.probe_samples = {}   # burst_id -> {host_key: [(offset_ms, delay_ms), ...]}
        self.probe_lock = threading.Lock()
        self.probe_burst = 0
        self.probe_running = threading.Lock()

        self.tty = None
        self.get_logger().info(
            f'명령 입력 후 Enter -> {sched_topic} 발행 (lead_time={self.lead_time}s, '
            f'resync {self.resync_period}s {self.resync_cmds}, 판정 허용 편차 '
            f'{self.sync_tolerance_ms}ms, q: 종료)')

    # ---------- 발행 ----------

    def next_seq(self, prefix=''):
        with self.seq_lock:
            self.seq += 1
            return f'{prefix}{self.host}-{self.seq}'

    def wait_for_subscribers(self, timeout=2.0):
        """수신 노드가 아직 발견되지 않았으면 잠시 기다린다 (첫 명령 유실 방지)."""
        deadline = time.time() + timeout
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)
        n = self.pub.get_subscription_count()
        if n == 0:
            self.get_logger().warn('수신 노드(serial_cmd_node)가 하나도 발견되지 않음')
        return n

    def send(self, cmds, targets, exec_t, seq_prefix='', periodic=None, probe=True):
        """periodic: (kind, k) 주기 전송이면 지정. probe: 발송 뒤 시계 프로브를 돌릴지"""
        n_sub = self.wait_for_subscribers()
        msg = ScheduledCmd()
        msg.exec_time = float_to_stamp(exec_t)
        msg.cmds = list(cmds)
        msg.gap = self.resync_gap if len(cmds) > 1 else 0.0
        msg.targets = list(targets)
        msg.seq_id = self.next_seq(seq_prefix)
        # 판정용 등록은 발행 전에 (ack 가 먼저 올 수 없게)
        expected = len(targets) if targets else n_sub
        entry = {
            'cmds': list(cmds), 'exec_t': exec_t, 'targets': list(targets),
            'expected': expected, 'acks': {}, 'periodic': periodic, 'done': False,
        }
        # ack 는 마지막 명령까지 다 보낸 뒤에 오므로 gap 만큼 더 기다린다
        last_t = exec_t + msg.gap * (len(cmds) - 1)
        entry['timer'] = threading.Timer(
            max(0.0, last_t + self.ack_wait - time.time()), self.finalize, args=(msg.seq_id,))
        entry['timer'].daemon = True
        with self.pending_lock:
            self.pending[msg.seq_id] = entry
        entry['timer'].start()
        self.pub.publish(msg)
        who = ','.join(targets) if targets else '전체'
        quiet_blink = periodic and periodic[0] == 'blink' and not self.resync_verbose
        if not quiet_blink:
            self.get_logger().info(
                f'[{msg.seq_id}] {list(cmds)} -> {who} (수신 노드 {n_sub}개), '
                f'{exec_t - time.time():.2f}s 후 실행')
        # 실행을 기다리는 동안 각 Pi 시계차를 새로 잰다 (판정 보조 자료)
        if probe:
            self.start_probe_burst()
        return msg.seq_id

    # ---------- 주기 전송 (resync / blink) ----------

    def start_periodic(self, kind, first_exec_t, targets, cmds, period, prefix):
        self.stop_periodic(kind, quiet=True)
        st = self.periodic[kind]
        st.update({'stop': threading.Event(), 'targets': list(targets), 'count': 0,
                   'next_t': None, 'period': period, 'cmds': list(cmds),
                   'stats': {'n': 0, 'pass': 0, 'max_spread': 0.0, 'first_k': 0}})
        st['thread'] = threading.Thread(
            target=self.periodic_loop, args=(kind, first_exec_t, st['stop'], period, cmds, prefix),
            daemon=True)
        st['thread'].start()
        who = ','.join(targets) if targets else '전체'
        self.get_logger().info(f'{kind} 시작: {period}s 마다 {list(cmds)} -> {who}')

    def stop_periodic(self, kind, quiet=False):
        st = self.periodic[kind]
        if st['thread'] and st['thread'].is_alive():
            st['stop'].set()
            st['thread'].join(timeout=1.0)
            if not quiet:
                self.get_logger().info(f'{kind} 중지 (총 {st["count"]}회 실행)')
        st['thread'] = None
        st['next_t'] = None

    def periodic_running(self, kind):
        st = self.periodic[kind]
        return st['thread'] is not None and st['thread'].is_alive()

    def periodic_loop(self, kind, first_exec_t, stop_evt, period, cmds, prefix):
        st = self.periodic[kind]
        k = 0
        while not stop_evt.is_set():
            k += 1
            exec_t = first_exec_t + k * period
            st['next_t'] = exec_t
            # 실행 시각 lead_time 전에 발행
            wait = exec_t - self.lead_time - time.time()
            if wait > 0 and stop_evt.wait(wait):
                return
            if stop_evt.is_set():
                return
            st['count'] = k
            st['last_exec_t'] = exec_t
            # blink 는 매번 프로브하면 트래픽이 많으므로 요약 주기마다 한 번
            probe = kind != 'blink' or k % self.blink_report_every == 1
            self.send(cmds, st['targets'], exec_t, seq_prefix=prefix,
                      periodic=(kind, k), probe=probe)

    # 기존 이름 유지 (resync)
    def start_resync(self, first_exec_t, targets):
        self.start_periodic('resync', first_exec_t, targets, self.resync_cmds,
                            self.resync_period, 'rs-')

    def stop_resync(self, quiet=False):
        self.stop_periodic('resync', quiet)

    def start_blink(self, first_exec_t, targets, period=None):
        self.start_periodic('blink', first_exec_t, targets, [self.blink_cmd],
                            period or self.blink_period, 'bk-')

    def stop_all_periodic(self, quiet=False):
        for kind in self.periodic:
            self.stop_periodic(kind, quiet)

    def pending_periodic_end(self):
        """이미 발행돼 Pi 에서 대기 중인 주기 명령이 모두 끝나는 시각"""
        end = 0.0
        for st in self.periodic.values():
            if st['last_exec_t']:
                n = len(st['cmds'])
                end = max(end, st['last_exec_t'] + self.resync_gap * (n - 1) + 0.05)
        return end

    # ---------- ack / 판정 ----------

    def ack_callback(self, ack):
        key = host_key(ack.robot_id, ack.host_addr)
        state = 'OK' if ack.ok else 'FAIL'
        extra = f' ({ack.message})' if ack.message else ''
        if ack.clock_synced:
            clk = f', 시계 {ack.clock_offset_ms:+.2f}ms ({ack.clock_ref})'
        else:
            clk = f', chrony 미동기({ack.clock_ref})' if ack.clock_ref else ', chrony 정보 없음'
        with self.pending_lock:
            entry = self.pending.get(ack.seq_id)
            if entry is not None:
                entry['acks'][key] = ack
                complete = len(entry['acks']) >= entry['expected']
            else:
                complete = False
        quiet = entry is not None and entry['periodic'] and not self.resync_verbose
        if not quiet:
            self.get_logger().info(
                f'  ack [{ack.seq_id}] {key}: {state} 전송오차 {ack.error_ms:+.2f}ms{clk}{extra}')
        if complete:
            self.finalize(ack.seq_id)

    def finalize(self, seq_id):
        """ack 를 모아 동시 실행 여부를 판정해 출력한다. 한 번만 실행됨."""
        with self.pending_lock:
            entry = self.pending.get(seq_id)
            if entry is None or entry['done']:
                return
            entry['done'] = True
            entry['timer'].cancel()
            del self.pending[seq_id]
        try:
            verdict, spread, lines = self.judge(entry)
        except Exception as e:   # 판정 실패가 ack 스레드를 죽이지 않도록
            self.get_logger().error(f'[{seq_id}] 판정 중 오류: {e!r}')
            return
        head = f'[{seq_id}] {entry["cmds"]}'
        kind, k = entry['periodic'] or (None, 0)
        if kind:
            head = f'{kind} #{k} {head}'
        n_ack = len(entry['acks'])
        spread_txt = f'편차 {spread:.2f}ms' if spread is not None else '편차 ?'
        summary = (f'=== 판정 {head}  {verdict}  {spread_txt} (허용 {self.sync_tolerance_ms:g}ms)  '
                   f'응답 {n_ack}/{entry["expected"]}')
        if kind == 'blink' and not self.resync_verbose:
            # 1초마다 줄이 쏟아지지 않도록: PASS 는 모아서 요약, FAIL 은 즉시 상세
            st = self.periodic['blink']['stats']
            if st['n'] == 0:
                st['first_k'] = k
            st['n'] += 1
            st['pass'] += verdict == 'PASS'
            st['max_spread'] = max(st['max_spread'], spread or 0.0)
            if st['n'] >= self.blink_report_every:
                self.get_logger().info(
                    f'=== blink #{st["first_k"]}~#{k}: PASS {st["pass"]}/{st["n"]}, '
                    f'최대 편차 {st["max_spread"]:.2f}ms (허용 {self.sync_tolerance_ms:g}ms)')
                st.update({'n': 0, 'pass': 0, 'max_spread': 0.0, 'first_k': 0})
            if verdict == 'PASS':
                return
            text = '\n'.join([summary] + ['    ' + l for l in lines])
        elif kind and not self.resync_verbose and verdict == 'PASS':
            text = summary
        else:
            text = '\n'.join([summary] + ['    ' + l for l in lines])
        # rclpy 로거는 같은 호출 지점에서 severity 를 바꾸면 예외를 내므로 줄을 나눠 호출
        if verdict == 'PASS':
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def judge(self, entry):
        """(verdict, spread_ms, 설명 줄들). verdict: PASS | FAIL | UNSURE"""
        acks = list(entry['acks'].values())
        problems = []
        notes = []
        if len(acks) < entry['expected']:
            problems.append(f'응답 누락: {len(acks)}/{entry["expected"]} '
                            f'(구독자 수 기준. 구버전 serial_cmd_node 가 아직 떠 있으면 명령은 받고 ack 는 '
                            f'못 보냄 -> 각 Pi 에서 pkill -f serial_cmd_node 후 재실행. '
                            f'ros2 topic echo 도 구독자로 세어짐)')
        for a in acks:
            if not a.ok:
                problems.append(f'{host_key(a.robot_id, a.host_addr)} 실행 실패: {a.message}')
        # robot_id 중복 (다른 IP 인데 같은 이름) -> 어느 Pi 인지 구분 불가
        by_id = {}
        for a in acks:
            by_id.setdefault(a.robot_id, set()).add(a.host_addr)
        for rid, addrs in by_id.items():
            if len(addrs) > 1:
                notes.append(f'robot_id "{rid}" 중복 ({", ".join(sorted(addrs))}). '
                             f'receiver.launch.py robot_id:=r1 처럼 Pi 마다 다르게 주세요')

        # 시계 기준(frame) 정하기. 모든 Pi 가 같은 chrony 기준을 따르면 chrony 오프셋을 그대로 쓴다.
        # 한 Pi 가 다른 Pi 들의 기준 서버 자신이면 그 Pi 의 오프셋은 0 으로 본다.
        synced = [a for a in acks if a.clock_synced]
        refs = {ref_name(a.clock_ref) for a in synced}
        addrs = {a.host_addr for a in acks} | {a.robot_id for a in acks}
        server_keys = set()
        if len(refs) > 1:
            others = refs & addrs
            if len(others) == 1 and len(refs - others) == 1:
                srv = next(iter(others))
                server_keys = {host_key(a.robot_id, a.host_addr) for a in acks
                               if srv in (a.host_addr, a.robot_id)}
                notes.append(f'{srv} 가 chrony 서버 (다른 Pi 는 이 시계를 따름)')
            else:
                problems.append(
                    f'Pi 들이 서로 다른 기준 서버를 따름: '
                    + ', '.join(f'{host_key(a.robot_id, a.host_addr)}->{ref_name(a.clock_ref)}'
                                for a in synced)
                    + '. GTGUI 로 모두 같은 서버에 맞추세요 (왕복 프로브 값으로 대략 판정)')
        use_chrony = not any('기준 서버' in p for p in problems)

        rows = []
        corrected = []
        unsure = 0.0
        for a in acks:
            key = host_key(a.robot_id, a.host_addr)
            probe = self.clock.get(key)
            probe_txt = (f'왕복측정 {probe["offset_ms"]:+.1f}±{probe["delay_ms"]/2:.1f}ms'
                         if probe else '왕복측정 없음')
            if a.clock_synced and use_chrony:
                off = 0.0 if key in server_keys else a.clock_offset_ms
                src = f'chrony {a.clock_offset_ms:+.2f}ms ({a.clock_ref})'
                if probe and abs(probe['offset_ms'] - off) > probe['delay_ms'] / 2 + 3.0:
                    notes.append(f'{key}: chrony 오프셋과 왕복측정이 안 맞음 '
                                 f'({off:+.1f} vs {probe["offset_ms"]:+.1f}ms). chrony 확인')
            elif probe:
                off = probe['offset_ms']
                unsure = max(unsure, probe['delay_ms'] / 2)
                src = 'chrony 없음'
                if a.clock_ref:
                    src = f'chrony 미동기({a.clock_ref})'
            else:
                problems.append(f'{key} 시계차를 알 수 없음 (chrony 미동기 + 왕복측정 없음)')
                rows.append(f'{key:<28} 전송 {a.error_ms:+.2f}ms  시계 ?  ({probe_txt})')
                continue
            real = a.error_ms - off
            corrected.append(real)
            rows.append(f'{key:<28} 전송 {a.error_ms:+.2f}ms  시계 {src}, {probe_txt}'
                        f'  => 실제 {real:+.2f}ms')

        spread = (max(corrected) - min(corrected)) if len(corrected) >= 2 else (
            0.0 if len(corrected) == 1 else None)
        if spread is not None and spread > self.sync_tolerance_ms:
            problems.append(f'편차 {spread:.2f}ms 가 허용 {self.sync_tolerance_ms:g}ms 초과')
        elif spread is not None and spread + unsure > self.sync_tolerance_ms:
            notes.append(f'왕복측정 불확실성 ±{unsure:.1f}ms 때문에 확신 불가. '
                         f'chrony 를 맞추면 정확해짐')

        if problems:
            verdict = 'FAIL'
        elif unsure > 0 and spread is not None and spread + unsure > self.sync_tolerance_ms:
            verdict = 'UNSURE'
        elif not acks:
            verdict = 'FAIL'
        else:
            verdict = 'PASS'
        lines = rows + [f'! {p}' for p in problems] + [f'* {n}' for n in notes]
        return verdict, spread, lines

    # ---------- 시계 프로브 ----------

    def start_probe_burst(self, report=False):
        """백그라운드로 프로브 1회(여러 발) 실행. 이미 실행 중이면 건너뜀."""
        if not self.probe_running.acquire(blocking=False):
            return False
        t = threading.Thread(target=self._probe_burst, args=(report,), daemon=True)
        t.start()
        return True

    def probe_burst_blocking(self, report=True):
        if not self.probe_running.acquire(blocking=True, timeout=3.0):
            return
        self._probe_burst(report)

    def _probe_burst(self, report):
        try:
            self.probe_burst += 1
            burst = f'pb{self.probe_burst}'
            with self.probe_lock:
                self.probe_samples[burst] = {}
            for i in range(self.probe_count):
                msg = ClockProbe()
                msg.seq_id = f'{burst}-{i}'
                msg.t1 = float_to_stamp(time.time())
                self.probe_pub.publish(msg)
                time.sleep(self.probe_interval)
            time.sleep(self.probe_settle)
            with self.probe_lock:
                samples = self.probe_samples.pop(burst, {})
            now = time.time()
            for key, lst in samples.items():
                best = min(lst, key=lambda s: s[1])    # 왕복 지연이 가장 짧은 표본이 가장 정확
                self.clock[key] = {'offset_ms': best[0], 'delay_ms': best[1],
                                   'n': len(lst), 't': now}
            if report:
                self.get_logger().info(self.clock_table(samples.keys() or self.clock.keys()))
        finally:
            self.probe_running.release()

    def probe_callback(self, r):
        t4 = time.time()
        t1, t2, t3 = stamp_to_float(r.t1), stamp_to_float(r.t2), stamp_to_float(r.t3)
        offset_ms = ((t2 - t1) + (t3 - t4)) / 2 * 1000.0   # Pi 시계 - PC 시계
        delay_ms = ((t4 - t1) - (t3 - t2)) * 1000.0
        burst = r.seq_id.rsplit('-', 1)[0]
        key = host_key(r.robot_id, r.host_addr)
        with self.probe_lock:
            samples = self.probe_samples.get(burst)
            if samples is not None:
                samples.setdefault(key, []).append((offset_ms, delay_ms))

    def clock_table(self, keys=None):
        keys = list(keys or self.clock.keys())
        if not keys:
            return '시계 프로브 응답 없음 (수신 노드가 없거나 구버전)'
        lines = ['Pi 시계 - PC 시계 (왕복 프로브, ±는 왕복지연/2 = 최대 오차):']
        for key in sorted(keys):
            c = self.clock.get(key)
            if not c:
                lines.append(f'  {key:<28} 응답 없음')
                continue
            lines.append(f'  {key:<28} {c["offset_ms"]:+7.2f}ms ±{c["delay_ms"]/2:.1f}ms '
                         f'(표본 {c["n"]}, {time.time() - c["t"]:.0f}s 전)')
        return '\n'.join(lines)

    # ---------- 입력 처리 ----------

    def handle_line(self, line):
        """True 를 돌려주면 종료."""
        if line == 'q':
            return True
        if line == 'status':
            parts = []
            for kind, st in self.periodic.items():
                if self.periodic_running(kind):
                    parts.append(f'{kind}=ON (period {st["period"]}s, cmds {st["cmds"]}, '
                                 f'targets {st["targets"] or "전체"}, 지금까지 {st["count"]}회, '
                                 f'다음 {st["next_t"] - time.time():.1f}s 후)')
                else:
                    parts.append(f'{kind}=OFF')
            self.get_logger().info(
                f'lead_time={self.lead_time}s, 허용 편차 {self.sync_tolerance_ms:g}ms, '
                f'수신 노드 {self.pub.get_subscription_count()}개, '
                + ', '.join(parts) + '\n' + self.clock_table())
            return False
        if line == 'probe':
            self.probe_burst_blocking(report=True)
            return False
        if line.startswith('tol'):
            try:
                self.sync_tolerance_ms = float(line[3:].strip())
                self.get_logger().info(f'판정 허용 편차 = {self.sync_tolerance_ms:g}ms')
            except ValueError:
                self.get_logger().warn('사용법: tol <ms>')
            return False
        if line.startswith('resync'):
            arg = line[len('resync'):].strip()
            if arg == 'off':
                self.stop_resync()
            elif arg == 'on':
                self.start_resync(time.time() + self.lead_time, self.periodic['resync']['targets'])
            else:
                self.get_logger().warn('사용법: resync on | resync off')
            return False

        targets = []
        if line.startswith('@'):
            head, _, rest = line.partition(' ')
            targets = [t for t in head[1:].split(',') if t]
            line = rest.strip()
            if not line:
                self.get_logger().warn('사용법: @r1,r2 <명령>')
                return False

        if line.startswith('blink'):
            arg = line[len('blink'):].strip()
            if arg == 'off':
                self.stop_periodic('blink')
                return False
            try:
                period = float(arg) if arg else self.blink_period
            except ValueError:
                self.get_logger().warn('사용법: blink [주기초] | blink off  (예: blink 0.5)')
                return False
            if period < 0.05:
                self.get_logger().warn('주기가 너무 짧음 (0.05s 이상)')
                return False
            # 장치 자체 깜빡임(1001)과 겹치지 않게 resync 는 멈춤
            self.stop_resync(quiet=True)
            # 첫 깜빡임은 lead_time 뒤, 이후 period 마다 (periodic_loop 가 k=1 부터 더하므로 period 를 뺀다)
            self.start_blink(time.time() + self.lead_time - period, targets, period)
            return False

        if not line.isascii():
            self.get_logger().warn(f'{line!r}: ASCII 가 아닌 문자는 시리얼로 보낼 수 없음 (한글 입력 상태?)')
            return False

        exec_t = time.time() + self.lead_time
        if line == self.stop_cmd:
            # 먼저 주기 전송을 멈추고, 이미 발행돼 Pi 에서 대기 중인 것이 있으면
            # 그것이 끝난 뒤에 정지가 실행되도록 시각을 뒤로 민다 (x 뒤에 1001 이 와서 다시 켜지는 것 방지)
            self.stop_all_periodic()
            after = self.pending_periodic_end()
            if after > exec_t:
                self.get_logger().info(f'대기 중인 주기 명령 뒤로 정지 시각 조정 (+{after - exec_t:.2f}s)')
                exec_t = after
        self.send([line], targets, exec_t)

        if line == self.periodic_cmd and self.resync_enabled:
            self.start_resync(exec_t, targets)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = BroadcastNode()
    node.tty = open('/dev/tty', 'r')
    # ack 수신을 위해 별도 스레드에서 spin, 메인 스레드는 터미널 입력
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            line = node.tty.readline()
            if not line:
                break
            line = line.strip()
            if line and node.handle_line(line):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all_periodic(quiet=True)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
