#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cascade_controller_node
헤딩 유지용 캐스케이드 PID + 추력 믹서.

  heading_sp ─▶ [외부: 헤딩 PID] ─▶ yaw_rate_sp ─▶ [내부: 요레이트 PID] ─▶ N(yaw)
                  입력: imu/data_filtered yaw        입력: imu/data_raw gyro z
  cmd_vel.linear.x ─▶ X(surge),  cmd_vel.linear.y ─▶ Y(sway)
  [X, Y, N] ─▶ 믹서(접선형 + 배치) ─▶ directions 부호 ─▶ thruster_pwm_cmd ─▶ pwm_node

좌표계는 ROS 표준(FLU): X+ 전진, Y+ 좌측, yaw+ 반시계(좌회전).
cmd_vel.angular.z 는 헤딩 유지 ON 이면 목표 헤딩을 돌리는 속도(rad/s),
OFF 이면 개루프 yaw 명령(manual_yaw_gain 배)으로 쓴다.

안전:
  - cmd_vel 이 cmd_timeout 동안 없으면 X=Y=0 (헤딩 유지는 계속)
  - IMU 가 imu_timeout 동안 없으면 전 채널 0
  - 믹서 출력이 max_cmd 를 넘으면 전 채널 같은 비율로 축소 (yaw 비율 유지)
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger


def wrap_pi(a):
    """각도를 [-pi, pi) 로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    """geometry_msgs/Quaternion -> yaw(rad), ZYX."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PID:
    """적분 클램프(anti-windup) + 출력 제한을 가진 단순 PID.
    derivative 는 외부에서 측정값 미분(예: 자이로)을 넘겨줄 수 있다."""

    def __init__(self, kp, ki, kd, out_limit, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = abs(out_limit)
        self.i_limit = abs(i_limit) if i_limit is not None else self.out_limit
        self.integral = 0.0
        self.prev_err = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def update(self, err, dt, d_meas=None):
        """err: 오차, dt: 초, d_meas: 측정값의 시간미분(주면 d(err)/dt = -d_meas 로 사용)."""
        if dt <= 0.0:
            dt = 1e-3
        # 적분 (클램프)
        self.integral += err * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        # 미분
        if d_meas is not None:
            d_err = -d_meas
        elif self.prev_err is None:
            d_err = 0.0
        else:
            d_err = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * d_err
        return max(-self.out_limit, min(self.out_limit, out))


class CascadeControllerNode(Node):

    def __init__(self):
        super().__init__('cascade_controller')
        p = self.declare_parameter

        # ---------- 루프 / 토픽 ----------
        self.control_rate = p('control_rate', 50.0).value          # 내부 루프 (Hz)
        self.outer_divider = int(p('outer_divider', 5).value)      # 외부 루프 = control_rate / divider
        cmd_vel_topic = p('cmd_vel_topic', 'cmd_vel').value
        imu_filtered_topic = p('imu_filtered_topic', 'imu/data_filtered').value
        imu_raw_topic = p('imu_raw_topic', 'imu/data_raw').value
        out_topic = p('cmd_topic', 'thruster_pwm_cmd').value
        self.cmd_timeout = p('cmd_timeout', 0.6).value
        self.imu_timeout = p('imu_timeout', 0.5).value
        self.init_settle_sec = p('init_settle_sec', 3.0).value   # madgwick 수렴 대기 후 초기 목표 헤딩 설정

        # ---------- 외부 루프: 헤딩 -> 요레이트 ----------
        self.yaw_rate_max = math.radians(p('yaw_rate_max_deg', 30.0).value)
        self.heading_pid = PID(
            p('heading_kp', 1.5).value,       # rad/s per rad
            p('heading_ki', 0.0).value,
            p('heading_kd', 0.0).value,
            out_limit=self.yaw_rate_max,
            i_limit=math.radians(p('heading_i_limit_deg', 10.0).value))
        self.heading_deadband = math.radians(p('heading_deadband_deg', 0.5).value)

        # ---------- 내부 루프: 요레이트 -> N ----------
        self.yaw_cmd_max = p('yaw_cmd_max', 0.4).value             # N 한계 (cmd 단위)
        self.rate_pid = PID(
            p('rate_kp', 0.6).value,          # cmd per rad/s
            p('rate_ki', 0.3).value,
            p('rate_kd', 0.0).value,
            out_limit=self.yaw_cmd_max,
            i_limit=p('rate_i_limit', 0.2).value)
        self.rate_ff = p('rate_ff', 0.0).value                     # 요레이트 피드포워드 (cmd per rad/s)
        self.manual_yaw_gain = p('manual_yaw_gain', 0.3).value     # 헤딩유지 OFF 때 angular.z -> N
        self.hold_enabled = bool(p('hold_enabled', True).value)
        # 발산 감지 (yaw 부호가 반대면 오차가 줄지 않고 계속 커진다)
        self.diverge_err = math.radians(p('diverge_err_deg', 60.0).value)
        self.diverge_sec = p('diverge_sec', 2.0).value
        self.t_diverge = None
        # yaw 부호 자동 캘리브레이션: 채널 하나씩 +명령 -> 자이로 z 부호 측정 -> yaw_mix[i]
        self.yawcal_cmd = p('yawcal_cmd', 0.12).value          # 테스트 세기 (물리 +, directions 적용 전)
        self.yawcal_sec = p('yawcal_sec', 1.5).value           # 채널당 구동 시간
        self.yawcal_settle_sec = p('yawcal_settle_sec', 3.0).value   # 채널 사이 최소 정지 대기
        self.yawcal_settle_max = p('yawcal_settle_max_sec', 12.0).value  # 최대 대기 (그래도 안 멈추면 진행)
        self.yawcal_quiet_deg = p('yawcal_quiet_deg', 3.0).value   # |gyro| 가 이 아래로 1 s 유지되면 정지로 판단
        self.yawcal_min_rate_deg = p('yawcal_min_rate_deg', 1.0).value  # 이보다 작으면 판정 불가
        self.cal = None   # None 이면 비활성, 아니면 dict(ch, phase, t0, samples)
        self.cal_pending = False   # IMU 준비 전에 요청되면 준비 즉시 시작

        # ---------- 믹서 ----------
        # 접선형 + 배치: ch1 좌측팔↑, ch2 우측팔↑, ch3 앞팔(횡), ch4 뒤팔(횡)
        # 전기 양(+) 명령(directions 적용 후) = ch1·2 전진 추력, ch3·4 좌측 추력 (실측).
        self.surge_mix = list(p('surge_mix', [1.0, 1.0, 0.0, 0.0]).value)
        self.sway_mix = list(p('sway_mix', [0.0, 0.0, 1.0, 1.0]).value)     # Y+ = 좌측 (ch3·4 전기 + = 좌측 추력, 실측)
        self.yaw_mix = list(p('yaw_mix', [-1.0, 1.0, 1.0, -1.0]).value)     # N+ = 반시계. 임시값 — calibrate_yaw_mix 로 측정해 확정
        self.directions = list(p('directions', [1.0, -1.0, 1.0, -1.0]).value)  # 배선/프로펠러 부호
        self.max_cmd = p('max_cmd', 0.5).value                     # 채널 출력 한계 (pwm_node 와 동일하게)
        self.xy_max = p('xy_max', 0.4).value                       # 조종 X/Y 한계
        self.deadband = p('deadband', 0.0).value                   # 시동 데드존 보정 (cmd 단위, 0=끔)
        self.deadband_min_cmd = p('deadband_min_cmd', 0.02).value  # 이보다 작은 명령은 0 (릴레이 채터링 방지)
        self.reverse_gain = p('reverse_gain', 1.0).value           # 역추력 보정 (1.6 ≈ 60% 보정)
        n = 4
        for name, v in (('surge_mix', self.surge_mix), ('sway_mix', self.sway_mix),
                        ('yaw_mix', self.yaw_mix), ('directions', self.directions)):
            if len(v) != n:
                raise ValueError(f'{name} 는 {n}개여야 합니다 (받음: {v})')

        # ---------- 상태 ----------
        self.yaw = None
        self.gyro_z = 0.0
        self.t_imu = 0.0
        self.t_imu_filt = 0.0
        self.heading_sp = None
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.wz_cmd = 0.0
        self.t_cmd = 0.0
        self.yaw_rate_sp = 0.0
        self.tick = 0
        self.last_out = None
        self.imu_lost_logged = False
        self.t_first_imu = None

        # ---------- ROS I/O ----------
        self.pub = self.create_publisher(Float32MultiArray, out_topic, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '~/debug', 10)
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(Imu, imu_filtered_topic, self.on_imu_filtered, 20)
        self.create_subscription(Imu, imu_raw_topic, self.on_imu_raw, 20)
        self.create_service(SetBool, '~/enable_hold', self.srv_enable_hold)
        self.create_service(Trigger, '~/set_heading_here', self.srv_set_heading_here)
        self.create_service(Trigger, '~/calibrate_yaw_mix', self.srv_calibrate_yaw_mix)

        self.dt = 1.0 / float(self.control_rate)
        self.timer = self.create_timer(self.dt, self.control_step)

        self.get_logger().info(
            f'cascade_controller 시작 (내부 {self.control_rate:.0f}Hz, 외부 '
            f'{self.control_rate / self.outer_divider:.0f}Hz, 헤딩유지={self.hold_enabled}, '
            f'yaw_rate_max={math.degrees(self.yaw_rate_max):.0f}°/s, max_cmd={self.max_cmd})')

    # ------------------------------------------------------------------ #
    # 콜백
    # ------------------------------------------------------------------ #
    def on_cmd_vel(self, msg):
        self.x_cmd = max(-self.xy_max, min(self.xy_max, msg.linear.x))
        self.y_cmd = max(-self.xy_max, min(self.xy_max, msg.linear.y))
        self.wz_cmd = msg.angular.z
        self.t_cmd = time.monotonic()

    def on_imu_filtered(self, msg):
        self.yaw = yaw_from_quat(msg.orientation)
        self.t_imu_filt = time.monotonic()
        if self.t_first_imu is None:
            self.t_first_imu = self.t_imu_filt
        # 필터 수렴 대기 후 초기 목표 헤딩 = 현재 yaw (그 전엔 출력 0)
        if self.heading_sp is None and self.t_imu_filt - self.t_first_imu >= self.init_settle_sec:
            self.heading_sp = self.yaw
            self.get_logger().info(f'초기 목표 헤딩 = 현재 yaw {math.degrees(self.yaw):.1f}°')

    def on_imu_raw(self, msg):
        self.gyro_z = msg.angular_velocity.z
        self.t_imu = time.monotonic()

    def srv_enable_hold(self, req, res):
        self.hold_enabled = bool(req.data)
        if self.hold_enabled and self.yaw is not None:
            self.heading_sp = self.yaw
        self.heading_pid.reset()
        self.rate_pid.reset()
        self.t_diverge = None
        res.success = True
        res.message = f'헤딩 유지 {"ON (목표=현재 헤딩)" if self.hold_enabled else "OFF (수동 yaw)"}'
        self.get_logger().info(res.message)
        return res

    def srv_set_heading_here(self, req, res):
        if self.yaw is None:
            res.success = False
            res.message = 'IMU 수신 전'
            return res
        self.heading_sp = self.yaw
        self.heading_pid.reset()
        res.success = True
        res.message = f'목표 헤딩 = {math.degrees(self.heading_sp):.1f}°'
        self.get_logger().info(res.message)
        return res

    def srv_calibrate_yaw_mix(self, req, res):
        """채널 하나씩 물리 + 명령을 주고 자이로 z 부호로 yaw_mix 를 측정한다 (비동기, 약 15 s).
        정지한 수중에서 실행. 헤딩 유지는 자동 OFF 되며 끝나면 t 로 다시 켠다."""
        if self.cal is not None:
            res.success = False
            res.message = '이미 캘리브레이션 중'
            return res
        if self.yaw is None or self.heading_sp is None:
            self.cal_pending = True
            self.hold_enabled = False
            res.success = True
            res.message = 'IMU 준비 전 — 준비되는 즉시 캘리브레이션 자동 시작 (로봇 정지 유지)'
            self.get_logger().warn(res.message)
            return res
        self.cal_start()
        res.success = True
        res.message = (f'yaw_mix 캘리브레이션 시작: 채널별 +{self.yawcal_cmd} x {self.yawcal_sec}s, '
                       '헤딩 유지 OFF. 로봇 정지 상태 유지!')
        return res

    def cal_start(self):
        self.cal_pending = False
        self.hold_enabled = False
        self.heading_pid.reset()
        self.rate_pid.reset()
        self.cal = {'ch': 0, 'phase': 'settle', 't0': time.monotonic(), 'samples': [],
                    'bias': [], 'result': [None] * 4}
        self.get_logger().warn(
            f'yaw_mix 캘리브레이션 시작: 채널별 +{self.yawcal_cmd} x {self.yawcal_sec}s, '
            '헤딩 유지 OFF. 로봇 정지 상태 유지!')

    def cal_step(self, now):
        """캘리브레이션 상태 머신. 발행할 4채널 물리 명령을 돌려준다."""
        c = self.cal
        el = now - c['t0']
        u = [0.0] * 4
        if c['phase'] == 'settle':
            # 자이로가 quiet 미만으로 1 s 이어질 때까지 대기 (관성 회전 소멸), 그동안 바이어스 샘플
            if abs(self.gyro_z) > math.radians(self.yawcal_quiet_deg):
                c['t_loud'] = now
                c['bias'] = []
            else:
                c['bias'].append(self.gyro_z)
            quiet = now - c.get('t_loud', c['t0']) >= 1.0
            if (el >= self.yawcal_settle_sec and quiet) or el >= self.yawcal_settle_max:
                if not quiet:
                    self.get_logger().warn(
                        f'  {self.yawcal_settle_max:.0f}s 지나도 회전이 안 멈춤 (|gyro| '
                        f'{math.degrees(abs(self.gyro_z)):.0f}°/s) — 다음 채널 측정 신뢰도 낮음')
                    c['bias'] = [self.gyro_z]
                if c['ch'] >= 4:
                    self.cal_finish()
                    return u
                c['phase'] = 'run'
                c['t0'] = now
                c['samples'] = []
                self.get_logger().info(f'  ch{c["ch"] + 1} 구동 중...')
        elif c['phase'] == 'run':
            u[c['ch']] = self.yawcal_cmd
            if el > self.yawcal_sec * 0.4:          # 가속 구간 지난 뒤 샘플
                c['samples'].append(self.gyro_z)
            if el >= self.yawcal_sec:
                bias = sum(c['bias']) / len(c['bias']) if c['bias'] else 0.0
                rate = (sum(c['samples']) / len(c['samples']) if c['samples'] else 0.0) - bias
                unreliable = abs(bias) > math.radians(self.yawcal_quiet_deg) or abs(rate) < 3 * abs(bias)
                c['result'][c['ch']] = None if unreliable else rate
                self.get_logger().info(
                    f'  ch{c["ch"] + 1}: 자이로 z 평균 {math.degrees(rate):+.2f}°/s '
                    f'(시작 전 잔류 회전 {math.degrees(bias):+.2f}°/s 제거)'
                    + ('  ← 잔류 회전이 커서 무효, 기존 부호 유지' if unreliable else ''))
                c['ch'] += 1
                c['phase'] = 'settle'
                c['t0'] = now
                c['bias'] = []
        return u

    def cal_finish(self):
        res = self.cal['result']
        self.cal = None
        thr = math.radians(self.yawcal_min_rate_deg)
        new_mix = []
        unclear = []
        for i, r in enumerate(res):
            if r is None or abs(r) < thr:
                new_mix.append(self.yaw_mix[i])
                unclear.append(i + 1)
            else:
                new_mix.append(1.0 if r > 0 else -1.0)
        self.yaw_mix = new_mix
        self.heading_pid.reset()
        self.rate_pid.reset()
        line = 'yaw_mix: [' + ', '.join(f'{v:+.1f}' for v in new_mix) + ']'
        self.get_logger().warn(f'yaw_mix 캘리브레이션 완료 → 적용됨. yaml 에 저장하세요:  {line}')
        if unclear:
            self.get_logger().error(
                f'ch{unclear} 측정 무효(반응 {self.yawcal_min_rate_deg}°/s 미만 또는 잔류 회전) — 기존 부호 유지. '
                '로봇이 완전히 멈춘 뒤 c 로 다시 실행')
        self.get_logger().info('헤딩 유지는 OFF 상태. 텔레옵 t 로 다시 켜면 목표=현재 헤딩')

    # ------------------------------------------------------------------ #
    # 제어 루프
    # ------------------------------------------------------------------ #
    def control_step(self):
        if not rclpy.ok():
            return
        now = time.monotonic()
        self.tick += 1

        # IMU 끊김 -> 전 채널 0
        imu_ok = (self.yaw is not None
                  and self.heading_sp is not None
                  and now - self.t_imu < self.imu_timeout
                  and now - self.t_imu_filt < self.imu_timeout)
        if not imu_ok:
            if not self.imu_lost_logged:
                self.get_logger().warn('IMU 수신 없음 — 전 채널 0 출력')
                self.imu_lost_logged = True
            self.heading_pid.reset()
            self.rate_pid.reset()
            self.publish([0.0] * 4)
            return
        if self.imu_lost_logged:
            self.get_logger().info('IMU 복구')
            self.imu_lost_logged = False
        if self.cal_pending and self.cal is None:
            self.cal_start()
        self.t_first_imu = None

        # 조종 입력 타임아웃 -> X=Y=0, 회전 입력 0
        cmd_ok = now - self.t_cmd < self.cmd_timeout
        x = self.x_cmd if cmd_ok else 0.0
        y = self.y_cmd if cmd_ok else 0.0
        wz = self.wz_cmd if cmd_ok else 0.0

        if self.cal is not None:
            # yaw 부호 캘리브레이션 중: 조종 입력 무시, 채널 하나씩 구동
            self.publish(self.mix_physical(self.cal_step(now)))
            return

        if self.hold_enabled:
            # angular.z 로 목표 헤딩을 돌린다 (키를 누른 동안 회전)
            if wz != 0.0:
                self.heading_sp = wrap_pi(self.heading_sp + wz * self.dt)
            # 발산 감지: 큰 오차가 diverge_sec 이상 지속되면 헤딩 유지 OFF
            err_now = abs(wrap_pi(self.heading_sp - self.yaw))
            if err_now > self.diverge_err:
                if self.t_diverge is None:
                    self.t_diverge = now
                elif now - self.t_diverge > self.diverge_sec:
                    self.hold_enabled = False
                    self.heading_pid.reset()
                    self.rate_pid.reset()
                    self.get_logger().error(
                        f'헤딩 오차 {math.degrees(err_now):.0f}° 가 {self.diverge_sec:.0f}s 이상 지속 — '
                        '발산으로 판단, 헤딩 유지 OFF. yaw_mix 부호 반대 가능성 (README "yaw 부호 검증")')
            else:
                self.t_diverge = None
            # 외부 루프 (분주)
            if self.hold_enabled and self.tick % self.outer_divider == 0:
                err = wrap_pi(self.heading_sp - self.yaw)
                if abs(err) < self.heading_deadband:
                    err = 0.0
                self.yaw_rate_sp = self.heading_pid.update(
                    err, self.dt * self.outer_divider, d_meas=self.gyro_z)
            # 내부 루프
            if self.hold_enabled:
                rate_err = self.yaw_rate_sp - self.gyro_z
                n = self.rate_pid.update(rate_err, self.dt) + self.rate_ff * self.yaw_rate_sp
                n = max(-self.yaw_cmd_max, min(self.yaw_cmd_max, n))
            else:
                n = 0.0   # 방금 발산으로 OFF 됨
        else:
            self.yaw_rate_sp = 0.0
            n = max(-self.yaw_cmd_max, min(self.yaw_cmd_max, self.manual_yaw_gain * wz))

        out = self.mix(x, y, n)
        self.publish(out)

        dbg = Float32MultiArray()
        dbg.data = [
            float(math.degrees(self.yaw)),
            float(math.degrees(self.heading_sp)),
            float(math.degrees(wrap_pi(self.heading_sp - self.yaw))),
            float(math.degrees(self.yaw_rate_sp)),
            float(math.degrees(self.gyro_z)),
            float(n), float(x), float(y),
        ]
        self.debug_pub.publish(dbg)

    def mix(self, x, y, n):
        """[X, Y, N] -> 4채널 물리 명령 -> 포화 축소 -> 데드존/역추력 보정 -> 배선 부호."""
        u = [self.surge_mix[i] * x + self.sway_mix[i] * y + self.yaw_mix[i] * n
             for i in range(4)]
        peak = max(abs(v) for v in u)
        if peak > self.max_cmd:
            s = self.max_cmd / peak
            u = [v * s for v in u]
        return self.mix_physical(u)

    def mix_physical(self, u):
        """4채널 물리 명령 -> 데드존/역추력 보정 -> 배선 부호 -> 전기 명령."""
        out = []
        for i, v in enumerate(u):
            if self.deadband > 0.0:
                if abs(v) < self.deadband_min_cmd:
                    v = 0.0
                else:
                    v = math.copysign(self.deadband + abs(v) * (1.0 - self.deadband), v)
            v *= self.directions[i]           # 배선/프로펠러 부호 (여기부터 전기적 부호)
            if v < 0.0:
                v *= self.reverse_gain        # ESC 역방향 추력 약함 보정
            out.append(max(-self.max_cmd, min(self.max_cmd, v)))
        return out

    def publish(self, values):
        values = [float(v) for v in values]
        # 변화 없으면 재발행 안 함 (pwm_node I2C/로그 부담 감소) — 단 1초마다 keepalive
        if values == self.last_out and self.tick % int(self.control_rate) != 0:
            return
        msg = Float32MultiArray()
        msg.data = values
        self.pub.publish(msg)
        self.last_out = values

    def shutdown(self):
        try:
            msg = Float32MultiArray()
            msg.data = [0.0] * 4
            self.pub.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CascadeControllerNode()
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
