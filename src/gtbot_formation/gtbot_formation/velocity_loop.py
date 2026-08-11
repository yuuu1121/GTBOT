import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import setpoints, world_to_body, yaw_of, wrap

class VelocityLoop(Node):
    def __init__(self):
        super().__init__('velocity_loop')
        # kpsi: yaw rate 플랜트 G≈12 rad/s/unit, 지연 τ≈0.5 s 실측 -> ζ≈0.7 되는 값이 0.085.
        # fix round 2: k_cf 기본 0.1->0.05 하향 — 진단(round2_log)에서 3.6m 스폰의
        # yaw_meas가 valid==True여도 평균 40~50° 오차(aux 포인트 부족으로 노이즈 큰 센트로이드)로
        # 확인됨. 노이즈 큰 측정에 덜 끌리도록 자이로 적분 비중을 높인다. kd: 자이로 rate 댐핑
        # (신규, bearing 전용 — hold 경로 무영향).
        for name, default in [('robot', 'gtbot'), ('kv', 1.5), ('kpsi', 0.1),
                              ('v_max', 0.5), ('rate', 20.0), ('ki', 1.0),
                              ('i_max', 0.6), ('log_csv', ''),
                              ('heading_mode', 'hold'), ('k_cf', 0.05), ('kd', 0.1),
                              ('yaw_source', 'odom'),
                              # odom_source='est'(2026-08-10, 플랫폼 중앙집중 설계):
                              # 위치·속도 피드백을 odometry가 아니라 플랫폼 LiDAR
                              # 추정(state_est의 상대위치·상대속도)에서 받는다 —
                              # 실기에서 로봇은 자기 위치 센서가 없고 플랫폼이
                              # 하행 전송하는 est가 유일한 소스. 홀드·v_ref가
                              # 플랫폼 상대 프레임이 되어 편대 유지 의미로도 정합.
                              # 'odom'은 구 캠페인 호환.
                              ('odom_source', 'odom'),
                              # k_yaw_off(2026-08-10): yaw = imu_yaw + yaw_off 의 오프셋 EMA 이득
                              # (유효 est 헤딩 표본당, ~10 Hz -> tau≈5 s). IMU 드리프트를 LiDAR
                              # 절대 헤딩으로 저주파 보정 + 실기 자북↔플랫폼 프레임 오프셋 자동
                              # 정렬(이게 없으면 실기는 베어링 목표와 IMU yaw 의 기준 프레임이
                              # 달라 수동 캘리브레이션 필수). 0.005 @10 Hz ≈ tau 20 s — 검출
                              # 상실 동역학(수 초)보다 훨씬 느려 '오염 측정→제어→검출 붕괴'
                              # 폐루프 결합을 끊는다(0.02 런 실측: 요 스윙 91~123°, S6 붕괴).
                              ('k_yaw_off', 0.005),
                              # est_vel_alpha(2026-08-10): rel_vel(KF 상대속도) EMA — est 잡음이
                              # 폐루프로 재유입(est->e_world->추력 지터, 실측 setpoint diff-std
                              # 0.023~0.028 = syaw 캡의 절반)돼 판 헤딩 적합을 열화시키는 것의
                              # 완화. 0.35 @10 Hz ≈ tau 0.3 s(플랜트 지연 0.5 s 아래).
                              ('est_vel_alpha', 0.35),
                              # e_deadband(2026-08-10): 속도오차가 이 값(m/s) 미만이면 병진
                              # 추력 침묵(요 제어는 유지) — est 잡음 크기의 오차를 쫓는
                              # 추력 지터가 정지유지 중 선체를 흔들어 판 헤딩 적합을
                              # 열화시키는 것의 차단. 0=비활성.
                              ('e_deadband', 0.0),
                              # vref_tau(2026-08-11): v_ref를 실측 속도로 되끄는 시상수(s).
                              # 0 = 기존 순수 적분기(비트 동일). 순수 적분 v_ref는 플랜트가
                              # 못 따라가면 실측 속도와 위상이 벌어지고(실측 ∠(v,v_ref)=
                              # 100~107°), 그 상태에서 a_cmd의 '감속' 지령은 v_ref를 줄이는
                              # 대신 **회전**시킨다 — v_ref가 52~67°/s로 돌고 로봇이 그걸
                              # 쫓아 공전한다(주기 5~7 s). 되끌면 정상상태가
                              # v_ref = v + a_cmd·tau 라 가속 지령이 곧 속도오차가 되어
                              # 감속 지령이 실제 감속으로 도달한다.
                              ('vref_tau', 0.0),
                              ('fallback_bearing', float('nan'))]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.kv, self.kpsi, self.v_max = p('kv'), p('kpsi'), p('v_max')
        self.ki, self.i_max = p('ki'), p('i_max')
        self.dt = 1.0 / p('rate')
        robot = p('robot')
        self.heading_mode, self.k_cf, self.kd = p('heading_mode'), p('k_cf'), p('kd')
        self.e_db, self.vref_tau = p('e_deadband'), p('vref_tau')
        self.fallback_bearing = float(p('fallback_bearing'))
        # yaw_source='imu'(2026-08-10): 헤딩 피드백을 odometry가 아니라 로봇 자체
        # IMU(AHRS 절대 yaw)에서 받는다 — 실기 hwt9053과 동일 의미론(시뮬 IMU도
        # 절대 자세 발행, odom과 0.1° 일치 실측). 'odom'은 구 캠페인 호환 기본값.
        from sensor_msgs.msg import Imu
        self.yaw_source = p('yaw_source')
        self.odom_source = p('odom_source')
        if self.odom_source == 'est' and self.heading_mode != 'bearing':
            raise ValueError("odom_source='est'는 bearing 모드(est 구독) 전용")
        self.imu_yaw = None
        self.t_imu = None
        self.create_subscription(Imu, f'/{robot}/imu', self.on_imu, 50)
        if self.heading_mode == 'bearing':
            from .mixer import cf_update  # noqa: 사용은 on_imu에서
            self.k_yaw_off, self.est_vel_alpha = p('k_yaw_off'), p('est_vel_alpha')
            self.yaw_off = None   # 초기화 포함 모든 갱신은 '스트릭 3 s + 준정지' 게이트 뒤
            #                       (on_est 참조) — 부트스트랩 원거리 표본(40~50° 오차 실측)
            #                       으로 초기화하면 로봇이 그만큼 돌아 검출이 죽고 45° 게이트가
            #                       교정을 막는 자기잠금이 된다(0.02 런 실측 붕괴).
            self.rel = None
            self.rel_vel = None
            self.yaw_meas = 0.0
            self.meas_valid = False
            self.yaw_hat = None
            self.t_cf = None
            self.t_est = None
            self.valid_since = None   # 연속 유효 스트릭 시작(0.5 s 끊기면 리셋) — bearing 진입 게이트
            self.yaw_ref_track = None  # 슬루잉 기준(_syaw_track) — 첫 사용 시 현재 자세로 초기화
            self.gyro_z = 0.0
            self.create_subscription(Float64MultiArray, f'/{robot}/state_est', self.on_est, 10)
            # 부트스트랩 폴백용 platform odometry(시뮬 한정 단순화의 연장 — 실물에서는
            # 운용자가 배치 시 로봇을 platform 쪽으로 지향시키는 것에 대응). est 성립
            # 후에는 bearing 분기(추정 기반)가 우선하므로 출력피드백 주장에는 부트스트랩
            # 구간만 관여한다. 스폰 상수 베어링은 platform이 로딩 중 표류하면 부정확해져
            # 런별 지향 품질 편차(부트 발화 복불복)를 만들었다(실측).
            self.plat_odom = None
            self.create_subscription(Odometry, '/platform/odometry', self.on_plat, 10)
        self.v_ref = np.zeros(2)
        self.a_cmd = np.zeros(2)
        self.ei = np.zeros(2)
        self.odom = None            # (pos2, v_world2, yaw)
        self.yaw0 = None
        self.t_odom = self.t_acc = None
        self.pos_hold = None              # 두절 위치 홀드 앵커(두절 진입 시 캡처)
        path = p('log_csv')
        self.log = open(path, 'w', buffering=1) if path else None
        if self.log:
            self.log.write('t,vx,vy,yaw,vrefx,vrefy,ax,ay,eix,eiy,s0,s1,s2,s3\n')
        self.create_subscription(Odometry, f'/{robot}/odometry', self.on_odom, 10)
        self.create_subscription(Float64MultiArray, f'/{robot}/accel_cmd', self.on_acc, 10)
        self.pub = self.create_publisher(Float64MultiArray, f'/{robot}/thrusters', 10)
        self.create_timer(self.dt, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _syaw_track(self, yaw, target):
        """대각 오차 획득의 한계 사이클 제거(2026-08-10 근본 수정): 명령 clip ±0.05 +
        kd·gyro_z 감쇠는 자이로 피드백 이득(G≈12 × kd=0.1)이 작동 지연 0.5 s와 결합해
        그 자체로 발진 구동이 된다(실측: 전 로봇 영구 자전, |wrap 오차| 중앙값 90° —
        판이 모서리로 서서 검출 사망). 대신 내부 기준 yaw_ref_track을 0.3 rad/s로
        목표까지 굴리고, 플랜트는 항상 소신호 오차만 추종한다 — 검증된 hold 모드
        P 루프(kpsi, ζ≈0.7)를 그대로 쓰는 기준 슬루잉."""
        if self.yaw_ref_track is None:
            self.yaw_ref_track = yaw                     # 현재 자세에서 출발(점프 없음)
        d = wrap(target - self.yaw_ref_track)
        step = 0.3 * self.dt
        self.yaw_ref_track = wrap(self.yaw_ref_track + float(np.clip(d, -step, step)))
        # ±0.06 캡: 정·역추력 비대칭 때문에 공통모드(syaw)가 크면 순 병진력이 생겨
        # '기생력 → 이탈 → 베어링 오차 증가 → 더 큰 syaw'의 폭주 루프가 된다(진단
        # 실측: syaw ~0.15 지속 구간에서 로봇 단조 이탈). 0.06이면 요 권위
        # ~0.45 rad/s로 슬루 0.3 rad/s 추종에 충분하면서 기생력은 절반 이하.
        return float(np.clip(self.kpsi * wrap(yaw - self.yaw_ref_track), -0.06, 0.06))

    def on_plat(self, msg):
        p_ = msg.pose.pose.position
        self.plat_odom = (np.array([p_.x, p_.y]), self.now())

    def on_acc(self, msg):
        self.a_cmd = np.array(msg.data[:2])
        self.t_acc = self.now()

    def on_est(self, msg):
        # valid==False(마커 가림 등)일 때 rel=(0,0) 더미값이 들어온다 — 이를 그대로 받으면
        # yaw_ref가 순간적으로 튀어 제어 발진의 원인이 된다(회전 중 실측 확인, task-4-report 참조).
        # t_est도 valid에서만 갱신해야 0.5s 게이트가 "마커 가림 → 자연 홀드 폴백"으로 동작한다.
        d = msg.data
        valid = bool(d[5])
        # fix round 2: yaw_meas 이상치 게이트 — valid==True라도 aux 센트로이드가 튀면(round2_log
        # 실측: 3.6m에서 valid 상태로도 40~50° 오차) 직전 yaw_hat 대비 45° 넘게 튀는 값은 버린다.
        # yaw_hat 미초기화(None) 시엔 게이트 불가하므로 그대로 받아 부트스트랩한다.
        if valid and self.yaw_hat is not None and abs(wrap(d[4] - self.yaw_hat)) > np.radians(45):
            valid = False
        self.meas_valid = valid
        if self.meas_valid:
            now = self.now()
            # 연속 유효 스트릭: 직전 유효에서 0.5 s 넘게 끊겼으면 스트릭 재시작.
            # bearing 분기는 스트릭 1 s 이상에서만 진입(아래) — 간헐 깜빡임 한 발로
            # 제어 체제가 스위칭하며 로봇을 흔드는 것을 차단(상태기계 3안).
            gap = self.t_est is None or now - self.t_est > 0.5
            if gap:
                self.valid_since = now
            self.rel = np.array(d[0:2])
            raw_vel = np.array(d[2:4])        # KF 상대속도 — odom_source='est' 피드백
            # EMA: est 잡음의 폐루프 재유입 완화(파라미터 주석 참조). 두절 후엔 재초기화.
            if self.rel_vel is None or gap:
                self.rel_vel = raw_vel
            else:
                self.rel_vel = self.rel_vel + self.est_vel_alpha * (raw_vel - self.rel_vel)
            self.yaw_meas = d[4]
            # yaw 오프셋 추정(IMU 드리프트·프레임 정렬 — 파라미터 주석 참조): 판 적합이
            # 신뢰되는 조건 — est 연속 유효 3 s 이상(안정 추적) + 준정지(|gyro|<0.05 rad/s,
            # 헤딩 오차가 운동 상관 실측) — 에서만 err=yaw_meas-imu_yaw 를 느리게 추종.
            # 초기화도 같은 게이트 뒤(자기잠금 방지, __init__ 주석). 이후 45° 게이트가
            # π-접힘 이상치 차단.
            if (self.k_yaw_off > 0.0                      # 0 = 추정기 완전 비활성(초기화 포함)
                    and self.imu_yaw is not None and self.t_imu is not None
                    and now - self.t_imu < 0.5
                    and now - self.valid_since >= 3.0 and abs(self.gyro_z) < 0.05):
                err = wrap(self.yaw_meas - self.imu_yaw)
                if self.yaw_off is None:
                    self.yaw_off = err
                elif abs(wrap(err - self.yaw_off)) < np.radians(45):
                    self.yaw_off = wrap(self.yaw_off + self.k_yaw_off * wrap(err - self.yaw_off))
            self.t_est = now

    def on_imu(self, msg):
        from .mixer import cf_update
        t = self.now()
        q = msg.orientation
        self.imu_yaw = yaw_of(q.x, q.y, q.z, q.w)   # AHRS 절대 yaw (yaw_source='imu' 피드백)
        self.t_imu = t
        if self.heading_mode != 'bearing':
            return
        self.gyro_z = msg.angular_velocity.z
        if self.yaw_hat is None:
            # plate 마커 캠페인: 초기화를 yaw_meas가 아니라 odometry yaw로 한다(실물:
            # 컴퍼스/IMU 절대 헤딩). plate yaw는 mod-180이라 브리지의 베어링 접기가
            # 지향 밖 자세에서 π-뒤집힌 측정을 낼 수 있는데, 그걸로 초기화하면 45°
            # 이상치 게이트가 이후 참값을 영원히 기각하는 자기잠금이 된다(실측:
            # yaw_hat 오염 → hold/bearing 모두 발진·자전).
            if self.odom is not None:
                self.yaw_hat = self.odom[2]
                self.t_cf = t
            return
        dt = max(0.0, min(t - self.t_cf, 0.1))
        self.t_cf = t
        self.yaw_hat = cf_update(self.yaw_hat, msg.angular_velocity.z, dt,
                                 self.yaw_meas, self.meas_valid, self.k_cf)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist.linear           # body(child) frame 가정 — S1이 실측 검증
        c, s = np.cos(yaw), np.sin(yaw)
        v_world = np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y])
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.odom = (pos, v_world, yaw)
        if self.yaw0 is None:
            self.yaw0 = yaw
        self.t_odom = self.now()

    def tick(self):
        t = self.now()
        if self.odom_source == 'est':
            # 플랫폼 중앙집중 실기 설계: 위치·속도 = 플랫폼 LiDAR est(상대 프레임,
            # 월드축 정렬). est 두절(가림·통신 단절) 시 기존 odom-두절과 동일하게
            # 무추력 — 로봇이 스스로 멈추는 검증된 안전 경로.
            if self.t_est is None or t - self.t_est > 0.5 or self.rel_vel is None:
                self.ei[:] = 0.0
                self.pub.publish(Float64MultiArray(data=[0.0] * 4))
                return
            pos, v_world = self.rel, self.rel_vel
            yaw = self.imu_yaw if self.imu_yaw is not None else 0.0
            if self.t_imu is None or t - self.t_imu > 0.5:
                self.ei[:] = 0.0
                self.pub.publish(Float64MultiArray(data=[0.0] * 4))
                return
        else:
            if self.odom is None or t - self.t_odom > 0.5:
                self.ei[:] = 0.0
                self.pub.publish(Float64MultiArray(data=[0.0] * 4))
                return
            pos, v_world, yaw = self.odom
        # 헤딩 피드백 소스 전환: 'imu'면 로봇 자체 AHRS yaw 사용(실기 hwt9053 동일
        # 의미론). 위치·속도는 여전히 odometry(시뮬 한정 단순화 — 실기 속도원 미정).
        if self.yaw_source == 'imu' and self.t_imu is not None and t - self.t_imu < 0.5:
            off = getattr(self, 'yaw_off', None)   # bearing 모드에서만 존재·추정됨
            yaw = wrap(self.imu_yaw + off) if off is not None else self.imu_yaw
        dropped = self.t_acc is None or t - self.t_acc > 0.5
        if dropped:
            self.a_cmd = np.zeros(2)
            # 두절 시 위치 홀드(2026-08-10): '참조=현재 속도'(자유 표류)는 부트 전
            # 구간이 길어진 plate 캠페인에서 로봇을 플랫폼/서로에게 표류-충돌시켜
            # 스핀을 만든다(진단 실측: r0가 1.08 m까지 접근 후 -100°/s 회전).
            # 두절 진입 시점 위치를 잡아 부드럽게 유지한다(odometry 사용은 기존
            # '시뮬 한정 단순화' 규약과 동일 위치. 이득·캡은 표류 정지 수준).
            if self.pos_hold is None:
                self.pos_hold = pos.copy()
            # P-only 홀드(적분 금지 — 지연 1.8 s 루프에서 적분이 이탈을 가속함을 실측).
            # sim 잔여 상수 외력은 ~0.2 m 정적 오프셋으로 수용(연관 게이트 0.6 안).
            # 이득 0.2/캡 0.08: sim 잔여 외력(~0.05 setpoint 상당)을 이겨 스테이션에
            # 실제로 복귀해야 한다(0.04 캡은 3~5 m 밖에서 힘 평형으로 미복귀 실측).
            self.v_ref = np.clip(0.2 * (self.pos_hold - pos), -0.08, 0.08)
            self.ei[:] = 0.0
        # 앵커는 최초 1회만 캡처하고 유지(2026-08-10): 두절마다 재캡처하면 워밍업
        # 가진으로 밀려난 자리를 새 기준으로 삼아 이탈이 누적된다(실측: 워밍업 44s에
        # |rel| 3~6m). 최초 스테이션 복귀가 검출 기하를 보존한다.
        self.v_ref = self.v_ref + self.a_cmd * self.dt
        # 기준 앵커링(파라미터 주석 참조) — 두절 홀드 경로는 자체 v_ref를 쓰므로 제외한다.
        if not dropped and self.vref_tau > 0.0:
            self.v_ref = self.v_ref - (self.v_ref - v_world) * (self.dt / self.vref_tau)
        if not dropped:
            # 두절 시 v_max 클램프 생략(2026-08-09 근본 수정): 리셋 직후 클램프하면
            # |v|>v_max에서 v_ref=0.2·v̂로 잘려 잔여 제동 명령이 남는다. 회전 중 body
            # 프레임 + 작동 지연 0.5 s에서 제동력 방향이 스핀만큼 돌아 제동이 가속으로
            # 뒤집히고(실측: a_cmd=0인데 |v| 1.2~1.4 m/s 폭주·채널 ±1 포화·자전 자기
            # 유지), 두절 설계 의도(오차 0 → 추력 침묵 → 항력 자연 감속)가 깨진다.
            # 정상 추적 경로의 클램프는 불변.
            n = np.linalg.norm(self.v_ref)
            if n > self.v_max:
                self.v_ref *= self.v_max / n
        e_world = self.v_ref - v_world
        # 데드밴드(파라미터 주석 참조): 오차가 잡음 바닥 미만이면 병진 무추력 + 적분 동결.
        if self.e_db > 0.0 and np.linalg.norm(e_world) < self.e_db:
            e_world = np.zeros(2)
        # 적분 기여를 ±i_max로 클램프(안티윈드업). i_max는 정상상태 항력을 이길 setpoint
        # 여유를 정하는 보정 노브 — mixer 병진 캡(±0.7) 아래에 둔다.
        lim = self.i_max / self.ki if self.ki else 0.0
        self.ei = np.clip(self.ei + e_world * self.dt, -lim, lim)
        e_body = world_to_body(*(e_world + (self.ki / self.kv) * self.ei), yaw)  # mixer 시그니처 불변 트릭: P+I 합성 오차
        # 실측(2026-08-06): [a,a,a,a] 양의 setpoint -> yaw 감소(sw=+0.1에서 -215deg/4s,
        # 정상 rate -12 rad/s per unit). 따라서 yaw>yaw0일 때 sw>0이어야 되돌린다.
        if (self.heading_mode == 'bearing' and self.rel is not None
                and self.t_est is not None and t - self.t_est < 0.5
                and self.valid_since is not None and t - self.valid_since >= 1.0):
            yaw_ref = np.arctan2(-self.rel[1], -self.rel[0])   # platform을 바라보는 방위
            self.last_yaw_ref = yaw_ref   # 두절 폴백용 최신 베어링 기억(스폰 상수 대체)
            # plate 캠페인(2026-08-09): 헤딩 피드백을 yaw_hat(CF: 자이로+plate yaw 융합)에서
            # **odometry yaw**로 전환. plate yaw는 π-대칭이라 미지향 자세에서 접기-뒤집힌
            # 측정이 CF를 오염시키고, 오염된 yaw_hat을 쫓는 제어가 자전→검출 상실→재오염의
            # 자기 유지 발산을 만든다(실측: 정렬 성공 직후 검출 버스트에서 재붕괴). 실물
            # 로봇은 온보드 컴퍼스/IMU로 자기 헤딩을 아는 게 표준이며, 이 저장소의 기존
            # '시뮬 한정 단순화' 규약(하위 속도 루프 odometry = 온보드 센서 대역)과 동일한
            # 위치의 단순화다. plate yaw 측정은 S4 게이트의 평가 대상으로 유지(제어 미사용).
            syaw = self._syaw_track(yaw, yaw_ref)
        else:
            # plate 부트스트랩(2026-08-09): bearing 모드에서 est가 아직/더는 없을 때의 폴백
            # 목표를 스폰 헤딩(yaw0)이 아니라 **배치 베어링**(fallback_bearing, 알려진 초기
            # 기하 — 실물도 배치 시점 기하는 알고 시작)으로 한다. 판 마커는 플랫폼을 향해야
            # 검출되므로, 스폰 헤딩 복귀 폴백은 판을 돌려버려 검출 부트스트랩을 막는다(실측).
            # fallback_bearing 미설정(NaN)이면 기존 hold 그대로 (hold 모드·구 캠페인 불변).
            # 적응형 폴백(2026-08-09): 스폰 상수 베어링은 platform이 표류하면 틀린 방향이
            # 되어, 두절마다 로봇을 엉뚱한 쪽으로 돌리고 검출 버스트가 되돌리는 왕복
            # 발진을 만든다(실측: 정렬 8~13° 도달 후 120~150° 스윙 반복). bearing 분기가
            # 마지막으로 쓴 yaw_ref를 유지하면 두절 동안 지향이 보존된다.
            fb = None
            if self.odom_source == 'est' and self.rel is not None:
                # est 모드: rel 자체가 플랫폼 상대 벡터 — plat_odom(월드)과 섞지 않는다.
                # 이 분기는 est 신선(위 게이트 통과)·스트릭 미충족(<1s)일 때만 온다.
                fb = float(np.arctan2(-self.rel[1], -self.rel[0]))
            elif self.heading_mode == 'bearing' and getattr(self, 'plat_odom', None) is not None \
                    and t - self.plat_odom[1] < 1.0:
                dp = self.plat_odom[0] - pos
                fb = float(np.arctan2(dp[1], dp[0]))     # platform odometry 기준 실제 베어링
            if fb is None:
                fb = getattr(self, 'last_yaw_ref', None)
            if fb is None:
                fb = self.fallback_bearing if (self.heading_mode == 'bearing'
                                               and not np.isnan(self.fallback_bearing)) else self.yaw0
            if self.heading_mode == 'bearing':
                syaw = self._syaw_track(yaw, fb)
            else:
                syaw = self.kpsi * wrap(yaw - fb)              # hold 모드(마스트 캠페인) 불변
        s = setpoints(e_body, syaw, self.kv)
        self.pub.publish(Float64MultiArray(data=list(s)))
        if self.log:
            row = [t, v_world[0], v_world[1], yaw, self.v_ref[0], self.v_ref[1],
                   self.a_cmd[0], self.a_cmd[1], self.ei[0], self.ei[1], *s]
            self.log.write(','.join(f'{x:.5f}' for x in row) + '\n')

def main():
    rclpy.init()
    rclpy.spin(VelocityLoop())
