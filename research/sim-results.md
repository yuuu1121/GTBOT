# 시뮬레이션 캠페인 결과 종합

`.superpowers/sdd/2026-08-03-sim-and-bibtex/` 하 Task 1~11 실행 결과. 모든 수치는 `results/*.json`에서 직접 복사(기억 서술 금지 원칙). 그림은 `results/*.png`. 전체 진행 서사는 `.superpowers/sdd/2026-08-03-sim-and-bibtex/progress.md` 레저 원본 참조.

## 캠페인 구조

| 단계 | 목적 | 산출 JSON | 결과 |
|---|---|---|---|
| 게이트 1 (E1 식별) | Θ 파라미터 식별 재현 | `e1_gate1.json` | PASS (개정 기준) |
| 게이트 2 재현 팔 | 공개 설정만으로 재현 | `e1_gate2.json` | FAIL — 재현 불가 확정 |
| 개선 식별 5라운드 | 데이터 기반 식별 개선 시도 | `e1_ident_improved.json`, `e1_gate2_local.json` | FAIL — 경로 기각 |
| 해석적 팔 | 참 그래디언트 LP | `e1_gate2_analytic.json` | PASS 5/5 |
| 게이트 3 (E2) | φ⁷ 편대유지 항 검증 | `e2_gate3.json`, `e2_conflict_run.json`, `e2_tuning_log.json` | PASS 4/4 |
| 게이트 4 (E3) | 확장성(N=3/5/7) | `e3_gate4.json` | PASS 3/3 |

---

## 게이트 1 — E1 식별 재현 (`e1_gate1.json`)

개정 기준(사용자 승인, 2026-08-03): **동결-Θ teacher-forced 1-step 예측오차에서 bilinear < linear**. 재귀 다단 개루프 예측은 판정에서 제외하고 기록만.

| 지표 | linear | bilinear |
|---|---|---|
| `frozen_1step_rmse` (전 구간 300스텝, teacher-forced) | 46.65117905761523 | 16.182183717529416 |
| `open_loop_rmse` (기록 전용, start=260, horizon=30) | 0.05570942136450889 | null |
| `open_loop_diverged` | false | true |
| `trP` | 1798.9593424359343 | 90100.0 |
| `zeta_rank` | 24 | 100 |
| `eps_peak_75win` (4구간 피크, teacher-forced ε 시계열) | [4.89, 6.11, 8.51, 11.12] | [11.47, 26.18, 29.45, 30.07] |

`gate1` 판정 블록:

```json
{
  "frozen_1step_bil_lt_lin": true,
  "eps_bil_lt_lin_fraction": 0.22,
  "ratio_report": 0.367935615876269,
  "pass": true
}
```

`metric_notes`(JSON 원문): `eps_bil_lt_lin_fraction`은 리셋 스케줄·과소결정 과도응답을 반영 — 모델 품질 증거 아님; bilinear `trP`=p0·dim은 리셋 항등식, `zeta_rank`는 표본 상한(100) — 둘 다 진단 증거로 쓸 수 없음.

**재귀 다단 발산 원인**(스펙 확정 문구): bilinear ζ의 이차 블록(outer(z2d,z2d))이 예측이 참값 범위 [−1,1]을 벗어나는 순간 오차를 제곱 증폭시켜 구조적으로 발산 — 오차 지수 매 스텝 2배.

---

## 게이트 2 재현 팔 — 공개 설정만으로 재현 불가 확정 (`e1_gate2.json`)

| Case | reached | min_target_dists | collision_free | min_robot_surf | min_wall_surf |
|---|---|---|---|---|---|
| I | false | [3.30, 3.09, 1.82] | false | 4.349 | -1.481 |
| II | false | [2.80, 2.57, 1.55] | false | 4.017 | -0.104 |
| III | false | [2.50, 2.47, 1.51] | false | -2.352 | 1.050 |
| IV | false | [2.93, 2.48, 2.05] | true | 0.181 | 2.340 |
| V | false | [2.93, 3.18, 1.39] | false | 1.240 | -3.316 |

`gate2.pass_conditions_1_2 = false`. `oracle_note`(JSON 원문): 참 그래디언트 오라클은 지평내 최소 기준 5/5 통과(0.03~0.15m) — 파이프라인 정상, 병목은 식별(identification) 자체.

식별 미결정계 진단(레저 확정 사실): a=901 vs ks=300(실측 rank=270, 영공간≥631), Θ 입력-그래디언트 부호일치율 46.7~55.6%(동전던지기 수준), φ⁴·φ⁵·φ⁶ 분산≈0.

`e1_w_tuning_log.json`: budget=10회, `any_pass: false`. 10회 전부 조건①②(목표도달+무충돌) 미통과. w[2](φ³ 가중치)를 5→320까지 64배 증폭해도 reached 불변 — φ³은 nr>~5m에서 그래디언트가 거의 0으로 소멸해 원거리 초기조건(Table 2 케이스 3~7m)에서 견인력을 만들지 못함. 튜닝 축(w1,w2,w5)만으로는 미해결로 확정.

이 로그는 커밋되지 않은 임시 스크립트의 산출물(생성 코드 저장소에 없음)이며 reached 판정은 정정 전 기준(최종 스텝)이다. 최종 리뷰에서 최적 시행(trial 8)을 정정 기준(지평 내 최소거리)으로 재실행한 결과 케이스별 최소거리 1.26~3.19m로 결론(어떤 w로도 미도달) 불변 확인.

---

## 개선 식별 5라운드 — 데이터 기반 경로 기각 (`e1_ident_improved.json`, `e1_gate2_local.json`)

| 라운드 | 방법 | sign_agreement | tail_rmse | 판정 |
|---|---|---|---|---|
| 1 | 에피소드 여기(ks=600, 리셋 없음) | 0.436 | 11.35 | 미달 |
| 2 | ζ축소 결정계(a=433) | 0.513 | 2.74 | 미달 (`e1_ident_improved.json` 최종 상태) |
| 3 | 국소 식별(회랑 24×25스텝) | Case I 0.733 / II 0.600 / III 0.767 / IV 0.700 / V 0.633 | Case I 0.141 / II 0.176 / III 0.162 / IV 0.333 / V 0.201 | 5케이스 전부 `ident_pass: false` (task-8b-report.md 라운드 3) |
| 4 | 속도 커버리지 + 국소성 조임 | Case I 0.810 / II 0.667 / III 0.738 / IV 0.690 / V 0.643 | Case I 0.103 / II 0.143 / III 0.076 / IV 0.078 / V 0.102 (`e1_gate2_local.json`) | Case I 근소 미달(tail 0.103>0.1), 나머지 미달 지속 |
| 5(최종) | 진단용 실제 제어 실행 | 제어궤적 실측 0.516 (정지 프로브 0.810 대비) | — | 5/5 미도달, 2/5 충돌 |

`e1_gate2_local.json`의 `control_traj_sign_agreement: 0.5161290322580645`(Case I) — 정지 프로브 부호일치(0.8095238095238095)가 궤적 위에서는 열화됨을 정량 확인.

**BREAKER 판정**(레저 확정): 데이터 기반 bilinear Koopman 식별은 전역(미결정)·결정계(모델클래스 한계)·국소(궤적 위 열화) 세 경로 모두에서 제어 불가 — 잘 문서화된 부정적 결과. 사용자 결정(2026-08-05)으로 해석적 그래디언트 팔로 전환.

---

## 해석적 팔 확정 — 게이트 2 정식 5/5 (`e1_gate2_analytic.json`)

컨트롤러: "analytic (finite-difference true 1-step gradient)".

| Case | min_target_dists | reached | collision_free |
|---|---|---|---|
| I | [0.032, 0.053, 0.130] | true | true |
| II | [0.060, 0.025, 0.049] | true | true |
| III | [0.069, 0.067, 0.035] | true | true |
| IV | [0.090, 0.069, 0.043] | true | true |
| V | [0.085, 0.040, 0.084] | true | true |

`gate2.pass_conditions_1_2 = true`. min_target_dists 전 케이스 0.025~0.130m 범위.

---

## 게이트 3 — E2 φ⁷ 편대유지 검증 (`e2_gate3.json`)

| 지표 | φ⁷ (기본값 σ=3.0, w7=3.0) | baseline |
|---|---|---|
| J0 | 151.4285283957277 | 151.4285283957277 |
| J_final | 0.00019143060460660488 | 6.097893858644961 |
| J_sum_k1 (ΣJ) | 5022.176268243457 | 9035.17563043624 |
| edge_final_abs_err (0-1/0-2/1-2) | 0.0133 / 0.0038 / 0.0000033 | 1.936 / 0.0018 / 1.532 |
| collision_free | true | false |

`gate3` 판정: `J_final_lt_10pct_J0=true`, `edges_within_0p3m=true`, `sumJ_phi7_lt_baseline=true`, `collision_free=true`, **`pass=true`** (4/4).

**튜닝 이력**(`e2_tuning_log.json`, σ=3/w7=3 기본값 도달 전): 다섯 σ/w7 조합(8/10, 6/20, 10/20, 5/15, 8/12) 모두 `edges_within_0p1_sigma`·`collision_free` 미충족 — 최종적으로 v_cruise 파라미터화(아래 이탈 목록) 후 σ=3/w7=3 **기본값**에서 4/4 통과로 귀결. 이 로그는 v_cruise 파라미터화(2026-08-05 스펙 개정) 이전 실행 기록으로 옛 게이트 키(`edges_within_0p1_sigma`, `peak_lt_baseline`)를 쓰며 현재 코드로는 재생성 불가.

**φ⁶/φ⁷ 상충 런**(`e2_conflict_run.json`, d*=4.6): J0=212.51, J_final=5.854, min_robot_surf=1.054 — 지정 거리에서 φ⁶(충돌회피)·φ⁷(편대유지) 상충 미관측.

---

## 게이트 4 — E3 확장성 N=3/5/7 (`e3_gate4.json`)

| N | ζ_dim | P_bytes | per_step_control_s | 실시간 여유(50ms 대비) | formation_J0 | formation_J_final | collision_free | gate4_pass |
|---|---|---|---|---|---|---|---|---|
| 3 | 1075 | 9245000 | ~0.80ms | 약 62배 | 36.0 | 0.06737031180296457 | true | true |
| 5 | 2941 | 69195848 | ~1.40ms | 약 35~36배 | 56.583592135001226 | 0.0007969542113370594 | true | true |
| 7 | 5727 | 262388232 | ~2.72ms | 약 18배 | 71.06040792565068 | 0.0039198924923083585 | true | true |

타이밍은 재실행마다 ±수% 변동 — 정본은 `results/e3_gate4.json`의 `per_step_control_s`. 결정론 필드(`zeta_dim`, `P_bytes`, `formation_J0`/`formation_J_final`, `gate4_pass`)는 위 표 값이 정본과 일치.

레저 요약(t_ctrl): ~0.80/1.40/2.72ms (비결정 — 정본 JSON 참조) — 3케이스 전부 50ms 대비 18배 이상 여유. `note`(JSON 원문): 식별 팔은 기각되었으므로 `zeta_dim`은 기각 팔의 스케일링 분석용, 제어 측정치는 해석적 팔 기준.

---

## 이탈 목록 (원논문 대비, 논문 본문에 명시 필요)

1. **게이트 1 지표 개정**: 사후오차 εₐ→사전오차(1-step) 기준으로 교체 — εₐ는 RLS 항등식 `εₐ(k)=ε(k)·ρ/(ρ+ζᵀPζ)`이라 파라미터 품질과 무관하게 작게 나옴(검증력 없음). 원논문 Figure 4/8 대조용으로만 기록.
2. **reached 판정**: 최종 스텝 기준이 아닌 지평 내 최소값 기준(그리디 관통).
3. **E2/E3 control_iters=300**.
4. **v_cruise=1.0**: 원논문 φ² 순항속도 지시(4m/s)가 φ⁶ 반응거리(0.3m, 제동거리 1.0m+)와 스케일 불일치해 장지평 충돌 유발 — 스코프 내 대안(할당/가중/거리축소) 3종 기각 후 v_cruise 인하로 해결. 파라미터화·기본값 이탈로 문서화.
5. **ζ축소·국소식별 시도**: 기각된 시도로 기록(위 "개선 식별 5라운드" 참조) — 데이터 기반 식별 경로가 안 된다는 부정적 결과 자체가 논문 자산(Rosenfelder 재확인).

---

## 그림 목록 (`results/*.png`)

| 파일 | 내용 |
|---|---|
| `e1_gate1_errors.png` | 게이트 1 ε 시계열(linear vs bilinear) |
| `e1_case_{I..V}_traj.png` | 게이트 2 재현 팔 5케이스 궤적(미통과) |
| `e1_case_{I..V}_traj_local.png` | 개선 식별(국소) 5케이스 궤적(미통과) |
| `e1_case_{I..V}_traj_analytic.png` | 해석적 팔 5케이스 궤적(전 케이스 통과) |
| `e2_traj_baseline.png` | E2 베이스라인(φ⁷ 없음) 궤적 |
| `e2_traj_phi7.png` | E2 φ⁷ 편대유지 궤적 |
| `e2_formation_error.png` | E2 형상오차(edge error) 시계열, φ⁷ vs baseline |

---

## 논문에 쓸 발견

**εₐ 항등식 아티팩트.** 원논문의 사후오차 지표 εₐ는 RLS 갱신식의 항등식 `εₐ(k)=ε(k)·ρ/(ρ+ζᵀPζ)`으로부터 나오는 값이라, 파라미터 Θ가 엉터리여도 P가 감쇠하면 작게 수렴한다 — 즉 "원논문 대비 몇 천 배 개선"류의 사후오차 비교는 식별 품질의 증거가 될 수 없다. 검증력 있는 지표는 사전추정오차(1-step teacher-forced) 뿐이며, 이 재정의가 게이트 1의 실제 판정 기준이 되었다(`frozen_1step_rmse` linear 46.65 vs bilinear 16.18).

**미결정계 정량 진단.** 게이트 2 공개 설정 재현이 실패한 근본 원인은 컨트롤러 버그가 아니라 식별 문제의 구조적 미결정성이다: 파라미터 차원 a=901 대 표본 ks=300, 실측 rank=270(영공간 ≥631), Θ 입력-그래디언트 부호일치율 46.7~55.6%(동전던지기 수준), φ⁴~φ⁶ 분산≈0. 참 그래디언트 오라클은 동일 파이프라인에서 5/5 통과(0.03~0.15m)해 파이프라인 자체는 정상임을 입증 — 병목은 순수 식별.

**순항속도-반응거리 스케일 불일치.** 원논문 φ² 순항속도 항의 지시값(4m/s)이 φ⁶ 반응거리(0.3m, 제동거리 1.0m+)와 자릿수가 맞지 않아 장지평에서 충돌을 유발한다. v_cruise를 1.0으로 낮추면(스코프 내 대안 3종은 기각) 기본 σ/w7 값에서 φ⁷ 팔이 4/4 전 축 통과(J 151→0.0002, ΣJ 5022<9035, 무충돌)한 반면 베이스라인은 형상오차 6.1로 열등 — 원논문 효용함수 설계의 장지평 한계를 드러내는 발견.

**φ⁷ 전 축 검증.** 편대유지 효용항 φ⁷는 게이트 3의 4개 독립 조건(J_final<10%J0, edge<0.3m, ΣJ<베이스라인, 무충돌) 전부 통과. 지정 거리 d*=4.6에서 φ⁶(충돌회피)·φ⁷(편대유지) 상충도 미관측 — 두 효용항이 같은 Koopman 파이프라인에서 공존 가능함을 실증.

**확장성.** N=3/5/7 전부 게이트 4 통과, 스텝당 제어 시간 0.81/1.44/2.78ms로 50ms 실시간 예산 대비 18~62배 여유. v_cruise 파라미터를 누락하면 N=3에서도 충돌이 재현되어(발견 3의 독립 확증) 스케일 불일치가 시나리오 크기와 무관한 구조적 문제임을 뒷받침.

---

## Stonefish 캠페인 (2026-08-06)

`.superpowers/sdd/2026-08-06-koopman-stonefish/` 하 Task 1~8 실행 결과. 수치 시뮬(이중적분기, 위 게이트 1~4)에서 검증된 φ⁷ 편대유지 Koopman 파이프라인을 Stonefish 실선박 동역학(platform 리더 + gtbot 팔로워 3대)으로 이식한 캠페인. 모든 수치는 `results/*.json`에서 직접 복사(기억 서술 금지 원칙).

### 아키텍처 (계층형)

수치 시뮬의 이중적분기 대신 검증된 실선박 동역학을 하위 속도 루프로 정형화해, Koopman/LP는 그 위의 가속도 명령 층으로만 이식했다(계층형 A안, B end-to-end 스러스터 식별·C 이식만은 브레인스토밍 단계에서 기각). `koopman_formation`(10→20 Hz, 최종 20 Hz)이 `/platform/odometry`·`/gtbot{,2,3}/odometry`를 구독해 로봇당 상태 z=[p,v]∈R⁴로 RLS 온라인 식별(sim/koopman.py·rls.py 재사용)과 LP(analytic 팔 기본)를 돌려 `/gtbot{,2,3}/accel_cmd`를 발행하고, `velocity_loop`×3(20 Hz)이 이를 받아 PI 속도 루프 + 검증된 믹서([E,W,N,S], +X=[0,0,-a,+a] / +Y=[-a,+a,0,0] / yaw=[a,a,a,a])로 스러스터 명령을 낸다. leader_pilot은 Koopman 밖 외생 입력으로 `/platform/thrusters`를 직접 구동.

### 리더 상대좌표계

`relative_state.py`가 각 팔로워의 위치·속도를 리더 기준 상대값(pos−pL, vel−vL)으로 조립해 koopman_formation에 넘긴다. 수치 시뮬의 targets·동작점(operating point)이 모두 이 상대좌표계로 상수화돼 있어, sim/ 모듈(koopman.py, rls.py, control.py, utility.py)을 코드 수정 없이 그대로 재사용할 수 있다는 것이 근거다.

### 게이트 S1 — 속도 추종 (`s1_stonefish.json`)

| axis | v_meas | v_expect | rel_err |
|---|---|---|---|
| x | [0.482860336491293, 0.002903907180515767] | [0.5, 0.0] | 0.034767843885104704 |
| y | [-0.0018620665427611972, 0.4796660015108756] | [0.0, 0.5] | 0.04083815795872214 |

`gate_s1_pass: true`.

통과까지 4단 수정 이력(Task 3):
1. **믹서 yaw 권한 예약**: 병진 오차가 크면 4채널 전부 ±1 포화돼 yaw 성분이 클립에 잘려 yaw 제어 권한이 0이 되고 yaw가 자유 드리프트 → 회전하는 몸체의 월드속도 평균이 "축간 혼입/재현성 붕괴"로 보였다. `mixer.setpoints()`에서 병진(sx,sy) ±0.7, yaw(sw) ±0.3을 각각 클립 후 가산하도록 예약.
2. **PI 루프**: P 전용 루프의 정상상태 오차(항력 균형점에서 e=0.35/kv 잔류)를 없애려 적분항(ki, 안티윈드업 i_max) 추가.
3. **yaw 유지 부호 반전(정귀환) + kpsi 0.1 감쇠 설계**: 벤치 개루프 실측(`[a,a,a,a]` 스텝)에서 양의 setpoint가 yaw를 **감소**시키는데 코드는 반대 부호(`syaw = kpsi*wrap(yaw0-yaw)`)를 가정해 정귀환 루프가 게이트 내내 로봇을 자전시키고 있었다. `syaw = kpsi*wrap(yaw-yaw0)`로 부호 반전 + yaw rate 플랜트 실측(G≈12 rad/s/unit, τ≈0.5s)에 맞춘 kpsi=0.1(ζ≈0.7 감쇠 설계) — 이 단계까지는 x rel_err 0.136 (v_meas 0.432, 0.002) / y rel_err 0.161 (v_meas 0.000, 0.420)로 아직 미달(`gate_s1_pass: false`, task-3-report.md 라운드 4).
4. **적분 안티윈드업 상한 i_max 0.4→0.6**: CSV 계측상 `ei`가 정확히 0.400에 물린 채 v가 0.432에서 정체 — 항력 균형 setpoint(≈0.5)보다 안티윈드업 상한이 낮은 게 병목이었다. 상한을 병진 캡 ±0.7 아래로 0.6까지 올려 통과.

### 게이트 S2 — 온라인 식별 (`s2_stonefish.json`)

| model | frozen_1step_rmse |
|---|---|
| linear | 0.05931626133210375 |
| bilinear | 0.04235342507017789 |

`gate_s2_pass: true` (`warmup_steps: 600`). 수치 캠페인(위 게이트 1~2)에서는 데이터 기반 bilinear Koopman 식별이 전역·결정계·국소 세 경로 모두에서 기각됐지만, 하위 속도루프로 폐루프 동역학이 정형화된 이 계층형 구조에서는 온라인 bilinear 식별(RLS)이 성립한다 — bilinear frozen 1-step RMSE가 linear보다 작다는 판정 기준을 충족.

### 게이트 S3 — 편대 형성·유지 (`s3_stonefish.json`)

| 지표 | 값 | 기준 |
|---|---|---|
| max_edge_err_tail30s | 0.23304293464882475 | < 0.3 |
| min_surface_dist | 1.1915661299594762 | > 0 |
| n_samples | 7378 | — |

`gate_s3_pass: true`. p50(전 구간) 0.024 m, LP nan_guard 0건(Task 7 커밋 로그).

통과까지 3단 서사(Task 7, 부정적 결과 포함 보존):

**① 식별모델(LP) 팔은 폐루프에서 발산 — 수치 캠페인의 부정적 결과가 현실 동역학에서 재현.** 워밍업이 초기 위치 근방을 흔드는데 RLS 선형화 기준점은 목표 오프셋이라, 제어 시작 상태에서 bilinear 기울기가 학습 분포 밖 외삽이 된다.

| 실행 | max_edge_err_tail30s | min_surface_dist | gate_s3_pass |
|---|---|---|---|
| `s3_stonefish_modelarm_baseline.json` | 65.92041886088734 | -0.1561316367496417 | false |
| `s3_stonefish_modelarm_sigma-1.5.json` | 54.33083952189884 | 9.795418022723135 | false |
| `s3_stonefish_modelarm_vlead-0.1.json` | 62.46915402562472 | 3.5691866377846067 | false |
| `s3_stonefish_modelarm_w3-10.json` | 147.40447560458279 | 3.0258978851016582 | false |

편대가 54~147 m로 발산, 노브(w3/sigma/v_lead) 3종 전부 무효.

**② analytic 팔 전환으로 발산 해소.** `koopman_node`에 `controller` 파라미터(기본 `'analytic'`, 참 그래디언트 — E1~E4에서 검증) 추가, `'model'` 팔은 비교군으로 보존. 리더 경로를 사각 → 직선(waypoints=[60,0], 사용자 승인)으로 재정의해 게이트 의미를 "정상상태 편대유지"로 명확화, 제어 전환 후 30 s 수렴 대기 뒤 180 s 관측. 턴 과도응답은 별도 보존:

| 실행 | max_edge_err_tail30s | min_surface_dist | gate_s3_pass |
|---|---|---|---|
| `s3_stonefish_square_turns.json` | 0.818261895920319 | 0.7863431808629333 | false |

**③ 지연 보상(사용자 승인) — 통과의 결정타.** 잔여 오차 0.48 m의 정체를 계측으로 판별: 부호 있는 edge 오차가 0 중심 진동(주기 3.5 s)하고 U는 전 tick \|U\|=u_max(LP 꼭짓점 = bang-bang), U→실가속 교차상관으로 작동지연 τ=0.16 s 실측(브리프 추정 1 s를 정정). 동일 제어기 + 이상적 이중적분기 대조실험(Task 7 커밋 로그 기록, JSON 미보존)에서 τ=0.10 s→0.011 m, τ=0.15 s→0.273 m의 절벽을 확인해 릴레이+지연 한계 사이클로 확정.

analytic_c 호출 전 X를 τ만큼 전파하는 지연 보상(X_pred = A_d·X + B_d·u_prev, `ab_matrices(3,τ)`, τ=0.16, u_prev=직전 발행 U)을 도입, LP·효용함수 프레임워크는 무변경. 함께 조정한 노브: rate 10→20 Hz + dt 0.1→0.05, u_max ±0.5→±0.3(±0.5는 velocity_loop v_ref가 v_max 클램프에 상시 접촉). 최종 `s3_stonefish.json` = `s3_stonefish_delaycomp-final.json`과 동치(참고: `s3_stonefish_delaycomp-u0.3.json`/`-u0.5.json`/`-u0.3-settled.json`은 u_max·정착 여부를 바꾼 중간 튜닝 시행 보존, τ 대조실험과는 별개).

### 시뮬레이터 판정 요약

스펙(`.sp/specs/2026-08-06-koopman-stonefish-design.md`) 확정 사항: Stonefish 채택, Isaac Lab 기각. 근거 3가지 — (1) RLS 온라인 식별은 수백~수천 스텝 순차 회귀이지 Isaac Lab의 존재 이유(수천 환경 병렬 RL)가 불필요, (2) Isaac Lab(PhysX)은 부력·유체항력·프로펠러 유입류 등 수중물리가 부재해 커스텀 force로 구현하면 "충실한 동역학에서 식별" 주장이 약해지는데 Stonefish는 이를 네이티브 제공하며 흘수·복원력·추력 비대칭까지 이미 실측 검증됨, (3) 온라인 식별은 실시간 순차 데이터가 본질이라 병렬화 이득 자체가 없음. 재고 조건: 대규모 RL 도입 시 Isaac Lab 재검토.

### 이탈 목록 (Stonefish 캠페인)

1. **여기(excitation) 입력**: 수치 캠페인 Table 1의 PRBS 대신 멀티사인 신호를 `/8.0` 스케일로 사용(`koopman_node.py`), 첫 시도로 게이트 S2 통과해 `/12.0` 재조정 불필요.
2. **마지막 여기 전이 평가 제외**: `frozen_1step_eval`은 `X_log[k+1]`을 참조하므로 워밍업 마지막 전이는 자기복제 편향 방지를 위해 평가에서 제외.
3. **koopman_formation 파라미터**: rate 20 Hz(dt=0.05, analytic_c의 B가 위치감도 dt²/2·속도감도 dt이므로 dt는 감쇠 가중 노브), u_min/u_max=∓0.3(계측 근거 — ±0.5는 v_ref가 v_max 클램프에 상시 접촉).
4. **controller 파라미터**: 기본값 `'analytic'`(참 그래디언트), `'model'`(식별 Θ 기반 LP)은 실험·비교군으로 코드에 보존.
5. **S3 리더 경로**: 사각 → 직선(waypoints=[60,0])으로 재정의, 정상상태 편대유지를 게이트 정의로 명확화(사용자 승인). 관측은 제어 전환 후 30 s 수렴 대기 뒤 180 s. 턴 과도응답은 `s3_stonefish_square_turns.json`에 별도 보존.
6. **지연 보상 도입**(사용자 승인): 작동지연 τ=0.16 s 보상을 위해 X를 A_d·X+B_d·u_prev로 전파하는 예측 단계 추가 — LP·효용함수 프레임워크는 무변경.
