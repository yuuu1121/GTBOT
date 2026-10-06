# 2026-10-06 — model 팔(bilinear Koopman Θ) Stonefish 런

`koopman_formation controller:=model|mpc` 검증 런. 오전 런(`koopman_model_sq4`, `*_leader_still_*`)은 공칭 Θ₀ + 그래디언트
확장 사전(ψ), 오후 런(`koopman_model_paper_sq4`)은 **원논문 형태**(φ 사전만·φ⁴ 제거·궤적 위 누적 재적합 Θ, 설명 노트 §11)
— 현재 노드의 model 팔은 후자다. 둘 다 제어 중 동결.
LiDAR 추정 기반, `formation.launch.py start_leader:=false` → S2 → `settle_wait` → `leader_pilot` 절차.
편대 오차는 `koopman_*.csv`의 GT 열(g0..g11, odometry 기준 상대상태)로 계산 — 이 환경에서 ros2 CLI가
SHM 프로파일 없이 토픽을 못 봐 정식 gate_s6는 돌리지 못했다.

| 파일 | 조건 | edge 오차 중앙 / p90 / 최대 [m] |
|:---|:---|:---|
| `koopman_mpc_sq4.csv` + 영상 2편 | **Koopman MPC 팔 최종 설정**(사전 dim 508·포화 좌표·넓은 식별 데이터, H=3, ρ=3, 식별 영역 밖 analytic 폴백, 적합 시드 0; 설명 노트 §12), 같은 4×4 경로(377 s) | **0.146 / 0.224 / 0.423**, 로봇간 최소 1.38 m, 포화 19 %, 부호 반전 13 %, 틱 최대 0.09 s |
| `koopman_mpc_sq4_seed1.csv` | 같은 최종 설정, 적합 시드 1 | 0.160 / 0.224 / 0.366, 포화 9 %, 폴백 55/7200 틱(0.8 %) — 시드를 바꿔도 analytic 보다 낫다 |
| `koopman_mpc_sq4_nofallback.csv` | 4차 런: 포화 좌표만으로 폴백 없이(적합 시드 0) | 0.137 / 0.224 / 0.371 — 가장 좋았지만 적합 시드 1 은 출발 직후 발산(2회, CSV 미수록)해 폴백을 되살렸다 |
| `koopman_mpc_leader_still.csv` | MPC 팔, 리더 정지 294 s | 0.145 / 0.242 / 0.940 — 최대는 t=268 s LiDAR 추정 튐(est−GT 0.93 m)을 따라간 것, 그 밖 구간 최대 0.32 |
| `koopman_mpc_sq4_rho1p5.csv` | MPC 팔, 입력 변화 패널티 ρ=1.5 | 0.152 / 0.252 / 0.500 — 포화 31 %; ρ=3 이 낫다 |
| `koopman_mpc_sq4_trust.csv` | 같은 MPC, 포화 좌표 없이 식별 영역(0.45 m) 밖 analytic 폴백(3차 런) | 0.151 / 0.233 / 0.447 — 포화 17 %, 반전 12 % |
| `koopman_mpc_sq4_rho0.csv` | 같은 MPC, 입력 변화 패널티 없음(ρ=0, 2차 런) | 0.157 / 0.292 / 0.867 — 포화 62 %·반전 39 %, est 속도 오차 RMS 0.15로 채터링 |
| `koopman_model_paper_sq4.csv` + 영상 2편 | **논문 형태 model 팔**, 리더 4×4 m 사각 1.7바퀴(377 s) | 0.190 / 0.367 / 0.569, 로봇간 최소 1.26 m 무충돌, 포화 0 % |
| `koopman_analytic_sq4.csv` + 영상 2편 | analytic 대조, 같은 4×4 경로(377 s) | 0.168 / 0.286 / 0.416 |
| `koopman_model_sq4.csv` + 영상 2편 | ψ 확장 사전 model 팔, 같은 4×4 경로(377 s) | 0.163 / 0.265 / 0.378, est 유효율 100 %, 무충돌 |
| `koopman_model_leader_still_run1.csv` | 리더 정지, 237 s | 0.164 / 0.285 / 0.393 (정착 후) |
| `koopman_model_leader_still_run2.csv` | 리더 정지, 249 s | 0.134 / 0.289 / 0.379 (정착 후) |
| `koopman_analytic_leader_still.csv` | analytic 대조, 리더 정지, 250 s | 0.164 / 0.258 / 0.405 (정착 후) |

영상: `*_topview.mp4`(탑뷰 10배속, 궤적·변 오차 HUD), `*_simview.mp4`(시뮬 카메라 3배속) — `koopman_mpc_sq4`, `koopman_model_paper_sq4`, `koopman_analytic_sq4`, `koopman_model_sq4` 각 2편.

Koopman MPC 팔(최종 설정, 적합 시드 0·1)은 analytic 대비 edge 중앙 −13 % / −5 %, p90 −22 %, 최대 ±0 / −12 %, station 중앙 0.095·0.106 vs 0.147 m. 입력은 더 쓴다(|U| 중앙 0.09~0.11 vs 0.04, 포화 9~19 % vs 4 %)지만 부호 반전은 적다(11~13 % vs 18 %). 런 이력: 1차는 식별 영역 밖(워밍업 끝 오차 0.8 m) 외삽으로 10 s 만에 발산(CSV 미수록) → 2차 ρ=0 채터링 → 3차 ρ=3 + 신뢰 영역 폴백 → 4차 포화 좌표로 폴백 제거(시드 0 만 성공) → 적합 시드 1 발산 2회 → 최종: 포화 좌표 + 넓은 식별 데이터 + 폴백 복원.

논문 형태 팔은 analytic보다 edge 중앙 +0.02 m, p90 +0.08 m 나쁘다 — 수치 시뮬의 격차(정상 0.12~0.14 vs 0.08 m)와 같은 방향이고, 1차식 모델의 표현력 한계다(설명 노트 §5·§11). ψ 확장 사전은 analytic과 같지만 J의 미분을 사전에 넣은 동어반복이라 노드에서 뺐다.

`koopman_arms_compare_sq4.mp4`: **4팔 비교 영상**(Stonefish 4×4 사각 런 4편을 같은 시각 축으로 재생, 10배속, `tools/render_arm_compare.py`) — Koopman MPC | analytic | Koopman 1-step(논문 꼴) | ψ 확장 사전. 세 줄: 윗줄 월드 좌표 탑뷰(각 런의 `*_topview.mp4` 프레임, 사각 경로를 실제로 도는지), 가운데 리더 고정 상대좌표(편대 변 오차), 아랫줄 최대 변 오차 시계열. 리더는 추력으로 움직여 런마다 진행 속도가 조금 달라 윗줄 위치는 서로 어긋난다.

`koopman_mpc_compare_numeric.mp4`: **수치 시뮬**(Stonefish 아님) 3분할 — 1-step QP(analytic) | Koopman Θ MPC H=5 | 참 모델 MPC H=5. 리더 사각 경로 0.1 m/s, 30 s마다 코너, 10배속. 코너 후 자리 오차 최대 0.138 / 0.277 / 0.087 m. 설명 노트 §10, `tools/koopman_mpc_lookahead.py`.

주의: 8×8 m 사각(기본 `leader_pilot` 경로)·직선 30 m 런은 플랫폼 원점 출발 시 리더 7.4 m 지점에서
수조 벽(반지름 ~8 m)에 닿아 깨진다 — 제어기와 무관하므로 여기 싣지 않았다. 4×4는 수조 안에 들어간다.
