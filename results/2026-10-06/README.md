# 2026-10-06 — model 팔(bilinear Koopman Θ) Stonefish 런

`koopman_formation controller:=model` (공칭 Θ₀ + 그래디언트 확장 사전, 제어 중 동결) 검증 런.
LiDAR 추정 기반, `formation.launch.py start_leader:=false` → S2 → `settle_wait` → `leader_pilot` 절차.
편대 오차는 `koopman_*.csv`의 GT 열(g0..g11, odometry 기준 상대상태)로 계산 — 이 환경에서 ros2 CLI가
SHM 프로파일 없이 토픽을 못 봐 정식 gate_s6는 돌리지 못했다.

| 파일 | 조건 | edge 오차 중앙 / p90 / 최대 [m] |
|:---|:---|:---|
| `koopman_model_sq4.csv` + 영상 2편 | 리더 4×4 m 사각 1.7바퀴(377 s) | 0.163 / 0.265 / 0.378, est 유효율 100 %, 무충돌 |
| `koopman_model_leader_still_run1.csv` | 리더 정지, 237 s | 0.164 / 0.285 / 0.393 (정착 후) |
| `koopman_model_leader_still_run2.csv` | 리더 정지, 249 s | 0.134 / 0.289 / 0.379 (정착 후) |
| `koopman_analytic_leader_still.csv` | analytic 대조, 리더 정지, 250 s | 0.164 / 0.258 / 0.405 (정착 후) |

영상: `koopman_model_sq4_topview.mp4`(탑뷰 10배속, 궤적·변 오차 HUD), `koopman_model_sq4_simview.mp4`(시뮬 카메라 3배속).

주의: 8×8 m 사각(기본 `leader_pilot` 경로)·직선 30 m 런은 플랫폼 원점 출발 시 리더 7.4 m 지점에서
수조 벽(반지름 ~8 m)에 닿아 깨진다 — 제어기와 무관하므로 여기 싣지 않았다. 4×4는 수조 안에 들어간다.
