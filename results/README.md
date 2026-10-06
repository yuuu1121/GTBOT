# results/ — 결과 산출물 색인

루트에는 게이트 스크립트(`gate_s1`~`gate_s6`)가 **매 실행마다 덮어쓰는 최신 판정**만 둔다
(`s1`~`s6_stonefish.json`, `s6_edges.csv`). 캠페인 기록은 시작일 접두어 폴더에 보관한다.
파일별 의미·판정 기준은 `research/sim-results.md`가 정본이다(파일명으로 참조).

| 폴더 | 내용 |
|:---|:---|
| `2026-08-05_numeric_e1-e3/` | 수치 캠페인 게이트 1~4: E1 원논문 재현(5케이스 궤적 png, 식별·해석적 팔 json), E2 φ⁷ 편대유지, E3 N=3/5/7 확장성. `sim/run_e1.py`·`run_e3.py`가 여기에 쓴다 |
| `2026-08-06_stonefish_s3_rounds/` | Stonefish S3(편대 형성·유지) 라운드별 판정 — model 팔 발산, analytic 전환, 지연 보상, rate/umax 스윕 |
| `2026-08-07_stonefish_s6_rounds/` | S6(LiDAR 추정 기반) 라운드별 판정 json + 원시 로그 csv(edges·koopman·probe·still) |
| `sweep/` | 저하 시나리오 스윕(편의·누락·파랑) S4·S5·S6 판정 |
| `feedback/` | 되먹임 프로브(ab_lead·ab_vel) csv·json, 2026-08-14~17 |
| `figs/` | 문서용 그림, `stonefish_scene.png` 포함 |
| `pre_real_matching_20260817/` | 실기 매칭 전 기준 |
| `real_lidar_data/` | 실기 LiDAR 백(rosbag) |
| `video/` | 영상(git 비추적) |
| `2026-10-06/` | model 팔 검증 런(ψ 확장 사전·논문 형태·analytic 대조, 4×4 사각 + 리더 정지) — 영상 6편, 런 csv 6개, 수치 MPC 비교 영상 |

새 결과는 `results/<YYYY-MM-DD>_<주제>/` + README 한 줄로 추가한다.
