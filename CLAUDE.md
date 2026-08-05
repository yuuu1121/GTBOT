# GTBOT — Koopman 기반 다중로봇 편대제어 논문 프로젝트

이 저장소는 논문 작업 프로젝트다. 다른 PC에서 `git clone` 후 이 폴더에서 Claude Code를 실행하면 이 파일과 `research/`를 읽고 맥락을 복원한다.

## 목표

**Tao & Zhao(2023)의 "효용함수(utility function)를 Koopman 상태변수로 선정 → 선형/bilinear 근사 → linprog 최적제어" 프레임워크를 다중로봇 편대주행(formation control)으로 확장**하는 논문을 쓴다. 방향은 확정됨(Koopman 기반).

## 확정된 사실 (이 위에서 다시 논쟁하지 말 것)

- **토대 논문 2편**: `papers/2305.03777v1.pdf` (Part I 이론), `papers/2305.03778v1.pdf` (Part II 시뮬레이션). 텍스트 추출본(.txt)도 같은 폴더에 있어 grep 가능.
- **핵심 확장 경로**: 원논문 효용함수의 φ⁶(로봇간 충돌회피, 상대거리 dij 클수록 좋음)를 **φ⁷(편대유지, dij가 목표값 근처일 때 최댓값, 예 `exp(−(dij−d_desired)²/σ²)`)** 로 일반화. 입력변수가 같아 Koopman변수화→bilinear근사→linprog 파이프라인 그대로 재사용.
- **관련연구 결론**: "효용함수+Koopman+linprog+다중로봇" 조합은 Tao&Zhao 외 없음 → 방향 참신함, 논문거리 됨. 상세는 `research/related-work.md`.
- **반드시 넘을 벽**: Rosenfelder et al. 2024 "Data Does Not Replace Geometry" (arXiv 2411.07192) — 비홀로노믹 실로봇서 bilinear Koopman이 기하구조 보존 못함. 반드시 인용·대응.
- **시뮬레이션 재현 한계 확정**: 공개 설정만으로는 데이터 기반 bilinear Koopman 식별이 미결정계(a=901 vs ks=300, rank 270)로 재현 불가 — 참 그래디언트 해석적 팔로 전환해 게이트 2 정식 5/5 확보. 상세는 `research/sim-results.md`.
- **φ⁷ 편대유지 항 시뮬레이션 검증 완료**: 게이트 3(E2) 4/4·게이트 4(E3, N=3/5/7) 3/3 전부 통과. 원논문 φ² 순항속도 vs φ⁶ 반응거리 스케일 불일치를 v_cruise=1.0 이탈로 해결.

## 다음 단계

1. ~~논문 아웃라인 (IMRaD) 작성~~ → `draft/outline.md` (완료)
2. ~~편대유지 효용항 φ⁷ 수식 설계~~ → `draft/phi7-design.md` (완료)
3. ~~Related Work 정리 + BibTeX~~ → `research/related-work.md`, `draft/references.bib` (완료)
4. ~~3-robot 원논문 재현~~ → 게이트 1~4 시뮬레이션 캠페인, `research/sim-results.md` (완료)
5. **본문 집필** (미착수) — outline.md·phi7-design.md·sim-results.md 기반으로 IMRaD 초안 작성
6. **투고 직전 관련연구 재검색** (미착수) — 아래 "주의" 절 참조

## 주의

- **투고 직전 관련연구 재검색 필수** — "부재의 증명"이라 새 논문이 나오면 뒤집힘. arXiv/IEEE 중심 조사였고 CDC/ACC/IROS 프로시딩·CNKI는 미스캔.
- 원논문은 코드·효용함수 가중치 수치 미공개 → 재현은 유도 패턴 따라 직접 구현.

## 작업 규칙

- 사용자 언어: 한국어.
- 논문 원고·조사 결과는 이 저장소에 커밋해 크로스머신 공유. 커밋 메시지는 무엇을·왜 중심으로.
