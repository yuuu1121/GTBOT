# 논문 아웃라인 (IMRaD)

작업 제목(가제): **Utility-Function-Based Koopman Approximation for Multi-Robot Formation Control via Linear Programming**

토대: `research/paper-analysis.md`(원논문 분석) + `research/related-work.md`(8편 조사). 본문은 영어로 작성 예정, 이 아웃라인은 한국어 작업 문서.

---

## I. Introduction

1. **동기**: 다중로봇 편대제어는 비선형 목적(편대유지+충돌회피+목표추종)의 실시간 최적화 문제. 기존 접근(MPC, 게임이론, 합의제어)은 비선형 최적화 비용이 크다.
2. **기존 틀**: Tao & Zhao(2023)가 효용함수 자체를 Koopman 상태변수로 선정 → bilinear 근사 → LP로 실시간 풀이하는 프레임워크 제시. 단, 목표추종+충돌회피만 다루고 **편대 개념 부재**.
3. **공백**: "효용함수-Koopman-LP" 조합을 편대제어로 확장한 연구 없음 (related-work.md 8편 검증).
4. **기여 (3개)**:
   - C1: 편대유지 효용항 φ⁷ 설계 — 기존 파이프라인(Koopman변수화→bilinear→RLS→linprog) 재사용 가능한 형태로.
   - C2: 3-robot 시뮬레이션에서 원논문 재현 + 편대 시나리오 검증 (linear vs bilinear 비교 포함).
   - C3: 로봇 수 확장성 분석 — 편대 항의 조합적(C(N,2)) 증가가 bilinear 파라미터 규모·LP 풀이 시간에 미치는 영향 (원논문 미검증 지점).
5. **스코프 명시 (Rosenfelder 대응 1차)**: 본 논문은 이중적분기(홀로노믹) 동역학 가정. 비홀로노믹 확장의 구조적 한계는 §VI에서 논의.

## II. Related Work

- **A. Koopman 기반 다중로봇/편대**: Wang 2022(EDMD 예측), Zhao & Zhu 2023(Stackelberg), Zhan 2024(DoS 강건) — 전부 목적함수 구조가 다름 → 표로 대비.
- **B. Bilinear Koopman 제어**: Bold 2023(단일 비홀로노믹), Higuchi & Sato 2026(강건 MPC), Rosenfelder 2024(데이터≠기하학 한계).
- **C. 효용함수 기반 접근**: Tao & Zhao Part I/II — 본 논문의 직접 토대. 서베이(arXiv 2408.04200)로 지형도 인용.
- 인용 목록·BibTeX 대상은 `research/related-work.md` §인용 목록.

## III. Preliminaries and Problem Formulation

1. **로봇 모델**: N대 이중적분기, 상태 X∈R⁴ᴺ, 제어=가속도, Δt=0.05s (원논문 동일).
2. **원논문 효용함수 요약**: u_i = Σⱼωᵢ⁽ʲ⁾φᵢ⁽ʲ⁾, 6항(방향정렬·속도·위치근접·정지·벽회피·로봇간회피).
3. **문제 정의**: 편대유지 목적 추가 — 로봇쌍 (i,j)의 상대거리 dij를 목표값 d*ij 근처로 유지하면서 목표점 도달 + 충돌회피.
4. **Koopman 틀 요약**: z₁=X(선형·정확), z₂=효용함수값(bilinear 근사), U-affine 구조 → LP 변환 가능성 보존.

## IV. Method: Formation-Aware Utility Design

1. **φ⁷ 설계**: 편대유지 항 — dij가 d*ij 근처일 때 최대. 후보 `exp(−(dij−d*ij)²/σ²)` (원논문 φ⁶와 같은 함수족: 로봇쌍 상대거리의 함수).
   - 설계 쟁점 ①: **거리 기반 편대는 회전·반사 대칭까지만 결정** — 절대 배치가 필요하면 앵커/방위 항 추가 필요. 본 논문 스코프: 거리 기반(상대 편대)으로 한정할지 결정.
   - 설계 쟁점 ②: φ⁶(회피)와 φ⁷(유지)의 가중치 상충 — d*ij > 안전거리 조건 명시.
   - 설계 쟁점 ③: σ 선택이 bilinear 근사 품질에 미치는 영향 (좁으면 비선형성 급증 → RLS 추정 악화 예상).
2. **Koopman 변수화·bilinear 근사**: 원논문 유도 패턴 그대로 φ⁷ 포함 효용에 적용. U-affine 구조 유지 증명(또는 유도).
3. **파라미터 추정**: RLS + 게인리셋 + 투영 (원논문 동일 — 재서술 최소화, 차이점만).
4. **LP 변환**: linprog 정식화. 편대 항 추가가 제약/목적 구조에 주는 변화.

## V. Simulations

1. **Setup**: 원논문 Part II 재현 조건(3-robot, 원형 벽 r=11m, 로봇 r=2m, 식별 300 iter, 제어 30 iter). 코드·가중치 미공개 → 유도 패턴 따라 직접 구현(재현 기준: 정성적 거동 + 사후추정오차 규모).
2. **E1 — 원논문 재현**: φ¹~φ⁶만으로 linear vs bilinear. 검증: bilinear만 성공, 추정오차 ~10⁻⁴ 재현.
3. **E2 — 편대 시나리오**: φ⁷ 추가, 삼각 편대 유지하며 목표 이동. 지표: 편대오차 Σ|dij−d*ij|, 목표도달시간, 최소 로봇간 거리.
4. **E3 — 확장성**: N=3,5,(가능하면 7...) — bilinear 파라미터 수, RLS 수렴, LP 풀이시간 vs 실시간 한계(Δt·제어주기).
5. **E4 (선택) — ablation**: σ·가중치 스윕, φ⁶/φ⁷ 상충 거동.

## VI. Discussion

1. **비홀로노믹 한계 (Rosenfelder 대응 2차)**: bilinear Koopman이 기하구조를 보존 못하는 실증적 근거 인용 → 본 방법이 이중적분기에서 유효한 이유와 비홀로노믹 확장 시 예상 실패 모드. 가능한 출구(기하 보존 lifting, 원논문 동작점 재정의 대안) 스케치.
2. **확장성 한계**: 파라미터 901×901(3-robot)이 N에 조합적 증가 — E3 결과 기반 논의.
3. **전역성**: 원논문 자인("전역적이지 않을 수 있음")이 편대 시나리오에서 어떻게 나타나는지.

## VII. Conclusion

기여 요약 + 향후과제(비홀로노믹, 실로봇, decentralized 병렬화 실측).

---

## 미결 사항 (아웃라인 확정 전 결정 필요)

- [ ] 거리 기반(상대 편대) vs 절대 배치 — 스코프 결정 (§IV 쟁점 ①)
- [ ] E3 최대 N — 계산 자원 보고 결정
- [ ] 투고처 미정 → 분량·포맷 제약 미반영
- [ ] 투고 직전 관련연구 재검색 (CLAUDE.md 주의사항)
