# 관련연구 조사 결과

deep-research 하네스(웹 검색 fan-out → 소스 fetch → 적대적 검증 → 종합)로 2026-08 조사. 8편 핵심 논문 검증. 원시 결과는 `deep-research-raw.json`.

## 결론 (한 문장)

**"Tao & Zhao의 효용함수-Koopman-linprog 프레임에 편대유지 항을 추가"하는 방향은 기존 문헌과 직접 겹치지 않아 논문거리가 된다.** 단, 비홀로노믹 실로봇 적용 시 "데이터가 기하학을 대체하지 못한다"는 확인된 한계(Rosenfelder 2024)를 정면으로 다뤄야 한다.

## 기존 Koopman-편대 연구 (목적함수 구조가 전부 다름 = 우리에게 유리)

| 연구 | 방법 | 목적 | Tao&Zhao와 차이 |
|---|---|---|---|
| Wang, Baba & Hikihara 2022 (arXiv 2205.04052) | EDMD | 리더-팔로워 위치 **예측** | 순수 예측 도구, 최적제어·linprog 없음. 옴니휠 2대만 검증 |
| Zhao & Zhu 2023 (arXiv 2309.16098) | Koopman + **Stackelberg 게임** | 리더가 팔로워 전략적 유도 | 비대칭 계층 구조, 효용함수 최적화 아님 |
| Zhan et al. 2024 (IEEE TSMC doc 10659151) | Koopman lifting | **DoS 공격 강건** 편대(신호복구) | 가변이득 강건 제어기, utility+linprog 아님. 비홀로노믹 NMR 대상 |
| Rosenfelder et al. 2024 (arXiv 2411.07192) | bilinear EDMD | 비홀로노믹 로봇 제어 | **"데이터≠기하학" 한계 규명** (실하드웨어). 반드시 인용 |
| Bold et al. 2023 (arXiv 2303.09144) | bilinear EDMD | 단일 비홀로노믹 로봇 사례연구 | 편대·다중로봇 아님 |
| Higuchi & Sato 2026 (arXiv 2607.25658) | bilinear Koopman **강건 MPC** | 단일 비선형 시스템 | 다중로봇/편대 전혀 없음 |
| (서베이) Koopman Operators in Robot Learning (arXiv 2408.04200) | — | 로보틱스 Koopman 전반 지형도 | utility-function 세부방법론은 좁은 틈새 |

## 차별성 (핵심)

**"효용함수(utility function)를 Koopman 상태변수로 선정 → 선형/bilinear 근사 → linprog"는 Tao&Zhao 특유.** 조사한 8편 모두 예측·게임이론·신호복구·강건MPC 등 다른 목적함수 구조. 상위 분야(Koopman+다중로봇)는 레드오션이지만 정확한 교차점(효용함수-Koopman-linprog-편대)은 비어 있음.

## 공백 (질문 4)

- (a) **bilinear Koopman을 편대에 직접 적용한 사례 없음** → 우리의 기회.
- (b) 로봇 수 확장성(3대 초과) 문헌 전반 취약.
- (c) 비홀로노믹 실로봇 편대 검증 취약.

## 유보 (반드시 유의)

1. "부재의 증명"이라 새 논문 나오면 뒤집힘 → **투고 직전 재검색 필수**.
2. arXiv/IEEE 중심 조사. CDC/ACC/IROS 프로시딩·CNKI 미스캔.
3. Tao&Zhao는 arXiv 프리프린트로 시작했으나 IEEE Transactions 정식 게재됨(ADS 2025ITCST..33..963Z) — 신뢰도 있음.
4. "항 하나 추가"는 기여 얇을 위험 → 비홀로노믹 대응 + 확장성 실험으로 두껍게.

## 인용 목록 (BibTeX 대상)

- arXiv:2305.03777, arXiv:2305.03778 (토대)
- arXiv:2205.04052, arXiv:2309.16098, IEEE 10659151 (Koopman-편대)
- arXiv:2411.07192, arXiv:2303.09144 (bilinear 비홀로노믹 한계)
- arXiv:2607.25658 (bilinear Koopman MPC), arXiv:2408.04200 (서베이)
