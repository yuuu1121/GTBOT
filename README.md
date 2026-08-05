# GTBOT

**Koopman 기반 다중로봇 편대제어 논문 프로젝트**

Tao & Zhao(2023)의 효용함수-Koopman-linprog 최적제어 프레임워크를 다중로봇 편대주행(formation control)으로 확장하는 연구.

## 구조

```
├── CLAUDE.md              프로젝트 맥락·방향·다음 단계 (다른 PC에서 Claude가 읽음)
├── papers/                토대 논문 2편 (PDF + 텍스트 추출본)
│   ├── 2305.03777v1.*     Part I: Concepts and Formulations
│   └── 2305.03778v1.*     Part II: Simulations and Evaluations
├── research/
│   ├── related-work.md    관련연구 조사 결과 (차별성·공백·인용목록)
│   ├── paper-analysis.md  토대 논문 Part I/II 정독 분석
│   ├── sim-results.md     시뮬레이션 캠페인 결과 종합
│   └── deep-research-raw.json   조사 원시 결과
├── sim/                   시뮬레이션 스택 (E1 재현·E2 편대유지·E3 확장성)
├── results/                게이트 판정 JSON·그림
└── draft/                 논문 원고 (작성 예정)
```

## 다른 PC에서 이어가기

```
git clone https://github.com/yuuu1121/GTBOT.git
cd GTBOT
claude          # CLAUDE.md와 research/를 읽고 맥락 복원
```

## 시뮬레이션 실행

```
python3 -m pytest sim/tests          # repo 루트에서 실행 — sim/tests/에 __init__.py 없어 -m 형태 필수
python3 -m sim.run_e1 --phase 1|2|2b|2c|2d
python3 -m sim.run_e2
python3 -m sim.run_e3
```

의존성: numpy, scipy≥1.6(linprog HiGHS), matplotlib, pytest.

## 현재 상태

단계 1~4 완료: ①IMRaD 아웃라인 ②편대유지 효용항 φ⁷ 수식 설계 ③Related Work + BibTeX ④시뮬레이션 캠페인(`research/sim-results.md` 참조). 남은 것은 본문 집필 + 투고 직전 관련연구 재검색.
