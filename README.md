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
│   └── deep-research-raw.json   조사 원시 결과
└── draft/                 논문 원고 (작성 예정)
```

## 다른 PC에서 이어가기

```
git clone https://github.com/yuuu1121/GTBOT.git
cd GTBOT
claude          # CLAUDE.md와 research/를 읽고 맥락 복원
```

## 현재 상태

방향 확정(참신성 검증 완료). 다음 단계는 `CLAUDE.md` 참조: ①IMRaD 아웃라인 ②편대유지 효용항 φ⁷ 수식 설계 ③Related Work + BibTeX ④3-robot 재현.
