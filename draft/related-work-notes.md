# Related Work 노트 (§II 초안 재료)

`research/related-work.md` 조사 결과를 `draft/outline.md` §II의 A/B/C 그룹 구조로 재구성한 것. 새 주장 추가 없음 — 문단화·인용 키 매핑만.

## A. Koopman 기반 다중로봇/편대

Wang, Baba & Hikihara (2022)는 EDMD 기반 Koopman 연산자로 리더-팔로워 편대의 위치를 **예측**하는 방법을 제시했다 [wang2022koopman]. 옴니휠 로봇 2대로만 검증되었고, 최적제어나 linprog 정식화는 다루지 않는 순수 예측 도구다.

Zhao & Zhu (2023)는 Koopman 연산자와 **Stackelberg 게임이론**을 결합해 리더가 팔로워의 전략적 반응을 유도하는 궤적 안내 기법을 제시했다 [zhao2023stackelberg]. 비대칭 계층 구조(리더-팔로워)를 다루며, 효용함수 최적화 프레임과는 목적함수 구조 자체가 다르다.

Zhan et al. (2024)은 Koopman lifting을 이용해 **DoS(서비스거부) 공격에 강건한** 편대 제어기를 설계했다 [zhan2024dos]. 통신 두절 시 신호 복구가 목표이며, 비홀로노믹 NMR(networked mobile robots)을 대상으로 가변이득 강건 제어기를 사용한다. 효용함수+linprog 구조는 포함하지 않는다.

세 연구 모두 Tao & Zhao의 "효용함수를 Koopman 상태변수로 선정" 프레임과는 목적함수 구조가 근본적으로 다르다는 공통점이 있다.

## B. Bilinear Koopman 제어

Bold et al. (2023)은 단일 비홀로노믹 로봇에 대한 bilinear EDMD 대리모델(surrogate model) 사례연구를 수행했다 [bold2023bilinear]. 편대나 다중로봇은 다루지 않는다.

Higuchi & Sato (2026)는 bilinear Koopman 기반 **강건 MPC**를 contraction metric과 결합해 미지 비선형 단일 시스템에 적용했다 [higuchi2026robust]. 다중로봇/편대 요소는 전혀 포함되지 않는다.

Rosenfelder et al. (2024)은 비홀로노믹 로봇의 실하드웨어 실험을 통해 bilinear Koopman 근사가 **기하구조(geometry)를 보존하지 못한다**는 한계를 실증적으로 규명했다 ("Data Does Not Replace Geometry") [rosenfelder2024data]. 본 논문이 반드시 정면으로 대응해야 할 핵심 제약이며, 이중적분기 가정 하에서의 유효 범위와 비홀로노믹 확장 시 예상 실패 모드를 §VI에서 논의할 근거가 된다.

## C. 효용함수 기반 접근 (본 논문의 직접 토대)

Tao & Zhao의 2편(Part I 개념·정식화 [tao2023part1], Part II 시뮬레이션·평가 [tao2023part2])은 효용함수 자체를 Koopman 상태변수로 선정하여 선형/bilinear 근사 후 linprog로 최적제어를 푸는 프레임워크를 제시한다. 본 논문의 직접적 토대이며, 목표추종+충돌회피만 다루고 편대유지 개념이 없다는 것이 확장 지점이다.

Koopman Operators in Robot Learning 서베이(2024/2026 게재)는 로보틱스 전반의 Koopman 연산자 적용 지형도를 정리한다 [koopman2024survey]. "효용함수를 상태변수로 선정"하는 세부방법론은 이 지형도 안에서도 좁은 틈새로 남아 있다는 것을 뒷받침하는 인용으로 사용한다.

## 차별성 요약

조사한 8편 모두 예측·게임이론·신호복구·강건MPC 등 Tao & Zhao와 다른 목적함수 구조를 갖는다. "효용함수-Koopman-linprog" 조합을 다중로봇 편대유지로 확장한 연구는 확인되지 않았다 — 상세 근거·유보사항은 `research/related-work.md` 참조.
