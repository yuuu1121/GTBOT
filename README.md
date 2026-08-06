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
├── src/gtbot_description/ Stonefish 시뮬레이션용 ROS 2 패키지 (씬·런치·메시)
├── src/gtbot_formation/   편대제어 ROS 2 패키지 (koopman_formation·velocity_loop·leader_pilot·게이트 S1~S3)
├── tools/decimate.py      원본 CAD 메시 감량 스크립트
├── models/                원본 CAD (git 비추적, 이 머신 로컬 전용)
├── results/                게이트 판정 JSON·그림
└── draft/                 논문 원고 (작성 예정)
```

로컬 폴더명은 `gtbot_ws`(Stonefish ROS 2 워크스페이스 겸용). GitHub 저장소명은 `GTBOT`로 유지(clone 시 대상 디렉토리명만 지정).

## 다른 PC에서 이어가기

```
git clone https://github.com/yuuu1121/GTBOT.git gtbot_ws
cd gtbot_ws
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

## Stonefish 씬 실행

감량 메시는 커밋돼 있어 clone만으로 씬 실행 가능. `tools/decimate.py` 재실행은 원본 `models/*.obj`(git 비추적, 이 머신 로컬 전용)가 필요.

```
source <Stonefish+stonefish_ros2가 빌드된 워크스페이스>/install/setup.bash   # 이 머신에선 /root/home/vlm_ws
colcon build --packages-select gtbot_description && source install/setup.bash
ros2 launch gtbot_description gtbot_world.launch.py
```

씬에는 로봇 편대 외에 `ObserverCam`(고정 카메라, `/gtbot_world/view` 토픽) — 씬 로드 검증·스크린샷 확보용 관측 장치이며 편대제어 로직과 무관.

## 게이트 S1~S3 재현 (Stonefish 편대제어)

```
ros2 launch gtbot_description gtbot_world.launch.py   # 시뮬 기동
ros2 launch gtbot_formation formation.launch.py        # koopman_formation·velocity_loop×3·leader_pilot 기동
# koopman_formation 로그에서 "S2: {...}" (제어 전환) 확인 후 30 s 대기
ros2 run gtbot_formation gate_s3                        # 정상상태 편대유지 판정(180 s)
```

상세 수치·판정 기준은 `research/sim-results.md` "Stonefish 캠페인" 절 참조.

## 현재 상태

단계 1~4 완료: ①IMRaD 아웃라인 ②편대유지 효용항 φ⁷ 수식 설계 ③Related Work + BibTeX ④시뮬레이션 캠페인(`research/sim-results.md` 참조). Stonefish 편대제어 캠페인(게이트 S1~S3 전부 통과, 2026-08-06) 완료. 남은 것은 본문 집필 + 투고 직전 관련연구 재검색.
