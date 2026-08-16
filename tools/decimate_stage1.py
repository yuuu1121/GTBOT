#!/usr/bin/env python3.11
"""1단계 메시 감량 — 검출과 무관한 시각 메시만 줄인다.

배경(2026-08-16 계측): 시뮬은 실시간을 놓치지 않는다(RTF 0.998)는데 렌더 기반
센서만 선언값의 3분의 1 이하다 — LiDAR 3.62 Hz(36%), 카메라 2.48 Hz(25%),
래스터화를 안 거치는 odometry만 10.00 Hz(100%). 병목이 렌더이므로 장면 삼각형
49만 개를 줄이는 것이 직접적인 처방이다.

이 단계에서 건드리는 것(검출 무관):
  ccw/cw.obj      프로펠러 16개. 수면 아래라 LiDAR 빔이 닿지 않고 강도 컷에도 걸린다.
  platform_viz    플랫폼 선체. 검출기가 z-밴드·강도로 이미 배제하는 대상.

건드리지 않는 것:
  GTBOT.obj       판이 붙어 있어 검출 캘리브레이션과 직결(2단계에서 S4 검증과 함께).
  *_phy.obj       부력·항력 계산에 쓰인다 — 바꾸면 거동이 달라져 게이트가 무효가 된다.

원본은 백업 후 덮어쓴다. 플랫폼 선체는 CAD 원본에서 다시 뽑는 편이 품질이 낫다.
"""
import pathlib
import shutil

import pymeshlab

ROOT = pathlib.Path(__file__).resolve().parent.parent
BAK = ROOT / 'src/gtbot_description/data/.mesh_backup_20260816'
BAK.mkdir(parents=True, exist_ok=True)

# (원본, 산출, 목표 면 수) — 원본이 CAD면 거기서, 아니면 현행 파일을 다시 줄인다
JOBS = [
    ('models/PKRCfloatingplatform.obj',
     'src/gtbot_description/data/objects/platform/meshes/platform_viz.obj', 30_000),
    ('src/gtbot_description/data/robots/gtbot/meshes/ccw.obj',
     'src/gtbot_description/data/robots/gtbot/meshes/ccw.obj', 500),
    ('src/gtbot_description/data/robots/gtbot/meshes/cw.obj',
     'src/gtbot_description/data/robots/gtbot/meshes/cw.obj', 500),
]


def faces(p):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(p))
    return ms.current_mesh().face_number()


for src, dst, target in JOBS:
    sp, dp = ROOT / src, ROOT / dst
    if dp.exists():                       # 되돌릴 수 있게 먼저 백업
        b = BAK / dp.name
        if not b.exists():
            shutil.copy2(dp, b)
            print(f'  백업 {dp.name} -> {b}')
    before = faces(dp) if dp.exists() else None
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(sp))
    ms.apply_filter('meshing_decimation_quadric_edge_collapse',
                    targetfacenum=target, preservenormal=True)
    ms.save_current_mesh(str(dp))
    after = faces(dp)
    print(f'{dst.split("/")[-1]:<20} {before} -> {after} 면 '
          f'({100*(1-after/before):.0f}% 감소)' if before else
          f'{dst.split("/")[-1]:<20} -> {after} 면')
print(f'\n백업 위치: {BAK}')
