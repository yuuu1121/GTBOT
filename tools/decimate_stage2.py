#!/usr/bin/env python3.11
"""2단계 메시 감량 — GTBOT.obj(로봇 시각, 판 포함) 8만 -> 3만 면.

1단계(프로펠러·플랫폼 선체)에서 LiDAR 3.62 -> 7.86 Hz, 카메라 2.48 -> 6.74 Hz를
얻었고 S4는 변화 없이 통과했다. 남은 삼각형 35.8만 중 24만이 이 파일 3대분이라
여기가 마지막 큰 덩어리다.

**원본은 CAD(models/GTBOT.obj)가 아니라 현행 data 파일에서 줄인다.** CAD 원본은
판을 붙이기 전 버전이고, 검출 파이프라인 전체가 판이 붙은 현행 형상에 맞춰
캘리브레이션돼 있다. 여기서 출처를 잘못 잡으면 판이 통째로 사라진다.

판은 평면이라 quadric edge collapse에서 오차 0으로 보존된다(평면은 삼각형 몇 개로
정확히 표현된다). 곡면인 선체만 거칠어진다. 그래도 감량 후 S4로 검출 품질을
확인해야 채택한다 — 이론이 아니라 실측으로 판정한다.
"""
import pathlib
import shutil

import pymeshlab

ROOT = pathlib.Path(__file__).resolve().parent.parent
BAK = ROOT / 'src/gtbot_description/data/.mesh_backup_20260816'
BAK.mkdir(parents=True, exist_ok=True)
DST = ROOT / 'src/gtbot_description/data/robots/gtbot/meshes/GTBOT.obj'
TARGET = 30_000

b = BAK / DST.name
if not b.exists():
    shutil.copy2(DST, b)
    print(f'백업 {DST.name} -> {b}')

ms = pymeshlab.MeshSet()
ms.load_new_mesh(str(DST))
m = ms.current_mesh()
before_f, before_bb = m.face_number(), m.bounding_box()
print(f'전  {before_f} 면, bbox 대각 {before_bb.diagonal():.4f}')
ms.apply_filter('meshing_decimation_quadric_edge_collapse',
                targetfacenum=TARGET, preservenormal=True, preserveboundary=True)
ms.save_current_mesh(str(DST))

ms2 = pymeshlab.MeshSet()
ms2.load_new_mesh(str(DST))
m2 = ms2.current_mesh()
print(f'후  {m2.face_number()} 면, bbox 대각 {m2.bounding_box().diagonal():.4f}')
d = abs(m2.bounding_box().diagonal() - before_bb.diagonal())
print(f'bbox 대각 변화 {d:.5f} — 형상 외곽이 유지됐는지의 1차 확인')
