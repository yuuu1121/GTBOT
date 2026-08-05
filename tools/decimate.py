#!/usr/bin/env python3.11
"""원본 CAD OBJ → Stonefish용 감량 메시. 사용: python3.11 tools/decimate.py
pymeshlab이 python3.11에만 설치됨 — 시스템 python3(3.10)엔 없음.
원본은 models/ 에서 읽기만 하고, 산출은 src/gtbot_description/data/ 아래에 쓴다."""
import pathlib
import pymeshlab

ROOT = pathlib.Path(__file__).resolve().parent.parent
JOBS = [  # (원본, 산출, 목표 삼각형 수)
    ("models/GTBOT.obj", "src/gtbot_description/data/robots/gtbot/meshes/gtbot_viz.obj", 150_000),
    ("models/GTBOT.obj", "src/gtbot_description/data/robots/gtbot/meshes/gtbot_phy.obj", 20_000),
    ("models/PKRCfloatingplatform.obj", "src/gtbot_description/data/objects/platform/meshes/platform_viz.obj", 150_000),
    ("models/PKRCfloatingplatform.obj", "src/gtbot_description/data/objects/platform/meshes/platform_phy.obj", 20_000),
]

for src, dst, target in JOBS:
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(ROOT / src))
    ms.apply_filter("meshing_decimation_quadric_edge_collapse",
                    targetfacenum=target, preservenormal=True)
    out = ROOT / dst
    out.parent.mkdir(parents=True, exist_ok=True)
    ms.save_current_mesh(str(out), save_vertex_normal=True)
    m = ms.current_mesh()
    size_mb = out.stat().st_size / 1e6
    print(f"{dst}: {m.face_number()} tris, {size_mb:.1f} MB")
    assert size_mb < 100, f"{dst} exceeds GitHub 100MB limit"
