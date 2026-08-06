"""기존 sim/ 패키지(gtbot_ws 루트) import 경로 주입 — 복사·재구현 금지(스펙)."""
import os, sys

def ensure():
    root = os.environ.get('GTBOT_WS_ROOT', '/root/home/gtbot_ws')
    if root not in sys.path:
        sys.path.insert(0, root)
