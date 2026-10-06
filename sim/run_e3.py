"""E3 게이트 4: 확장성 계측 — 해석적 그래디언트 제어 팔(식별 팔은 기각, 레저 참조).
실행: python3 -m sim.run_e3 [--n 3 5 7]"""
import argparse, json, pathlib, time
import numpy as np
from .scenario import e3_scenario
from .experiment import run_phase2
from .koopman import zeta_dim
from .run_e2 import formation_error

RESULTS = pathlib.Path(__file__).resolve().parent.parent / "results" / "2026-08-05_numeric_e1-e3"
NOTE = ("identification arm rejected (see ledger) — zeta_dim/P_bytes reported as analysis of "
        "the rejected arm's scaling; control measurements are for the analytic arm")

def measure(n_robots):
    sc = e3_scenario(n_robots)                 # dataclasses.replace 미사용 — _e3_init 보존 (레저)
    a = zeta_dim(sc.n_robots, sc.m)
    init = sc._e3_init[:2 * sc.n_robots].reshape(-1, 2)
    t0 = time.perf_counter()
    r = run_phase2(sc, None, "bilinear", init, sc.targets, sc.control_iters, controller="analytic")
    t_ctrl = (time.perf_counter() - t0) / sc.control_iters
    J = formation_error(r.X_log, sc.formation, sc.n_robots)
    return {"N": n_robots, "zeta_dim": a, "P_bytes": 8 * a * a, "per_step_control_s": t_ctrl,
            "realtime_ok": bool(t_ctrl < sc.dt),
            "formation_J0": float(J[0]), "formation_J_final": float(J[-1]),
            "min_robot_surf": float(r.min_robot_surf), "min_wall_surf": float(r.min_wall_surf),
            "collision_free": bool(r.min_robot_surf > 0 and r.min_wall_surf > 0),
            "reached": bool((r.min_target_dists < 0.5).all()),
            "controller": "analytic"}

def gate4(row):
    return (row["realtime_ok"] and row["collision_free"] and row["reached"]
            and row["formation_J_final"] < 0.1 * row["formation_J0"])

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--n", type=int, nargs="+", default=[3, 5, 7])
    rows = [measure(n) for n in ap.parse_args().n]
    for row in rows:
        row["note"] = NOTE
        row["gate4_pass"] = gate4(row)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "e3_gate4.json").write_text(json.dumps(rows, indent=2))
    print(json.dumps(rows, indent=2))

if __name__ == "__main__":
    main()
