"""E2 게이트 3: 편대 유지 (φ⁷) vs 베이스라인 (같은 리프팅, w7=0). 실행: python3 -m sim.run_e2
게이트 3 미달 시 σ·w7 파라미터 반복(스펙: 최대 5회, e2_tuning_log.json 기록, 이 두 값만 허용)."""
import dataclasses, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from .scenario import e2_scenario, E2_INIT, E2_TARGETS
from .experiment import run_phase2
from .run_e1 import _plot_traj, RESULTS

def formation_error(X_log, formation, n):
    J = []
    for X in X_log:
        pos = X[:2 * n].reshape(n, 2)
        J.append(sum((np.linalg.norm(pos[a] - pos[b]) - d) ** 2 for (a, b), d in formation.items()))
    return np.array(J)

def _run_arm(use_phi7, sigma, w7):
    sc = e2_scenario(use_phi7_weight=use_phi7)
    sc = dataclasses.replace(sc, sigma=sigma,
                              w_robot=np.array([-1.0, -1.0, 5.0, 2.0, -2.0, -2.0, w7 if use_phi7 else 0.0]))
    r = run_phase2(sc, None, "bilinear", E2_INIT, E2_TARGETS, sc.control_iters, controller="analytic")
    J = formation_error(r.X_log, sc.formation, sc.n_robots)
    pos_f = r.X_log[-1][:2 * sc.n_robots].reshape(sc.n_robots, 2)
    edge_final = {f"{a}-{b}": abs(float(np.linalg.norm(pos_f[a] - pos_f[b])) - d)
                  for (a, b), d in sc.formation.items()}
    stats = {"J0": float(J[0]), "J_final": float(J[-1]), "J_peak": float(J.max()),
             "edge_final_abs_err": edge_final, "controller": "analytic",
             "collision_free": bool(r.min_robot_surf > 0 and r.min_wall_surf > 0)}
    return sc, r, J, stats

def run_gate3(sigma=3.0, w7=3.0):
    """두 팔 실행 + 게이트 3 판정. sigma·w7는 phi7 팔만 오버라이드(베이스라인 w7=0 고정, 차원 통제)."""
    sc7, r7, J7, p = _run_arm(True, sigma, w7)
    scb, rb, Jb, b = _run_arm(False, sigma, w7)
    gate = {"J_final_lt_10pct_J0": p["J_final"] < 0.1 * p["J0"],
            "edges_within_0p1_sigma": all(v < 0.1 * sigma for v in p["edge_final_abs_err"].values()),
            "peak_lt_baseline": p["J_peak"] < b["J_peak"],
            "collision_free": p["collision_free"]}
    gate["pass"] = all(gate.values())
    out = {"phi7": p, "baseline": b, "gate3": gate, "params": {"sigma": sigma, "w7": w7}}
    return out, {"phi7": (sc7, r7, J7), "baseline": (scb, rb, Jb)}

def _score(gate):
    return sum(gate[k] for k in ("J_final_lt_10pct_J0", "edges_within_0p1_sigma",
                                  "peak_lt_baseline", "collision_free"))

def main():
    out, runs = run_gate3()                          # 1차: 기본값(sigma=3.0, w7=3.0)
    if not out["gate3"]["pass"]:
        tuning_log = [{"sigma": 3.0, "w7": 3.0, "gate3": out["gate3"]}]
        for sigma, w7 in ((8.0, 10.0), (6.0, 20.0), (10.0, 20.0), (5.0, 15.0), (8.0, 12.0)):
            trial, trial_runs = run_gate3(sigma, w7)
            tuning_log.append({"sigma": sigma, "w7": w7, "gate3": trial["gate3"]})
            if _score(trial["gate3"]) > _score(out["gate3"]):
                out, runs = trial, trial_runs
            if trial["gate3"]["pass"]:
                break
        (RESULTS / "e2_tuning_log.json").write_text(json.dumps(tuning_log, indent=2))
    for arm in ("phi7", "baseline"):
        sc, r, J = runs[arm]
        np.save(RESULTS / f"e2_J_{arm}.npy", J)
        _plot_traj(r.X_log, np.asarray(E2_TARGETS), sc, RESULTS / f"e2_traj_{arm}.png")
    (RESULTS / "e2_gate3.json").write_text(json.dumps(out, indent=2))
    fig, ax = plt.subplots()
    for arm in ("phi7", "baseline"):
        ax.plot(np.load(RESULTS / f"e2_J_{arm}.npy"), label=arm)
    ax.set_xlabel("k"); ax.set_ylabel("J(k)"); ax.legend()
    fig.savefig(RESULTS / "e2_formation_error.png", dpi=150)
    print(json.dumps(out["gate3"], indent=2))
    if out["gate3"]["pass"]:
        conflict_run()
    return out

def conflict_run(d_star=4.6):
    """φ⁶/φ⁷ 상충 관찰(게이트 통과 시에만): d*를 2Rr+0.6 근처로 낮춰 phi7 팔 1회 실행, 판정 없음."""
    sigma, w7 = 3.0, 3.0
    sc = e2_scenario(use_phi7_weight=True)
    sc = dataclasses.replace(sc, sigma=sigma, formation={k: d_star for k in sc.formation})
    r = run_phase2(sc, None, "bilinear", E2_INIT, E2_TARGETS, sc.control_iters, controller="analytic")
    J = formation_error(r.X_log, sc.formation, sc.n_robots)
    report = {"d_star": d_star, "J0": float(J[0]), "J_final": float(J[-1]), "J_peak": float(J.max()),
              "min_robot_surf": float(r.min_robot_surf)}
    (RESULTS / "e2_conflict_run.json").write_text(json.dumps(report, indent=2))
    return report

if __name__ == "__main__":
    main()
