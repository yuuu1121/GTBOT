"""E1 게이트 1: 식별 재현 (w 무관·결정적). 실행: python3 -m sim.run_e1 --phase 1"""
import argparse, json, pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from .scenario import e1_scenario, TABLE2_CASES
from .experiment import run_phase1, run_phase2, open_loop_eval, frozen_1step_eval, operating_point

RESULTS = pathlib.Path(__file__).resolve().parent.parent / "results"

def gate1():
    sc = e1_scenario()
    out = {}
    res = {m: run_phase1(sc, m) for m in ("linear", "bilinear")}
    for m, r in res.items():
        f1 = frozen_1step_eval(sc, m, r.rls, r.X_log, r.U_log)
        ol = open_loop_eval(sc, m, r.rls, r.X_log, r.U_log, start=260, horizon=30)
        diverged = bool(not np.isfinite(ol).all())         # 재귀 개루프는 기록 전용 (스펙 개정)
        out[m] = {
            "eps_peak_75win": [float(np.abs(r.eps_log[s:s + 75]).max()) for s in range(0, 300, 75)],
            "eps_norm_series": np.linalg.norm(r.eps_log, axis=1).tolist(),
            "eps_a_norm_series": np.linalg.norm(r.eps_a_log, axis=1).tolist(),
            "frozen_1step_rmse": float(np.sqrt((f1 ** 2).mean())),
            "open_loop_diverged": diverged,
            "open_loop_rmse": None if diverged else float(np.sqrt((ol ** 2).mean())),
            "viol": r.viol_flags, "trP": r.trP, "zeta_rank": r.zeta_rank}
    lin, bil = out["linear"], out["bilinear"]
    better = np.array(bil["eps_norm_series"]) < np.array(lin["eps_norm_series"])
    out["gate1"] = {                                  # 스펙 개정: 동결-Θ teacher-forced 1-step 판정 (LP가 소비하는 것과 용도 일치)
        "frozen_1step_bil_lt_lin": bool(bil["frozen_1step_rmse"] < lin["frozen_1step_rmse"]),
        "eps_bil_lt_lin_fraction": float(better.mean()),
        "ratio_report": float(np.median(np.array(lin["eps_norm_series"]) /
                                        np.maximum(np.array(bil["eps_norm_series"]), 1e-300))),
        "pass": bool(bil["frozen_1step_rmse"] < lin["frozen_1step_rmse"])}
    out["metric_notes"] = ("eps_bil_lt_lin_fraction은 리셋 스케줄·과소결정 과도응답을 반영 — 모델 품질 증거 아님; "
                            "bilinear trP=p0*dim은 리셋 항등식, zeta_rank는 표본 상한(100) — 둘 다 진단 증거로 쓸 수 없음")
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "e1_gate1.json").write_text(json.dumps(out, indent=2, allow_nan=False))
    fig, axes = plt.subplots(2, 1, figsize=(8, 6))
    for m in ("linear", "bilinear"):
        axes[0].semilogy(out[m]["eps_norm_series"], label=f"{m} |eps| (a priori)")
        axes[1].semilogy(out[m]["eps_a_norm_series"], label=f"{m} |eps_a| (Fig4/8 대조)")
    for ax in axes:
        ax.legend(); ax.set_xlabel("k")
    fig.savefig(RESULTS / "e1_gate1_errors.png", dpi=150)
    print(json.dumps(out["gate1"], indent=2))

def _plot_traj(X_log, targets, sc, path):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.add_patch(plt.Circle((0, 0), sc.wall_radius, fill=False))
    n = sc.n_robots
    for i in range(n):
        xs, ys = X_log[:, 2 * i], X_log[:, 2 * i + 1]
        ax.plot(xs, ys)
        ax.add_patch(plt.Circle((xs[-1], ys[-1]), sc.robot_radius, alpha=0.3))
        ax.plot(*targets[i], "x")
    ax.set_aspect("equal"); ax.set_xlim(-12, 12); ax.set_ylim(-12, 12)
    fig.savefig(path, dpi=150); plt.close(fig)

def gate2():
    import copy
    sc = e1_scenario()
    base = run_phase1(sc, "bilinear")
    report = {}
    for case, (init, tgt) in TABLE2_CASES.items():   # 식별 1회, 케이스별 deepcopy (:1450~1453)
        r = run_phase2(sc, copy.deepcopy(base.rls), "bilinear", init, tgt, sc.control_iters)
        report[case] = {"final_target_dists": r.final_target_dists.tolist(),
                        "min_target_dists": r.min_target_dists.tolist(),
                        "reached": bool((r.min_target_dists < 0.5).all()),     # 스펙: 지평 내 0.5m 이내
                        "min_robot_surf": r.min_robot_surf, "min_wall_surf": r.min_wall_surf,
                        "collision_free": bool(r.min_robot_surf > 0 and r.min_wall_surf > 0),
                        "lp_events": r.lp_events}
        _plot_traj(r.X_log, tgt, sc, RESULTS / f"e1_case_{case}_traj.png")
    report["gate2"] = {
        "pass_conditions_1_2": all(v["reached"] and v["collision_free"]
                                   for k, v in report.items() if k != "gate2"),
        "condition3": "Case IV/V 그림 육안 확인 필요 — 자동 판정 아님 (스펙 게이트 2 ③)"}
    report["oracle_note"] = ("참 그래디언트 오라클은 지평내 최소 기준 5/5 통과(0.03~0.15m) — "
                              "파이프라인 정상, 병목은 식별 (리뷰어 검증, task-8-report.md 참조)")
    (RESULTS / "e1_gate2.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["gate2"], indent=2))

def sign_agreement(rls, sc, probe_states, w_full, h=1e-4, tol=1e-9):
    """모델 c vs 유한차분 참 1-step 그래디언트의 부호 일치율 (검증 전용 — 제어에 미사용)."""
    from .dynamics import ab_matrices
    from .control import input_objective
    from .utility import z2_vector
    A, B = ab_matrices(sc.n_robots, sc.dt)
    z10, z20 = operating_point(sc)
    n_in = 2 * sc.n_robots
    match, total = 0, 0
    for X in probe_states:
        z2 = z2_vector(X, sc)
        c_model, _ = input_objective(rls.theta, X - z10, z2 - z20, w_full, n_in, reduced=sc.reduced_lifting)
        base = w_full @ z2_vector(A @ X, sc)
        for l in range(n_in):
            U = np.zeros(n_in); U[l] = h
            c_true = (w_full @ z2_vector(A @ X + B @ U, sc) - base) / h
            if abs(c_true) > tol:
                total += 1
                match += int(np.sign(c_true) == np.sign(c_model[l]))
    return match / max(total, 1)

def gate2_improved():
    import copy, dataclasses
    from .scenario import IMPROVED_EPISODES, improved_episode_state, e3_input
    sc = dataclasses.replace(e1_scenario(), ks=600, reduced_lifting=True)   # 폴백: g1·g3 제거 ζ(a=433) — 600샘플 대비 결정계
    episodes = [improved_episode_state(i) for i in range(len(IMPROVED_EPISODES))]
    p1 = run_phase1(sc, "bilinear", input_fn=lambda k: e3_input(k, sc.n_robots),
                    episodes=episodes, p_reset=False)     # 시불변계 — P 리셋 없음, P0=100I가 ridge
    probes = episodes + [np.concatenate([np.array(init).ravel(), np.zeros(6)])
                         for init, _ in TABLE2_CASES.values()]
    sa = sign_agreement(p1.rls, sc, probes, sc.w_full)
    tail_rmse = float(np.sqrt((p1.eps_log[-75:] ** 2).mean()))
    ident = {"sign_agreement": sa, "tail_eps_rmse": tail_rmse, "viol": p1.viol_flags,
             "gate8b_pass": bool(sa >= 0.8 and tail_rmse < 0.1)}
    (RESULTS / "e1_ident_improved.json").write_text(json.dumps(ident, indent=2))
    print(json.dumps(ident, indent=2))
    if not ident["gate8b_pass"]:
        return                                            # 폴백(ζ 축소)은 컨트롤러 결정 — 여기서 멈춤
    report = {}
    for case, (init, tgt) in TABLE2_CASES.items():
        r = run_phase2(sc, copy.deepcopy(p1.rls), "bilinear", init, tgt, sc.control_iters)
        report[case] = {"min_target_dists": r.min_target_dists.tolist(),
                        "reached": bool((r.min_target_dists < 0.5).all()),
                        "min_robot_surf": r.min_robot_surf, "min_wall_surf": r.min_wall_surf,
                        "collision_free": bool(r.min_robot_surf > 0 and r.min_wall_surf > 0)}
        _plot_traj(r.X_log, tgt, sc, RESULTS / f"e1_case_{case}_traj_improved.png")
    report["gate2"] = {"pass_conditions_1_2": all(v["reached"] and v["collision_free"]
                                                  for v in report.values() if isinstance(v, dict) and "reached" in v),
                       "identification": "improved (episodic, no P reset, ks=600)"}
    (RESULTS / "e1_gate2_improved.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["gate2"], indent=2))

def gate2_local():
    """국소 식별(라운드 3): 케이스별 전개점=목표, 회랑 근방 에피소드로 식별 → 케이스별 국소 게이트 판정."""
    import copy, dataclasses
    from .scenario import local_episodes, e3_input
    report = {}
    for case, (init, tgt) in TABLE2_CASES.items():
        sc_c = dataclasses.replace(e1_scenario(), targets=np.asarray(tgt), ks=1200, reduced_lifting=True)
        p1 = run_phase1(sc_c, "bilinear", input_fn=lambda k: 0.3 * e3_input(k, 3),
                        episodes=local_episodes(init, tgt), episode_len=15, p_reset=False)
        init_a, tgt_a = np.array(init), np.array(tgt)
        direction = tgt_a - init_a
        unit = direction / np.linalg.norm(direction, axis=1, keepdims=True)
        probe_pos = [init_a, tgt_a] + [init_a + f * direction for f in (0.25, 0.5, 0.75)]
        probes = [np.concatenate([p.ravel(), np.zeros(6)]) for p in probe_pos]
        for f in (0.25, 0.75):        # 목표 방향 2 m/s 속도 중간점 — 제어 상태 분포 커버 (라운드 4)
            probes.append(np.concatenate([(init_a + f * direction).ravel(), (2.0 * unit).ravel()]))
        sa = sign_agreement(p1.rls, sc_c, probes, sc_c.w_full)
        tail_rmse = float(np.sqrt((p1.eps_log[-15:] ** 2).mean()))
        ident_pass = bool(sa >= 0.8 and tail_rmse < 0.1)
        entry = {"sign_agreement": sa, "tail_rmse": tail_rmse, "ident_pass": ident_pass}
        if ident_pass:                                        # 국소 게이트 통과 케이스만 Phase II 진행
            r = run_phase2(sc_c, copy.deepcopy(p1.rls), "bilinear", init, tgt, 30)
            entry.update({"min_target_dists": r.min_target_dists.tolist(),
                         "reached": bool((r.min_target_dists < 0.5).all()),
                         "min_robot_surf": r.min_robot_surf, "min_wall_surf": r.min_wall_surf,
                         "collision_free": bool(r.min_robot_surf > 0 and r.min_wall_surf > 0)})
            _plot_traj(r.X_log, tgt, sc_c, RESULTS / f"e1_case_{case}_traj_local.png")
        report[case] = entry
    report["gate2"] = {"pass_conditions_1_2": all(v.get("ident_pass") and v.get("reached") and v.get("collision_free")
                                                  for v in report.values())}
    (RESULTS / "e1_gate2_local.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--phase", default="1")
    args = p.parse_args()
    if args.phase == "1":
        gate1()
    elif args.phase == "2":
        gate2()
    elif args.phase == "2b":
        gate2_improved()
    elif args.phase == "2c":
        gate2_local()
