"""E1 게이트 1: 식별 재현 (w 무관·결정적). 실행: python3 -m sim.run_e1 --phase 1"""
import argparse, json, pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from .scenario import e1_scenario, TABLE2_CASES
from .experiment import run_phase1, open_loop_eval

RESULTS = pathlib.Path(__file__).resolve().parent.parent / "results"

def gate1():
    sc = e1_scenario()
    out = {}
    res = {m: run_phase1(sc, m) for m in ("linear", "bilinear")}
    for m, r in res.items():
        ol = open_loop_eval(sc, m, r.rls, r.X_log, r.U_log, start=260, horizon=30)
        out[m] = {
            "eps_peak_75win": [float(np.abs(r.eps_log[s:s + 75]).max()) for s in range(0, 300, 75)],
            "eps_norm_series": np.linalg.norm(r.eps_log, axis=1).tolist(),
            "eps_a_norm_series": np.linalg.norm(r.eps_a_log, axis=1).tolist(),
            "open_loop_rmse": float(np.sqrt((ol ** 2).mean())),
            "viol": r.viol_flags, "trP": r.trP, "zeta_rank": r.zeta_rank}
    lin, bil = out["linear"], out["bilinear"]
    better = np.array(bil["eps_norm_series"]) < np.array(lin["eps_norm_series"])
    out["gate1"] = {                                  # 스펙: 방향성 판정, 배수는 보고값
        "open_loop_bil_lt_lin": bool(bil["open_loop_rmse"] < lin["open_loop_rmse"]),
        "eps_bil_lt_lin_fraction": float(better.mean()),
        "ratio_report": float(np.median(np.array(lin["eps_norm_series"]) /
                                        np.maximum(np.array(bil["eps_norm_series"]), 1e-300))),
        "pass": bool(bil["open_loop_rmse"] < lin["open_loop_rmse"] and better.mean() > 0.5)}
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "e1_gate1.json").write_text(json.dumps(out, indent=2))
    fig, axes = plt.subplots(2, 1, figsize=(8, 6))
    for m in ("linear", "bilinear"):
        axes[0].semilogy(out[m]["eps_norm_series"], label=f"{m} |eps| (a priori)")
        axes[1].semilogy(out[m]["eps_a_norm_series"], label=f"{m} |eps_a| (Fig4/8 대조)")
    for ax in axes:
        ax.legend(); ax.set_xlabel("k")
    fig.savefig(RESULTS / "e1_gate1_errors.png", dpi=150)
    print(json.dumps(out["gate1"], indent=2))

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--phase", type=int, default=1)
    args = p.parse_args()
    if args.phase == 1:
        gate1()
