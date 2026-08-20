#!/usr/bin/env python3
"""
Plot baseline vs candidate trajectory (XY top-down view) from regression bags.

ATE 수치 비교는 RANSAC 비결정성 때문에 의미가 없지만,
궤적 모양(shape)은 시각적으로 비교 가능합니다.
두 궤적이 거의 겹치면 알고리즘 회귀 없음을 의미.
"""
import sys
from pathlib import Path

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
except ImportError as e:
    sys.exit(f"[plot] ImportError: {e}. Source ROS2 humble first.")

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("[plot] matplotlib not installed. apt install python3-matplotlib")


def read_xyz(bag_dir: Path, topic: str = "/slam/fast_lio_loc/odometry") -> np.ndarray:
    storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage, converter)

    rows = []
    while reader.has_next():
        topic_name, data, t_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        rows.append((t_ns * 1e-9, p.x, p.y, p.z))
    return np.array(rows, dtype=np.float64)


def main() -> int:
    base_dir = Path("/tmp/fast_lio_localization_regression")
    out_path = base_dir / "trajectory_comparison.png"

    baseline = read_xyz(base_dir / "baseline" / "odom_bag")
    candidate = read_xyz(base_dir / "candidate" / "odom_bag")

    if len(baseline) == 0 or len(candidate) == 0:
        print(f"[plot] empty trajectories: baseline={len(baseline)} candidate={len(candidate)}")
        return 1

    _, axes = plt.subplots(1, 3, figsize=(18, 6))

    # XY top-down
    ax = axes[0]
    ax.plot(baseline[:, 1], baseline[:, 2], "b-", label=f"baseline ({len(baseline)} pts)", alpha=0.7, lw=1.5)
    ax.plot(candidate[:, 1], candidate[:, 2], "r--", label=f"candidate ({len(candidate)} pts)", alpha=0.7, lw=1.5)
    ax.scatter([baseline[0, 1]], [baseline[0, 2]], c="green", s=80, marker="o", label="start", zorder=5)
    ax.scatter([baseline[-1, 1]], [baseline[-1, 2]], c="black", s=80, marker="x", label="end", zorder=5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY trajectory (top-down)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # X over time
    ax = axes[1]
    t0 = baseline[0, 0]
    ax.plot(baseline[:, 0] - t0, baseline[:, 1], "b-", label="baseline x", alpha=0.7, lw=1)
    ax.plot(baseline[:, 0] - t0, baseline[:, 2], "b--", label="baseline y", alpha=0.7, lw=1)
    ax.plot(candidate[:, 0] - candidate[0, 0], candidate[:, 1], "r-", label="candidate x", alpha=0.7, lw=1)
    ax.plot(candidate[:, 0] - candidate[0, 0], candidate[:, 2], "r--", label="candidate y", alpha=0.7, lw=1)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [m]")
    ax.set_title("X / Y over time")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Z over time
    ax = axes[2]
    ax.plot(baseline[:, 0] - t0, baseline[:, 3], "b-", label="baseline z", alpha=0.7, lw=1.5)
    ax.plot(candidate[:, 0] - candidate[0, 0], candidate[:, 3], "r--", label="candidate z", alpha=0.7, lw=1.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("z [m]")
    ax.set_title("Z over time")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.suptitle(
        f"Baseline (post-B-1) vs Candidate (B-2a) localization trajectory\n"
        f"baseline: {len(baseline)} msgs, candidate: {len(candidate)} msgs",
        fontsize=12,
    )
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    print(f"[plot] saved: {out_path}")

    # Quick numeric summary
    if len(baseline) > 10 and len(candidate) > 10:
        # Compare last 100 points (final converged state)
        n_compare = min(100, len(baseline), len(candidate))
        b_end = baseline[-n_compare:, 1:4].mean(axis=0)
        c_end = candidate[-n_compare:, 1:4].mean(axis=0)
        diff = np.linalg.norm(b_end - c_end)
        print(f"[plot] mean position of last {n_compare} points:")
        print(f"  baseline:  ({b_end[0]:.3f}, {b_end[1]:.3f}, {b_end[2]:.3f})")
        print(f"  candidate: ({c_end[0]:.3f}, {c_end[1]:.3f}, {c_end[2]:.3f})")
        print(f"  diff:      {diff*100:.2f} cm")

    return 0


if __name__ == "__main__":
    sys.exit(main())
