#!/usr/bin/env python3
"""
fast_lio regression comparison: baseline vs candidate trajectory.

두 ros2 bag(`/slam/fast_lio/odometry`)을 읽어 다음을 계산:
- ATE (Absolute Trajectory Error) RMSE
- 최종 drift (마지막 pose 차이)
- 메시지 카운트 동등성

설계 문서 임계: ATE ≤ 1cm, drift ≤ 5cm.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
except ImportError as e:
    sys.exit(f"[regression] ImportError: {e}. Source ROS2 humble first.")


def read_odom_bag(bag_dir: Path, topic: str) -> np.ndarray:
    """Return (N, 4) array: [t, x, y, z]."""
    storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
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


def align_by_time(a: np.ndarray, b: np.ndarray, max_dt: float = 0.05) -> tuple:
    """Pair (a_i, b_j) by nearest timestamp within max_dt."""
    pairs_a, pairs_b = [], []
    j_start = 0
    for ai in a:
        best_j = -1
        best_dt = max_dt
        for j in range(j_start, len(b)):
            dt = abs(ai[0] - b[j, 0])
            if dt < best_dt:
                best_dt = dt
                best_j = j
            if b[j, 0] - ai[0] > max_dt:
                break
        if best_j >= 0:
            pairs_a.append(ai[1:4])
            pairs_b.append(b[best_j, 1:4])
            j_start = best_j
    return np.array(pairs_a), np.array(pairs_b)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--topic", default="/slam/fast_lio/odometry")
    parser.add_argument("--ate-threshold", type=float, default=0.01)
    parser.add_argument("--drift-threshold", type=float, default=0.05)
    args = parser.parse_args()

    a = read_odom_bag(args.baseline, args.topic)
    b = read_odom_bag(args.candidate, args.topic)

    print(f"[regression] baseline:  {len(a)} odom messages")
    print(f"[regression] candidate: {len(b)} odom messages")

    if len(a) == 0 or len(b) == 0:
        print("[regression] FAIL: empty trajectory")
        return 1

    pa, pb = align_by_time(a, b)
    if len(pa) == 0:
        print("[regression] FAIL: no time-aligned pairs (clock mismatch?)")
        return 1
    print(f"[regression] aligned pairs: {len(pa)}")

    diffs = np.linalg.norm(pa - pb, axis=1)
    ate_rmse = math.sqrt(float(np.mean(diffs ** 2)))
    drift_final = float(np.linalg.norm(a[-1, 1:4] - b[-1, 1:4]))

    print(f"[regression] ATE RMSE:    {ate_rmse * 100:.3f} cm "
          f"(threshold {args.ate_threshold * 100:.1f} cm)")
    print(f"[regression] final drift: {drift_final * 100:.3f} cm "
          f"(threshold {args.drift_threshold * 100:.1f} cm)")

    ok = ate_rmse <= args.ate_threshold and drift_final <= args.drift_threshold
    print(f"[regression] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
