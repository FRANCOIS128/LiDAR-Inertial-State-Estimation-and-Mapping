#!/usr/bin/env python3
"""Compare estimated trajectory with ground truth."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_traj(path):
    t, xyz, quat = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["timestamp"]))
            xyz.append([float(row["x"]), float(row["y"]), float(row["z"])])
            quat.append(
                [
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                ]
            )
    return np.array(t), np.array(xyz), np.array(quat)


def interpolate(t_src, x_src, t_query):
    out = np.zeros((len(t_query), x_src.shape[1]))
    for i in range(x_src.shape[1]):
        out[:, i] = np.interp(t_query, t_src, x_src[:, i])
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated", default="output/estimated_trajectory.csv")
    parser.add_argument("--gt", default="output/ground_truth.csv")
    parser.add_argument("--out", default="results/position_error.png")
    parser.add_argument("--metrics", default="results/metrics.csv")
    args = parser.parse_args()

    t_est, p_est, _ = load_traj(args.estimated)
    t_gt, p_gt, _ = load_traj(args.gt)

    p_gt_i = interpolate(t_gt, p_gt, t_est)
    err = p_est - p_gt_i
    dist = np.linalg.norm(err, axis=1)
    rmse = float(np.sqrt(np.mean(dist**2)))
    ate = rmse  # same world frame, I do not do Sim(3) align
    mae = float(np.mean(dist))
    final = float(dist[-1]) if len(dist) else float("nan")

    Path(args.metrics).parent.mkdir(parents=True, exist_ok=True)
    with open(args.metrics, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerow(["rmse_m", f"{rmse:.6f}"])
        w.writerow(["ate_rmse_m", f"{ate:.6f}"])
        w.writerow(["mae_m", f"{mae:.6f}"])
        w.writerow(["final_error_m", f"{final:.6f}"])
        w.writerow(["samples", str(len(dist))])

    fig, ax = plt.subplots(figsize=(5, 3.5))
    ax.plot(t_est, dist)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m)")
    ax.set_title("Position error")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)

    print(f"RMSE      : {rmse:.4f} m")
    print(f"ATE RMSE  : {ate:.4f} m")
    print(f"MAE       : {mae:.4f} m")
    print(f"final err : {final:.4f} m")
    print("wrote", args.out)
    print("wrote", args.metrics)


if __name__ == "__main__":
    main()
