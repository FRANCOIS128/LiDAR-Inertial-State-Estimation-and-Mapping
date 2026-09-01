#!/usr/bin/env python3
"""Plot trajectory and map."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_xyz(path):
    t, xyz = [], []
    with open(path) as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(",")
            t.append(float(parts[0]))
            xyz.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(t), np.array(xyz)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated", default="output/estimated_trajectory.csv")
    parser.add_argument("--gt", default="output/ground_truth.csv")
    parser.add_argument("--out", default="results/trajectory.png")
    parser.add_argument("--map", default="output/map_xy.csv")
    parser.add_argument("--map-out", default="results/map.png")
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(5, 4))
    if Path(args.estimated).exists():
        _, p = load_xyz(args.estimated)
        ax.plot(p[:, 0], p[:, 1], label="estimated")
    if Path(args.gt).exists():
        _, p = load_xyz(args.gt)
        ax.plot(p[:, 0], p[:, 1], "--", label="ground truth")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectory")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print("wrote", args.out)

    if Path(args.map).exists():
        pts = np.loadtxt(args.map, delimiter=",", skiprows=1)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)
        keep = (pts[:, 2] > 0.4) & (pts[:, 2] < 2.4)
        pts = pts[keep] if np.any(keep) else pts

        mfig, maxis = plt.subplots(figsize=(5, 4))
        maxis.scatter(pts[:, 0], pts[:, 1], s=1, c="tab:blue")
        maxis.set_xlabel("x (m)")
        maxis.set_ylabel("y (m)")
        maxis.set_title("Map")
        maxis.axis("equal")
        maxis.grid(True)
        mfig.tight_layout()
        mfig.savefig(args.map_out, dpi=120)
        print("wrote", args.map_out)


if __name__ == "__main__":
    main()
