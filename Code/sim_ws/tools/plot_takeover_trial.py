#!/usr/bin/env python3
"""Create separate, single-axis English plots for a takeover trial."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def numeric(rows, key):
    return np.asarray([float(row[key]) for row in rows], dtype=float)


def load_route(path):
    values = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.reader(handle):
            if not row or row[0].lstrip().startswith("#"):
                continue
            values.append([float(row[0]), float(row[1])])
    return np.asarray(values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("waypoints", type=Path)
    args = parser.parse_args()
    trajectory = read_csv(args.trial_dir / "trajectory.csv")
    route = load_route(args.waypoints)

    fig, axis = plt.subplots(figsize=(9.0, 5.5), constrained_layout=True)
    axis.plot(route[:, 0], route[:, 1], "--", color="0.55", linewidth=1.2, label="Reference route")
    axis.plot(numeric(trajectory, "ego_x"), numeric(trajectory, "ego_y"),
              color="#1764ab", linewidth=2.0, label="Ego actual trajectory")
    axis.plot(numeric(trajectory, "lead_x"), numeric(trajectory, "lead_y"),
              color="#d1495b", linewidth=2.0, label="Lead actual trajectory")
    axis.set_title("Dynamic Overtaking: Actual Trajectories")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    fig.savefig(args.trial_dir / "trajectory_en.png", dpi=180)
    plt.close(fig)

    prediction_path = args.trial_dir / "predictions.csv"
    if not prediction_path.exists():
        return
    predictions = read_csv(prediction_path)
    snapshots = {}
    for row in predictions:
        snapshots.setdefault(row["status_stamp_s"], []).append(row)
    selected = None
    for snapshot in snapshots.values():
        if snapshot[0].get("behavior") == "OVERTAKE":
            selected = snapshot
            break
    if selected is None and snapshots:
        selected = next(iter(snapshots.values()))
    if not selected:
        return
    reference = np.asarray([
        [float(row["reference_x"]), float(row["reference_y"])]
        for row in selected if row["reference_x"] not in ("", "None")
    ])
    lead = np.asarray([
        [float(row["lead_prediction_x"]), float(row["lead_prediction_y"])]
        for row in selected if row["lead_prediction_x"] not in ("", "None")
    ])
    fig, axis = plt.subplots(figsize=(8.0, 5.5), constrained_layout=True)
    axis.plot(route[:, 0], route[:, 1], "--", color="0.65", linewidth=1.0, label="Reference route")
    if reference.size:
        axis.plot(reference[:, 0], reference[:, 1], color="#1764ab", linewidth=2.2,
                  label="Time-aware ego reference")
    if lead.size:
        axis.plot(lead[:, 0], lead[:, 1], "o-", color="#d1495b", markersize=3,
                  label="Constant-velocity lead prediction")
    axis.set_title("Representative Time-Aligned Prediction")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    fig.savefig(args.trial_dir / "prediction_en.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
