#!/usr/bin/env python3
"""Create separate, single-axis English plots for a takeover trial."""

import argparse
import csv
import json
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
    parser.add_argument('--output-dir', type=Path,
                        help='Optional presentation output; raw trial data are never changed')
    args = parser.parse_args()
    trajectory = read_csv(args.trial_dir / "trajectory.csv")
    route = load_route(args.waypoints)
    output = args.output_dir or args.trial_dir
    output.mkdir(parents=True, exist_ok=True)

    fig, axis = plt.subplots(figsize=(7.5, 9.0), constrained_layout=True)
    axis.plot(route[:, 0], route[:, 1], "--", color="0.35", linewidth=1.2, label="Reference route")
    axis.plot(numeric(trajectory, "ego_x"), numeric(trajectory, "ego_y"),
              color="#1764ab", linewidth=2.0, label="Ego actual trajectory")
    axis.plot(numeric(trajectory, "lead_x"), numeric(trajectory, "lead_y"),
              '-.', color="#d1495b", linewidth=2.0, label="Lead actual trajectory")
    summary = json.loads((args.trial_dir / 'summary.json').read_text())
    times = numeric(trajectory, 'time_s')
    for i, event in enumerate(summary.get('overtaking', {}).get('truth_events', []), 1):
        row = trajectory[int(np.argmin(np.abs(times-event['completion_time_s'])))]
        x, y = float(row['ego_x']), float(row['ego_y'])
        axis.plot(x, y, 's', color='black', markersize=5)
        axis.annotate(f"Pass {i} rejoined: {event['completion_time_s']:.1f} s",
                      (x, y), xytext=(-8, 12 if i % 2 else -18), textcoords='offset points',
                      ha='right', fontsize=9)
    axis.set_title("Dynamic Overtaking: Actual Trajectories")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_aspect('equal', adjustable='box')
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    fig.savefig(output / "trajectory_en.png", dpi=180)
    plt.close(fig)

    prediction_path = args.trial_dir / "predictions.csv"
    if not prediction_path.exists():
        return
    predictions = read_csv(prediction_path)
    snapshots = {}
    for row in predictions:
        snapshots.setdefault(row["status_stamp_s"], []).append(row)
    entries = []
    previous_behavior = None
    for snapshot in snapshots.values():
        behavior = snapshot[0].get('behavior')
        if behavior == 'OVERTAKE' and previous_behavior != behavior:
            entries.append(snapshot)
        previous_behavior = behavior
    if not entries and snapshots:
        entries.append(next(iter(snapshots.values())))
    for index, selected in enumerate(entries, 1):
        plot_prediction(output, route, selected, index)


def plot_prediction(trial_dir, route, selected, index):
    reference = np.asarray([
        [float(row["reference_x"]), float(row["reference_y"])]
        for row in selected if row["reference_x"] not in ("", "None")
    ])
    lead = np.asarray([
        [float(row["lead_prediction_x"]), float(row["lead_prediction_y"])]
        for row in selected if row["lead_prediction_x"] not in ("", "None")
    ])
    fig, axis = plt.subplots(figsize=(8.0, 5.5), constrained_layout=True)
    axis.plot(route[:, 0], route[:, 1], "--", color="0.35", linewidth=1.0, label="Reference route")
    if reference.size:
        axis.plot(reference[:, 0], reference[:, 1], color="#1764ab", linewidth=2.2,
                  label="Time-aware ego reference")
    if lead.size:
        axis.plot(lead[:, 0], lead[:, 1], "o-", color="#d1495b", markersize=3,
                  label="Lead motion prediction")
    axis.set_title(f"Entry {index}: Time-Aligned Model Forecast (not ground truth)")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    shown = [points for points in (reference, lead) if points.size]
    if shown:
        points = np.vstack(shown)
        axis.set_xlim(points[:, 0].min()-1, points[:, 0].max()+1)
        axis.set_ylim(points[:, 1].min()-1, points[:, 1].max()+1)
        axis.set_aspect('equal', adjustable='box')
    axis.grid(alpha=0.25)
    axis.legend(loc='upper left', bbox_to_anchor=(1.03, 1.), fontsize=9)
    name = 'prediction_en.png' if index == 1 else f'prediction_entry_{index}_en.png'
    fig.savefig(trial_dir / name, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
