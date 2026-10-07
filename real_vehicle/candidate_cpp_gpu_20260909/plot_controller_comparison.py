#!/usr/bin/env python3
"""Render real-run PP/PID/MPC trajectory comparisons from frozen traces."""
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
ROUTE_FILE = WORKSPACE / "routes/manual_route_20260904_215501_clearance_v1.csv"
MAP_FILE = WORKSPACE / "maps/ps4_slam_20260904_163930.pgm"
OUTPUT = ROOT / "comparison_figures"

RUNS = {
    "PP v25": ROOT / "ground_route_pp_20260910_v25/run_logs/20260910_131124-pf-pp-ground-attempt",
    "Heading PID v1": ROOT / "ground_route_pid_20260910_v1/run_logs/20260910_231649-pf-pp-ground-attempt",
    "Linear MPC v3（未完成）": ROOT / "ground_route_mpc_20260911_v3/run_logs/20260911_121131-pf-pp-ground-attempt",
    "Dynamic MPC v5": ROOT / "ground_route_dynamic_mpc_20260912_v5/run_logs/20260912_112104-pf-pp-ground-attempt",
}
COLORS = {
    "PP v25": "#2563eb",
    "Heading PID v1": "#d97706",
    "Linear MPC v3（未完成）": "#788c32",
    "Dynamic MPC v5": "#c026d3",
}
LINESTYLES = {
    "PP v25": "-",
    "Heading PID v1": "-.",
    "Linear MPC v3（未完成）": ":",
    "Dynamic MPC v5": "--",
}


def load_route():
    with ROUTE_FILE.open() as stream:
        rows = list(csv.DictReader(stream))
    return np.array([[float(row["x"]), float(row["y"]), float(row["yaw"])] for row in rows])


def load_motion_trace(path):
    data = json.loads((path / "tracking_trace.json").read_text())
    commands = data["controller"]
    active = [item for item in commands if item.get("speed_command", 0) > 0]
    start = active[0]["received_mono"]
    completed = [item for item in commands if item.get("complete") is True]
    if completed:
        end = completed[0]["received_mono"]
        complete = True
    else:
        furthest = max(item.get("progress_index", 0) for item in commands)
        end = next(item["received_mono"] for item in commands if item.get("progress_index", 0) == furthest)
        complete = False
    poses = [item for item in data["poses"] if start - 0.1 <= item["received_mono"] <= end + 0.1]
    return poses, complete


def route_errors(poses, route):
    samples = []
    for pose in poses:
        distances = np.square(route[:, 0] - pose["x"]) + np.square(route[:, 1] - pose["y"])
        index = int(np.argmin(distances))
        x, y, yaw = route[index]
        cte = -math.sin(yaw) * (pose["x"] - x) + math.cos(yaw) * (pose["y"] - y)
        heading = math.atan2(math.sin(pose["yaw"] - yaw), math.cos(pose["yaw"] - yaw))
        samples.append((index, cte * 100.0, math.degrees(heading)))
    # Median per route index prevents different callback counts from changing line weight.
    values = []
    for index in sorted(set(item[0] for item in samples)):
        at_index = [item for item in samples if item[0] == index]
        values.append((index, float(np.median([item[1] for item in at_index])),
                       float(np.median([item[2] for item in at_index]))))
    return np.asarray(values)


def setup_map(ax, image, extent):
    ax.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=254,
              interpolation="nearest", alpha=0.68, zorder=0)
    ax.set_xlim(-1.45, 2.25)
    ax.set_ylim(-0.38, 2.72)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#cbd5e1", linewidth=0.6, alpha=0.45)
    ax.set_xlabel("地图 x（m）")
    ax.set_ylabel("地图 y（m）")


def main():
    plt.rcParams.update({
        # Matplotlib 3.1 exposes the system TTC under its first face name.
        # The glyph coverage includes Simplified Chinese.
        "font.family": "Noto Sans CJK JP",
        "axes.edgecolor": "#475569",
        "axes.labelcolor": "#1f2937",
        "xtick.color": "#475569",
        "ytick.color": "#475569",
        "text.color": "#111827",
        "figure.facecolor": "#f8fafc",
        "axes.facecolor": "#ffffff",
    })
    OUTPUT.mkdir(exist_ok=True)
    route = load_route()
    map_image = np.asarray(Image.open(MAP_FILE))
    resolution = 0.05
    origin_x, origin_y = -5.45, -1.2
    extent = [origin_x, origin_x + map_image.shape[1] * resolution,
              origin_y, origin_y + map_image.shape[0] * resolution]
    traces = {name: load_motion_trace(path) for name, path in RUNS.items()}

    single_names = {
        "PP v25": "pp_v25_path.png",
        "Heading PID v1": "pid_v1_path.png",
        "Linear MPC v3（未完成）": "linear_mpc_v3_path.png",
        "Dynamic MPC v5": "dynamic_mpc_v5_path.png",
    }
    for name, (poses, complete) in traces.items():
        fig, ax = plt.subplots(figsize=(8.5, 7.5))
        setup_map(ax, map_image, extent)
        ax.plot(route[:, 0], route[:, 1], color="#111827", linewidth=2.2,
                linestyle=(0, (4, 3)), label="参考路线", zorder=2)
        xs = [pose["x"] for pose in poses]
        ys = [pose["y"] for pose in poses]
        ax.plot(xs, ys, color=COLORS[name], linewidth=2.8, label="实车 PF 轨迹", zorder=3)
        ax.scatter(route[0, 0], route[0, 1], s=80, marker="o", facecolor="white",
                   edgecolor="#111827", linewidth=1.5, zorder=4, label="起点")
        ax.scatter(xs[-1], ys[-1], s=85, marker="s" if complete else "X",
                   color=COLORS[name], edgecolor="white", linewidth=1.0, zorder=4,
                   label="完成" if complete else "中断位置")
        ax.set_title(name + " 实车路径跟踪", fontsize=16, fontweight="bold", pad=12)
        ax.legend(loc="upper right", fontsize=10, framealpha=0.94)
        note = "完成路线 149/149" if complete else "路线索引 114/149 中断"
        fig.text(0.5, 0.025, note + "；轨迹为闭环中使用的粒子滤波位姿。",
                 ha="center", fontsize=10, color="#475569")
        fig.tight_layout(rect=[0, 0.045, 1, 1])
        fig.savefig(OUTPUT / single_names[name], dpi=200, bbox_inches="tight")
        plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex=True, sharey=True)
    for ax, (name, (poses, complete)) in zip(axes.flat, traces.items()):
        setup_map(ax, map_image, extent)
        ax.plot(route[:, 0], route[:, 1], color="#111827", linewidth=2.0,
                linestyle=(0, (4, 3)), label="参考路线", zorder=2)
        xs = [pose["x"] for pose in poses]
        ys = [pose["y"] for pose in poses]
        ax.plot(xs, ys, color=COLORS[name], linewidth=2.5, label="实车 PF 轨迹", zorder=3)
        ax.scatter(route[0, 0], route[0, 1], s=70, marker="o", facecolor="white",
                   edgecolor="#111827", linewidth=1.5, zorder=4, label="起点")
        marker = "s" if complete else "X"
        ax.scatter(xs[-1], ys[-1], s=75, marker=marker, color=COLORS[name],
                   edgecolor="white", linewidth=1.0, zorder=4,
                   label="完成" if complete else "中断位置")
        ax.set_title(name, fontsize=13, fontweight="bold")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.92)
    fig.suptitle("F1TENTH 路径跟踪实车轨迹对比", fontsize=17, fontweight="bold", y=0.98)
    fig.text(0.5, 0.012, "相同参考路线与地图；Linear MPC v3 在索引 114/149 中断，其余均完成。",
             ha="center", fontsize=10, color="#475569")
    fig.tight_layout(rect=[0, 0.035, 1, 0.96])
    fig.savefig(OUTPUT / "path_tracking_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 8))
    setup_map(ax, map_image, extent)
    ax.plot(route[:, 0], route[:, 1], color="#111827", linewidth=2.4,
            linestyle=(0, (4, 3)), label="参考路线", zorder=2)
    for name, (poses, complete) in traces.items():
        ax.plot([pose["x"] for pose in poses], [pose["y"] for pose in poses],
                color=COLORS[name], linestyle=LINESTYLES[name], linewidth=2.2,
                label=name, zorder=3)
    ax.scatter(route[0, 0], route[0, 1], s=80, marker="o", facecolor="white",
               edgecolor="#111827", linewidth=1.5, zorder=4)
    ax.set_title("四种控制器实车轨迹叠加", fontsize=16, fontweight="bold", pad=12)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.94)
    fig.text(0.5, 0.025, "轨迹为闭环中使用的粒子滤波位姿，并非独立真值。",
             ha="center", fontsize=10, color="#475569")
    fig.tight_layout(rect=[0, 0.045, 1, 1])
    fig.savefig(OUTPUT / "path_tracking_overlay.png", dpi=200, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for name, (poses, _) in traces.items():
        errors = route_errors(poses, route)
        axes[0].plot(errors[:, 0], errors[:, 1], color=COLORS[name],
                     linestyle=LINESTYLES[name], linewidth=2.0, label=name)
        axes[1].plot(errors[:, 0], errors[:, 2], color=COLORS[name],
                     linestyle=LINESTYLES[name], linewidth=2.0, label=name)
    for ax in axes:
        ax.axhline(0, color="#111827", linewidth=0.9)
        ax.grid(color="#cbd5e1", linewidth=0.7, alpha=0.7)
        ax.set_xlim(0, len(route) - 1)
        ax.legend(loc="upper right", ncol=2, fontsize=9, framealpha=0.94)
    axes[0].axhline(10, color="#64748b", linewidth=0.8, linestyle=":")
    axes[0].axhline(-10, color="#64748b", linewidth=0.8, linestyle=":")
    axes[0].set_ylabel("横向误差（cm）")
    axes[0].set_title("横向误差随路线进度", fontsize=14, fontweight="bold")
    axes[1].set_ylabel("航向误差（°）")
    axes[1].set_xlabel("参考路线索引（0–149）")
    axes[1].set_title("航向误差随路线进度", fontsize=14, fontweight="bold")
    fig.suptitle("PP、PID 与 MPC 跟踪误差对比", fontsize=17, fontweight="bold", y=0.985)
    fig.text(0.5, 0.012, "每个路线索引取该次实车 PF 样本中位数；Linear MPC v3 为未完成路线。",
             ha="center", fontsize=10, color="#475569")
    fig.tight_layout(rect=[0, 0.035, 1, 0.96])
    fig.savefig(OUTPUT / "tracking_error_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    print(json.dumps({"output": [str(path) for path in sorted(OUTPUT.glob("*.png"))]},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
