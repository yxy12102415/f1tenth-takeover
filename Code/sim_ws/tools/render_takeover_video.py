#!/usr/bin/env python3
"""Low-load offline video of recorded simulation poses, not a fresh trial."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=int, default=12)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("Refusing to overwrite video")
    if not 1 <= args.fps <= 30:
        raise SystemExit("fps must be 1..30")
    workspace = Path(__file__).resolve().parents[1]
    config = json.loads((args.trial / "config.json").read_text())
    summary = json.loads((args.trial / "summary.json").read_text())
    with (args.trial / "trajectory.csv").open() as f:
        rows = list(csv.DictReader(f))
    times = np.array([float(r["time_s"]) for r in rows])
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise SystemExit("Need strictly increasing recorded timestamps")
    poses = {who: np.array([[float(r[f"{who}_{k}"]) for k in ("x", "y", "yaw")]
                            for r in rows]) for who in ("ego", "lead")}
    map_path = workspace / "src/f1tenth_gym_ros/maps" / (config["map"] + ".yaml")
    info = yaml.safe_load(map_path.read_text())
    if abs(info["origin"][2]) > 1e-8:
        raise SystemExit("Rotated map origins are not supported")
    map_image = np.asarray(Image.open(map_path.parent / info["image"]).convert("L"))
    route_path = workspace / "src/f1tenth_mpc-main/mpc/waypoints" / (config["map"] + "_mpc.csv")
    route = np.loadtxt(route_path, delimiter=",", comments="#")
    h, w = map_image.shape
    ox, oy, _ = info["origin"]
    res = info["resolution"]
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    fig.subplots_adjust(left=.07, right=.98, bottom=.12, top=.81)
    ax.imshow(map_image, cmap="gray", vmin=0, vmax=255, origin="upper",
              extent=(ox, ox+w*res, oy, oy+h*res), interpolation="nearest")
    ax.plot(route[:, 0], route[:, 1], "--", color="0.45", lw=1, label="Reference route")
    trails, cars, labels = {}, {}, {}
    for who, color, style in (("ego", "#0072B2", "-"), ("lead", "#D55E00", ":")):
        trails[who], = ax.plot([], [], style, color=color, lw=2,
                              label=f"{who.title()} recorded trajectory")
        cars[who] = Polygon(np.zeros((4, 2)), closed=True, facecolor=color,
                            edgecolor="black", linewidth=1, zorder=5)
        ax.add_patch(cars[who])
        labels[who] = ax.text(0, 0, who.upper(), fontsize=9, weight="bold",
                              bbox=dict(facecolor="white", alpha=.85, edgecolor="none"), zorder=6)
    ax.set_aspect("equal")
    ax.set_xlabel("Map x [m]")
    ax.set_ylabel("Map y [m]")
    ax.legend(loc="upper left", fontsize=8)
    fig.text(.07, .96, "Fitzpatrick | LiDAR-based Dynamic Overtaking", fontsize=18, weight="bold")
    fig.text(.07, .915, f"RECORDED SIMULATION REPLAY | {config['run_id']} | 1x speed", fontsize=10)
    status = fig.text(.07, .855, "", fontsize=11, family="monospace")
    fig.text(.07, .035, "Recorded ground-truth poses for visualization only. No new simulation or real-vehicle run.",
             fontsize=9)
    corners = np.array([[.29, .155], [.29, -.155], [-.29, -.155], [-.29, .155]])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=args.fps, codec="libx264",
                         extra_args=["-threads", "1", "-preset", "veryfast",
                                     "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    playback = np.arange(times[0], times[-1] + 1 / args.fps, 1 / args.fps)
    with writer.saving(fig, str(args.output), 100):
        for frame, t in enumerate(playback):
            i = min(len(rows)-1, max(0, np.searchsorted(times, t, side="right")-1))
            row = rows[i]
            for who in poses:
                x, y, yaw = poses[who][i]
                rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
                valid = np.isfinite([x, y, yaw]).all() and t-times[i] < .25
                cars[who].set_visible(valid)
                labels[who].set_visible(valid)
                if valid:
                    cars[who].set_xy(corners @ rot.T + [x, y])
                    labels[who].set_position((x+.25, y+.25))
                past = poses[who][:i+1, :2].copy()
                past[np.r_[False, np.diff(times[:i+1]) > .25]] = np.nan
                trails[who].set_data(past[:, 0], past[:, 1])
            x, y = poses["ego"][i, :2]
            ax.set_xlim(x-7, x+7)
            ax.set_ylim(y-3.8, y+3.8)
            status.set_text(
                f"t={times[i]:5.1f}s   {row['behavior']:18s}   Tracker: {row['tracker_mode']}\n"
                f"Ego {float(row['ego_speed_mps']):.2f} m/s   Lead {float(row['lead_speed_mps']):.2f} m/s"
                f"   Body clearance {float(row['body_clearance_m']):.2f} m")
            if frame == min(len(playback)-1, 8*args.fps):
                fig.savefig(args.output.with_suffix(".preview.png"))
            writer.grab_frame()
            if frame % (args.fps*10) == 0:
                print(f"Rendered {frame}/{len(playback)} frames", flush=True)
    plt.close(fig)
    manifest = {
        "kind": "offline_recorded_simulation_replay", "source_trial": str(args.trial.resolve()),
        "source_sha256": hashlib.sha256((args.trial / "trajectory.csv").read_bytes()).hexdigest(),
        "fps": args.fps, "frames": len(playback), "resolution": [1280, 720],
        "sampling": "Previous recorded sample held at 1x time; no interpolation. Gaps >0.25s hidden.",
        "camera": "Ego-centered moving map window; equal metric aspect",
        "vehicle_dimensions_m": [0.58, 0.31], "historical_summary": summary,
        "description": "Blue ego passes orange lead on the mapped reference route. Labels distinguish vehicles.",
    }
    args.output.with_suffix(".json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(args.output)


if __name__ == "__main__":
    main()
