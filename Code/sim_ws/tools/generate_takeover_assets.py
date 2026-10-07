#!/usr/bin/env python3
"""Generate v8-compatible map/route assets for laptop takeover simulation."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / "src" / "f1tenth_gym_ros" / "maps"
WAYPOINTS = ROOT / "src" / "f1tenth_mpc-main" / "mpc" / "waypoints"
OUTPUT = ROOT / "takeover_assets"


def base_route_name(map_name: str) -> str:
    for suffix in (
        "_obstacle_center",
        "_obstacle_left",
        "_obstacle_right",
        "_visibility_left",
        "_visibility_right",
    ):
        if map_name.endswith(suffix):
            return map_name[: -len(suffix)]
    return map_name


def load_route(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 4:
                continue
            rows.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
    if len(rows) < 40:
        raise RuntimeError(f"Route too short: {path}")
    first = rows[0]
    if math.hypot(rows[-1][0] - first[0], rows[-1][1] - first[1]) > 1e-8:
        rows.append(first.copy())
    else:
        rows[-1][:3] = first[:3]
    speeds = [float(np.clip(row[3], 0.70, 2.0)) for row in rows]
    speeds[0] = speeds[-1] = 0.0
    return [[row[0], row[1], row[2]] for row in rows], speeds


def generate(map_name: str) -> Path:
    map_yaml_path = MAPS / f"{map_name}.yaml"
    if not map_yaml_path.exists():
        raise FileNotFoundError(map_yaml_path)
    metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    image_path = map_yaml_path.parent / metadata["image"]
    image = np.asarray(Image.open(image_path).convert("L"))
    # Generated benchmark maps use white for drivable floor, black for hard
    # boundaries/obstacles, and gray for unknown outside the course.
    free = image >= 250
    resolution = float(metadata["resolution"])
    clearance = distance_transform_edt(free) * resolution
    route_name = base_route_name(map_name)
    route, speed = load_route(WAYPOINTS / f"{route_name}_mpc.csv")
    output = {
        "route_mode": "closed_single_lap_stop",
        "route": route,
        "speed": speed,
        "resolution": resolution,
        "origin": [float(metadata["origin"][0]), float(metadata["origin"][1])],
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "clearance": clearance.reshape(-1).tolist(),
        "source": {
            "map_yaml": str(map_yaml_path.relative_to(ROOT)),
            "map_image": str(image_path.relative_to(ROOT)),
            "route": str((WAYPOINTS / f"{route_name}_mpc.csv").relative_to(ROOT)),
            "free_pixel_threshold": 250,
        },
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    destination = OUTPUT / f"{map_name}.json"
    destination.write_text(json.dumps(output, separators=(",", ":")) + "\n", encoding="utf-8")
    return destination


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_names", nargs="+", help="Gym map names")
    args = parser.parse_args()
    for map_name in args.map_names:
        print(generate(map_name))


if __name__ == "__main__":
    main()
