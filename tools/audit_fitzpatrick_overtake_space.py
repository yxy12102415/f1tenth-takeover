#!/usr/bin/env python3
"""Audit footprint and curvature feasibility along the v8 Fitzpatrick route."""

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/runtime_assets.json"


def runs(mask, arc, values):
    found, start = [], None
    for index, valid in enumerate(mask + [False]):
        if valid and start is None:
            start = index
        elif not valid and start is not None:
            if index - start > 1:
                found.append({
                    "start_s_m": arc[start],
                    "end_s_m": arc[index - 1],
                    "length_m": arc[index - 1] - arc[start],
                    "minimum_margin_m": min(values[start:index]),
                })
            start = None
    return sorted(found, key=lambda item: item["length_m"], reverse=True)


def main():
    data = json.loads(ASSETS.read_text())
    route = data["route"]
    resolution = data["resolution"]
    origin_x, origin_y = data["origin"]
    width, height, grid = data["width"], data["height"], data["clearance"]

    def clearance(x, y):
        column = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        if not (0 <= column < width and 0 <= row < height):
            return 0.0
        return grid[(height - 1 - row) * width + column]

    arc = [0.0]
    for before, after in zip(route, route[1:]):
        arc.append(arc[-1] + math.hypot(after[0] - before[0], after[1] - before[1]))
    curvature = []
    for index in range(len(route)):
        lo, hi = max(0, index - 1), min(len(route) - 1, index + 1)
        delta_yaw = (route[hi][2] - route[lo][2] + math.pi) % (2 * math.pi) - math.pi
        curvature.append(abs(delta_yaw / max(1e-6, arc[hi] - arc[lo])))

    report = {
        "route_length_m": arc[-1],
        "pass_offset_m": 0.82,
        "vehicle_length_m": 0.58,
        "vehicle_width_m": 0.31,
        "required_center_clearance_m": 0.25,
        "required_corner_clearance_m": 0.05,
        "straight_curvature_threshold_1pm": 0.15,
        "sides": {},
    }
    for side, label in ((1, "left"), (-1, "right")):
        margins = []
        for x, y, yaw in route:
            x -= math.sin(yaw) * side * 0.82
            y += math.cos(yaw) * side * 0.82
            corners = []
            for longitudinal in (-0.29, 0.29):
                for lateral in (-0.155, 0.155):
                    corners.append(clearance(
                        x + longitudinal * math.cos(yaw) - lateral * math.sin(yaw),
                        y + longitudinal * math.sin(yaw) + lateral * math.cos(yaw),
                    ) - 0.05)
            margins.append(min(corners + [clearance(x, y) - 0.25]))
        footprint_valid = [value >= 0.0 for value in margins]
        straight_valid = [
            valid and k <= 0.15 for valid, k in zip(footprint_valid, curvature)
        ]
        report["sides"][label] = {
            "footprint_valid_fraction": sum(footprint_valid) / len(footprint_valid),
            "longest_footprint_intervals": runs(footprint_valid, arc, margins)[:5],
            "longest_straight_intervals": runs(straight_valid, arc, margins)[:5],
        }
    output = ROOT / "results" / "takeover_sim_20261007" / "fitzpatrick_space_audit.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
