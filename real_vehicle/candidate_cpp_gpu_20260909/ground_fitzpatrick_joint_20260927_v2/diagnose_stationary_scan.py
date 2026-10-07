#!/usr/bin/env python3
"""Read-only stationary scan diagnosis; does not open or command the VESC."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
PLANNER = WORKSPACE / "candidate_cpp_gpu_20260909" / "open_route_frenet_v1_20260923"
sys.path.insert(0, str(PLANNER))
from return_lattice_planner import ClosedLoopLatticePlanner
from avoidance_controller import LASER_OFFSET_X, remove_map_wall_returns


def clusters(points, radius=0.10):
    unseen = set(range(len(points)))
    groups = []
    while unseen:
        seed = unseen.pop()
        group = {seed}
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            if not unseen:
                break
            candidate = np.fromiter(unseen, dtype=int)
            near = candidate[
                np.linalg.norm(points[candidate] - points[current], axis=1) <= radius
            ]
            for index in near.tolist():
                unseen.remove(index)
                group.add(index)
                frontier.append(index)
        groups.append(np.asarray(sorted(group), dtype=int))
    return groups


class Capture(Node):
    def __init__(self):
        super().__init__("stationary_scan_diagnostic", start_parameter_services=False)
        self.scans = []
        self.create_subscription(
            LaserScan, "/baseline/raw_scan", self.scans.append, qos_profile_sensor_data
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    parser.add_argument("--progress", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    route = WORKSPACE / "routes/manual_route_20260923_172359_controller_ready.csv"
    map_file = WORKSPACE / "maps/381_2_0.yaml"
    planner = ClosedLoopLatticePlanner(route, map_file, static_echo_tolerance_m=0.25)
    planner.progress_index = args.progress

    rclpy.init()
    node = Capture()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and len(node.scans) < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.scans:
        raise SystemExit("no scan received")

    output = []
    all_dynamic = []
    for scan in node.scans:
        origin = np.array([
            args.x + LASER_OFFSET_X * math.cos(args.yaw),
            args.y + LASER_OFFSET_X * math.sin(args.yaw),
        ])
        points = []
        directions = []
        for index, raw in enumerate(scan.ranges):
            distance = float(raw)
            if (not math.isfinite(distance) or distance < scan.range_min
                    or distance > min(scan.range_max, 7.0)):
                continue
            angle = args.yaw + scan.angle_min + index * scan.angle_increment
            direction = np.array([math.cos(angle), math.sin(angle)])
            points.append(origin + distance * direction)
            directions.append(direction)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        directions = np.asarray(directions, dtype=float).reshape(-1, 2)
        dynamic = remove_map_wall_returns(planner.static_map, points, directions)
        all_dynamic.append(dynamic)
        output.append({"raw_count": len(points), "dynamic_count": len(dynamic)})

    dynamic = all_dynamic[-1]
    remaining = float(np.sum(planner.segment_m[args.progress:]))
    window = planner.route[planner.window_indices(horizon_m=remaining), :2]
    details = []
    if len(dynamic):
        route_distance = np.min(
            np.linalg.norm(dynamic[:, None, :] - window[None, :, :], axis=2), axis=1
        )
        vehicle_distance = np.linalg.norm(dynamic - np.array([args.x, args.y]), axis=1)
        relevant = dynamic[route_distance < 0.80]
        relevant_route_distance = route_distance[route_distance < 0.80]
        relevant_vehicle_distance = vehicle_distance[route_distance < 0.80]
        for group in sorted(clusters(relevant), key=len, reverse=True):
            cloud = relevant[group]
            details.append({
                "points": len(group),
                "center": np.mean(cloud, axis=0).tolist(),
                "bounds": [float(cloud[:, 0].min()), float(cloud[:, 0].max()),
                           float(cloud[:, 1].min()), float(cloud[:, 1].max())],
                "minimum_route_distance_m": float(np.min(relevant_route_distance[group])),
                "minimum_vehicle_distance_m": float(np.min(relevant_vehicle_distance[group])),
                "minimum_map_clearance_m": float(np.min(planner.static_map.clearance_many(cloud))),
                "maximum_map_clearance_m": float(np.max(planner.static_map.clearance_many(cloud))),
            })
    report = {
        "vesc_started": False,
        "pose": [args.x, args.y, args.yaw],
        "progress_index": args.progress,
        "frames": output,
        "dynamic_points_map_xy_m": dynamic.tolist(),
        "clusters_near_current_lap": details,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
