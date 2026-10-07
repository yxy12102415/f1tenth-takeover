#!/usr/bin/env python3
"""Generate deterministic, physically reproducible F1TENTH benchmark maps.

The generated maps are simulator environments only.  The ego online stack must
continue to plan from LiDAR rather than consuming the global occupancy map.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import yaml
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
MAPS = ROOT / "src" / "f1tenth_gym_ros" / "maps"
WAYPOINTS = ROOT / "src" / "f1tenth_mpc-main" / "mpc" / "waypoints"
MANIFEST = MAPS / "benchmark_manifest.json"

VERSION = "1.5"
RESOLUTION_M = 0.025
TRACK_WIDTH_M = 2.20
BOUNDARY_THICKNESS_M = 0.10
WAYPOINT_SPACING_M = 0.05
WAYPOINT_MAX_SPEED_MPS = 2.4
WAYPOINT_MIN_SPEED_MPS = 0.8
WAYPOINT_LATERAL_ACCEL_MPS2 = 0.65
BACKGROUND = 205
FREE = 255
OCCUPIED = 0

OBSTACLE_LENGTH_M = 0.32
OBSTACLE_WIDTH_M = 0.35
OBSTACLE_HEIGHT_M = 0.30
OBSTACLE_LATERAL_OFFSETS_M = {
    "center": 0.0,
    "left": 0.70,
    "right": -0.70,
}
LEAD_DETOUR_OFFSETS_M = {
    "center": 0.85,
    "left": 0.0,
    "right": 0.0,
}
LEAD_DETOUR_HALF_SPAN_M = 3.0
VISIBILITY_SCREEN_OFFSETS_M = (0.0,)
VISIBILITY_SCREEN_WIDTH_M = 0.60
VISIBILITY_LEAD_DETOUR_HALF_SPAN_M = 4.5
VISIBILITY_LEAD_OFFSETS_M = {
    "left": 0.70,
    "right": -0.70,
}


def line_points(start, end, spacing=0.02):
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(2, int(math.ceil(distance / spacing)))
    return [
        (
            start[0] + (end[0] - start[0]) * i / count,
            start[1] + (end[1] - start[1]) * i / count,
        )
        for i in range(count)
    ]


def arc_points(center, radius, start_angle, end_angle, spacing=0.02):
    arc_length = abs(end_angle - start_angle) * radius
    count = max(3, int(math.ceil(arc_length / spacing)))
    return [
        (
            center[0] + radius * math.cos(start_angle + (end_angle - start_angle) * i / count),
            center[1] + radius * math.sin(start_angle + (end_angle - start_angle) * i / count),
        )
        for i in range(count)
    ]


def rounded_rectangle_centerline(xmin, ymin, xmax, ymax, radius):
    return (
        line_points((xmin + radius, ymin), (xmax - radius, ymin))
        + arc_points((xmax - radius, ymin + radius), radius, -math.pi / 2.0, 0.0)
        + line_points((xmax, ymin + radius), (xmax, ymax - radius))
        + arc_points((xmax - radius, ymax - radius), radius, 0.0, math.pi / 2.0)
        + line_points((xmax - radius, ymax), (xmin + radius, ymax))
        + arc_points((xmin + radius, ymax - radius), radius, math.pi / 2.0, math.pi)
        + line_points((xmin, ymax - radius), (xmin, ymin + radius))
        + arc_points((xmin + radius, ymin + radius), radius, math.pi, 3.0 * math.pi / 2.0)
    )


def stadium_centerline():
    """Return a long calibration stadium with gentle 4 m-radius turns."""
    left_x, right_x = 6.0, 18.0
    center_y, radius = 7.0, 4.0
    return (
        line_points((left_x, center_y - radius), (right_x, center_y - radius))
        + arc_points((right_x, center_y), radius, -math.pi / 2.0, math.pi / 2.0)
        + line_points((right_x, center_y + radius), (left_x, center_y + radius))
        + arc_points((left_x, center_y), radius, math.pi / 2.0, 3.0 * math.pi / 2.0)
    )


def takeover_stadium_centerline():
    """Long stadium whose 24 m straight contains a complete pass and rejoin."""
    left_x, right_x = 6.0, 30.0
    center_y, radius = 7.0, 4.0
    return (
        line_points((left_x, center_y - radius), (right_x, center_y - radius))
        + arc_points((right_x, center_y), radius, -math.pi / 2.0, math.pi / 2.0)
        + line_points((right_x, center_y + radius), (left_x, center_y + radius))
        + arc_points((left_x, center_y), radius, math.pi / 2.0, 3.0 * math.pi / 2.0)
    )


def asymmetric_centerline():
    points = []
    count = 1800
    for i in range(count):
        theta = -math.pi / 2.0 + 2.0 * math.pi * i / count
        x = 5.0 + 3.0 * math.cos(theta) + 0.25 * math.cos(2.0 * theta)
        y = 4.0 + 2.15 * math.sin(theta) + 0.35 * math.sin(2.0 * theta)
        points.append((x, y))
    return points


def resample_closed(points, spacing=WAYPOINT_SPACING_M):
    closed = list(points) + [points[0]]
    cumulative = [0.0]
    for p0, p1 in zip(closed, closed[1:]):
        cumulative.append(cumulative[-1] + math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    total = cumulative[-1]
    targets = [i * spacing for i in range(max(3, int(math.floor(total / spacing))))]

    sampled = []
    segment = 0
    for target in targets:
        while segment + 1 < len(cumulative) and cumulative[segment + 1] < target:
            segment += 1
        segment_length = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if segment_length <= 1e-12 else (target - cumulative[segment]) / segment_length
        p0, p1 = closed[segment], closed[segment + 1]
        sampled.append((p0[0] + ratio * (p1[0] - p0[0]), p0[1] + ratio * (p1[1] - p0[1])))

    rows = []
    count = len(sampled)
    for i, (x, y) in enumerate(sampled):
        previous = sampled[(i - 1) % count]
        following = sampled[(i + 1) % count]
        yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        previous_yaw = math.atan2(y - previous[1], x - previous[0])
        next_yaw = math.atan2(following[1] - y, following[0] - x)
        yaw_delta = (next_yaw - previous_yaw + math.pi) % (2.0 * math.pi) - math.pi
        local_ds = max(
            0.5
            * (
                math.hypot(x - previous[0], y - previous[1])
                + math.hypot(following[0] - x, following[1] - y)
            ),
            1e-6,
        )
        curvature = yaw_delta / local_ds
        # Curvature-aware lead profile: use a normal indoor F1TENTH speed on
        # straights and slow before bends according to v^2*kappa <= a_lat.
        # The lead controller looks ahead on these waypoints, so the speed
        # reduction is commanded before entering an arc rather than inside it.
        v_ref = min(
            WAYPOINT_MAX_SPEED_MPS,
            max(
                WAYPOINT_MIN_SPEED_MPS,
                math.sqrt(
                    WAYPOINT_LATERAL_ACCEL_MPS2
                    / max(abs(curvature), 1e-3)
                ),
            ),
        )
        rows.append(
            {
                "s_m": i * total / count,
                "x_m": x,
                "y_m": y,
                "yaw_rad": yaw,
                "curvature_radpm": curvature,
                "v_ref_mps": v_ref,
            }
        )
    return rows, total


def pose_at_s(rows, station):
    lap_length = rows[-1]["s_m"] + (rows[1]["s_m"] - rows[0]["s_m"])
    station %= lap_length
    row = min(rows, key=lambda item: abs(item["s_m"] - station))
    return [row["x_m"], row["y_m"], row["yaw_rad"]]


def recompute_waypoint_geometry(rows):
    """Recompute yaw, curvature and v_ref after a deterministic detour."""
    updated = []
    count = len(rows)
    for index, row in enumerate(rows):
        previous = rows[(index - 1) % count]
        following = rows[(index + 1) % count]
        x, y = row["x_m"], row["y_m"]
        yaw = math.atan2(
            following["y_m"] - previous["y_m"],
            following["x_m"] - previous["x_m"],
        )
        previous_yaw = math.atan2(y - previous["y_m"], x - previous["x_m"])
        next_yaw = math.atan2(following["y_m"] - y, following["x_m"] - x)
        yaw_delta = (next_yaw - previous_yaw + math.pi) % (2.0 * math.pi) - math.pi
        local_ds = max(
            0.5 * (
                math.hypot(x - previous["x_m"], y - previous["y_m"])
                + math.hypot(following["x_m"] - x, following["y_m"] - y)
            ),
            1e-6,
        )
        curvature = yaw_delta / local_ds
        v_ref = min(
            WAYPOINT_MAX_SPEED_MPS,
            max(
                WAYPOINT_MIN_SPEED_MPS,
                math.sqrt(WAYPOINT_LATERAL_ACCEL_MPS2 / max(abs(curvature), 1e-3)),
            ),
        )
        updated.append({
            "s_m": row["s_m"],
            "x_m": x,
            "y_m": y,
            "yaw_rad": yaw,
            "curvature_radpm": curvature,
            "v_ref_mps": v_ref,
        })
    return updated


def make_lead_detour(
    rows,
    lap_length,
    obstacle_station,
    peak_offset,
    half_span=LEAD_DETOUR_HALF_SPAN_M,
):
    """Apply a smooth cosine lateral detour around one obstacle station."""
    shifted = []
    for row in rows:
        delta = (
            (row["s_m"] - obstacle_station + 0.5 * lap_length) % lap_length
            - 0.5 * lap_length
        )
        lateral = 0.0
        if abs(delta) < half_span:
            lateral = peak_offset * 0.5 * (
                1.0 + math.cos(math.pi * delta / half_span)
            )
        shifted.append({
            **row,
            "x_m": row["x_m"] - lateral * math.sin(row["yaw_rad"]),
            "y_m": row["y_m"] + lateral * math.cos(row["yaw_rad"]),
        })
    return recompute_waypoint_geometry(shifted)


def world_to_pixel(point, height_px):
    return (
        int(round(point[0] / RESOLUTION_M)),
        height_px - 1 - int(round(point[1] / RESOLUTION_M)),
    )


def obstacle_polygon(
    pose,
    lateral_offset,
    length=OBSTACLE_LENGTH_M,
    width=OBSTACLE_WIDTH_M,
):
    x, y, yaw = pose
    tangent = (math.cos(yaw), math.sin(yaw))
    normal = (-math.sin(yaw), math.cos(yaw))
    center = (x + lateral_offset * normal[0], y + lateral_offset * normal[1])
    half_length = length * 0.5
    half_width = width * 0.5
    corners = []
    for along, lateral in (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ):
        corners.append(
            (
                center[0] + along * tangent[0] + lateral * normal[0],
                center[1] + along * tangent[1] + lateral * normal[1],
            )
        )
    return center, corners


def draw_map(course, rows, obstacle=None):
    width_px = int(round(course["size_m"][0] / RESOLUTION_M))
    height_px = int(round(course["size_m"][1] / RESOLUTION_M))
    image = Image.new("L", (width_px, height_px), color=BACKGROUND)
    draw = ImageDraw.Draw(image)
    pixels = [world_to_pixel((row["x_m"], row["y_m"]), height_px) for row in rows]
    pixels.append(pixels[0])
    outer_width = int(round((TRACK_WIDTH_M + 2.0 * BOUNDARY_THICKNESS_M) / RESOLUTION_M))
    inner_width = int(round(TRACK_WIDTH_M / RESOLUTION_M))
    draw.line(pixels, fill=OCCUPIED, width=outer_width, joint="curve")
    draw.line(pixels, fill=FREE, width=inner_width, joint="curve")
    obstacles = [] if obstacle is None else (
        obstacle if isinstance(obstacle, list) else [obstacle]
    )
    for item in obstacles:
        draw.polygon(
            [world_to_pixel(point, height_px) for point in item["corners_world_m"]],
            fill=OCCUPIED,
        )
    return image


def write_map_yaml(path, image_name):
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                f"resolution: {RESOLUTION_M:.6f}",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.45",
                "free_thresh: 0.196",
                "",
            ]
        )
    )


def write_centerline(path, rows):
    fields = ["s_m", "x_m", "y_m", "yaw_rad", "curvature_radpm", "v_ref_mps"]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fields, lineterminator="\n"
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({key: f"{row[key]:.6f}" for key in fields})


def write_waypoints(path, rows):
    with path.open("w", newline="") as handle:
        handle.write("# x_m,y_m,yaw_rad,v_ref_mps\n")
        writer = csv.writer(handle, lineterminator="\n")
        for row in rows:
            writer.writerow(
                [
                    f"{row['x_m']:.6f}",
                    f"{row['y_m']:.6f}",
                    f"{row['yaw_rad']:.6f}",
                    f"{row['v_ref_mps']:.6f}",
                ]
            )


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def build_courses():
    return [
        {
            "course_id": "benchmark_takeover",
            "purpose": "dynamic-overtaking benchmark with a 24 m passing straight",
            "size_m": [36.0, 14.0],
            "geometry": {
                "type": "stadium",
                "straight_length_m": 24.0,
                "turn_radius_m": 4.0,
            },
            "raw_points": takeover_stadium_centerline(),
            "obstacle_station_m": 15.0,
            "zones": {
                "warmup": [0.0, 5.0],
                "approach": [5.0, 9.0],
                "passing": [9.0, 18.0],
                "rejoin": [18.0, 22.0],
            },
        },
        {
            "course_id": "benchmark_straight",
            "purpose": "calibration and nominal following on two long straight sections",
            "size_m": [24.0, 14.0],
            "geometry": {
                "type": "stadium",
                "straight_length_m": 12.0,
                "turn_radius_m": 4.0,
            },
            "raw_points": stadium_centerline(),
            "obstacle_station_m": 9.0,
            "zones": {
                "warmup": [0.0, 6.5],
                "approach": [6.5, 8.5],
                "occlusion_obstacle": [8.5, 10.5],
                "rejoin": [10.5, 13.0],
            },
        },
        {
            "course_id": "benchmark_occlusion_a",
            "purpose": "primary large rounded-square turning benchmark",
            "size_m": [14.0, 14.0],
            "geometry": {
                "type": "rounded_square",
                "bounds_m": [2.0, 2.0, 12.0, 12.0],
                "side_length_m": 10.0,
                "turn_radius_m": 3.0,
            },
            "raw_points": rounded_rectangle_centerline(2.0, 2.0, 12.0, 12.0, 3.0),
            "obstacle_station_m": 6.0,
            "zones": {
                "warmup": [0.0, 4.0],
                "approach": [4.0, 5.5],
                "occlusion_obstacle": [5.5, 8.0],
                "rejoin": [8.0, 11.0],
            },
        },
        {
            "course_id": "benchmark_occlusion_b",
            "purpose": "held-out asymmetric-curvature generalization benchmark",
            "size_m": [10.0, 8.0],
            "geometry": {
                "type": "asymmetric_closed_curve",
                "equation": "x=5+3cos(t)+0.25cos(2t), y=4+2.15sin(t)+0.35sin(2t)",
            },
            "raw_points": asymmetric_centerline(),
            "obstacle_station_fraction": 0.24,
            "zones_fraction": {
                "warmup": [0.0, 0.14],
                "approach": [0.14, 0.20],
                "occlusion_obstacle": [0.20, 0.32],
                "rejoin": [0.32, 0.45],
            },
        },
    ]


def generate():
    MAPS.mkdir(parents=True, exist_ok=True)
    WAYPOINTS.mkdir(parents=True, exist_ok=True)
    manifest = {
        "benchmark_version": VERSION,
        "coordinate_convention": "x right, y up; obstacle lateral offset l>0 is left of travel",
        "resolution_m_per_pixel": RESOLUTION_M,
        "track_width_m": TRACK_WIDTH_M,
        "boundary_thickness_m": BOUNDARY_THICKNESS_M,
        "waypoint_speed_profile": {
            "maximum_speed_mps": WAYPOINT_MAX_SPEED_MPS,
            "minimum_speed_mps": WAYPOINT_MIN_SPEED_MPS,
            "lateral_acceleration_limit_mps2": WAYPOINT_LATERAL_ACCEL_MPS2,
            "formula": "v_ref=clip(sqrt(a_lat/max(abs(curvature),1e-3)),v_min,v_max)",
        },
        "online_map_policy": "evaluation/environment only; never an ego planning input",
        "maps": {},
        "files_sha256": {},
    }
    generated = []

    for course in build_courses():
        rows, lap_length = resample_closed(course.pop("raw_points"))
        if "obstacle_station_fraction" in course:
            obstacle_station = course.pop("obstacle_station_fraction") * lap_length
            course["obstacle_station_m"] = obstacle_station
        else:
            obstacle_station = course["obstacle_station_m"]
        if "zones_fraction" in course:
            course["zones"] = {
                name: [bounds[0] * lap_length, bounds[1] * lap_length]
                for name, bounds in course.pop("zones_fraction").items()
            }

        ego_station = 0.25
        opponent_station = ego_station + 1.5
        ego_spawn = pose_at_s(rows, ego_station)
        opponent_spawn = pose_at_s(rows, opponent_station)
        initial_center_distance = math.hypot(
            opponent_spawn[0] - ego_spawn[0], opponent_spawn[1] - ego_spawn[1]
        )
        course["lap_length_m"] = lap_length
        course["track_width_m"] = TRACK_WIDTH_M
        course["boundary_thickness_m"] = BOUNDARY_THICKNESS_M
        course["resolution_m_per_pixel"] = RESOLUTION_M
        course["spawn"] = {
            "ego": ego_spawn,
            "opponent": opponent_spawn,
            "ego_station_s_m": ego_station,
            "opponent_station_s_m": opponent_station,
            "arc_separation_m": 1.5,
            "center_distance_m": initial_center_distance,
        }

        course_id = course["course_id"]
        centerline_path = MAPS / f"{course_id}_centerline.csv"
        waypoints_name = f"{course_id}_mpc.csv"
        waypoints_path = WAYPOINTS / waypoints_name
        course_path = MAPS / f"{course_id}_course.yaml"
        obstacles_path = MAPS / f"{course_id}_obstacles.yaml"
        write_centerline(centerline_path, rows)
        write_waypoints(waypoints_path, rows)
        generated.extend([centerline_path, waypoints_path])

        base_name = course_id
        base_image_path = MAPS / f"{base_name}.png"
        base_yaml_path = MAPS / f"{base_name}.yaml"
        draw_map(course, rows).save(base_image_path, optimize=False)
        write_map_yaml(base_yaml_path, base_image_path.name)
        generated.extend([base_image_path, base_yaml_path])
        map_names = [base_name]
        map_variant_labels = {base_name: None}
        map_waypoints = {base_name: waypoints_name}

        obstacle_specs = {
            "dimensions_m": {
                "length": OBSTACLE_LENGTH_M,
                "width": OBSTACLE_WIDTH_M,
                "height": OBSTACLE_HEIGHT_M,
            },
            "station_s_m": obstacle_station,
            "lateral_offset_convention": "positive is left of direction of travel",
            "variants": {},
        }
        if obstacle_station is not None:
            obstacle_pose = pose_at_s(rows, obstacle_station)
            for label, lateral_offset in OBSTACLE_LATERAL_OFFSETS_M.items():
                center, corners = obstacle_polygon(obstacle_pose, lateral_offset)
                obstacle = {
                    "label": label,
                    "station_s_m": obstacle_station,
                    "lateral_offset_m": lateral_offset,
                    "center_world_m": list(center),
                    "yaw_rad": obstacle_pose[2],
                    "corners_world_m": [list(point) for point in corners],
                }
                name = f"{course_id}_obstacle_{label}"
                image_path = MAPS / f"{name}.png"
                yaml_path = MAPS / f"{name}.yaml"
                draw_map(course, rows, obstacle).save(image_path, optimize=False)
                write_map_yaml(yaml_path, image_path.name)
                generated.extend([image_path, yaml_path])
                detour_rows = make_lead_detour(
                    rows,
                    lap_length,
                    obstacle_station,
                    LEAD_DETOUR_OFFSETS_M[label],
                )
                detour_waypoints_name = f"{name}_mpc.csv"
                detour_waypoints_path = WAYPOINTS / detour_waypoints_name
                write_waypoints(detour_waypoints_path, detour_rows)
                generated.append(detour_waypoints_path)
                obstacle_specs["variants"][label] = {
                    "map_name": name,
                    "lead_waypoints_file": detour_waypoints_name,
                    "lead_detour_peak_offset_m": LEAD_DETOUR_OFFSETS_M[label],
                    "lead_detour_half_span_m": LEAD_DETOUR_HALF_SPAN_M,
                    **obstacle,
                }
                map_names.append(name)
                map_variant_labels[name] = label
                map_waypoints[name] = detour_waypoints_name

        # A mirrored pair used only to test the visibility hypothesis.  One
        # wider foam-board obstacle creates a measurable occlusion boundary
        # while keeping both +/- lateral corridors collision-free.  The
        # map geometry is identical in both trials; only the lead detour is
        # mirrored, so a controller that always prefers one side is exposed.
        if course_id == "benchmark_straight":
            screen_components = []
            for station_offset in VISIBILITY_SCREEN_OFFSETS_M:
                component_station = obstacle_station + station_offset
                component_pose = pose_at_s(rows, component_station)
                component_center, component_corners = obstacle_polygon(
                    component_pose, 0.0, width=VISIBILITY_SCREEN_WIDTH_M
                )
                screen_components.append({
                    "station_s_m": component_station,
                    "lateral_offset_m": 0.0,
                    "center_world_m": list(component_center),
                    "yaw_rad": component_pose[2],
                    "corners_world_m": [list(point) for point in component_corners],
                })
            for expected_side, lead_offset in VISIBILITY_LEAD_OFFSETS_M.items():
                label = f"visibility_{expected_side}"
                name = f"benchmark_straight_{label}"
                image_path = MAPS / f"{name}.png"
                yaml_path = MAPS / f"{name}.yaml"
                draw_map(course, rows, screen_components).save(
                    image_path, optimize=False
                )
                write_map_yaml(yaml_path, image_path.name)
                generated.extend([image_path, yaml_path])
                detour_rows = make_lead_detour(
                    rows,
                    lap_length,
                    obstacle_station,
                    lead_offset,
                    half_span=VISIBILITY_LEAD_DETOUR_HALF_SPAN_M,
                )
                detour_waypoints_name = f"{name}_mpc.csv"
                detour_waypoints_path = WAYPOINTS / detour_waypoints_name
                write_waypoints(detour_waypoints_path, detour_rows)
                generated.append(detour_waypoints_path)
                obstacle_specs["variants"][label] = {
                    "map_name": name,
                    "lead_waypoints_file": detour_waypoints_name,
                    "lead_detour_peak_offset_m": lead_offset,
                    "lead_detour_half_span_m": (
                        VISIBILITY_LEAD_DETOUR_HALF_SPAN_M
                    ),
                    "expected_visibility_side": expected_side,
                    "screen_component_count": len(screen_components),
                    "screen_component_width_m": VISIBILITY_SCREEN_WIDTH_M,
                    "screen_station_offsets_m": list(
                        VISIBILITY_SCREEN_OFFSETS_M
                    ),
                    "components": screen_components,
                }
                map_names.append(name)
                map_variant_labels[name] = label
                map_waypoints[name] = detour_waypoints_name

        course["map_names"] = map_names
        course["centerline_file"] = centerline_path.name
        course["waypoints_file"] = waypoints_name
        course_path.write_text(yaml.safe_dump(course, sort_keys=False))
        obstacles_path.write_text(yaml.safe_dump(obstacle_specs, sort_keys=False))
        generated.extend([course_path, obstacles_path])

        for map_name in map_names:
            obstacle_label = map_variant_labels[map_name]
            selected_waypoints = map_waypoints[map_name]
            manifest["maps"][map_name] = {
                "course_id": course_id,
                "purpose": course["purpose"],
                "waypoints_file": selected_waypoints,
                "spawn": course["spawn"],
                "obstacle_label": obstacle_label,
                "course_spec": course_path.name,
                "obstacle_spec": obstacles_path.name,
            }

    for path in sorted(generated):
        manifest["files_sha256"][str(path.relative_to(ROOT))] = sha256(path)
    MANIFEST.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"Generated {len(manifest['maps'])} standardized maps; manifest: {MANIFEST}")


def print_lookup(map_name):
    if not MANIFEST.exists():
        raise SystemExit(f"Missing {MANIFEST}; run this script without --lookup first")
    manifest = json.loads(MANIFEST.read_text())
    config = manifest["maps"].get(map_name)
    if config is None:
        raise SystemExit(f"Map '{map_name}' is not a standardized benchmark map")
    print(config["waypoints_file"])
    for value in config["spawn"]["ego"]:
        print(f"{value:.9f}")
    for value in config["spawn"]["opponent"]:
        print(f"{value:.9f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lookup", metavar="MAP_NAME", help="print launch values for a generated map")
    args = parser.parse_args()
    if args.lookup:
        print_lookup(args.lookup)
    else:
        generate()


if __name__ == "__main__":
    main()
