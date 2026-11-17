#!/usr/bin/env python3
"""ROS-independent Frenet lattice planner adapted to the 381 map.

The stable predecessor sampled only nine terminal lateral offsets with one
fixed two-metre transition.  This planner samples lateral terminal position,
outbound transition distance and return distance.  Every lateral profile is a
quintic polynomial in route arc length, so position, slope and curvature are
continuous at both ends.
"""

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


@dataclass
class Candidate:
    valid: bool
    offset_m: float
    path: np.ndarray
    score: float
    minimum_static_clearance_m: float
    minimum_obstacle_clearance_m: float
    maximum_curvature_1pm: float
    reason: str = "ok"
    transition_m: float = 0.0
    return_m: float = 0.0
    lateral_jerk_cost: float = 0.0
    footprint_collision: bool = False
    score_terms: dict = None


class StaticMap:
    def __init__(self, yaml_path):
        config = yaml.safe_load(Path(yaml_path).read_text())
        image_path = Path(config["image"])
        if not image_path.is_absolute():
            image_path = Path(yaml_path).parent / image_path
        image = np.asarray(Image.open(image_path))
        free = image > int(round(255.0 * (1.0 - float(config["free_thresh"]))))
        self.distance = distance_transform_edt(free) * float(config["resolution"])
        self.resolution = float(config["resolution"])
        self.origin = np.asarray(config["origin"][:2], dtype=float)
        self.height, self.width = image.shape

    def clearance_many(self, points):
        points = np.asarray(points, dtype=float)
        cols = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        rows_bottom = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        rows = self.height - 1 - rows_bottom
        valid = (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)
        result = np.zeros(len(points), dtype=float)
        result[valid] = self.distance[rows[valid], cols[valid]]
        return result


def load_route(path):
    rows = []
    with Path(path).open(newline="") as stream:
        for row in csv.reader(stream):
            try:
                rows.append([float(row[0]), float(row[1]), float(row[2])])
            except (ValueError, IndexError):
                continue
    route = np.asarray(rows, dtype=float)
    if len(route) < 40:
        raise ValueError("route has too few poses")
    seam = np.linalg.norm(route[-1, :2] - route[0, :2])
    return route, bool(seam <= 0.08)


def load_closed_route(path):
    """Compatibility loader retained for closed-route callers and tests."""
    route, closed_loop = load_route(path)
    if not closed_loop:
        raise ValueError("route is not closed at normal waypoint spacing")
    return route


def smoothstep5(value):
    value = np.clip(value, 0.0, 1.0)
    return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)


def inverse_smoothstep5(value):
    """Invert monotonic quintic smoothstep without adding a scipy dependency."""
    target = float(np.clip(value, 0.0, 1.0))
    low, high = 0.0, 1.0
    for _ in range(32):
        middle = 0.5 * (low + high)
        if float(smoothstep5(middle)) < target:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def quintic_coefficients(start_position, start_slope, start_curvature,
                         end_position, end_slope, end_curvature, length):
    """Return coefficients for a spatial quintic Frenet profile d(s)."""
    length = float(length)
    if not math.isfinite(length) or length <= 1e-6:
        raise ValueError("quintic length must be positive")
    a0 = float(start_position)
    a1 = float(start_slope)
    a2 = 0.5 * float(start_curvature)
    matrix = np.array([
        [length ** 3, length ** 4, length ** 5],
        [3.0 * length ** 2, 4.0 * length ** 3, 5.0 * length ** 4],
        [6.0 * length, 12.0 * length ** 2, 20.0 * length ** 3],
    ])
    right = np.array([
        float(end_position) - a0 - a1 * length - a2 * length ** 2,
        float(end_slope) - a1 - 2.0 * a2 * length,
        float(end_curvature) - 2.0 * a2,
    ])
    return np.r_[a0, a1, a2, np.linalg.solve(matrix, right)]


def quintic_coefficients_batch(start_position, start_slope, start_curvature,
                               end_position, end_slope, end_curvature, length):
    """Vectorized closed form of ``quintic_coefficients``."""
    p0, v0, c0, p1, v1, c1, length = np.broadcast_arrays(
        np.asarray(start_position, dtype=float),
        np.asarray(start_slope, dtype=float),
        np.asarray(start_curvature, dtype=float),
        np.asarray(end_position, dtype=float),
        np.asarray(end_slope, dtype=float),
        np.asarray(end_curvature, dtype=float),
        np.asarray(length, dtype=float),
    )
    if np.any(~np.isfinite(length)) or np.any(length <= 1e-6):
        raise ValueError("quintic length must be positive")
    a0 = p0
    a1 = v0
    a2 = 0.5 * c0
    a3 = (20.0*(p1-p0) - (8.0*v1+12.0*v0)*length
          - (3.0*c0-c1)*length**2) / (2.0*length**3)
    a4 = (30.0*(p0-p1) + (14.0*v1+16.0*v0)*length
          + (3.0*c0-2.0*c1)*length**2) / (2.0*length**4)
    a5 = (12.0*(p1-p0) - (6.0*v1+6.0*v0)*length
          - (c0-c1)*length**2) / (2.0*length**5)
    return np.stack((a0, a1, a2, a3, a4, a5), axis=-1)


def evaluate_quintic(coefficients, arc_length):
    """Evaluate position and first three spatial derivatives."""
    a0, a1, a2, a3, a4, a5 = np.asarray(coefficients, dtype=float)
    s = np.asarray(arc_length, dtype=float)
    position = a0 + a1*s + a2*s**2 + a3*s**3 + a4*s**4 + a5*s**5
    slope = a1 + 2*a2*s + 3*a3*s**2 + 4*a4*s**3 + 5*a5*s**4
    curvature = 2*a2 + 6*a3*s + 12*a4*s**2 + 20*a5*s**3
    jerk = 6*a3 + 24*a4*s + 60*a5*s**2
    return position, slope, curvature, jerk


def vehicle_curvature(path, half_window=5):
    points = np.asarray(path)[:, :2]
    if len(points) < 2 * half_window + 1:
        return 0.0
    first = points[:-2 * half_window]
    middle = points[half_window:-half_window]
    last = points[2 * half_window:]
    a_vec = middle - first
    c_vec = last - first
    denominator = (
        np.linalg.norm(a_vec, axis=1)
        * np.linalg.norm(last - middle, axis=1)
        * np.linalg.norm(c_vec, axis=1)
    )
    cross = np.abs(a_vec[:, 0] * c_vec[:, 1] - a_vec[:, 1] * c_vec[:, 0])
    valid = denominator > 1e-9
    if not np.any(valid):
        return 0.0
    return float(np.max(2.0 * cross[valid] / denominator[valid]))


class ClosedLoopLatticePlanner:
    """Multi-horizon Frenet lattice for closed loops and stop-at-end routes."""

    def __init__(
        self,
        route_path,
        map_path,
        offsets=None,
        transition_lengths_m=None,
        return_lengths_m=None,
        horizon_m=3.2,
        footprint_radius_m=0.25,
        vehicle_half_width_m=0.15,
        vehicle_half_length_m=0.23,
        obstacle_margin_m=0.0,
        obstacle_horizon_m=2.0,
        static_echo_tolerance_m=0.15,
        clear_hold_scans=5,
        candidate_switch_margin=0.12,
        minimum_commit_scans=3,
        minimum_initial_avoidance_offset_m=0.20,
        proactive_avoidance_clearance_m=0.22,
    ):
        self.route, self.closed_loop = load_route(route_path)
        self.route_mode = "closed_loop" if self.closed_loop else "open_stop_at_end"
        self.static_map = StaticMap(map_path)
        self.offsets = np.asarray(
            np.linspace(-0.40, 0.40, 9) if offsets is None else offsets,
            dtype=float,
        )
        self.transition_lengths_m = np.asarray(
            (1.20, 1.50, 1.80, 2.10, 2.40)
            if transition_lengths_m is None else transition_lengths_m,
            dtype=float,
        )
        self.return_lengths_m = np.asarray(
            (1.20, 1.60, 2.00)
            if return_lengths_m is None else return_lengths_m,
            dtype=float,
        )
        if not np.any(np.isclose(self.offsets, 0.0)):
            raise ValueError("lattice must include the center route")
        if (not len(self.transition_lengths_m) or not len(self.return_lengths_m)
                or np.any(self.transition_lengths_m <= 0.0)
                or np.any(self.return_lengths_m <= 0.0)):
            raise ValueError("Frenet transition lengths must be positive")
        self.horizon_m = float(horizon_m)
        self.footprint_radius_m = float(footprint_radius_m)
        self.vehicle_half_width_m = float(vehicle_half_width_m)
        self.vehicle_half_length_m = float(vehicle_half_length_m)
        self.obstacle_margin_m = float(obstacle_margin_m)
        self.dynamic_clearance_m = self.vehicle_half_width_m + self.obstacle_margin_m
        self.obstacle_horizon_m = float(obstacle_horizon_m)
        if (self.vehicle_half_width_m <= 0.0 or self.vehicle_half_length_m <= 0.0
                or self.obstacle_margin_m < 0.0):
            raise ValueError("vehicle dimensions must be positive and margin nonnegative")
        self.static_echo_tolerance_m = float(static_echo_tolerance_m)
        self.clear_hold_scans = max(1, int(clear_hold_scans))
        self.candidate_switch_margin = float(candidate_switch_margin)
        self.minimum_commit_scans = max(1, int(minimum_commit_scans))
        self.minimum_initial_avoidance_offset_m = float(
            minimum_initial_avoidance_offset_m
        )
        self.proactive_avoidance_clearance_m = float(
            proactive_avoidance_clearance_m
        )
        if (not math.isfinite(self.candidate_switch_margin)
                or self.candidate_switch_margin < 0.0):
            raise ValueError("candidate switch margin must be nonnegative")
        if (not math.isfinite(self.minimum_initial_avoidance_offset_m)
                or self.minimum_initial_avoidance_offset_m < 0.0):
            raise ValueError("minimum initial avoidance offset must be nonnegative")
        if (not math.isfinite(self.proactive_avoidance_clearance_m)
                or self.proactive_avoidance_clearance_m
                < self.dynamic_clearance_m):
            raise ValueError(
                "proactive avoidance clearance must be at least the dynamic clearance"
            )
        self.progress_index = 0
        self.committed_offset_m = 0.0
        self.committed_transition_m = 0.0
        self.committed_return_m = 0.0
        self._center_clear_scans = 0
        self.mode = "FOLLOWING"
        self.commit_age_scans = 0
        self.candidate_switch_count = 0
        self.last_switch_reason = "initialized"
        self._planning_start_slope = 0.0
        self._planning_start_curvature = 0.0
        self._last_frenet_state = None
        self._planning_context = None
        self._planning_obstacle_tree = None
        self._center_path = None
        self.center_path_reused = False
        segment = np.linalg.norm(np.diff(self.route[:, :2], axis=0), axis=1)
        if self.closed_loop:
            segment = np.r_[
                segment,
                np.linalg.norm(self.route[0, :2] - self.route[-1, :2]),
            ]
        else:
            # Keep one segment entry per waypoint so progress indices retain
            # the same shape.  The terminal waypoint contributes no distance.
            segment = np.r_[segment, 0.0]
        self.segment_m = segment

    def nearest_progress(self, x, y):
        if self.closed_loop:
            indices = (self.progress_index + np.arange(-8, 81)) % len(self.route)
        else:
            first = max(0, self.progress_index - 8)
            last = min(len(self.route), self.progress_index + 81)
            indices = np.arange(first, last, dtype=int)
        delta = self.route[indices, :2] - np.array([x, y])
        nearest = int(indices[np.argmin(np.sum(delta * delta, axis=1))])
        self.progress_index = nearest
        return nearest

    def window_indices(self, start=None, horizon_m=None):
        if start is None:
            index = self.progress_index
        elif self.closed_loop:
            index = int(start) % len(self.route)
        else:
            index = int(np.clip(int(start), 0, len(self.route) - 1))
        horizon = self.horizon_m if horizon_m is None else min(
            self.horizon_m, max(0.0, float(horizon_m))
        )
        indices = [index]
        distance = 0.0
        while distance < horizon and len(indices) <= len(self.route):
            if not self.closed_loop and index >= len(self.route) - 1:
                break
            distance += self.segment_m[index]
            index = ((index + 1) % len(self.route)
                     if self.closed_loop else index + 1)
            indices.append(index)
        return np.asarray(indices, dtype=int)

    def _local_obstacles(self, base, local_s, obstacle_points):
        obstacle_points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2)
        if not len(obstacle_points):
            return obstacle_points, None
        distance_to_base = np.linalg.norm(
            base[:, None, :2] - obstacle_points[None, :, :], axis=2
        )
        nearest_base = np.argmin(distance_to_base, axis=0)
        relevant = (
            (np.min(distance_to_base, axis=0) < 0.80)
            & (local_s[nearest_base] <= self.obstacle_horizon_m)
        )
        points = obstacle_points[relevant]
        if not len(points):
            return points, None
        return points, float(np.max(local_s[nearest_base[relevant]]))

    def _lateral_profile(self, local_s, offset, start_offset_m,
                         transition_m, return_m, last_obstacle_s):
        transition_m = float(transition_m)
        if abs(offset) < 1e-9 and self._terminal_window():
            transition_m = min(transition_m, max(.001, float(local_s[-1])-.25))
        return_m = float(return_m)
        outbound = quintic_coefficients(
            start_offset_m,
            self._planning_start_slope,
            self._planning_start_curvature,
            offset, 0.0, 0.0,
            transition_m,
        )
        outbound_s = np.minimum(local_s, transition_m)
        profile, slope, curvature, jerk = evaluate_quintic(outbound, outbound_s)
        after_outbound = local_s >= transition_m
        profile[after_outbound] = offset
        slope[after_outbound] = 0.0
        curvature[after_outbound] = 0.0
        jerk[after_outbound] = 0.0

        if last_obstacle_s is not None and abs(offset) > 1e-9:
            return_start_s = max(
                transition_m,
                float(last_obstacle_s) + self.dynamic_clearance_m + 0.10,
            )
            returning = local_s >= return_start_s
            if np.any(returning):
                inbound = quintic_coefficients(
                    offset, 0.0, 0.0, 0.0, 0.0, 0.0, return_m
                )
                inbound_s = np.minimum(local_s[returning] - return_start_s, return_m)
                values = evaluate_quintic(inbound, inbound_s)
                profile[returning], slope[returning], curvature[returning], jerk[returning] = values
        return profile, slope, curvature, jerk

    def _footprint_collision(self, path, obstacle_points):
        """Check a conservative three-disc swept vehicle envelope."""
        if not len(obstacle_points) or not len(path):
            return False
        tree = self._planning_obstacle_tree or cKDTree(obstacle_points)
        longitudinal_offsets = np.array([
            -max(0.0, self.vehicle_half_length_m - self.vehicle_half_width_m),
            0.0,
            max(0.0, self.vehicle_half_length_m - self.vehicle_half_width_m),
        ])
        tangent = np.c_[np.cos(path[:, 2]), np.sin(path[:, 2])]
        disc_centers = (
            path[:, None, :2]
            + tangent[:, None, :] * longitudinal_offsets[None, :, None]
        ).reshape(-1, 2)
        nearest, _ = tree.query(disc_centers, k=1)
        return bool(np.min(nearest) <= self.dynamic_clearance_m)

    def _candidate(self, offset, obstacle_points, horizon_m=None,
                   start_offset_m=0.0, transition_m=None, return_m=None):
        transition_m = float(
            self.transition_lengths_m[len(self.transition_lengths_m) // 2]
            if transition_m is None else transition_m
        )
        return_m = float(
            self.return_lengths_m[len(self.return_lengths_m) // 2]
            if return_m is None else return_m
        )
        if self._planning_context is None:
            indices = self.window_indices(horizon_m=horizon_m)
            base = self.route[indices]
            local_s = np.r_[
                0.0,
                np.cumsum(np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1)),
            ]
            local_obstacles, last_obstacle_s = self._local_obstacles(
                base, local_s, obstacle_points
            )
        else:
            base, local_s, local_obstacles, last_obstacle_s = self._planning_context
        offset = float(offset)
        start_offset_m = float(start_offset_m)
        profile, _, _, jerk = self._lateral_profile(
            local_s, offset, start_offset_m,
            transition_m, return_m, last_obstacle_s,
        )

        path = np.empty_like(base)
        path[:, 0] = base[:, 0] - np.sin(base[:, 2]) * profile
        path[:, 1] = base[:, 1] + np.cos(base[:, 2]) * profile
        if len(path) < 2:
            path[:, 2] = base[:, 2]
        else:
            delta = np.gradient(path[:, :2], axis=0)
            path[:, 2] = np.arctan2(delta[:, 1], delta[:, 0])

        static_clearance = float(np.min(self.static_map.clearance_many(path[:, :2])))
        if len(local_obstacles):
            tree = self._planning_obstacle_tree or cKDTree(local_obstacles)
            obstacle_clearance = float(np.min(tree.query(path[:, :2], k=1)[0]))
        else:
            obstacle_clearance = math.inf
        footprint_collision = self._footprint_collision(path, local_obstacles)
        if len(path) < 2:
            maximum_curvature = 0.0
        elif abs(offset) < 1e-9 and abs(start_offset_m) < 1e-9:
            path_ds = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
            yaw_delta = np.abs(np.arctan2(
                np.sin(np.diff(path[:, 2])), np.cos(np.diff(path[:, 2]))
            ))
            maximum_curvature = float(np.max(yaw_delta / np.maximum(path_ds, 1e-6)))
        else:
            maximum_curvature = vehicle_curvature(path)
        baseline_curvature = vehicle_curvature(base)
        curvature_limit = max(math.tan(0.27) / 0.25 + 0.03, baseline_curvature + 0.03)
        valid = (
            static_clearance >= self.footprint_radius_m
            and not footprint_collision
            and obstacle_clearance >= self.dynamic_clearance_m
            and maximum_curvature <= curvature_limit
        )
        terminal_heading_ok = (not (self._terminal_window() and abs(offset)<1e-9)
            or abs(math.atan2(math.sin(path[-1,2]-self.route[-1,2]),
                             math.cos(path[-1,2]-self.route[-1,2]))) <= .10)
        valid = valid and terminal_heading_ok and len(path)>=2
        lateral_jerk_cost = float(np.trapz(jerk * jerk, local_s)) if len(local_s) > 1 else 0.0
        obstacle_surplus = obstacle_clearance - self.dynamic_clearance_m
        obstacle_cost = (
            0.0 if not math.isfinite(obstacle_surplus)
            else 4.0 * max(0.0, 0.35 - obstacle_surplus) ** 2
        )
        wall_cost = 5.0 * max(
            0.0, self.footprint_radius_m + 0.10 - static_clearance
        ) ** 2
        transition_continuity = (
            0.0 if self.committed_transition_m <= 0.0
            else 0.20 * abs(transition_m - self.committed_transition_m)
        )
        return_continuity = (
            0.0 if self.committed_return_m <= 0.0
            else 0.10 * abs(return_m - self.committed_return_m)
        )
        reference_cost = 1.0 * abs(offset)
        curvature_cost = 0.35 * maximum_curvature + 0.003 * lateral_jerk_cost
        continuity_cost = (
            1.5 * abs(offset - self.committed_offset_m)
            + transition_continuity + return_continuity
            + 0.04 * abs(transition_m - 1.80)
            + 0.02 * abs(return_m - 1.60)
        )
        score = reference_cost + curvature_cost + obstacle_cost + wall_cost + continuity_cost
        reason = "ok" if valid else (
            "terminal_path_too_short" if len(path)<2 else
            "static_clearance" if static_clearance < self.footprint_radius_m else
            "obstacle_footprint" if footprint_collision else
            "obstacle_clearance" if obstacle_clearance < self.dynamic_clearance_m else
            "terminal_heading" if not terminal_heading_ok else "curvature"
        )
        return Candidate(
            valid, offset, path, float(score), static_clearance,
            obstacle_clearance, maximum_curvature, reason,
            transition_m, return_m, lateral_jerk_cost, footprint_collision,
            {
                "reference": float(reference_cost),
                "curvature_jerk": float(curvature_cost),
                "obstacle": float(obstacle_cost),
                "wall": float(wall_cost),
                "continuity": float(continuity_cost),
            },
        )

    @staticmethod
    def _evaluate_quintic_batch(coefficients, arc_length):
        coefficients = np.asarray(coefficients, dtype=float)
        s = np.asarray(arc_length, dtype=float)
        a0, a1, a2, a3, a4, a5 = (
            coefficients[:, index:index + 1] for index in range(6)
        )
        position = a0 + a1*s + a2*s**2 + a3*s**3 + a4*s**4 + a5*s**5
        slope = a1 + 2*a2*s + 3*a3*s**2 + 4*a4*s**3 + 5*a5*s**4
        curvature = 2*a2 + 6*a3*s + 12*a4*s**2 + 20*a5*s**3
        jerk = 6*a3 + 24*a4*s + 60*a5*s**2
        return position, slope, curvature, jerk

    def _batch_candidates(self, specifications, start_offset_m):
        """Evaluate a complete candidate set with shared vectorized geometry."""
        if not specifications:
            return []
        base, local_s, local_obstacles, last_obstacle_s = self._planning_context
        offsets = np.asarray([item[0] for item in specifications], dtype=float)
        transitions = np.asarray([item[1] for item in specifications], dtype=float)
        if self._terminal_window():
            transitions = np.where(np.abs(offsets) < 1e-9,
                np.minimum(transitions, max(.001, float(local_s[-1])-.25)), transitions)
        returns = np.asarray([item[2] for item in specifications], dtype=float)
        count = len(specifications)
        outbound_coefficients = quintic_coefficients_batch(
            np.full(count, start_offset_m),
            np.full(count, self._planning_start_slope),
            np.full(count, self._planning_start_curvature),
            offsets, np.zeros(count), np.zeros(count), transitions,
        )
        outbound_s = np.minimum(local_s[None, :], transitions[:, None])
        profile, _, _, jerk = self._evaluate_quintic_batch(
            outbound_coefficients, outbound_s
        )
        after_outbound = local_s[None, :] >= transitions[:, None]
        profile = np.where(after_outbound, offsets[:, None], profile)
        jerk = np.where(after_outbound, 0.0, jerk)

        if last_obstacle_s is not None:
            returning_candidates = np.abs(offsets) > 1e-9
            return_start = np.maximum(
                transitions,
                float(last_obstacle_s) + self.dynamic_clearance_m + 0.10,
            )
            returning = (
                returning_candidates[:, None]
                & (local_s[None, :] >= return_start[:, None])
            )
            inbound_coefficients = quintic_coefficients_batch(
                offsets, np.zeros(count), np.zeros(count),
                np.zeros(count), np.zeros(count), np.zeros(count), returns,
            )
            inbound_s = np.minimum(
                np.maximum(0.0, local_s[None, :] - return_start[:, None]),
                returns[:, None],
            )
            inbound_profile, _, _, inbound_jerk = self._evaluate_quintic_batch(
                inbound_coefficients, inbound_s
            )
            profile = np.where(returning, inbound_profile, profile)
            jerk = np.where(returning, inbound_jerk, jerk)

        x = base[None, :, 0] - np.sin(base[None, :, 2]) * profile
        y = base[None, :, 1] + np.cos(base[None, :, 2]) * profile
        if len(base) < 2:
            yaw = np.broadcast_to(base[None, :, 2], (count, len(base))).copy()
        else:
            yaw = np.arctan2(
                np.gradient(y, axis=1),
                np.gradient(x, axis=1),
            )
        paths = np.stack((x, y, yaw), axis=2)

        static_values = self.static_map.clearance_many(
            paths[:, :, :2].reshape(-1, 2)
        ).reshape(count, len(base))
        static_clearances = np.min(static_values, axis=1)

        if len(local_obstacles):
            tree = self._planning_obstacle_tree
            longitudinal_offsets = np.array([
                -max(0.0, self.vehicle_half_length_m - self.vehicle_half_width_m),
                0.0,
                max(0.0, self.vehicle_half_length_m - self.vehicle_half_width_m),
            ])
            tangent = np.stack((np.cos(yaw), np.sin(yaw)), axis=2)
            disc_centers = (
                paths[:, :, None, :2]
                + tangent[:, :, None, :] * longitudinal_offsets[None, None, :, None]
            )
            footprint_distance = tree.query(disc_centers.reshape(-1, 2), k=1)[0]
            disc_distance = footprint_distance.reshape(count, len(base), 3)
            # The middle footprint disc is exactly the path center.  Reuse its
            # nearest-neighbour results for center clearance instead of issuing
            # a second KD-tree query over every candidate waypoint.
            obstacle_clearances = disc_distance[:, :, 1].min(axis=1)
            footprint_collisions = (
                disc_distance.min(axis=(1, 2))
                <= self.dynamic_clearance_m
            )
        else:
            obstacle_clearances = np.full(count, math.inf)
            footprint_collisions = np.zeros(count, dtype=bool)

        if len(base) < 11:
            maximum_curvatures = np.zeros(count)
        else:
            first = paths[:, :-10, :2]
            middle = paths[:, 5:-5, :2]
            last = paths[:, 10:, :2]
            a_vec = middle - first
            c_vec = last - first
            denominator = (
                np.linalg.norm(a_vec, axis=2)
                * np.linalg.norm(last - middle, axis=2)
                * np.linalg.norm(c_vec, axis=2)
            )
            cross = np.abs(a_vec[:, :, 0] * c_vec[:, :, 1]
                           - a_vec[:, :, 1] * c_vec[:, :, 0])
            curvature = np.zeros_like(cross)
            valid_denominator = denominator > 1e-9
            curvature[valid_denominator] = (
                2.0 * cross[valid_denominator] / denominator[valid_denominator]
            )
            maximum_curvatures = np.max(curvature, axis=1)
        baseline_curvature = vehicle_curvature(base)
        curvature_limit = max(
            math.tan(0.27) / 0.25 + 0.03,
            baseline_curvature + 0.03,
        )
        valid = (
            (static_clearances >= self.footprint_radius_m)
            & ~footprint_collisions
            & (obstacle_clearances >= self.dynamic_clearance_m)
            & (maximum_curvatures <= curvature_limit)
        )
        terminal_heading_ok = np.ones(count,dtype=bool)
        if self._terminal_window():
            end_error = np.abs(np.arctan2(np.sin(yaw[:,-1]-self.route[-1,2]),
                                         np.cos(yaw[:,-1]-self.route[-1,2])))
            terminal_heading_ok = (np.abs(offsets)>1e-9) | (end_error<=.10)
            valid &= terminal_heading_ok
        valid &= len(base)>=2
        if len(local_s) > 1:
            lateral_jerk_costs = np.trapz(jerk * jerk, x=local_s, axis=1)
        else:
            lateral_jerk_costs = np.zeros(count)
        obstacle_surplus = obstacle_clearances - self.dynamic_clearance_m
        obstacle_cost = np.where(
            np.isfinite(obstacle_surplus),
            4.0 * np.maximum(0.0, 0.35 - obstacle_surplus) ** 2,
            0.0,
        )
        wall_cost = 5.0 * np.maximum(
            0.0, self.footprint_radius_m + 0.10 - static_clearances
        ) ** 2
        transition_continuity = (
            np.zeros(count) if self.committed_transition_m <= 0.0
            else 0.20 * np.abs(transitions - self.committed_transition_m)
        )
        return_continuity = (
            np.zeros(count) if self.committed_return_m <= 0.0
            else 0.10 * np.abs(returns - self.committed_return_m)
        )
        scores = (
            np.abs(offsets)
            + 0.35 * maximum_curvatures
            + 0.003 * lateral_jerk_costs
            + obstacle_cost
            + wall_cost
            + 1.5 * np.abs(offsets - self.committed_offset_m)
            + transition_continuity
            + return_continuity
            + 0.04 * np.abs(transitions - 1.80)
            + 0.02 * np.abs(returns - 1.60)
        )
        candidates = []
        for index in range(count):
            reason = "ok" if valid[index] else (
                "terminal_path_too_short" if len(base)<2 else
                "static_clearance"
                if static_clearances[index] < self.footprint_radius_m else
                "obstacle_footprint" if footprint_collisions[index] else
                "obstacle_clearance"
                if obstacle_clearances[index] < self.dynamic_clearance_m else
                "terminal_heading" if not terminal_heading_ok[index] else "curvature"
            )
            candidates.append(Candidate(
                bool(valid[index]), float(offsets[index]), paths[index].copy(),
                float(scores[index]), float(static_clearances[index]),
                float(obstacle_clearances[index]), float(maximum_curvatures[index]),
                reason, float(transitions[index]), float(returns[index]),
                float(lateral_jerk_costs[index]), bool(footprint_collisions[index]),
                {
                    "reference": float(abs(offsets[index])),
                    "curvature_jerk": float(
                        0.35 * maximum_curvatures[index]
                        + 0.003 * lateral_jerk_costs[index]),
                    "obstacle": float(obstacle_cost[index]),
                    "wall": float(wall_cost[index]),
                    "continuity": float(
                        1.5 * abs(offsets[index] - self.committed_offset_m)
                        + transition_continuity[index]
                        + return_continuity[index]
                        + 0.04 * abs(transitions[index] - 1.80)
                        + 0.02 * abs(returns[index] - 1.60)),
                },
            ))
        return candidates

    def _terminal_window(self):
        return (not self.closed_loop and self._planning_context is not None
                and np.linalg.norm(self._planning_context[0][-1, :2]-self.route[-1, :2]) < 1e-6)

    def _reuse_center(self, x, y, obstacles):
        """Keep a spatially fixed recovery; recheck its swept footprint each scan."""
        old = self._center_path
        if old is None or abs(self.committed_offset_m) > 1e-9:
            return None
        nearest = int(np.argmin(np.sum((old.path[:, :2]-[x,y])**2, axis=1)))
        path = old.path[max(0, nearest-1):]
        if len(path) < 2 or np.linalg.norm(old.path[nearest,:2]-[x,y]) > .15:
            return None
        length = float(np.sum(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)))
        terminal = np.linalg.norm(path[-1,:2]-self.route[-1,:2]) < 1e-6
        if length < .80 and not terminal:
            return None
        static = float(np.min(self.static_map.clearance_many(path[:,:2])))
        clearance = math.inf
        footprint = math.inf
        if len(obstacles):
            tree = cKDTree(obstacles)
            clearance = float(np.min(tree.query(path[:,:2])[0]))
            half = max(0.,self.vehicle_half_length_m-self.vehicle_half_width_m)
            tangent = np.c_[np.cos(path[:,2]),np.sin(path[:,2])]
            centers = path[:,None,:2]+tangent[:,None,:]*np.array([-half,0.,half])[None,:,None]
            footprint = float(np.min(tree.query(centers.reshape(-1,2))[0]))
        if (static < self.footprint_radius_m or clearance < self.dynamic_clearance_m
                or footprint < self.dynamic_clearance_m):
            return None
        return replace(old, path=path.copy(), minimum_static_clearance_m=static,
                       minimum_obstacle_clearance_m=clearance)

    def plan(self, x, y, obstacle_points=(), horizon_m=None, heading=None):
        self.center_path_reused = False
        result, candidates = self._plan(x,y,obstacle_points,horizon_m,heading)
        if result.valid and abs(result.offset_m) < 1e-9:
            self._center_path = replace(result, path=result.path.copy())
        else:
            self._center_path = None
        return result, candidates

    def _plan(self, x, y, obstacle_points=(), horizon_m=None, heading=None):
        progress = self.nearest_progress(x, y)
        base = self.route[progress]
        normal = np.array([-math.sin(base[2]), math.cos(base[2])])
        start_offset_m = float(np.dot(np.array([x, y]) - base[:2], normal))
        maximum_offset = max(0.60, float(np.max(np.abs(self.offsets))) + 0.10)
        start_offset_m = float(np.clip(start_offset_m, -maximum_offset, maximum_offset))
        start_slope = 0.0
        if heading is not None and math.isfinite(float(heading)):
            heading_error = math.atan2(
                math.sin(float(heading) - float(base[2])),
                math.cos(float(heading) - float(base[2])),
            )
            if self.closed_loop or progress < len(self.route) - 1:
                next_index = (progress + 1) % len(self.route)
                reference_delta = math.atan2(
                    math.sin(float(self.route[next_index, 2] - base[2])),
                    math.cos(float(self.route[next_index, 2] - base[2])),
                )
                reference_segment = self.segment_m[progress]
            else:
                previous = max(0, progress - 1)
                reference_delta = math.atan2(
                    math.sin(float(base[2] - self.route[previous, 2])),
                    math.cos(float(base[2] - self.route[previous, 2])),
                )
                reference_segment = self.segment_m[previous]
            reference_curvature = reference_delta / max(reference_segment, 1e-6)
            start_slope = math.tan(float(np.clip(heading_error, -0.60, 0.60)))
            start_slope *= 1.0 - reference_curvature * start_offset_m
            start_slope = float(np.clip(start_slope, -0.80, 0.80))
        start_curvature = 0.0
        if self._last_frenet_state is not None:
            previous_index, previous_slope, previous_curvature = self._last_frenet_state
            forward = ((progress - previous_index) % len(self.route)
                       if self.closed_loop else progress - previous_index)
            if forward == 0:
                start_curvature = previous_curvature
            elif 0 < forward <= 12:
                distance = float(sum(
                    self.segment_m[(previous_index + step) % len(self.route)]
                    for step in range(forward)
                ))
                measured = (start_slope - previous_slope) / max(distance, 0.05)
                start_curvature = 0.50 * previous_curvature + 0.50 * float(
                    np.clip(measured, -0.60, 0.60)
                )
        self._planning_start_slope = start_slope
        self._planning_start_curvature = start_curvature
        self._last_frenet_state = (progress, start_slope, start_curvature)
        obstacle_points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2)
        if len(obstacle_points):
            # Returns coincident with the frozen occupancy boundary are mapped
            # walls. Only points inside mapped free space are new obstacles.
            obstacle_points = obstacle_points[
                self.static_map.clearance_many(obstacle_points)
                > self.static_echo_tolerance_m
            ]
        indices = self.window_indices(horizon_m=horizon_m)
        plan_base = self.route[indices]
        plan_s = np.r_[
            0.0,
            np.cumsum(np.linalg.norm(np.diff(plan_base[:, :2], axis=0), axis=1)),
        ]
        local_obstacles, last_obstacle_s = self._local_obstacles(
            plan_base, plan_s, obstacle_points
        )
        self._planning_context = (
            plan_base, plan_s, local_obstacles, last_obstacle_s
        )
        self._planning_obstacle_tree = (
            cKDTree(local_obstacles) if len(local_obstacles) else None
        )
        center_candidates = self._batch_candidates([
            (0.0, transition_m, self.return_lengths_m[0])
            for transition_m in self.transition_lengths_m
        ], start_offset_m)
        candidates = list(center_candidates)
        valid_centers = [item for item in center_candidates if item.valid]
        center = min(valid_centers or center_candidates, key=lambda item: item.score)
        cached = self._reuse_center(x,y,obstacle_points)
        if cached is not None:
            center = cached
            self.center_path_reused = True
        proactive_avoidance = (
            center.valid
            and abs(self.committed_offset_m) < 1e-9
            and len(local_obstacles) > 0
            and center.minimum_obstacle_clearance_m
            <= self.proactive_avoidance_clearance_m
        )
        if (center.valid and abs(self.committed_offset_m) < 1e-9
                and not proactive_avoidance):
            self._center_clear_scans = 0
            self.mode = "FOLLOWING"
            self.commit_age_scans = 0
            self.last_switch_reason = "center_clear"
            return center, candidates

        avoidance_specifications = [
            (float(offset), float(transition_m), float(return_m))
            for offset in self.offsets if abs(float(offset)) > 1e-9
            for transition_m in self.transition_lengths_m
            for return_m in self.return_lengths_m
        ]
        candidates.extend(self._batch_candidates(
            avoidance_specifications, start_offset_m
        ))

        if abs(self.committed_offset_m) > 1e-9:
            sign = math.copysign(1.0, self.committed_offset_m)
            allowed = [item for item in candidates if item.offset_m * sign > 0.0 and item.valid]
            if center.valid:
                self._center_clear_scans += 1
                if self._center_clear_scans >= self.clear_hold_scans:
                    self.clear_commit(mode="RETURNING")
                    self.last_switch_reason = "center_clear_hold_satisfied"
                    return center, candidates
                allowed = [item for item in allowed if abs(item.offset_m) > 1e-9]
                if not allowed:
                    # The committed side is no longer traversable, but the
                    # reference centerline is safe. Rejoin center immediately
                    # instead of manufacturing an all-candidates-blocked stop.
                    self.clear_commit(mode="RETURNING")
                    self.last_switch_reason = "committed_side_invalid_center_safe"
                    return center, candidates
            else:
                self._center_clear_scans = 0
        else:
            allowed = [item for item in candidates if item.valid and abs(item.offset_m) > 1e-9]
            # A partially observed box can make a 0.10 m path look valid at
            # first sight, then invalidate every same-side continuation once
            # its full face becomes visible in a bend.  Commit to at least a
            # 0.20 m lateral maneuver when such a candidate exists.  If the
            # map only admits a smaller offset, retain the previous fallback
            # instead of converting a traversable corridor into a stop.
            decisive = [
                item for item in allowed
                if abs(item.offset_m) + 1e-9
                >= self.minimum_initial_avoidance_offset_m
            ]
            if decisive:
                allowed = decisive
        if not allowed:
            if center.valid:
                # The early trigger creates time to choose a useful lateral
                # path, but it is not itself a stop boundary.  Continue on the
                # still-safe centerline when the lattice has no valid bypass;
                # the unchanged dynamic clearance remains the hard boundary.
                self._center_clear_scans = 0
                self.mode = "FOLLOWING"
                self.commit_age_scans = 0
                self.last_switch_reason = (
                    "proactive_clearance_no_valid_avoidance"
                )
                return center, candidates
            self.mode = "BLOCKED"
            self.last_switch_reason = "all_candidates_blocked"
            return Candidate(False, self.committed_offset_m, np.empty((0, 3)), math.inf,
                             0.0, 0.0, 0.0, "all_candidates_blocked"), candidates
        best = min(allowed, key=lambda item: item.score)
        selected = best
        incumbent = None
        if abs(self.committed_offset_m) > 1e-9:
            matching = [
                item for item in allowed
                if abs(item.offset_m - self.committed_offset_m) < 1e-9
                and abs(item.transition_m - self.committed_transition_m) < 1e-9
                and abs(item.return_m - self.committed_return_m) < 1e-9
            ]
            if matching:
                incumbent = min(matching, key=lambda item: item.score)
        if (incumbent is not None
                and (self.commit_age_scans < self.minimum_commit_scans
                     or best.score + self.candidate_switch_margin
                     >= incumbent.score)):
            selected = incumbent
            self.last_switch_reason = "incumbent_hysteresis"
        elif incumbent is None and abs(self.committed_offset_m) > 1e-9:
            self.last_switch_reason = "incumbent_invalid"
        elif abs(self.committed_offset_m) > 1e-9:
            self.last_switch_reason = "score_improvement_exceeded_margin"
        else:
            self.last_switch_reason = (
                "proactive_clearance_select_avoidance"
                if proactive_avoidance
                else "center_blocked_select_avoidance"
            )
        changed = (
            abs(selected.offset_m - self.committed_offset_m) > 1e-9
            or abs(selected.transition_m - self.committed_transition_m) > 1e-9
            or abs(selected.return_m - self.committed_return_m) > 1e-9
        )
        if changed:
            self.candidate_switch_count += 1
            self.commit_age_scans = 0
        else:
            self.commit_age_scans += 1
        self.committed_offset_m = selected.offset_m
        self.committed_transition_m = selected.transition_m
        self.committed_return_m = selected.return_m
        self.mode = "AVOIDING"
        if not center.valid:
            self._center_clear_scans = 0
        return selected, candidates

    def clear_commit(self, mode="FOLLOWING"):
        self.committed_offset_m = 0.0
        self.committed_transition_m = 0.0
        self.committed_return_m = 0.0
        self._center_clear_scans = 0
        self.commit_age_scans = 0
        self.mode = str(mode)
