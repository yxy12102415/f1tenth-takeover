#!/usr/bin/env python3
"""Evaluation-only logger for dynamic overtaking trials.

Opponent ground truth is subscribed here for scoring. It is never republished
and is not an input to the tracker, behavior planner, Frenet planner, or MPC.
"""

import csv
import json
import math
import os
from collections import Counter
from datetime import datetime, timezone

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rectangle(x, y, yaw, length=0.58, width=0.31):
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-math.sin(yaw), math.cos(yaw)])
    center = np.array([x, y])
    a, b = 0.5 * length * forward, 0.5 * width * lateral
    return np.array([center + a + b, center + a - b, center - a - b, center - a + b])


def overlap_axis(first, second, axis):
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return True
    axis = axis / norm
    a, b = first @ axis, second @ axis
    return a.max() >= b.min() and b.max() >= a.min()


def overlap(first, second):
    for polygon in (first, second):
        for index in range(4):
            edge = polygon[(index + 1) % 4] - polygon[index]
            if not overlap_axis(first, second, np.array([-edge[1], edge[0]])):
                return False
    return True


def point_segment_distance(point, start, end):
    edge = end - start
    fraction = 0.0 if edge @ edge < 1e-12 else np.clip((point - start) @ edge / (edge @ edge), 0.0, 1.0)
    return float(np.linalg.norm(point - (start + fraction * edge)))


def polygon_distance(first, second):
    if overlap(first, second):
        return 0.0
    value = float("inf")
    for a, b in ((first, second), (second, first)):
        for point in a:
            for index in range(4):
                value = min(value, point_segment_distance(
                    point, b[index], b[(index + 1) % 4]
                ))
    return value


class TruthPassCounter:
    """Evaluation-only encounter counter using continuous unwrapped progress.

    Each lap-relative encounter must start behind, contain OVERTAKE, and end
    fully ahead/aligned for 0.5 s. Repeated status messages cannot add passes.
    """
    def __init__(self, route_length):
        self.length = route_length
        self.armed = None
        self.started = None
        self.saw_overtake = False
        self.aligned_since = None
        self.counted = set()
        self.events = []

    def update(self, time, delta, route_error, heading_error, behavior, safe=True):
        encounter = int(round(delta / self.length))
        gap = encounter*self.length-delta
        if self.armed is None and encounter not in self.counted and 1.35 < gap < 7.:
            self.armed, self.started = encounter, time
            self.saw_overtake = False
            self.aligned_since = None
        if self.armed is None:
            return
        if behavior == "OVERTAKE":
            self.saw_overtake = True
        ahead = delta-self.armed*self.length
        aligned = (safe and self.saw_overtake and ahead > 1.20 and
                   route_error < .10 and abs(heading_error) < .15 and behavior == "CRUISE")
        if not aligned:
            self.aligned_since = None
        elif self.aligned_since is None:
            self.aligned_since = time
        elif time-self.aligned_since >= .5:
            self.events.append(dict(encounter_lap=self.armed, start_time_s=self.started,
                                    completion_time_s=time, lead_distance_m=ahead))
            self.counted.add(self.armed)
            self.armed = None


class TakeoverEvaluator(Node):
    def __init__(self):
        super().__init__("takeover_evaluator")
        for name, default in (
            ("scenario", "constant_speed"),
            ("expected_outcome", "safe"),
            ("run_id", "trial"),
            ("results_dir", "results/takeover/trial"),
            ("map_name", "benchmark_straight"),
            ("waypoints_path", ""),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("seed", 0)
        self.declare_parameter("require_lead", True)
        self.require_lead = bool(self.get_parameter("require_lead").value)
        self.declare_parameter("sample_rate", 20.0)

        self.scenario = str(self.get_parameter("scenario").value)
        self.expected_outcome = str(self.get_parameter("expected_outcome").value)
        self.run_id = str(self.get_parameter("run_id").value)
        self.output_dir = os.path.abspath(str(self.get_parameter("results_dir").value))
        os.makedirs(self.output_dir, exist_ok=True)
        self.map_name = str(self.get_parameter("map_name").value)
        self.seed = int(self.get_parameter("seed").value)
        self.start = self.get_clock().now()
        self.finalized = False

        waypoint_path = str(self.get_parameter("waypoints_path").value)
        self.route = self.load_route(waypoint_path)
        self.route_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.route, axis=0), axis=1))]
        self.route_length = float(self.route_arc[-1] + np.linalg.norm(self.route[-1]-self.route[0]))
        self.truth_passes = TruthPassCounter(self.route_length)
        maps = os.path.join(get_package_share_directory("f1tenth_gym_ros"), "maps")
        metadata = yaml.safe_load(open(os.path.join(maps, self.map_name + ".yaml"), encoding="utf-8"))
        self.map_resolution = float(metadata["resolution"])
        self.map_origin = np.array(metadata["origin"][:2], dtype=float)
        self.map_image = np.asarray(Image.open(os.path.join(maps, metadata["image"])).convert("L"))

        self.ego = self.opp = self.estimate = self.control = None
        self.tracker = {}
        self.status = {}
        self.last_status_receive = None
        self.rows = []
        self.prediction_rows = []
        self.behaviors = Counter()
        self.tracker_modes = Counter()
        self.loss_count = 0
        self.loss_start = None
        self.reacquisition_times = []
        self.in_collision = False
        self.collision_events = 0
        self.in_boundary_violation = False
        self.boundary_events = 0
        self.lead_boundary_events = 0
        self.in_lead_boundary_violation = False
        self.overtake_time = None
        self.truth_overtake_confirmed = False
        self.last_steering = None
        self.last_speed_command = None
        self.last_sample_time = None
        self.last_ego_index = None
        self.ego_unwrapped_s = 0.0
        self.last_opp_index = None
        self.opp_unwrapped_s = 0.0

        self.create_subscription(Odometry, "/ego_racecar/odom", lambda msg: setattr(self, "ego", msg), 20)
        # Evaluation-only truth. No publication from this node.
        self.create_subscription(Odometry, "/opp_racecar/odom", lambda msg: setattr(self, "opp", msg), 20)
        self.create_subscription(Odometry, "/ego_racecar/opp_odom_ekf", lambda msg: setattr(self, "estimate", msg), 20)
        self.create_subscription(AckermannDriveStamped, "/drive", lambda msg: setattr(self, "control", msg), 20)
        self.create_subscription(String, "/ego_tracker/status", self.tracker_callback, 20)
        self.create_subscription(String, "/takeover/status", self.status_callback, 20)
        rate = max(1.0, float(self.get_parameter("sample_rate").value))
        self.create_timer(1.0 / rate, self.sample)
        self.create_timer(1.0, self.write_summary)

        self.write_metadata()
        self.get_logger().info(f"Takeover evaluation results: {self.output_dir}")

    @staticmethod
    def load_route(path):
        values = []
        with open(path, newline="", encoding="utf-8") as handle:
            for row in csv.reader(handle):
                if not row or row[0].lstrip().startswith("#") or len(row) < 2:
                    continue
                values.append([float(row[0]), float(row[1])])
        return np.asarray(values)

    def tracker_callback(self, msg):
        try:
            current = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        mode = current.get("mode", "INVALID")
        previous = self.tracker.get("mode")
        now = self.elapsed()
        if mode == "LOST" and previous != "LOST":
            self.loss_count += 1
            self.loss_start = now
        if mode == "TRACKED" and self.loss_start is not None:
            self.reacquisition_times.append(now - self.loss_start)
            self.loss_start = None
        self.tracker = current

    def status_callback(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_status_receive = self.get_clock().now()
        if int(self.status.get("overtakes_completed", 0)) > 0 and self.overtake_time is None:
            self.overtake_time = self.elapsed()
        stamp = float(self.status.get("stamp", self.elapsed()))
        reference = self.status.get("reference_path", [])
        prediction = self.status.get("target_prediction", [])
        timed = self.status.get("timed_reference", [])
        mpc = self.status.get("mpc_prediction", [])
        mpc_speed = (self.status.get("controller") or {}).get("predicted_speed_mps", [])
        for index in range(max(len(reference), len(prediction), len(mpc))):
            ref = reference[index] if index < len(reference) else [None, None, None]
            pred = prediction[index] if index < len(prediction) else [None] * 5
            schedule = timed[index] if index < len(timed) else [None] * 4
            actual = mpc[index] if index < len(mpc) else [None] * 3
            self.prediction_rows.append({
                "status_stamp_s": stamp,
                "plan_stamp_s": self.status.get("plan_stamp"),
                "prediction_model": self.status.get("prediction_model"),
                "behavior": self.status.get("behavior", "UNAVAILABLE"),
                "index": index,
                "reference_x": ref[0], "reference_y": ref[1], "reference_yaw": ref[2],
                "prediction_t_s": pred[0], "lead_prediction_x": pred[1],
                "lead_prediction_y": pred[2], "lead_prediction_yaw": pred[3],
                "lead_prediction_sigma_m": pred[4],
                "reference_t_s": schedule[0], "reference_s_m": schedule[1],
                "reference_d_m": schedule[2], "reference_speed_mps": schedule[3],
                "mpc_t_s": index * 0.05 if index < len(mpc) else None,
                "mpc_x": actual[0], "mpc_y": actual[1], "mpc_yaw": actual[2],
                "mpc_speed_mps": mpc_speed[index] if index < len(mpc_speed) and index < len(mpc) else None,
            })

    def elapsed(self):
        return (self.get_clock().now() - self.start).nanoseconds * 1e-9

    def route_state(self, x, y, ego=True):
        index = int(np.argmin(np.sum((self.route - np.array([x, y])) ** 2, axis=1)))
        previous = self.last_ego_index if ego else self.last_opp_index
        if previous is None:
            # Preserve the vehicles' initial separation.  Starting both
            # unwrapped progress counters at zero made a lead vehicle that
            # spawned several metres ahead appear level with the ego car in
            # the evaluator (the controller itself was unaffected).
            delta = index
        else:
            delta = index - previous
            half = len(self.route) // 2
            if delta > half:
                delta -= len(self.route)
            elif delta < -half:
                delta += len(self.route)
        if ego:
            self.last_ego_index = index
            self.ego_unwrapped_s += delta
            progress = self.ego_unwrapped_s
        else:
            self.last_opp_index = index
            self.opp_unwrapped_s += delta
            progress = self.opp_unwrapped_s
        return index, progress, float(np.linalg.norm(self.route[index] - np.array([x, y])))

    def boundary_violation(self, body):
        height, width = self.map_image.shape
        for x, y in body:
            col = int(math.floor((x - self.map_origin[0]) / self.map_resolution))
            row = height - 1 - int(math.floor((y - self.map_origin[1]) / self.map_resolution))
            if row < 0 or row >= height or col < 0 or col >= width or self.map_image[row, col] < 250:
                return True
        # Inspect occupied cells under the entire oriented body. Corner-only
        # checking misses a small obstacle between corners or along an edge.
        lo = np.floor((body.min(axis=0) - self.map_origin) / self.map_resolution).astype(int)
        hi = np.floor((body.max(axis=0) - self.map_origin) / self.map_resolution).astype(int)
        roi = self.map_image[height - 1 - hi[1]:height - lo[1], lo[0]:hi[0] + 1]
        for rr, cc in np.argwhere(roi < 250):
            x = self.map_origin[0] + (lo[0] + cc + 0.5) * self.map_resolution
            y = self.map_origin[1] + (hi[1] - rr + 0.5) * self.map_resolution
            if overlap(body, rectangle(x, y, 0.0, self.map_resolution, self.map_resolution)):
                return True
        return False

    def sample(self):
        if self.ego is None or (self.require_lead and self.opp is None):
            return
        time = self.elapsed()
        ego_x, ego_y, ego_yaw = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y, yaw_from_odom(self.ego)
        opp_x, opp_y, opp_yaw = (self.opp.pose.pose.position.x, self.opp.pose.pose.position.y, yaw_from_odom(self.opp)) if self.opp else (float('nan'), float('nan'), float('nan'))
        ego_speed = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)
        opp_speed = math.hypot(self.opp.twist.twist.linear.x, self.opp.twist.twist.linear.y) if self.opp else float('nan')
        # nav_msgs/Odometry defines twist in child_frame_id.  Gym therefore
        # reports longitudinal/lateral velocity in each vehicle body frame,
        # while the EKF estimate is expressed in the map frame.  Rotate truth
        # before comparing it with the estimate (the magnitude above is frame
        # invariant and remains useful for following metrics).
        opp_body_vx = self.opp.twist.twist.linear.x if self.opp else float('nan')
        opp_body_vy = self.opp.twist.twist.linear.y if self.opp else float('nan')
        opp_world_vx = math.cos(opp_yaw) * opp_body_vx - math.sin(opp_yaw) * opp_body_vy
        opp_world_vy = math.sin(opp_yaw) * opp_body_vx + math.cos(opp_yaw) * opp_body_vy
        ego_body, opp_body = rectangle(ego_x, ego_y, ego_yaw), rectangle(opp_x, opp_y, opp_yaw)
        body_clearance = polygon_distance(ego_body, opp_body) if self.opp else float('nan')
        collision = overlap(ego_body, opp_body) if self.opp else False
        if collision and not self.in_collision:
            self.collision_events += 1
        self.in_collision = collision
        boundary = self.boundary_violation(ego_body)
        if boundary and not self.in_boundary_violation:
            self.boundary_events += 1
        self.in_boundary_violation = boundary
        lead_boundary = self.boundary_violation(opp_body) if self.opp else False
        if lead_boundary and not self.in_lead_boundary_violation:
            self.lead_boundary_events += 1
        self.in_lead_boundary_violation = lead_boundary

        ego_index, ego_progress, route_error = self.route_state(ego_x, ego_y, True)
        opp_progress = self.route_state(opp_x, opp_y, False)[1] if self.opp else None
        truth_gap = float('nan')
        if self.opp is not None:
            def progress_m(index_progress):
                lap, index = divmod(int(index_progress), len(self.route))
                return lap*self.route_length + self.route_arc[index]
            delta = progress_m(ego_progress)-progress_m(opp_progress)
            tangent = self.route[(ego_index+1) % len(self.route)]-self.route[(ego_index-1) % len(self.route)]
            heading_error = math.atan2(math.sin(ego_yaw-math.atan2(tangent[1], tangent[0])),
                                       math.cos(ego_yaw-math.atan2(tangent[1], tangent[0])))
            self.truth_passes.update(time, delta, route_error, heading_error,
                                    self.status.get('behavior', ''),
                                    self.collision_events == 0 and self.boundary_events == 0
                                    and self.lead_boundary_events == 0)
            truth_gap = round(delta/self.route_length)*self.route_length-delta
        if self.opp is not None and int(self.status.get('overtakes_completed', 0)) > 0:
            forward_clearance = (ego_x - opp_x) * math.cos(ego_yaw) + (ego_y - opp_y) * math.sin(ego_yaw)
            # Independent evaluation of complete passage and route rejoin.
            # 0.70 m exceeds the longitudinal extent of two 0.58 x 0.31 m
            # rectangles even with relative heading; truth never leaves this node.
            if forward_clearance > 0.70 and route_error < 0.15:
                self.truth_overtake_confirmed = True
        estimate_error = velocity_error = float("nan")
        if self.estimate is not None and self.opp is not None:
            estimate_error = math.hypot(self.estimate.pose.pose.position.x - opp_x,
                                        self.estimate.pose.pose.position.y - opp_y)
            velocity_error = math.hypot(self.estimate.twist.twist.linear.x - opp_world_vx,
                                        self.estimate.twist.twist.linear.y - opp_world_vy)
        speed_command = float(self.control.drive.speed) if self.control else 0.0
        steering = float(self.control.drive.steering_angle) if self.control else 0.0
        dt = time - self.last_sample_time if self.last_sample_time is not None else float("nan")
        steering_rate = (steering - self.last_steering) / dt if self.last_steering is not None and dt > 1e-6 else float("nan")
        acceleration = (speed_command - self.last_speed_command) / dt if self.last_speed_command is not None and dt > 1e-6 else float("nan")
        self.last_sample_time, self.last_steering, self.last_speed_command = time, steering, speed_command
        behavior = self.status.get("behavior", "UNAVAILABLE")
        tracker_mode = self.tracker.get("mode", "UNAVAILABLE")
        self.behaviors[behavior] += 1
        self.tracker_modes[tracker_mode] += 1
        controller = self.status.get("controller") or {}
        self.rows.append({
            "time_s": time,
            "ego_x": ego_x, "ego_y": ego_y, "ego_yaw": ego_yaw, "ego_speed_mps": ego_speed,
            "lead_x": opp_x, "lead_y": opp_y, "lead_yaw": opp_yaw, "lead_speed_mps": opp_speed,
            "estimated_lead_x": self.estimate.pose.pose.position.x if self.estimate else None,
            "estimated_lead_y": self.estimate.pose.pose.position.y if self.estimate else None,
            "tracking_position_error_m": estimate_error,
            "tracking_velocity_error_mps": velocity_error,
            "tracker_mode": tracker_mode,
            "behavior": behavior,
            "behavior_transition": self.status.get("transition"),
            "stop": self.status.get("stop"),
            "stop_reason": self.status.get("stop_reason"),
            "planner_reason": self.status.get("planner_reason"),
            "speed_cap_mps": self.status.get("speed_cap_mps"),
            "controller_error": controller.get("controller_error"),
            "timed_position_error_m": controller.get("timed_position_error_m"),
            "timed_speed_error_mps": controller.get("timed_speed_error_mps"),
            "timed_reference_age_s": controller.get("timed_reference_age_s"),
            "timed_horizon_max_position_error_m": controller.get("timed_horizon_max_position_error_m"),
            "timed_horizon_speed_rmse_mps": controller.get("timed_horizon_speed_rmse_mps"),
            "mpc_dynamic_check_passed": controller.get("mpc_dynamic_check_passed"),
            "mpc_predicted_body_clearance_m": controller.get("mpc_minimum_predicted_body_clearance_m"),
            "transition_length_m": self.status.get("transition_length_m"),
            "target_offset_m": self.status.get("target_offset_m"),
            "evaluated_candidates": self.status.get("evaluated_candidates"),
            "planning_backend": self.status.get("planning_backend", "frenet_candidates"),
            "sqp_calls": self.status.get("sqp_calls", 0),
            "sqp_failures": self.status.get("sqp_failures", 0),
            "sqp_timeouts": self.status.get("sqp_timeouts", 0),
            "sqp_solve_ms": (self.status.get("sqp_last") or {}).get("solve_ms"),
            "sqp_message": (self.status.get("sqp_last") or {}).get("message"),
            "sqp_geometry": json.dumps(self.status.get("sqp_geometry", {}), sort_keys=True),
            "return_preview_checks": self.status.get("return_preview_checks", 0),
            "sqp_preview_checks": self.status.get("sqp_preview_checks", 0),
            "overtakes_completed": self.status.get("overtakes_completed", 0),
            "truth_overtakes_completed": len(self.truth_passes.events),
            "truth_route_gap_m": truth_gap,
            "committed_side": self.status.get("committed_side"),
            "ego_route_index_unwrapped": ego_progress,
            "lead_route_index_unwrapped": opp_progress,
            "route_lateral_error_m": route_error,
            "body_clearance_m": body_clearance,
            "collision": int(collision),
            "boundary_violation": int(boundary),
            "lead_boundary_violation": int(lead_boundary),
            "speed_command_mps": speed_command,
            "acceleration_command_est_mps2": acceleration,
            "mpc_acceleration_command_mps2": controller.get("acceleration_command_mps2"),
            "mpc_acceleration_limit_mps2": controller.get("configured_acceleration_limit_mps2"),
            "steering_angle_rad": steering,
            "steering_rate_radps": steering_rate,
            "planner_valid": self.status.get("planner_valid"),
            "planning_ms": self.status.get("planning_ms"),
            "mpc_solve_ms": controller.get("mpc_solve_ms"),
            "planner_failures": self.status.get("planner_failures"),
            "controller_failures": self.status.get("controller_failures"),
            "solver_timeouts": self.status.get("solver_timeouts"),
            "infeasible_count": self.status.get("infeasible_count"),
            "solver_recoveries": self.status.get("solver_recoveries"),
            "rejected_track_jumps": self.status.get("rejected_track_jumps"),
            "predicted_body_clearance_m": self.status.get("minimum_predicted_body_clearance_m"),
            "target_age_s": self.status.get("target_age_s"),
        })

    @staticmethod
    def finite_values(rows, key):
        return np.asarray([row[key] for row in rows if row.get(key) is not None and np.isfinite(row[key])], dtype=float)

    def summary(self):
        duration = self.elapsed()
        position = self.finite_values(self.rows, "tracking_position_error_m")
        velocity = self.finite_values(self.rows, "tracking_velocity_error_mps")
        measured = [row for row in self.rows if row.get('tracker_mode') == 'TRACKED']
        tracked_position = self.finite_values(measured, 'tracking_position_error_m')
        body = self.finite_values(self.rows, "body_clearance_m")
        route_error = self.finite_values(self.rows, "route_lateral_error_m")
        steering_rate = self.finite_values(self.rows, "steering_rate_radps")
        acceleration = self.finite_values(self.rows, "acceleration_command_est_mps2")
        planning = self.finite_values(self.rows, "planning_ms")
        solve = self.finite_values(self.rows, "mpc_solve_ms")
        timed_position = self.finite_values(self.rows, "timed_position_error_m")
        timed_speed = self.finite_values(self.rows, "timed_speed_error_mps")
        horizon_position = self.finite_values(self.rows, "timed_horizon_max_position_error_m")
        horizon_speed = self.finite_values(self.rows, "timed_horizon_speed_rmse_mps")
        rejoined = [row for row in self.rows if self.overtake_time is not None
                    and row['time_s'] >= self.overtake_time and row['behavior'] == 'CRUISE']
        rejoin_error = self.finite_values(rejoined, 'route_lateral_error_m')
        rejection_reasons = Counter(
            row.get("controller_error") for row in self.rows
            if row.get("controller_error")
        )
        overtakes = int(self.status.get("overtakes_completed", 0))
        safe = bool(self.rows) and self.collision_events == 0 and self.boundary_events == 0
        behavior_samples = dict(self.behaviors)
        if self.expected_outcome == "single_pass_and_rejoin":
            expected_behavior_pass = (overtakes >= 1 and len(self.truth_passes.events) >= 1
                                      and self.lead_boundary_events == 0)
        elif self.expected_outcome == "two_overtakes":
            expected_behavior_pass = (overtakes >= 2 and len(self.truth_passes.events) >= 2
                                      and self.lead_boundary_events == 0)
        elif self.expected_outcome in ("overtake", "fitzpatrick_long_straight_overtake"):
            expected_behavior_pass = overtakes > 0 and self.truth_overtake_confirmed
        elif self.expected_outcome == "follow_wait_or_cancel":
            expected_behavior_pass = (
                overtakes == 0
                and sum(behavior_samples.get(state, 0) for state in ("FOLLOW", "WAIT", "CANCEL")) > 0
            )
        elif self.expected_outcome == "route_cruise":
            progress = [row['ego_route_index_unwrapped'] for row in self.rows]
            expected_behavior_pass = (len(progress) > 1 and progress[-1] - progress[0] > 0.9 * len(self.route)
                                      and overtakes == 0)
        else:
            # Perturbation and v8 regression cases have safety/recovery
            # expectations rather than a mandatory behavior outcome.
            expected_behavior_pass = True
        passed = safe and expected_behavior_pass
        return {
            "scenario": self.scenario,
            "run_id": self.run_id,
            "seed": self.seed,
            "duration_s": duration,
            "samples": len(self.rows),
            "passed": passed,
            "acceptance": {
                "expected_outcome": self.expected_outcome,
                "collision_free": self.collision_events == 0,
                "boundary_safe": self.boundary_events == 0,
                "expected_behavior_satisfied": expected_behavior_pass,
            },
            "safety": {
                "collision_events": self.collision_events,
                "boundary_violation_events": self.boundary_events,
                "lead_boundary_violation_events": self.lead_boundary_events,
                "minimum_body_clearance_m": float(body.min()) if body.size else None,
            },
            "overtaking": {
                "truth_verified_pass_and_rejoin": self.truth_overtake_confirmed,
                "completed": overtakes,
                "truth_completed": len(self.truth_passes.events),
                "truth_events": self.truth_passes.events,
                "first_completion_time_s": self.overtake_time,
                "behavior_sample_counts": behavior_samples,
            },
            "tracking": {
                "tracked_only_position_rmse_m": float(np.sqrt(np.mean(tracked_position ** 2))) if tracked_position.size else None,
                "position_rmse_m": float(np.sqrt(np.mean(position ** 2))) if position.size else None,
                "velocity_rmse_mps": float(np.sqrt(np.mean(velocity ** 2))) if velocity.size else None,
                "loss_count": self.loss_count,
                "reacquisition_times_s": self.reacquisition_times,
                "tracker_mode_sample_counts": dict(self.tracker_modes),
            },
            "control": {
                "timed_horizon_position_error_p95_m": float(np.percentile(horizon_position,95)) if horizon_position.size else None,
                "timed_horizon_speed_rmse_p95_mps": float(np.percentile(horizon_speed,95)) if horizon_speed.size else None,
                "timed_position_rmse_m": float(np.sqrt(np.mean(timed_position ** 2))) if timed_position.size else None,
                "timed_speed_rmse_mps": float(np.sqrt(np.mean(timed_speed ** 2))) if timed_speed.size else None,
                "mpc_dynamic_check_pass_samples": sum(row.get("mpc_dynamic_check_passed") is True for row in self.rows),
                "post_rejoin_route_rmse_m": float(np.sqrt(np.mean(rejoin_error ** 2))) if rejoin_error.size else None,
                "post_rejoin_route_max_error_m": float(rejoin_error.max()) if rejoin_error.size else None,
                "route_lateral_rmse_m": float(np.sqrt(np.mean(route_error ** 2))) if route_error.size else None,
                "maximum_abs_steering_rate_radps": float(np.max(np.abs(steering_rate))) if steering_rate.size else None,
                "maximum_abs_acceleration_command_mps2": float(np.max(np.abs(acceleration))) if acceleration.size else None,
            },
            "runtime": {
                "planning_mean_ms": float(planning.mean()) if planning.size else None,
                "planning_p95_ms": float(np.percentile(planning, 95)) if planning.size else None,
                "mpc_mean_ms": float(solve.mean()) if solve.size else None,
                "mpc_p95_ms": float(np.percentile(solve, 95)) if solve.size else None,
                "planner_failures": int(self.status.get("planner_failures", 0)),
                "planning_backend": self.status.get("planning_backend", "frenet_candidates"),
                "sqp_calls": self.status.get("sqp_calls", 0),
                "sqp_failures": self.status.get("sqp_failures", 0),
                "sqp_timeouts": self.status.get("sqp_timeouts", 0),
                "controller_failures": int(self.status.get("controller_failures", 0)),
                "solver_timeouts": int(self.status.get("solver_timeouts", 0)),
                "infeasible_count": int(self.status.get("infeasible_count", 0)),
                "solver_recoveries": int(self.status.get("solver_recoveries", 0)),
                "rejected_track_jumps": int(self.status.get("rejected_track_jumps", 0)),
                "freshness_triggered_replans": int(self.status.get("freshness_triggered_replans", 0)),
                "controller_rejection_reasons": dict(rejection_reasons),
                "return_preview_checks": self.status.get("return_preview_checks", 0),
                "return_preview_rejections": self.status.get("return_preview_rejections", {}),
                "sqp_preview_checks": self.status.get("sqp_preview_checks", 0),
                "sqp_preview_rejections": self.status.get("sqp_preview_rejections", {}),
            },
            "ground_truth_policy": "Opponent ground truth is used only by its own controller and the independent evaluator, never by ego tracking/planning/MPC.",
        }

    def write_metadata(self):
        payload = {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "scenario": self.scenario,
            "expected_outcome": self.expected_outcome,
            "run_id": self.run_id,
            "seed": self.seed,
            "map_name": self.map_name,
            "require_lead": self.require_lead,
            "ground_truth_consumer": "takeover_evaluator only",
        }
        with open(os.path.join(self.output_dir, "metadata.json"), "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)

    def write_summary(self):
        if not self.rows:
            return
        destination=os.path.join(self.output_dir,"summary.json")
        temporary=destination+".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(self.summary(), handle, indent=2, sort_keys=True)
        os.replace(temporary,destination)

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        if self.rows:
            with open(os.path.join(self.output_dir, "trajectory.csv"), "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0]), lineterminator="\n")
                writer.writeheader()
                writer.writerows(self.rows)
        if self.prediction_rows:
            with open(os.path.join(self.output_dir, "predictions.csv"), "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.prediction_rows[0]), lineterminator="\n")
                writer.writeheader()
                writer.writerows(self.prediction_rows)
        self.write_summary()

    def destroy_node(self):
        self.finalize()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoverEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
