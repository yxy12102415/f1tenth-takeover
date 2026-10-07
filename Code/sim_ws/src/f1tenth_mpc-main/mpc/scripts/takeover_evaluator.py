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
        self.overtake_time = None
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
        if mode == "TRACKED" and previous == "LOST" and self.loss_start is not None:
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
        for index in range(max(len(reference), len(prediction))):
            ref = reference[index] if index < len(reference) else [None, None, None]
            pred = prediction[index] if index < len(prediction) else [None] * 5
            self.prediction_rows.append({
                "status_stamp_s": stamp,
                "behavior": self.status.get("behavior", "UNAVAILABLE"),
                "index": index,
                "reference_x": ref[0], "reference_y": ref[1], "reference_yaw": ref[2],
                "prediction_t_s": pred[0], "lead_prediction_x": pred[1],
                "lead_prediction_y": pred[2], "lead_prediction_yaw": pred[3],
                "lead_prediction_sigma_m": pred[4],
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
        return False

    def sample(self):
        if self.ego is None or self.opp is None:
            return
        time = self.elapsed()
        ego_x, ego_y, ego_yaw = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y, yaw_from_odom(self.ego)
        opp_x, opp_y, opp_yaw = self.opp.pose.pose.position.x, self.opp.pose.pose.position.y, yaw_from_odom(self.opp)
        ego_speed = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)
        opp_speed = math.hypot(self.opp.twist.twist.linear.x, self.opp.twist.twist.linear.y)
        # nav_msgs/Odometry defines twist in child_frame_id.  Gym therefore
        # reports longitudinal/lateral velocity in each vehicle body frame,
        # while the EKF estimate is expressed in the map frame.  Rotate truth
        # before comparing it with the estimate (the magnitude above is frame
        # invariant and remains useful for following metrics).
        opp_body_vx = self.opp.twist.twist.linear.x
        opp_body_vy = self.opp.twist.twist.linear.y
        opp_world_vx = math.cos(opp_yaw) * opp_body_vx - math.sin(opp_yaw) * opp_body_vy
        opp_world_vy = math.sin(opp_yaw) * opp_body_vx + math.cos(opp_yaw) * opp_body_vy
        ego_body, opp_body = rectangle(ego_x, ego_y, ego_yaw), rectangle(opp_x, opp_y, opp_yaw)
        body_clearance = polygon_distance(ego_body, opp_body)
        collision = overlap(ego_body, opp_body)
        if collision and not self.in_collision:
            self.collision_events += 1
        self.in_collision = collision
        boundary = self.boundary_violation(ego_body)
        if boundary and not self.in_boundary_violation:
            self.boundary_events += 1
        self.in_boundary_violation = boundary

        _, ego_progress, route_error = self.route_state(ego_x, ego_y, True)
        _, opp_progress, _ = self.route_state(opp_x, opp_y, False)
        estimate_error = velocity_error = float("nan")
        if self.estimate is not None:
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
            "committed_side": self.status.get("committed_side"),
            "ego_route_index_unwrapped": ego_progress,
            "lead_route_index_unwrapped": opp_progress,
            "route_lateral_error_m": route_error,
            "body_clearance_m": body_clearance,
            "collision": int(collision),
            "boundary_violation": int(boundary),
            "speed_command_mps": speed_command,
            "acceleration_command_est_mps2": acceleration,
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
        body = self.finite_values(self.rows, "body_clearance_m")
        route_error = self.finite_values(self.rows, "route_lateral_error_m")
        steering_rate = self.finite_values(self.rows, "steering_rate_radps")
        acceleration = self.finite_values(self.rows, "acceleration_command_est_mps2")
        planning = self.finite_values(self.rows, "planning_ms")
        solve = self.finite_values(self.rows, "mpc_solve_ms")
        rejection_reasons = Counter(
            row.get("controller_error") for row in self.rows
            if row.get("controller_error")
        )
        overtakes = int(self.status.get("overtakes_completed", 0))
        safe = bool(self.rows) and self.collision_events == 0 and self.boundary_events == 0
        behavior_samples = dict(self.behaviors)
        if self.expected_outcome in ("overtake", "fitzpatrick_long_straight_overtake"):
            expected_behavior_pass = overtakes > 0
        elif self.expected_outcome == "follow_wait_or_cancel":
            expected_behavior_pass = (
                overtakes == 0
                and sum(behavior_samples.get(state, 0) for state in ("FOLLOW", "WAIT", "CANCEL")) > 0
            )
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
                "minimum_body_clearance_m": float(body.min()) if body.size else None,
            },
            "overtaking": {
                "completed": overtakes,
                "first_completion_time_s": self.overtake_time,
                "behavior_sample_counts": behavior_samples,
            },
            "tracking": {
                "position_rmse_m": float(np.sqrt(np.mean(position ** 2))) if position.size else None,
                "velocity_rmse_mps": float(np.sqrt(np.mean(velocity ** 2))) if velocity.size else None,
                "loss_count": self.loss_count,
                "reacquisition_times_s": self.reacquisition_times,
                "tracker_mode_sample_counts": dict(self.tracker_modes),
            },
            "control": {
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
                "controller_failures": int(self.status.get("controller_failures", 0)),
                "solver_timeouts": int(self.status.get("solver_timeouts", 0)),
                "infeasible_count": int(self.status.get("infeasible_count", 0)),
                "solver_recoveries": int(self.status.get("solver_recoveries", 0)),
                "rejected_track_jumps": int(self.status.get("rejected_track_jumps", 0)),
                "controller_rejection_reasons": dict(rejection_reasons),
            },
            "ground_truth_policy": "Opponent ground truth is consumed only by takeover_evaluator for scoring.",
        }

    def write_metadata(self):
        payload = {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "scenario": self.scenario,
            "expected_outcome": self.expected_outcome,
            "run_id": self.run_id,
            "seed": self.seed,
            "map_name": self.map_name,
            "ground_truth_consumer": "takeover_evaluator only",
        }
        with open(os.path.join(self.output_dir, "metadata.json"), "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)

    def write_summary(self):
        if not self.rows:
            return
        with open(os.path.join(self.output_dir, "summary.json"), "w", encoding="utf-8") as handle:
            json.dump(self.summary(), handle, indent=2, sort_keys=True)

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
