#!/usr/bin/env python3
import csv
import json
import math
import os
from datetime import datetime, timezone

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class BaselineEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('baseline_evaluator')

        self.declare_parameter('scenario', 'constant_speed')
        self.declare_parameter('map_name', 'Melbourne_map')
        self.declare_parameter('run_id', '')
        self.declare_parameter('results_root', 'results')
        self.declare_parameter('follow_distance', 0.5)
        self.declare_parameter('estimate_timeout', 0.4)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('control_topic', '/drive')
        self.declare_parameter('method', 'nominal')

        self.scenario = str(self.get_parameter('scenario').value)
        self.map_name = str(self.get_parameter('map_name').value)
        self.method = str(self.get_parameter('method').value)
        run_id = str(self.get_parameter('run_id').value)
        if not run_id:
            run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        results_root = os.path.abspath(os.path.expanduser(str(self.get_parameter('results_root').value)))
        self.output_dir = os.path.join(results_root, self.scenario, run_id)
        os.makedirs(self.output_dir, exist_ok=True)

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.estimate_timeout = float(self.get_parameter('estimate_timeout').value)
        sample_rate = max(float(self.get_parameter('sample_rate').value), 1.0)

        self.ego = None
        self.ground_truth = None
        self.estimate = None
        self.estimate_receive_time = None
        self.tracker_status = {}
        self.tracker_status_time = None
        self.tracker_rejection_counts = {}
        self.latest_planning_ms = float('nan')
        self.latest_solve_ms = float('nan')
        self.latest_planner_valid = None
        self.latest_front_clearance = float('nan')
        self.latest_path_heading_error = float('nan')
        self.latest_reference_speed = float('nan')
        self.latest_dynamic_follow_distance = self.follow_distance
        self.latest_prediction_horizon = float('nan')
        self.latest_safe_speed_cap = float('nan')
        self.latest_control_mode = 'UNINITIALIZED'
        self.latest_risk_score = float('nan')
        self.latest_raw_risk_score = float('nan')
        self.latest_risk_speed_scale = float('nan')
        self.latest_sigma_long = float('nan')
        self.latest_sigma_lat = float('nan')
        self.latest_visibility = float('nan')
        self.latest_planner_decision = {}
        self.start_time = self.get_clock().now()
        self.last_steering = None
        self.last_control_time = None
        self.ever_had_estimate = False
        self.ever_had_measurement = False
        self.target_lost = False
        self.loss_start_time = None
        self.loss_count = 0
        self.reacquisition_times = []

        self.position_error_sq = []
        self.velocity_error_sq = []
        self.follow_error_sq = []
        self.dynamic_follow_error_sq = []
        self.lead_distances = []
        self.speed_error_sq = []
        self.steering_rates = []
        self.planning_times = []
        self.solve_times = []
        self.planner_results = []
        self.front_clearances = []
        self.reference_speeds = []
        self.safe_speed_caps = []
        self.dynamic_follow_distances = []
        self.prediction_horizons = []
        self.control_mode_counts = {}
        self.risk_scores = []
        self.raw_risk_scores = []
        self.risk_speed_scales = []
        self.sigma_longs = []
        self.sigma_lats = []
        self.visibilities = []
        self.candidate_selection_counts = {}
        self.candidate_switch_count = 0
        self.no_checked_path_count = 0
        self.multi_candidate_decision_count = 0
        self.selected_best_visibility_count = 0
        self.selected_visibility_advantages = []
        self.visibility_advantages_over_geometric = []
        self.visibility_margin_cleared_count = 0
        self.visibility_changed_selection_count = 0
        self.corridor_refresh_requested_count = 0
        self.corridor_refresh_succeeded_count = 0
        self.tracking_samples = 0
        self.estimate_valid_samples = 0
        self.measurement_valid_samples = 0
        self.coasting_samples = 0
        self.measurement_ages = []

        self.files = {}
        self.writers = {}
        self._open_csv('tracking', [
            'time_s', 'estimate_valid', 'measurement_valid', 'estimate_age_s',
            'measurement_age_s', 'tracker_mode', 'miss_reason',
            'candidate_count', 'association_error_m', 'association_gate_m',
            'position_covariance_trace', 'existence_probability',
            'reacquisition_confirmation_count', 'reacquisition_motion_mps',
            'reacquisition_motion_rmse_m',
            'estimated_x', 'estimated_y', 'estimated_vx', 'estimated_vy',
            'ground_truth_x', 'ground_truth_y', 'ground_truth_vx', 'ground_truth_vy',
            'position_error_m', 'velocity_error_mps',
        ])
        self._open_csv('following', [
            'time_s', 'lead_distance_m', 'follow_distance_error_m',
            'dynamic_follow_distance_m', 'dynamic_follow_error_m',
            'reference_speed_mps', 'safe_speed_cap_mps',
            'prediction_horizon_s', 'control_mode',
            'ego_speed_mps', 'lead_speed_mps', 'speed_error_mps',
            'ego_x', 'ego_y', 'ego_yaw', 'lead_x', 'lead_y', 'lead_yaw',
        ])
        self._open_csv('control', [
            'time_s', 'speed_command_mps', 'steering_angle_rad', 'steering_rate_radps',
        ])
        self._open_csv('runtime', [
            'time_s', 'planning_time_ms', 'mpc_solve_time_ms', 'planner_valid',
            'front_clearance_m', 'path_heading_error_rad',
        ])
        self._open_csv('decision', [
            'time_s', 'method', 'selected_candidate', 'valid_candidate_count',
            'candidate_switched', 'hard_failure', 'risk_score', 'sigma_long_m',
            'raw_risk_score', 'risk_speed_scale', 'sigma_lat_m',
            'measurement_age_s', 'predicted_visibility',
            'candidate_scores_json', 'candidate_diagnostics_json',
        ])
        self._open_csv('decision_events', [
            'time_s', 'method', 'selected_candidate', 'geometric_preference',
            'valid_candidate_count', 'candidate_switched', 'hard_failure',
            'visibility_margin', 'visibility_advantage_over_geometric',
            'visibility_margin_cleared', 'visibility_changed_selection',
            'corridor_refresh_requested', 'corridor_refresh_succeeded',
            'decision_json',
        ])

        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_callback, 20)
        # Ground truth is intentionally consumed only in this evaluator.
        self.create_subscription(Odometry, '/opp_racecar/odom', self.ground_truth_callback, 20)
        self.create_subscription(Odometry, '/ego_racecar/opp_odom_ekf', self.estimate_callback, 20)
        self.create_subscription(String, '/ego_tracker/status', self.tracker_status_callback, 20)
        self.control_topic = str(self.get_parameter('control_topic').value)
        self.create_subscription(AckermannDriveStamped, self.control_topic, self.control_callback, 20)
        self.create_subscription(Float64, '/ego_mpc/planning_time_ms', self.planning_callback, 20)
        self.create_subscription(Float64, '/ego_mpc/solve_time_ms', self.solve_callback, 20)
        self.create_subscription(Bool, '/ego_mpc/planner_valid', self.planner_valid_callback, 20)
        self.create_subscription(
            Float64, '/ego_mpc/front_clearance_m', self.front_clearance_callback, 20
        )
        self.create_subscription(
            Float64, '/ego_mpc/path_heading_error_rad', self.path_heading_error_callback, 20
        )
        self.create_subscription(
            Float64, '/ego_mpc/reference_speed_mps', self.reference_speed_callback, 20
        )
        self.create_subscription(
            Float64,
            '/ego_mpc/dynamic_follow_distance_m',
            self.dynamic_follow_distance_callback,
            20,
        )
        self.create_subscription(
            Float64,
            '/ego_mpc/target_prediction_horizon_s',
            self.prediction_horizon_callback,
            20,
        )
        self.create_subscription(
            Float64, '/ego_mpc/safe_speed_cap_mps', self.safe_speed_cap_callback, 20
        )
        self.create_subscription(
            String, '/ego_mpc/control_mode', self.control_mode_callback, 20
        )
        self.create_subscription(Float64, '/ego_mpc/risk_score', self.risk_callback, 20)
        self.create_subscription(
            Float64, '/ego_mpc/raw_risk_score', self.raw_risk_callback, 20
        )
        self.create_subscription(
            Float64, '/ego_mpc/risk_speed_scale', self.risk_speed_scale_callback, 20
        )
        self.create_subscription(Float64, '/ego_mpc/sigma_long_m', self.sigma_long_callback, 20)
        self.create_subscription(Float64, '/ego_mpc/sigma_lat_m', self.sigma_lat_callback, 20)
        self.create_subscription(
            Float64, '/ego_mpc/predicted_visibility', self.visibility_callback, 20
        )
        self.create_subscription(
            String, '/ego_mpc/planner_decision', self.planner_decision_callback, 20
        )
        self.sample_timer = self.create_timer(1.0 / sample_rate, self.sample)
        self.summary_timer = self.create_timer(1.0, self.write_summary)

        self._write_metadata(run_id)
        self.get_logger().info(f'Baseline results: {self.output_dir}')

    def _open_csv(self, name, fieldnames) -> None:
        handle = open(os.path.join(self.output_dir, f'{name}.csv'), 'w', newline='')
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        handle.flush()
        self.files[name] = handle
        self.writers[name] = writer

    def _write_metadata(self, run_id: str) -> None:
        metadata = {
            'scenario': self.scenario,
            'map_name': self.map_name,
            'method': self.method,
            'run_id': run_id,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'measurement_source': 'lidar',
            'ground_truth_topic': '/opp_racecar/odom',
            'ground_truth_role': 'evaluation_only',
            'estimated_target_topic': '/ego_racecar/opp_odom_ekf',
            'evaluated_control_topic': self.control_topic,
            'follow_distance_m': self.follow_distance,
        }
        with open(os.path.join(self.output_dir, 'metadata.json'), 'w') as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    @staticmethod
    def speed(msg: Odometry) -> float:
        return math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    @staticmethod
    def yaw(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    @classmethod
    def map_velocity(cls, msg: Odometry):
        """Rotate Gym's base-link twist into the odometry/map frame."""
        yaw = cls.yaw(msg)
        body_vx = float(msg.twist.twist.linear.x)
        body_vy = float(msg.twist.twist.linear.y)
        return (
            math.cos(yaw) * body_vx - math.sin(yaw) * body_vy,
            math.sin(yaw) * body_vx + math.cos(yaw) * body_vy,
        )

    def ego_callback(self, msg: Odometry) -> None:
        self.ego = msg

    def ground_truth_callback(self, msg: Odometry) -> None:
        self.ground_truth = msg

    def estimate_callback(self, msg: Odometry) -> None:
        now = self.get_clock().now()
        self.estimate = msg
        self.estimate_receive_time = now
        self.ever_had_estimate = True

    def tracker_status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            status = {
                'mode': 'INVALID',
                'measurement_age_s': None,
                'miss_reason': 'invalid_status',
            }
        self.tracker_status = status
        self.tracker_status_time = self.get_clock().now()
        miss_reason = str(status.get('miss_reason', 'unknown'))
        if miss_reason not in ('none', ''):
            self.tracker_rejection_counts[miss_reason] = (
                self.tracker_rejection_counts.get(miss_reason, 0) + 1
            )

    def control_callback(self, msg: AckermannDriveStamped) -> None:
        now = self.get_clock().now()
        steering = float(msg.drive.steering_angle)
        steering_rate = float('nan')
        if self.last_steering is not None and self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            if dt > 1e-4:
                steering_rate = (steering - self.last_steering) / dt
                if math.isfinite(steering_rate):
                    self.steering_rates.append(steering_rate)
        self.last_steering = steering
        self.last_control_time = now
        self.writers['control'].writerow({
            'time_s': self.elapsed(),
            'speed_command_mps': float(msg.drive.speed),
            'steering_angle_rad': steering,
            'steering_rate_radps': steering_rate,
        })

    def planning_callback(self, msg: Float64) -> None:
        self.latest_planning_ms = float(msg.data)
        if math.isfinite(self.latest_planning_ms):
            self.planning_times.append(self.latest_planning_ms)

    def solve_callback(self, msg: Float64) -> None:
        self.latest_solve_ms = float(msg.data)
        if math.isfinite(self.latest_solve_ms):
            self.solve_times.append(self.latest_solve_ms)

    def planner_valid_callback(self, msg: Bool) -> None:
        self.latest_planner_valid = bool(msg.data)
        self.planner_results.append(self.latest_planner_valid)

    def front_clearance_callback(self, msg: Float64) -> None:
        self.latest_front_clearance = float(msg.data)
        if math.isfinite(self.latest_front_clearance):
            self.front_clearances.append(self.latest_front_clearance)

    def path_heading_error_callback(self, msg: Float64) -> None:
        self.latest_path_heading_error = float(msg.data)

    def reference_speed_callback(self, msg: Float64) -> None:
        self.latest_reference_speed = float(msg.data)
        if math.isfinite(self.latest_reference_speed):
            self.reference_speeds.append(self.latest_reference_speed)

    def dynamic_follow_distance_callback(self, msg: Float64) -> None:
        self.latest_dynamic_follow_distance = float(msg.data)
        if math.isfinite(self.latest_dynamic_follow_distance):
            self.dynamic_follow_distances.append(
                self.latest_dynamic_follow_distance
            )

    def prediction_horizon_callback(self, msg: Float64) -> None:
        self.latest_prediction_horizon = float(msg.data)
        if math.isfinite(self.latest_prediction_horizon):
            self.prediction_horizons.append(self.latest_prediction_horizon)

    def safe_speed_cap_callback(self, msg: Float64) -> None:
        self.latest_safe_speed_cap = float(msg.data)
        if math.isfinite(self.latest_safe_speed_cap):
            self.safe_speed_caps.append(self.latest_safe_speed_cap)

    def control_mode_callback(self, msg: String) -> None:
        self.latest_control_mode = str(msg.data)
        self.control_mode_counts[self.latest_control_mode] = (
            self.control_mode_counts.get(self.latest_control_mode, 0) + 1
        )

    def risk_callback(self, msg: Float64) -> None:
        self.latest_risk_score = float(msg.data)
        if math.isfinite(self.latest_risk_score):
            self.risk_scores.append(self.latest_risk_score)

    def raw_risk_callback(self, msg: Float64) -> None:
        self.latest_raw_risk_score = float(msg.data)
        if math.isfinite(self.latest_raw_risk_score):
            self.raw_risk_scores.append(self.latest_raw_risk_score)

    def risk_speed_scale_callback(self, msg: Float64) -> None:
        self.latest_risk_speed_scale = float(msg.data)
        if math.isfinite(self.latest_risk_speed_scale):
            self.risk_speed_scales.append(self.latest_risk_speed_scale)

    def sigma_long_callback(self, msg: Float64) -> None:
        self.latest_sigma_long = float(msg.data)
        if math.isfinite(self.latest_sigma_long):
            self.sigma_longs.append(self.latest_sigma_long)

    def sigma_lat_callback(self, msg: Float64) -> None:
        self.latest_sigma_lat = float(msg.data)
        if math.isfinite(self.latest_sigma_lat):
            self.sigma_lats.append(self.latest_sigma_lat)

    def visibility_callback(self, msg: Float64) -> None:
        self.latest_visibility = float(msg.data)
        if math.isfinite(self.latest_visibility):
            self.visibilities.append(self.latest_visibility)

    def planner_decision_callback(self, msg: String) -> None:
        try:
            decision = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        selected = str(decision.get('selected_candidate', 'none'))
        self.candidate_selection_counts[selected] = (
            self.candidate_selection_counts.get(selected, 0) + 1
        )
        self.candidate_switch_count += int(bool(decision.get('switched', False)))
        self.no_checked_path_count += int(bool(decision.get('hard_failure', False)))
        self.corridor_refresh_requested_count += int(bool(
            decision.get('corridor_refresh_requested', False)
        ))
        self.corridor_refresh_succeeded_count += int(bool(
            decision.get('corridor_refresh_succeeded', False)
        ))
        valid_candidate_count = int(decision.get('valid_candidate_count') or 0)
        try:
            visibility_advantage = float(
                decision.get('visibility_advantage_over_geometric')
            )
        except (TypeError, ValueError):
            visibility_advantage = float('nan')
        if valid_candidate_count >= 2 and math.isfinite(visibility_advantage):
            self.visibility_advantages_over_geometric.append(visibility_advantage)
            self.visibility_margin_cleared_count += int(bool(
                decision.get('visibility_margin_cleared', False)
            ))
            self.visibility_changed_selection_count += int(bool(
                decision.get('visibility_changed_selection', False)
            ))
        visible_candidates = []
        for candidate in decision.get('candidates', []):
            if not candidate.get('valid', False):
                continue
            try:
                visibility = float(candidate.get('visibility'))
            except (TypeError, ValueError):
                continue
            if math.isfinite(visibility):
                visible_candidates.append((str(candidate.get('label')), visibility))
        if len(visible_candidates) >= 2:
            self.multi_candidate_decision_count += 1
            selected_visibility = next(
                (value for label, value in visible_candidates if label == selected),
                None,
            )
            alternatives = [
                value for label, value in visible_candidates if label != selected
            ]
            if selected_visibility is not None and alternatives:
                advantage = selected_visibility - max(alternatives)
                self.selected_visibility_advantages.append(advantage)
                if advantage >= -1e-9:
                    self.selected_best_visibility_count += 1
        self.latest_planner_decision = decision
        self.writers['decision_events'].writerow({
            'time_s': self.elapsed(),
            'method': self.method,
            'selected_candidate': selected,
            'geometric_preference': decision.get('geometric_preference', ''),
            'valid_candidate_count': decision.get('valid_candidate_count', ''),
            'candidate_switched': int(bool(decision.get('switched', False))),
            'hard_failure': int(bool(decision.get('hard_failure', False))),
            'visibility_margin': decision.get('visibility_margin', ''),
            'visibility_advantage_over_geometric': (
                visibility_advantage if math.isfinite(visibility_advantage) else ''
            ),
            'visibility_margin_cleared': int(bool(
                decision.get('visibility_margin_cleared', False)
            )),
            'visibility_changed_selection': int(bool(
                decision.get('visibility_changed_selection', False)
            )),
            'corridor_refresh_requested': int(bool(
                decision.get('corridor_refresh_requested', False)
            )),
            'corridor_refresh_succeeded': int(bool(
                decision.get('corridor_refresh_succeeded', False)
            )),
            'decision_json': json.dumps(decision, sort_keys=True),
        })
        self.files['decision_events'].flush()

    def sample(self) -> None:
        now = self.get_clock().now()
        estimate_age = float('inf')
        if self.estimate_receive_time is not None:
            estimate_age = (now - self.estimate_receive_time).nanoseconds * 1e-9
        status_age = float('inf')
        if self.tracker_status_time is not None:
            status_age = (now - self.tracker_status_time).nanoseconds * 1e-9
        tracker_mode = str(self.tracker_status.get('mode', 'UNINITIALIZED'))
        reported_measurement_age = self.tracker_status.get('measurement_age_s')
        if reported_measurement_age is None or not math.isfinite(status_age):
            measurement_age = float('inf')
        else:
            measurement_age = max(0.0, float(reported_measurement_age) + status_age)
        status_fresh = status_age <= self.estimate_timeout
        estimate_valid = (
            self.estimate is not None
            and estimate_age <= self.estimate_timeout
            and status_fresh
            and tracker_mode in ('TRACKED', 'COASTING')
        )
        measurement_valid = (
            status_fresh
            and measurement_age <= self.estimate_timeout
            and tracker_mode in ('TRACKED', 'COASTING')
        )

        self.tracking_samples += 1
        self.estimate_valid_samples += int(estimate_valid)
        self.measurement_valid_samples += int(measurement_valid)
        self.coasting_samples += int(tracker_mode == 'COASTING')
        if math.isfinite(measurement_age):
            self.measurement_ages.append(measurement_age)

        if measurement_valid:
            self.ever_had_measurement = True
            if self.target_lost and self.loss_start_time is not None:
                reacquisition = (now - self.loss_start_time).nanoseconds * 1e-9
                self.reacquisition_times.append(reacquisition)
                self.target_lost = False
                self.loss_start_time = None
        elif self.ever_had_measurement and not self.target_lost:
            self.target_lost = True
            self.loss_count += 1
            self.loss_start_time = now

        if self.ground_truth is not None:
            gt = self.ground_truth
            gt_vx, gt_vy = self.map_velocity(gt)
            row = {
                'time_s': self.elapsed(), 'estimate_valid': int(estimate_valid),
                'measurement_valid': int(measurement_valid),
                'estimate_age_s': estimate_age,
                'measurement_age_s': measurement_age,
                'tracker_mode': tracker_mode,
                'miss_reason': self.tracker_status.get('miss_reason', 'unknown'),
                'candidate_count': self.tracker_status.get('candidate_count', ''),
                'association_error_m': self.tracker_status.get('association_error_m', ''),
                'association_gate_m': self.tracker_status.get('association_gate_m', ''),
                'position_covariance_trace': self.tracker_status.get(
                    'position_covariance_trace', ''
                ),
                'existence_probability': self.tracker_status.get(
                    'existence_probability', ''
                ),
                'reacquisition_confirmation_count': self.tracker_status.get(
                    'reacquisition_confirmation_count', ''
                ),
                'reacquisition_motion_mps': self.tracker_status.get(
                    'reacquisition_motion_mps', ''
                ),
                'reacquisition_motion_rmse_m': self.tracker_status.get(
                    'reacquisition_motion_rmse_m', ''
                ),
                'estimated_x': '', 'estimated_y': '', 'estimated_vx': '', 'estimated_vy': '',
                'ground_truth_x': gt.pose.pose.position.x,
                'ground_truth_y': gt.pose.pose.position.y,
                'ground_truth_vx': gt_vx,
                'ground_truth_vy': gt_vy,
                'position_error_m': '', 'velocity_error_mps': '',
            }
            if estimate_valid:
                est = self.estimate
                position_error = math.hypot(
                    est.pose.pose.position.x - gt.pose.pose.position.x,
                    est.pose.pose.position.y - gt.pose.pose.position.y,
                )
                velocity_error = math.hypot(
                    est.twist.twist.linear.x - gt_vx,
                    est.twist.twist.linear.y - gt_vy,
                )
                self.position_error_sq.append(position_error ** 2)
                self.velocity_error_sq.append(velocity_error ** 2)
                row.update({
                    'estimated_x': est.pose.pose.position.x,
                    'estimated_y': est.pose.pose.position.y,
                    'estimated_vx': est.twist.twist.linear.x,
                    'estimated_vy': est.twist.twist.linear.y,
                    'position_error_m': position_error,
                    'velocity_error_mps': velocity_error,
                })
            self.writers['tracking'].writerow(row)

        if self.ego is not None and self.ground_truth is not None:
            lead_distance = math.hypot(
                self.ground_truth.pose.pose.position.x - self.ego.pose.pose.position.x,
                self.ground_truth.pose.pose.position.y - self.ego.pose.pose.position.y,
            )
            follow_error = lead_distance - self.follow_distance
            dynamic_follow_error = lead_distance - self.latest_dynamic_follow_distance
            ego_speed = self.speed(self.ego)
            lead_speed = self.speed(self.ground_truth)
            speed_error = ego_speed - lead_speed
            self.follow_error_sq.append(follow_error ** 2)
            self.dynamic_follow_error_sq.append(dynamic_follow_error ** 2)
            self.lead_distances.append(lead_distance)
            self.speed_error_sq.append(speed_error ** 2)
            self.writers['following'].writerow({
                'time_s': self.elapsed(), 'lead_distance_m': lead_distance,
                'follow_distance_error_m': follow_error,
                'dynamic_follow_distance_m': self.latest_dynamic_follow_distance,
                'dynamic_follow_error_m': dynamic_follow_error,
                'reference_speed_mps': self.latest_reference_speed,
                'safe_speed_cap_mps': self.latest_safe_speed_cap,
                'prediction_horizon_s': self.latest_prediction_horizon,
                'control_mode': self.latest_control_mode,
                'ego_speed_mps': ego_speed, 'lead_speed_mps': lead_speed,
                'speed_error_mps': speed_error,
                'ego_x': self.ego.pose.pose.position.x,
                'ego_y': self.ego.pose.pose.position.y,
                'ego_yaw': self.yaw(self.ego),
                'lead_x': self.ground_truth.pose.pose.position.x,
                'lead_y': self.ground_truth.pose.pose.position.y,
                'lead_yaw': self.yaw(self.ground_truth),
            })

        self.writers['runtime'].writerow({
            'time_s': self.elapsed(),
            'planning_time_ms': self.latest_planning_ms,
            'mpc_solve_time_ms': self.latest_solve_ms,
            'planner_valid': '' if self.latest_planner_valid is None else int(self.latest_planner_valid),
            'front_clearance_m': self.latest_front_clearance,
            'path_heading_error_rad': self.latest_path_heading_error,
        })
        decision = self.latest_planner_decision
        candidate_scores = {
            item.get('label', 'unknown'): item.get('score')
            for item in decision.get('candidates', [])
        }
        self.writers['decision'].writerow({
            'time_s': self.elapsed(),
            'method': self.method,
            'selected_candidate': decision.get('selected_candidate', ''),
            'valid_candidate_count': decision.get('valid_candidate_count', ''),
            'candidate_switched': int(bool(decision.get('switched', False))),
            'hard_failure': int(bool(decision.get('hard_failure', False))),
            'risk_score': self.latest_risk_score,
            'raw_risk_score': self.latest_raw_risk_score,
            'risk_speed_scale': self.latest_risk_speed_scale,
            'sigma_long_m': self.latest_sigma_long,
            'sigma_lat_m': self.latest_sigma_lat,
            'measurement_age_s': decision.get('measurement_age_s', ''),
            'predicted_visibility': self.latest_visibility,
            'candidate_scores_json': json.dumps(candidate_scores, sort_keys=True),
            'candidate_diagnostics_json': json.dumps(
                decision.get('candidates', []), sort_keys=True
            ),
        })
        for handle in self.files.values():
            handle.flush()

    @staticmethod
    def rmse(squared_values):
        return math.sqrt(float(np.mean(squared_values))) if squared_values else None

    @staticmethod
    def stats(values):
        if not values:
            return {'mean': None, 'p95': None, 'max': None}
        array = np.asarray(values, dtype=float)
        return {
            'mean': float(np.mean(array)),
            'p95': float(np.percentile(array, 95)),
            'max': float(np.max(array)),
        }

    def write_summary(self) -> None:
        planner_attempts = len(self.planner_results)
        planner_successes = sum(self.planner_results)
        summary = {
            'scenario': self.scenario,
            'map_name': self.map_name,
            'method': self.method,
            'duration_s': self.elapsed(),
            'tracking': {
                'position_rmse_m': self.rmse(self.position_error_sq),
                'velocity_rmse_mps': self.rmse(self.velocity_error_sq),
                'target_loss_count': self.loss_count,
                'reacquisition_time_s': self.stats(self.reacquisition_times),
                'estimate_availability': (
                    self.estimate_valid_samples / self.tracking_samples
                    if self.tracking_samples else None
                ),
                'measurement_availability': (
                    self.measurement_valid_samples / self.tracking_samples
                    if self.tracking_samples else None
                ),
                'coasting_fraction': (
                    self.coasting_samples / self.tracking_samples
                    if self.tracking_samples else None
                ),
                'measurement_age_s': self.stats(self.measurement_ages),
                'rejection_reason_counts': self.tracker_rejection_counts,
            },
            'following': {
                'distance_rmse_m': self.rmse(self.follow_error_sq),
                'dynamic_distance_rmse_m': self.rmse(self.dynamic_follow_error_sq),
                'minimum_lead_distance_m': min(self.lead_distances) if self.lead_distances else None,
                'speed_rmse_mps': self.rmse(self.speed_error_sq),
                'reference_speed_mps': self.stats(self.reference_speeds),
                'safe_speed_cap_mps': self.stats(self.safe_speed_caps),
                'dynamic_follow_distance_m': self.stats(
                    self.dynamic_follow_distances
                ),
                'prediction_horizon_s': self.stats(self.prediction_horizons),
                'control_mode_counts': self.control_mode_counts,
            },
            'control': {
                'steering_rate_rms_radps': (
                    math.sqrt(float(np.mean(np.square(self.steering_rates))))
                    if self.steering_rates else None
                ),
            },
            'runtime': {
                'planning_time_ms': self.stats(self.planning_times),
                'mpc_solve_time_ms': self.stats(self.solve_times),
                'planner_attempts': planner_attempts,
                'planner_successes': planner_successes,
                'planner_failures': planner_attempts - planner_successes,
                'planner_success_rate': (
                    planner_successes / planner_attempts if planner_attempts else None
                ),
                'front_clearance_m': self.stats(self.front_clearances),
            },
            'risk_aware_planning': {
                'risk_score': self.stats(self.risk_scores),
                'raw_risk_score': self.stats(self.raw_risk_scores),
                'risk_speed_scale': self.stats(self.risk_speed_scales),
                'sigma_long_m': self.stats(self.sigma_longs),
                'sigma_lat_m': self.stats(self.sigma_lats),
                'predicted_visibility': self.stats(self.visibilities),
                'candidate_selection_counts': self.candidate_selection_counts,
                'candidate_switch_count': self.candidate_switch_count,
                'no_collision_checked_path_count': self.no_checked_path_count,
                'multi_candidate_decision_count': self.multi_candidate_decision_count,
                'selected_best_visibility_count': self.selected_best_visibility_count,
                'selected_visibility_advantage': self.stats(
                    self.selected_visibility_advantages
                ),
                'visibility_advantage_over_geometric': self.stats(
                    self.visibility_advantages_over_geometric
                ),
                'visibility_margin_cleared_count': (
                    self.visibility_margin_cleared_count
                ),
                'visibility_changed_selection_count': (
                    self.visibility_changed_selection_count
                ),
                'corridor_refresh_requested_count': (
                    self.corridor_refresh_requested_count
                ),
                'corridor_refresh_succeeded_count': (
                    self.corridor_refresh_succeeded_count
                ),
            },
        }
        path = os.path.join(self.output_dir, 'summary.json')
        temp_path = path + '.tmp'
        with open(temp_path, 'w') as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        os.replace(temp_path, path)

    def close(self) -> None:
        self.write_summary()
        for handle in self.files.values():
            handle.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaselineEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 launch can invalidate the context before spin observes the
        # shutdown guard.  Suppress only this normal teardown race.
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
