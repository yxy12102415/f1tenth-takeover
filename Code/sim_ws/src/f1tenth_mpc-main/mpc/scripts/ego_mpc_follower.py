#!/usr/bin/env python3
import heapq
import json
import math
import time
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt
from scipy.sparse import block_diag, csc_matrix
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker


@dataclass
class mpc_config:
    NXK: int = 4
    NU: int = 2
    # One second of preview at 20 Hz is needed to enter F1TENTH-scale turns
    # before the lead-vehicle anchor has already moved around the corner.
    TK: int = 20
    Rk: list = field(default_factory=lambda: np.diag([0.01, 90.0]))
    Rdk: list = field(default_factory=lambda: np.diag([0.02, 260.0]))
    Qk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    Qfk: list = field(default_factory=lambda: np.diag([25.0, 25.0, 0.12, 0.8]))
    DTK: float = 0.05
    WB: float = 0.33
    MIN_STEER: float = -0.50
    MAX_STEER: float = 0.50
    MAX_SPEED: float = 2.4
    MIN_SPEED: float = 0.0
    MAX_ACCEL: float = 0.8


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return quat.as_euler('zxy', degrees=False)[0]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EgoMPCFollower(Node):
    def __init__(self):
        super().__init__('ego_mpc_follower')

        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('target_odom_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('target_status_topic', '/ego_tracker/status')
        self.declare_parameter('drive_topic', '/drive_nominal')
        self.declare_parameter('debug_drive_topic', '/ego_mpc_debug_cmd')
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('target_timeout', 0.3)
        self.declare_parameter('target_state_timeout', 0.4)
        self.declare_parameter('tracker_status_timeout', 0.4)
        self.declare_parameter('scan_timeout', 0.4)
        self.declare_parameter('odom_timeout', 0.4)
        self.declare_parameter('recovery_valid_cycles', 3)
        self.declare_parameter('control_tolerance', 1e-3)
        self.declare_parameter('max_transient_planner_failures', 2)
        self.declare_parameter('min_speed_command', 0.3)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_accel', 0.8)
        self.declare_parameter('max_steer', 0.4189)
        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('wall_stop_distance', 0.45)
        self.declare_parameter('wall_slow_distance', 0.8)
        self.declare_parameter('cbf_front_gamma', 2.0)
        self.declare_parameter('cbf_q_accel', 1.0)
        self.declare_parameter('cbf_q_steer', 30.0)
        self.declare_parameter('mpc_steer_input_cost', 20.0)
        self.declare_parameter('mpc_steer_rate_cost', 70.0)
        self.declare_parameter('mpc_heading_cost', 0.8)
        self.declare_parameter('mpc_speed_cost', 0.12)
        self.declare_parameter('astar_resolution', 0.18)
        self.declare_parameter('astar_grid_radius', 5.5)
        self.declare_parameter('astar_obstacle_inflation', 0.35)
        self.declare_parameter('astar_max_scan_range', 6.0)
        self.declare_parameter('target_occupancy_exclusion_radius', 0.45)
        self.declare_parameter('target_center_offset', 0.29)
        self.declare_parameter('hybrid_astar_yaw_bins', 24)
        self.declare_parameter('hybrid_astar_step_distance', 0.25)
        self.declare_parameter('hybrid_astar_max_expansions', 5000)
        self.declare_parameter('hybrid_astar_time_budget_ms', 120.0)
        self.declare_parameter('target_heading_alpha', 0.15)
        self.declare_parameter('target_heading_min_speed', 0.4)
        self.declare_parameter('target_heading_full_speed', 1.0)
        self.declare_parameter('target_yaw_rate_alpha', 0.2)
        self.declare_parameter('max_target_yaw_rate', 1.5)
        self.declare_parameter('target_prediction_horizon', 1.0)
        self.declare_parameter('time_headway', 0.15)
        self.declare_parameter('maximum_follow_distance', 1.8)
        self.declare_parameter('gap_speed_gain', 0.6)
        self.declare_parameter('max_catchup_speed_bonus', 0.6)
        self.declare_parameter('minimum_motion_gap', 0.75)
        self.declare_parameter('safe_gap_gamma', 1.0)
        self.declare_parameter('minimum_safe_distance', 0.9)
        self.declare_parameter('goal_yaw_weight', 0.25)
        self.declare_parameter('coasting_min_speed_scale', 0.25)
        self.declare_parameter('turn_slowdown_gain', 0.55)
        self.declare_parameter('turn_min_speed_scale', 0.45)
        self.declare_parameter('method', 'nominal')
        # A candidate must represent a genuinely distinct corridor while
        # retaining margin to the observed track wall. The former 0.67 m
        # minimum put the path on the wall-inflation boundary in the mirrored
        # benchmark. The original 0.58 m seed still clears the compact center
        # obstacle and leaves a symmetric wall margin.
        self.declare_parameter('corridor_lateral_offset', 0.58)
        self.declare_parameter('corridor_min_lateral_offset', 0.58)
        self.declare_parameter('corridor_max_lateral_offset', 0.88)
        self.declare_parameter('corridor_lateral_offset_step', 0.05)
        self.declare_parameter('corridor_boundary_min_points', 6)
        self.declare_parameter('corridor_boundary_min_span', 0.55)
        self.declare_parameter('corridor_boundary_fit_max_rmse', 0.18)
        self.declare_parameter('corridor_curve_blend_distance', 0.60)
        self.declare_parameter('corridor_blockage_max_range', 4.6)
        self.declare_parameter('corridor_blockage_confirmation_scans', 3)
        self.declare_parameter('corridor_blockage_match_radius', 0.25)
        self.declare_parameter('corridor_geometry_refresh_period', 0.20)
        self.declare_parameter('candidate_switch_margin', 0.05)
        self.declare_parameter('candidate_rejoin_margin', 0.25)
        self.declare_parameter('candidate_planning_budget_ms', 60.0)
        self.declare_parameter('method_astar_resolution', 0.12)
        self.declare_parameter('method_obstacle_inflation', 0.20)
        self.declare_parameter('planning_start_clearance_radius', 0.24)
        self.declare_parameter('method_clearance_cost_gain', 1.2)
        self.declare_parameter('method_clearance_cost_scale', 0.45)
        self.declare_parameter('method_astar_max_expansions', 15000)
        self.declare_parameter('visibility_fov_deg', 190.0)
        self.declare_parameter('visibility_max_range', 6.0)
        self.declare_parameter('visibility_samples', 5)
        self.declare_parameter('visibility_prediction_horizon', 3.0)
        self.declare_parameter('uncertainty_sigma_reference', 0.35)
        self.declare_parameter('uncertainty_distance_gain', 1.5)
        self.declare_parameter('measurement_age_distance_gain', 0.4)
        self.declare_parameter('visibility_distance_gain', 0.8)
        self.declare_parameter('risk_sigma_weight', 0.4)
        self.declare_parameter('risk_age_weight', 0.35)
        self.declare_parameter('risk_visibility_weight', 0.25)
        self.declare_parameter('risk_speed_gain', 0.65)
        self.declare_parameter('minimum_risk_speed_scale', 0.3)
        self.declare_parameter('risk_release_alpha', 0.15)
        self.declare_parameter('candidate_length_weight', 0.55)
        self.declare_parameter('candidate_curvature_weight', 0.35)
        self.declare_parameter('candidate_clearance_weight', 0.8)
        self.declare_parameter('candidate_rejoin_weight', 0.05)
        self.declare_parameter('candidate_visibility_weight', 2.5)
        self.declare_parameter('candidate_visibility_margin', 0.075)
        self.declare_parameter('candidate_switch_weight', 0.05)
        self.declare_parameter('fault_inject_scan_timeout', False)
        self.declare_parameter('fault_inject_odom_timeout', False)
        self.declare_parameter('fault_inject_target_timeout', False)
        self.declare_parameter('fault_inject_planner_failure', False)
        self.declare_parameter('fault_inject_solver_failure', False)

        ego_topic = self.get_parameter('ego_odom_topic').value
        target_topic = self.get_parameter('target_odom_topic').value
        target_status_topic = self.get_parameter('target_status_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        debug_drive_topic = self.get_parameter('debug_drive_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.target_state_timeout = float(self.get_parameter('target_state_timeout').value)
        self.tracker_status_timeout = float(self.get_parameter('tracker_status_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.recovery_valid_cycles = max(1, int(self.get_parameter('recovery_valid_cycles').value))
        self.control_tolerance = max(float(self.get_parameter('control_tolerance').value), 0.0)
        self.max_transient_planner_failures = max(
            0, int(self.get_parameter('max_transient_planner_failures').value)
        )
        self.min_speed_command = float(self.get_parameter('min_speed_command').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_accel = float(self.get_parameter('max_accel').value)
        max_steer = float(self.get_parameter('max_steer').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.wall_stop_distance = float(self.get_parameter('wall_stop_distance').value)
        self.wall_slow_distance = float(self.get_parameter('wall_slow_distance').value)
        self.cbf_front_gamma = float(self.get_parameter('cbf_front_gamma').value)
        self.cbf_q_accel = float(self.get_parameter('cbf_q_accel').value)
        self.cbf_q_steer = float(self.get_parameter('cbf_q_steer').value)
        mpc_steer_input_cost = max(
            0.0, float(self.get_parameter('mpc_steer_input_cost').value)
        )
        mpc_steer_rate_cost = max(
            0.0, float(self.get_parameter('mpc_steer_rate_cost').value)
        )
        mpc_heading_cost = max(
            0.0, float(self.get_parameter('mpc_heading_cost').value)
        )
        mpc_speed_cost = max(
            0.0, float(self.get_parameter('mpc_speed_cost').value)
        )
        self.astar_resolution = float(self.get_parameter('astar_resolution').value)
        self.astar_grid_radius = float(self.get_parameter('astar_grid_radius').value)
        self.astar_obstacle_inflation = float(self.get_parameter('astar_obstacle_inflation').value)
        self.astar_max_scan_range = float(self.get_parameter('astar_max_scan_range').value)
        self.target_occupancy_exclusion_radius = max(
            0.0,
            float(self.get_parameter('target_occupancy_exclusion_radius').value),
        )
        self.target_center_offset = max(
            0.0, float(self.get_parameter('target_center_offset').value)
        )
        self.hybrid_astar_yaw_bins = int(self.get_parameter('hybrid_astar_yaw_bins').value)
        self.hybrid_astar_step_distance = float(self.get_parameter('hybrid_astar_step_distance').value)
        self.hybrid_astar_max_expansions = int(self.get_parameter('hybrid_astar_max_expansions').value)
        self.hybrid_astar_time_budget_ms = max(
            1.0, float(self.get_parameter('hybrid_astar_time_budget_ms').value)
        )
        self.target_heading_alpha = float(np.clip(
            self.get_parameter('target_heading_alpha').value, 0.0, 1.0
        ))
        self.target_heading_min_speed = max(
            0.0, float(self.get_parameter('target_heading_min_speed').value)
        )
        self.target_heading_full_speed = max(
            self.target_heading_min_speed + 1e-3,
            float(self.get_parameter('target_heading_full_speed').value),
        )
        self.target_yaw_rate_alpha = float(np.clip(
            self.get_parameter('target_yaw_rate_alpha').value, 0.0, 1.0
        ))
        self.max_target_yaw_rate = max(
            0.0, float(self.get_parameter('max_target_yaw_rate').value)
        )
        self.target_prediction_horizon = float(np.clip(
            self.get_parameter('target_prediction_horizon').value,
            0.0,
            self.config_horizon_seconds(),
        ))
        self.time_headway = max(0.0, float(self.get_parameter('time_headway').value))
        self.maximum_follow_distance = max(
            self.follow_distance,
            float(self.get_parameter('maximum_follow_distance').value),
        )
        self.gap_speed_gain = max(0.0, float(self.get_parameter('gap_speed_gain').value))
        self.max_catchup_speed_bonus = max(
            0.0, float(self.get_parameter('max_catchup_speed_bonus').value)
        )
        self.minimum_motion_gap = max(
            0.0, float(self.get_parameter('minimum_motion_gap').value)
        )
        self.safe_gap_gamma = max(
            0.0, float(self.get_parameter('safe_gap_gamma').value)
        )
        self.minimum_safe_distance = max(
            0.0, float(self.get_parameter('minimum_safe_distance').value)
        )
        self.goal_yaw_weight = max(0.0, float(self.get_parameter('goal_yaw_weight').value))
        self.coasting_min_speed_scale = float(np.clip(
            self.get_parameter('coasting_min_speed_scale').value, 0.0, 1.0
        ))
        self.turn_slowdown_gain = max(
            0.0, float(self.get_parameter('turn_slowdown_gain').value)
        )
        self.turn_min_speed_scale = float(np.clip(
            self.get_parameter('turn_min_speed_scale').value, 0.0, 1.0
        ))
        self.method = str(self.get_parameter('method').value).strip().lower()
        valid_methods = {'nominal', 'corridor', 'visibility', 'adaptive', 'full'}
        if self.method not in valid_methods:
            raise ValueError(
                f'Unsupported method {self.method!r}; choose one of {sorted(valid_methods)}'
            )
        self.use_corridor = self.method in {'corridor', 'visibility', 'full'}
        self.use_visibility = self.method in {'visibility', 'full'}
        self.use_adaptive_risk = self.method in {'adaptive', 'full'}
        self.require_collision_checked_path = self.method != 'nominal'
        self.astar_clearance_cost_gain = 0.0
        self.astar_clearance_cost_scale = 0.45
        self.planning_start_clearance_radius = (
            max(
                0.0,
                float(self.get_parameter('planning_start_clearance_radius').value),
            )
            if self.method != 'nominal' else 0.0
        )
        if self.method != 'nominal':
            self.astar_resolution = max(
                0.05, float(self.get_parameter('method_astar_resolution').value)
            )
            self.astar_obstacle_inflation = max(
                0.1, float(self.get_parameter('method_obstacle_inflation').value)
            )
            self.astar_clearance_cost_gain = max(
                0.0, float(self.get_parameter('method_clearance_cost_gain').value)
            )
            self.astar_clearance_cost_scale = max(
                0.05, float(self.get_parameter('method_clearance_cost_scale').value)
            )
            self.hybrid_astar_max_expansions = max(
                self.hybrid_astar_max_expansions,
                int(self.get_parameter('method_astar_max_expansions').value),
            )
        self.corridor_lateral_offset = max(
            0.0, float(self.get_parameter('corridor_lateral_offset').value)
        )
        self.corridor_min_lateral_offset = max(
            0.2, float(self.get_parameter('corridor_min_lateral_offset').value)
        )
        self.corridor_max_lateral_offset = max(
            self.corridor_min_lateral_offset,
            float(self.get_parameter('corridor_max_lateral_offset').value),
        )
        self.corridor_lateral_offset = float(np.clip(
            self.corridor_lateral_offset,
            self.corridor_min_lateral_offset,
            self.corridor_max_lateral_offset,
        ))
        self.corridor_lateral_offset_step = max(
            0.02,
            float(self.get_parameter('corridor_lateral_offset_step').value),
        )
        self.corridor_boundary_min_points = max(
            4, int(self.get_parameter('corridor_boundary_min_points').value)
        )
        self.corridor_boundary_min_span = max(
            0.2, float(self.get_parameter('corridor_boundary_min_span').value)
        )
        self.corridor_boundary_fit_max_rmse = max(
            0.02, float(self.get_parameter('corridor_boundary_fit_max_rmse').value)
        )
        self.corridor_curve_blend_distance = max(
            0.1, float(self.get_parameter('corridor_curve_blend_distance').value)
        )
        self.corridor_blockage_max_range = max(
            1.0, float(self.get_parameter('corridor_blockage_max_range').value)
        )
        self.corridor_blockage_confirmation_scans = max(
            1,
            int(self.get_parameter('corridor_blockage_confirmation_scans').value),
        )
        self.corridor_blockage_match_radius = max(
            0.05, float(self.get_parameter('corridor_blockage_match_radius').value)
        )
        self.corridor_geometry_refresh_period = max(
            0.05,
            float(self.get_parameter('corridor_geometry_refresh_period').value),
        )
        self.candidate_switch_margin = max(
            0.0, float(self.get_parameter('candidate_switch_margin').value)
        )
        self.candidate_rejoin_margin = max(
            self.candidate_switch_margin,
            float(self.get_parameter('candidate_rejoin_margin').value),
        )
        self.candidate_planning_budget_ms = max(
            3.0, float(self.get_parameter('candidate_planning_budget_ms').value)
        )
        self.visibility_fov_rad = math.radians(float(np.clip(
            self.get_parameter('visibility_fov_deg').value, 1.0, 360.0
        )))
        self.visibility_max_range = max(
            0.1, float(self.get_parameter('visibility_max_range').value)
        )
        self.visibility_samples = max(
            2, int(self.get_parameter('visibility_samples').value)
        )
        self.visibility_prediction_horizon = max(
            mpc_config.DTK,
            float(self.get_parameter('visibility_prediction_horizon').value),
        )
        self.uncertainty_sigma_reference = max(
            1e-3, float(self.get_parameter('uncertainty_sigma_reference').value)
        )
        self.uncertainty_distance_gain = max(
            0.0, float(self.get_parameter('uncertainty_distance_gain').value)
        )
        self.measurement_age_distance_gain = max(
            0.0, float(self.get_parameter('measurement_age_distance_gain').value)
        )
        self.visibility_distance_gain = max(
            0.0, float(self.get_parameter('visibility_distance_gain').value)
        )
        risk_weights = np.array([
            max(0.0, float(self.get_parameter('risk_sigma_weight').value)),
            max(0.0, float(self.get_parameter('risk_age_weight').value)),
            max(0.0, float(self.get_parameter('risk_visibility_weight').value)),
        ])
        if float(np.sum(risk_weights)) <= 1e-9:
            risk_weights[:] = (0.4, 0.35, 0.25)
        self.risk_weights = risk_weights / np.sum(risk_weights)
        self.risk_speed_gain = max(
            0.0, float(self.get_parameter('risk_speed_gain').value)
        )
        self.minimum_risk_speed_scale = float(np.clip(
            self.get_parameter('minimum_risk_speed_scale').value, 0.0, 1.0
        ))
        self.risk_release_alpha = float(np.clip(
            self.get_parameter('risk_release_alpha').value, 0.0, 1.0
        ))
        self.candidate_weights = {
            'length': max(0.0, float(self.get_parameter('candidate_length_weight').value)),
            'curvature': max(0.0, float(self.get_parameter('candidate_curvature_weight').value)),
            'clearance': max(0.0, float(self.get_parameter('candidate_clearance_weight').value)),
            'rejoin': max(0.0, float(self.get_parameter('candidate_rejoin_weight').value)),
            'visibility': max(0.0, float(self.get_parameter('candidate_visibility_weight').value)),
            'switch': max(0.0, float(self.get_parameter('candidate_switch_weight').value)),
        }
        self.candidate_visibility_margin = float(np.clip(
            self.get_parameter('candidate_visibility_margin').value, 0.0, 1.0
        ))

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.debug_drive_pub = self.create_publisher(AckermannDriveStamped, debug_drive_topic, 10)
        self.pred_path_pub = self.create_publisher(Marker, '/ego_mpc_pred_path_vis', 1)
        self.ref_path_pub = self.create_publisher(Marker, '/ego_mpc_ref_path_vis', 1)
        self.astar_path_pub = self.create_publisher(Marker, '/ego_astar_path_vis', 1)
        self.planning_time_pub = self.create_publisher(Float64, '/ego_mpc/planning_time_ms', 10)
        self.solve_time_pub = self.create_publisher(Float64, '/ego_mpc/solve_time_ms', 10)
        self.planner_valid_pub = self.create_publisher(Bool, '/ego_mpc/planner_valid', 10)
        self.front_clearance_pub = self.create_publisher(
            Float64, '/ego_mpc/front_clearance_m', 10
        )
        self.path_heading_error_pub = self.create_publisher(
            Float64, '/ego_mpc/path_heading_error_rad', 10
        )
        self.reference_speed_pub = self.create_publisher(
            Float64, '/ego_mpc/reference_speed_mps', 10
        )
        self.dynamic_follow_distance_pub = self.create_publisher(
            Float64, '/ego_mpc/dynamic_follow_distance_m', 10
        )
        self.prediction_horizon_pub = self.create_publisher(
            Float64, '/ego_mpc/target_prediction_horizon_s', 10
        )
        self.safe_speed_cap_pub = self.create_publisher(
            Float64, '/ego_mpc/safe_speed_cap_mps', 10
        )
        self.control_mode_pub = self.create_publisher(
            String, '/ego_mpc/control_mode', 10
        )
        self.risk_score_pub = self.create_publisher(Float64, '/ego_mpc/risk_score', 10)
        self.raw_risk_score_pub = self.create_publisher(
            Float64, '/ego_mpc/raw_risk_score', 10
        )
        self.risk_speed_scale_pub = self.create_publisher(
            Float64, '/ego_mpc/risk_speed_scale', 10
        )
        self.sigma_long_pub = self.create_publisher(Float64, '/ego_mpc/sigma_long_m', 10)
        self.sigma_lat_pub = self.create_publisher(Float64, '/ego_mpc/sigma_lat_m', 10)
        self.visibility_pub = self.create_publisher(
            Float64, '/ego_mpc/predicted_visibility', 10
        )
        self.planner_decision_pub = self.create_publisher(
            String, '/ego_mpc/planner_decision', 10
        )
        self.create_subscription(Odometry, ego_topic, self.ego_callback, 10)
        self.create_subscription(Odometry, target_topic, self.target_callback, 10)
        self.create_subscription(String, target_status_topic, self.target_status_callback, 10)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.control_loop)

        self.config = mpc_config()
        self.config.MAX_SPEED = max_speed
        self.config.MAX_ACCEL = max_accel
        self.config.MAX_STEER = max_steer
        self.config.MIN_STEER = -max_steer
        self.config.Rk[1, 1] = mpc_steer_input_cost
        self.config.Rdk[1, 1] = mpc_steer_rate_cost
        self.config.Qk[3, 3] = mpc_heading_cost
        self.config.Qfk[3, 3] = mpc_heading_cost
        self.config.Qk[2, 2] = mpc_speed_cost
        self.config.Qfk[2, 2] = mpc_speed_cost
        self.ego = None
        self.ego_stamp = None
        self.target = None
        self.target_stamp = None
        self.tracker_status_stamp = None
        self.tracker_mode = 'UNINITIALIZED'
        self.tracker_measurement_age = float('inf')
        self.tracker_position_covariance = float('inf')
        self.tracker_miss_reason = 'no_status'
        self.scan = None
        self.scan_stamp = None
        self.latest_astar_path = []
        self.filtered_target_yaw = None
        self.filtered_target_yaw_rate = 0.0
        self.target_heading_stamp = None
        self.current_reference_speed = 0.0
        self.current_dynamic_follow_distance = self.follow_distance
        self.current_prediction_horizon = 0.0
        self.current_safe_speed_cap = 0.0
        self.current_risk_score = 0.0
        self.current_raw_risk_score = 0.0
        self.current_risk_speed_scale = 1.0
        self.filtered_risk_score = 0.0
        self.current_sigma_long = 0.0
        self.current_sigma_lat = 0.0
        self.current_visibility = 1.0
        self.current_planner_decision = {}
        self.selected_candidate = 'center'
        self.committed_path = []
        self.committed_path_label = None
        self.active_blockage = None
        self.pending_blockage = None
        self.planning_hard_failure = False
        self.start_time = self.get_clock().now()
        self.oa = None
        self.odelta = None
        self.valid_cycle_count = 0
        self.planner_failure_count = 0
        self.using_straight_fallback = False
        self.last_stop_reason = None

        self.mpc_prob_init()
        self.get_logger().info(
            f'Following method={self.method}, corridor={self.use_corridor}, '
            f'visibility={self.use_visibility}, adaptive_risk={self.use_adaptive_risk}'
        )

    def ego_callback(self, msg: Odometry) -> None:
        self.ego = msg
        self.ego_stamp = self.get_clock().now()

    def target_callback(self, msg: Odometry) -> None:
        self.target = msg
        now = self.get_clock().now()
        self.target_stamp = now
        target_vx = float(msg.twist.twist.linear.x)
        target_vy = float(msg.twist.twist.linear.y)
        target_speed = math.hypot(target_vx, target_vy)
        if target_speed >= self.target_heading_min_speed:
            measured_yaw = math.atan2(target_vy, target_vx)
            if self.filtered_target_yaw is None:
                self.filtered_target_yaw = measured_yaw
                self.filtered_target_yaw_rate = 0.0
            else:
                previous_yaw = self.filtered_target_yaw
                yaw_delta = normalize_angle(measured_yaw - self.filtered_target_yaw)
                self.filtered_target_yaw = normalize_angle(
                    self.filtered_target_yaw + self.target_heading_alpha * yaw_delta
                )
                if self.target_heading_stamp is not None:
                    dt = (now - self.target_heading_stamp).nanoseconds * 1e-9
                    if dt > 1e-3:
                        measured_yaw_rate = normalize_angle(
                            self.filtered_target_yaw - previous_yaw
                        ) / dt
                        measured_yaw_rate = float(np.clip(
                            measured_yaw_rate,
                            -self.max_target_yaw_rate,
                            self.max_target_yaw_rate,
                        ))
                        self.filtered_target_yaw_rate = (
                            (1.0 - self.target_yaw_rate_alpha)
                            * self.filtered_target_yaw_rate
                            + self.target_yaw_rate_alpha * measured_yaw_rate
                        )
            self.target_heading_stamp = now
        else:
            self.filtered_target_yaw_rate *= 0.9

    def target_status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            mode = str(status.get('mode', 'UNINITIALIZED'))
            measurement_age = status.get('measurement_age_s')
            covariance = status.get('position_covariance_trace')
            self.tracker_mode = mode
            self.tracker_measurement_age = (
                float(measurement_age) if measurement_age is not None else float('inf')
            )
            self.tracker_position_covariance = (
                float(covariance) if covariance is not None else float('inf')
            )
            self.tracker_miss_reason = str(status.get('miss_reason', 'unknown'))
            self.tracker_status_stamp = self.get_clock().now()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.tracker_mode = 'INVALID'
            self.tracker_measurement_age = float('inf')
            self.tracker_miss_reason = 'invalid_status'
            self.tracker_status_stamp = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_stamp = self.get_clock().now()

    def fault_enabled(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def publish_safe_stop(
        self,
        reason: str,
        reset_validity: bool = True,
        reset_warm_start: bool = True,
    ) -> None:
        if reset_validity:
            self.valid_cycle_count = 0
        if reset_warm_start:
            self.oa = None
            self.odelta = None
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'ego_racecar/base_link'
        drive.drive.speed = 0.0
        drive.drive.steering_angle = 0.0
        drive.drive.acceleration = 0.0
        self.drive_pub.publish(drive)
        self.reference_speed_pub.publish(Float64(data=0.0))
        self.safe_speed_cap_pub.publish(Float64(data=0.0))
        self.control_mode_pub.publish(String(data=f'STOP:{reason}'))
        if reason != self.last_stop_reason:
            self.get_logger().warn(f'SAFETY STOP: {reason}')
            self.last_stop_reason = reason

    def publish_transient_deceleration(self, reason: str) -> None:
        self.valid_cycle_count = 0
        current_speed = math.hypot(
            self.ego.twist.twist.linear.x,
            self.ego.twist.twist.linear.y,
        )
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'ego_racecar/base_link'
        drive.drive.speed = max(0.0, current_speed - self.config.MAX_ACCEL * self.config.DTK)
        drive.drive.steering_angle = 0.0
        drive.drive.acceleration = -self.config.MAX_ACCEL
        self.drive_pub.publish(drive)
        if reason != self.last_stop_reason:
            self.get_logger().warn(f'SAFETY DECEL: {reason}')
            self.last_stop_reason = reason

    def input_age(self, stamp) -> float:
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def effective_measurement_age(self) -> float:
        status_age = self.input_age(self.tracker_status_stamp)
        if not math.isfinite(self.tracker_measurement_age) or not math.isfinite(status_age):
            return float('inf')
        return max(0.0, self.tracker_measurement_age + status_age)

    def sector_min_distance(self, angle_min_deg: float, angle_max_deg: float) -> float:
        if self.scan is None:
            return float('inf')

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)
        best = float('inf')

        for idx, dist in enumerate(self.scan.ranges):
            if not math.isfinite(dist):
                continue
            angle = self.scan.angle_min + idx * self.scan.angle_increment
            if angle_min <= angle <= angle_max:
                best = min(best, dist)

        return best

    def control_loop(self) -> None:
        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 < self.startup_delay:
            self.publish_safe_stop('startup delay')
            return

        if self.fault_enabled('fault_inject_scan_timeout') or self.input_age(self.scan_stamp) > self.scan_timeout:
            self.publish_safe_stop('scan timeout')
            return
        if self.fault_enabled('fault_inject_odom_timeout') or self.input_age(self.ego_stamp) > self.odom_timeout:
            self.publish_safe_stop('odom timeout')
            return
        if self.fault_enabled('fault_inject_target_timeout'):
            self.publish_safe_stop('target timeout')
            return
        if self.input_age(self.target_stamp) > self.target_state_timeout:
            self.publish_safe_stop('target state timeout')
            return
        if self.input_age(self.tracker_status_stamp) > self.tracker_status_timeout:
            self.publish_safe_stop('tracker status timeout')
            return
        measurement_age = self.effective_measurement_age()
        if self.tracker_mode in ('LOST', 'UNINITIALIZED', 'INVALID') or measurement_age > self.target_timeout:
            self.publish_safe_stop(
                f'target measurement timeout ({self.tracker_miss_reason})'
            )
            return

        ego_yaw = yaw_from_odom(self.ego)
        ego_speed = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)
        ego_state = State(
            x=self.ego.pose.pose.position.x,
            y=self.ego.pose.pose.position.y,
            v=max(ego_speed, 0.1),
            yaw=ego_yaw,
        )
        gap_distance = math.hypot(
            self.target.pose.pose.position.x - ego_state.x,
            self.target.pose.pose.position.y - ego_state.y,
        )
        front_dist = self.sector_min_distance(-12.0, 12.0)
        self.update_safety_margin(front_dist)
        x0 = [ego_state.x, ego_state.y, ego_state.v, ego_state.yaw]

        planning_start = time.perf_counter()
        ref_path, planner_valid = self.build_reference_trajectory(ego_state)
        planning_ms = (time.perf_counter() - planning_start) * 1000.0
        self.planning_time_pub.publish(Float64(data=float(planning_ms)))
        self.planner_valid_pub.publish(Bool(data=bool(planner_valid)))
        heading_step = min(3, self.config.TK)
        path_heading_error = normalize_angle(ref_path[3, heading_step] - ego_state.yaw)
        self.front_clearance_pub.publish(Float64(data=float(front_dist)))
        self.path_heading_error_pub.publish(Float64(data=float(path_heading_error)))
        self.reference_speed_pub.publish(
            Float64(data=float(self.current_reference_speed))
        )
        self.dynamic_follow_distance_pub.publish(
            Float64(data=float(self.current_dynamic_follow_distance))
        )
        self.prediction_horizon_pub.publish(
            Float64(data=float(self.current_prediction_horizon))
        )
        self.safe_speed_cap_pub.publish(
            Float64(data=float(self.current_safe_speed_cap))
        )
        self.risk_score_pub.publish(Float64(data=float(self.current_risk_score)))
        self.raw_risk_score_pub.publish(
            Float64(data=float(self.current_raw_risk_score))
        )
        self.risk_speed_scale_pub.publish(
            Float64(data=float(self.current_risk_speed_scale))
        )
        self.sigma_long_pub.publish(Float64(data=float(self.current_sigma_long)))
        self.sigma_lat_pub.publish(Float64(data=float(self.current_sigma_lat)))
        self.visibility_pub.publish(Float64(data=float(self.current_visibility)))
        self.planner_decision_pub.publish(String(data=json.dumps(
            self.current_planner_decision, sort_keys=True
        )))
        if self.planning_hard_failure:
            self.planner_failure_count += 1
            self.publish_safe_stop('no collision-checked planner path')
            return
        if not planner_valid:
            self.planner_failure_count += 1
            if not self.using_straight_fallback:
                self.get_logger().warn(
                    'PLANNER FALLBACK: using unchecked straight-line pursuit path'
                )
            self.using_straight_fallback = True
            self.control_mode_pub.publish(String(data='STRAIGHT_FALLBACK'))
        else:
            self.planner_failure_count = 0
            self.using_straight_fallback = False
            mode = 'COASTING' if self.tracker_mode == 'COASTING' else 'TRACKED'
            self.control_mode_pub.publish(String(data=mode))

        solve_start = time.perf_counter()
        (
            self.oa,
            self.odelta,
            ox,
            oy,
            _ov,
            _oyaw,
            _path_predict,
        ) = self.linear_mpc_control(ref_path, x0, self.oa, self.odelta)
        solve_ms = (time.perf_counter() - solve_start) * 1000.0
        self.solve_time_pub.publish(Float64(data=float(solve_ms)))

        if self.oa is None or self.odelta is None:
            self.publish_safe_stop('MPC solver failure')
            return

        accel_cmd = float(self.oa[0])
        steer_cmd = float(self.odelta[0])
        if not math.isfinite(accel_cmd) or not math.isfinite(steer_cmd):
            self.publish_safe_stop('invalid MPC control')
            return
        if abs(accel_cmd) > self.config.MAX_ACCEL + self.control_tolerance:
            self.publish_safe_stop('acceleration outside safety limits')
            return
        if (
            steer_cmd < self.config.MIN_STEER - self.control_tolerance
            or steer_cmd > self.config.MAX_STEER + self.control_tolerance
        ):
            self.publish_safe_stop('steering outside safety limits')
            return
        accel_cmd = float(np.clip(accel_cmd, -self.config.MAX_ACCEL, self.config.MAX_ACCEL))
        steer_cmd = float(np.clip(steer_cmd, self.config.MIN_STEER, self.config.MAX_STEER))

        # CBF-like longitudinal safety filter.  If the current speed is above
        # the admissible v_lead + gamma*(gap-d_safe), request maximum bounded
        # deceleration.  Otherwise prevent the next MPC step from accelerating
        # through that bound.  This remains subordinate to the final arbiter.
        if ego_state.v > self.current_safe_speed_cap:
            accel_cmd = -self.config.MAX_ACCEL
        else:
            accel_cmd = min(
                accel_cmd,
                (self.current_safe_speed_cap - ego_state.v) / self.config.DTK,
            )
            accel_cmd = float(np.clip(
                accel_cmd,
                -self.config.MAX_ACCEL,
                self.config.MAX_ACCEL,
            ))

        self.valid_cycle_count += 1
        if self.valid_cycle_count < self.recovery_valid_cycles:
            self.publish_safe_stop(
                f'waiting for {self.recovery_valid_cycles} consecutive valid cycles',
                reset_validity=False,
                reset_warm_start=False,
            )
            return

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'ego_racecar/base_link'
        drive.drive.steering_angle = steer_cmd
        commanded_speed = np.clip(
            ego_state.v + accel_cmd * self.config.DTK,
            self.config.MIN_SPEED,
            self.config.MAX_SPEED,
        )
        drive.drive.speed = float(commanded_speed)
        self.drive_pub.publish(drive)
        self.last_stop_reason = None

        debug_drive = AckermannDriveStamped()
        debug_drive.header = drive.header
        debug_drive.drive.steering_angle = steer_cmd
        debug_drive.drive.acceleration = accel_cmd
        debug_drive.drive.speed = drive.drive.speed
        self.debug_drive_pub.publish(debug_drive)

        self.visualize_path(ref_path, self.ref_path_pub, 0.9, 0.1, 0.9, 'ego_ref')
        self.visualize_xy_path(ox, oy, self.pred_path_pub, 0.1, 0.9, 0.1, 'ego_pred')
        self.visualize_point_path(self.latest_astar_path, self.astar_path_pub, 0.0, 0.45, 1.0, 'ego_astar')

    def build_reference_trajectory(self, ego_state: State):
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ex = self.ego.pose.pose.position.x
        ey = self.ego.pose.pose.position.y
        tx = self.target.pose.pose.position.x
        ty = self.target.pose.pose.position.y
        tvx = float(self.target.twist.twist.linear.x)
        tvy = float(self.target.twist.twist.linear.y)
        target_speed = math.hypot(tvx, tvy)
        gap_distance = math.hypot(tx - ex, ty - ey)

        line_of_sight_yaw = math.atan2(ty - ey, tx - ex)
        target_yaw = line_of_sight_yaw
        if self.filtered_target_yaw is not None:
            speed_weight = np.clip(
                (target_speed - self.target_heading_min_speed)
                / (self.target_heading_full_speed - self.target_heading_min_speed),
                0.0,
                1.0,
            )
            measurement_age = self.effective_measurement_age()
            age_weight = np.clip(
                1.0 - measurement_age / max(self.target_timeout, 1e-3),
                0.0,
                1.0,
            )
            heading_weight = float(speed_weight * age_weight)
            target_yaw = normalize_angle(
                line_of_sight_yaw
                + heading_weight
                * normalize_angle(self.filtered_target_yaw - line_of_sight_yaw)
            )

        sigma_long, sigma_lat = self.target_position_sigmas(target_yaw)
        measurement_age = self.effective_measurement_age()
        dynamic_follow_distance = float(np.clip(
            self.follow_distance
            + self.time_headway * ego_state.v
            + (
                self.uncertainty_distance_gain * sigma_long
                + self.measurement_age_distance_gain * measurement_age
                if self.use_adaptive_risk else 0.0
            ),
            self.follow_distance,
            self.maximum_follow_distance,
        ))

        if gap_distance <= self.minimum_motion_gap:
            desired_speed = 0.0
        else:
            gap_correction = float(np.clip(
                self.gap_speed_gain * (gap_distance - dynamic_follow_distance),
                -target_speed,
                self.max_catchup_speed_bonus,
            ))
            desired_speed = float(np.clip(
                target_speed + gap_correction,
                0.0,
                self.config.MAX_SPEED,
            ))

        heading_demand = abs(normalize_angle(target_yaw - ego_state.yaw))
        turn_speed_scale = max(
            self.turn_min_speed_scale,
            1.0 - self.turn_slowdown_gain * heading_demand,
        )
        desired_speed *= turn_speed_scale

        if self.tracker_mode == 'COASTING':
            measurement_age = self.effective_measurement_age()
            coast_scale = max(
                self.coasting_min_speed_scale,
                1.0 - measurement_age / max(self.target_timeout, 1e-3),
            )
            desired_speed *= coast_scale

        prediction_horizon = self.target_prediction_horizon
        predicted_tx, predicted_ty, predicted_target_yaw = self.predict_target_pose(
            tx,
            ty,
            target_speed,
            target_yaw,
            self.filtered_target_yaw_rate,
            prediction_horizon,
        )
        path, planner_valid, decision = self.select_candidate_path(
            ego_state=ego_state,
            predicted_target=(predicted_tx, predicted_ty, predicted_target_yaw),
            target_state=(tx, ty, target_speed, target_yaw),
            dynamic_follow_distance=dynamic_follow_distance,
            desired_speed=desired_speed,
        )
        selected_visibility = float(decision.get('selected_visibility', 1.0))
        normalized_sigma = float(np.clip(
            sigma_long / self.uncertainty_sigma_reference, 0.0, 1.0
        ))
        normalized_age = float(np.clip(
            measurement_age / max(self.target_timeout, 1e-3), 0.0, 1.0
        ))
        raw_risk_score = 0.0
        risk_score = 0.0
        risk_speed_scale = 1.0
        if self.use_adaptive_risk:
            raw_risk_score = float(np.clip(
                self.risk_weights[0] * normalized_sigma
                + self.risk_weights[1] * normalized_age
                + self.risk_weights[2] * (1.0 - selected_visibility),
                0.0,
                1.0,
            ))
            # Risk increases take effect immediately; release is filtered so
            # scan quantization cannot repeatedly accelerate/decelerate the
            # MPC reference after an occlusion clears.
            if raw_risk_score >= self.filtered_risk_score:
                self.filtered_risk_score = raw_risk_score
            else:
                self.filtered_risk_score = (
                    (1.0 - self.risk_release_alpha) * self.filtered_risk_score
                    + self.risk_release_alpha * raw_risk_score
                )
            risk_score = float(np.clip(self.filtered_risk_score, 0.0, 1.0))
            risk_speed_scale = max(
                self.minimum_risk_speed_scale,
                1.0 - self.risk_speed_gain * risk_score,
            )
            # Sigma and measurement age already enlarge the preliminary anchor
            # distance above.  In full mode, visibility adds the missing third
            # risk component after candidate evaluation.  The final safety cap
            # below uses this complete adaptive distance.
            dynamic_follow_distance = float(np.clip(
                dynamic_follow_distance
                + self.visibility_distance_gain * (1.0 - selected_visibility),
                self.follow_distance,
                self.maximum_follow_distance,
            ))
            # Risk limits positive closing speed, not the lead vehicle's base
            # speed.  Scaling the absolute command makes the ego permanently
            # slower than a moving lead, grows the gap until the lead exits
            # LiDAR range, and is therefore counterproductive for visibility.
            if desired_speed > target_speed:
                desired_speed = (
                    target_speed
                    + risk_speed_scale * (desired_speed - target_speed)
                )

        safe_follow_distance = max(self.minimum_safe_distance, dynamic_follow_distance)
        safe_speed_cap = float(np.clip(
            target_speed + self.safe_gap_gamma * (gap_distance - safe_follow_distance),
            0.0,
            self.config.MAX_SPEED,
        ))
        if self.use_adaptive_risk:
            if safe_speed_cap > target_speed:
                safe_speed_cap = (
                    target_speed
                    + risk_speed_scale * (safe_speed_cap - target_speed)
                )

        self.planning_hard_failure = False
        if not planner_valid or len(path) < 2:
            if self.require_collision_checked_path:
                self.planning_hard_failure = True
                desired_speed = 0.0
                safe_speed_cap = 0.0
                path = [
                    (ex, ey),
                    (ex + 0.01 * math.cos(ego_state.yaw),
                     ey + 0.01 * math.sin(ego_state.yaw)),
                ]
            else:
                # Retain the historical nominal fallback only for the baseline.
                fallback_distance = max(gap_distance - dynamic_follow_distance, 0.0)
                fallback_goal_x = ex + fallback_distance * math.cos(line_of_sight_yaw)
                fallback_goal_y = ey + fallback_distance * math.sin(line_of_sight_yaw)
                path = [(ex, ey), (fallback_goal_x, fallback_goal_y)]
        self.latest_astar_path = path
        self.current_reference_speed = desired_speed
        self.current_dynamic_follow_distance = dynamic_follow_distance
        self.current_prediction_horizon = prediction_horizon
        self.current_safe_speed_cap = safe_speed_cap
        self.current_risk_score = risk_score
        self.current_raw_risk_score = raw_risk_score
        self.current_risk_speed_scale = risk_speed_scale
        self.current_sigma_long = sigma_long
        self.current_sigma_lat = sigma_lat
        self.current_visibility = selected_visibility
        decision.update({
            'method': self.method,
            'measurement_age_s': measurement_age,
            'sigma_long_m': sigma_long,
            'sigma_lat_m': sigma_lat,
            'risk_score': risk_score,
            'raw_risk_score': raw_risk_score,
            'risk_speed_scale': risk_speed_scale,
            'hard_failure': self.planning_hard_failure,
        })
        self.current_planner_decision = decision

        continuous_reference_yaw = ego_state.yaw
        for i in range(self.config.TK + 1):
            progress = min(i * desired_speed * self.config.DTK, self.path_length(path))
            rx, ry, ryaw = self.sample_path(path, progress, target_yaw)
            # Keep the MPC yaw reference continuous across the +/-pi boundary.
            # A raw atan2 jump from +pi to -pi is physically a tiny change, but
            # the quadratic objective otherwise interprets it as a 2*pi error
            # and commands steering in the wrong direction at turn exits.
            continuous_reference_yaw += normalize_angle(
                ryaw - continuous_reference_yaw
            )
            ref_traj[0, i] = rx
            ref_traj[1, i] = ry
            ref_traj[2, i] = desired_speed
            ref_traj[3, i] = continuous_reference_yaw

        return ref_traj, planner_valid

    def target_covariance_matrices(self):
        pose_cov = np.asarray(self.target.pose.covariance, dtype=float).reshape(6, 6)
        twist_cov = np.asarray(self.target.twist.covariance, dtype=float).reshape(6, 6)
        position_covariance = pose_cov[np.ix_([0, 1], [0, 1])]
        velocity_covariance = twist_cov[np.ix_([0, 1], [0, 1])]
        position_covariance = self.sanitize_covariance(position_covariance)
        velocity_covariance = self.sanitize_covariance(velocity_covariance)
        return position_covariance, velocity_covariance

    @staticmethod
    def sanitize_covariance(covariance):
        covariance = np.nan_to_num(
            np.asarray(covariance, dtype=float), nan=0.0, posinf=4.0, neginf=0.0
        )
        covariance = 0.5 * (covariance + covariance.T)
        values, vectors = np.linalg.eigh(covariance)
        values = np.clip(values, 1e-6, 4.0)
        return vectors @ np.diag(values) @ vectors.T

    def target_position_sigmas(self, target_yaw):
        position_covariance, _ = self.target_covariance_matrices()
        tangent = np.array([math.cos(target_yaw), math.sin(target_yaw)])
        normal = np.array([-math.sin(target_yaw), math.cos(target_yaw)])
        sigma_long = math.sqrt(max(float(tangent @ position_covariance @ tangent), 0.0))
        sigma_lat = math.sqrt(max(float(normal @ position_covariance @ normal), 0.0))
        return sigma_long, sigma_lat

    def select_candidate_path(
        self,
        ego_state,
        predicted_target,
        target_state,
        dynamic_follow_distance,
        desired_speed,
    ):
        context = self.build_planning_context(ego_state)
        if context is None:
            return [], False, {
                'selected_candidate': 'none',
                'selected_visibility': 0.0,
                'valid_candidate_count': 0,
                'candidates': [],
            }

        blockage = (
            self.find_frontal_blockage(ego_state, context)
            if self.use_corridor else None
        )
        # Outside a confirmed blockage, corridor must be identical to the
        # nominal centre-path controller.  Evaluating arbitrary lateral goals
        # here caused the ablation to weave before an obstacle was detected.
        labels_and_offsets = [('center', 0.0)]
        # Once a compact obstacle blocks the following corridor, spend the
        # bounded search budget on the two meaningful passing choices.  A
        # direct center path is not an admissible candidate until the obstacle
        # has passed behind the ego vehicle.
        if blockage is not None:
            labels_and_offsets = [
                ('left', blockage.get('left_offset', self.corridor_lateral_offset)),
                ('right', blockage.get('right_offset', -self.corridor_lateral_offset)),
            ]
            committed_side = blockage.get('chosen_side')
            if committed_side in {'left', 'right'}:
                # Never cross the obstacle corridor after committing a side.
                # Such a switch can be collision-free yet destroy target
                # visibility and creates a path discontinuity for the MPC.
                labels_and_offsets = [
                    item for item in labels_and_offsets
                    if item[0] == committed_side
                ]
            else:
                # The corridor builder has already collision-checked a family
                # of offsets on each side.  If at least one side has a valid
                # seed, do not spend a full Hybrid-A* budget trying a side
                # known to be closed.  That extra search previously delayed a
                # visibility decision beyond the final-command watchdog.
                feasible_seed_sides = [
                    item for item in labels_and_offsets
                    if blockage.get(f'{item[0]}_valid_option_count', 0) > 0
                ]
                if feasible_seed_sides:
                    labels_and_offsets = feasible_seed_sides
                labels_and_offsets.sort(
                    key=lambda item: blockage.get(
                        f'{item[0]}_valid_option_count', 0
                    ),
                    reverse=True,
                )

        px, py, pyaw = predicted_target
        normal_x = -math.sin(pyaw)
        normal_y = math.cos(pyaw)
        per_candidate_budget = (
            self.hybrid_astar_time_budget_ms
            if self.method == 'nominal'
            else self.candidate_planning_budget_ms / max(len(labels_and_offsets), 1)
        )
        evaluate_candidate_visibility = self.use_visibility and (
            self.use_adaptive_risk or len(labels_and_offsets) > 1
        )
        candidates = []
        for label, lateral_offset in labels_and_offsets:
            goal_x = (
                px - dynamic_follow_distance * math.cos(pyaw)
                + lateral_offset * normal_x
            )
            goal_y = (
                py - dynamic_follow_distance * math.sin(pyaw)
                + lateral_offset * normal_y
            )
            guided_by_obstacle = blockage is not None and label != 'center'
            if guided_by_obstacle:
                # A lateral terminal offset alone does not describe a pass: if
                # the moving rear anchor is still abreast of the obstacle, all
                # three goals can remain on its near side.  Put side-corridor
                # goals just beyond the compact LiDAR blockage so Hybrid A*
                # must produce a complete, collision-checked pass.
                gateway = blockage[f'{label}_gateway']
                goal_x, goal_y = gateway
                lateral_offset = blockage.get(
                    f'{label}_offset', lateral_offset
                )
            path = []
            valid = False
            planner_source = 'none'
            if guided_by_obstacle:
                if (
                    blockage.get('chosen_side') == label
                    and self.committed_path_label == label
                ):
                    path = self.reuse_collision_checked_path(
                        ego_state, context
                    )
                    if path:
                        planner_source = 'committed_path'
                if not path:
                    path = self.collision_checked_corridor_path(
                        ego_state, blockage, label, context
                    )
                valid = len(path) >= 2
                if valid and planner_source == 'none':
                    planner_source = 'lidar_corridor'
            if not valid:
                path, valid = self.plan_astar_path(
                    ego_state,
                    goal_x,
                    goal_y,
                    blockage['yaw'] if guided_by_obstacle else pyaw,
                    context=context,
                    time_budget_ms=per_candidate_budget,
                )
                if valid:
                    planner_source = 'hybrid_astar'
            if not valid or len(path) < 2:
                candidates.append({
                    'label': label,
                    'lateral_offset_m': lateral_offset,
                    'guided_by_obstacle': guided_by_obstacle,
                    'planner_source': planner_source,
                    'corridor_valid_option_count': (
                        blockage.get(f'{label}_valid_option_count', 0)
                        if blockage is not None else None
                    ),
                    'corridor_seed_collision': (
                        blockage.get(f'{label}_collision')
                        if blockage is not None else None
                    ),
                    'corridor_current_collision': (
                        blockage.get(f'{label}_current_collision')
                        if blockage is not None else None
                    ),
                    'target_cluster_match_error_m': context.get(
                        'target_cluster_match_error_m'
                    ),
                    'target_cluster_point_count': context.get(
                        'target_cluster_point_count', 0
                    ),
                    'valid': False,
                })
                continue
            if evaluate_candidate_visibility:
                visibility, visibility_profile = self.predict_path_visibility(
                    path, ego_state, target_state, desired_speed, context
                )
            else:
                visibility = 1.0
                visibility_profile = []
            components = self.candidate_cost_components(
                path, lateral_offset, visibility, context
            )
            score = sum(
                self.candidate_weights[name] * value
                for name, value in components.items()
            )
            candidates.append({
                'label': label,
                'lateral_offset_m': lateral_offset,
                'guided_by_obstacle': guided_by_obstacle,
                'planner_source': planner_source,
                'corridor_valid_option_count': (
                    blockage.get(f'{label}_valid_option_count', 0)
                    if blockage is not None else None
                ),
                'target_cluster_match_error_m': context.get(
                    'target_cluster_match_error_m'
                ),
                'target_cluster_point_count': context.get(
                    'target_cluster_point_count', 0
                ),
                'goal_x': float(goal_x),
                'goal_y': float(goal_y),
                'valid': True,
                'visibility': visibility,
                'visibility_profile': (
                    visibility_profile if len(labels_and_offsets) > 1 else []
                ),
                'score': float(score),
                'components': components,
                'heading_error_1m_rad': float(normalize_angle(
                    self.sample_path(path, 1.0, ego_state.yaw)[2]
                    - ego_state.yaw
                )),
                'path': path,
            })
            # Corridor-only ablation takes the first feasible passing side and
            # commits it.  This prevents a second bounded search from starving
            # the command stream during initial activation.
            if (
                self.method == 'corridor'
                and blockage is not None
            ):
                break

        valid_candidates = [candidate for candidate in candidates if candidate['valid']]
        if not valid_candidates:
            public_candidates = [dict(candidate) for candidate in candidates]
            attempted_labels = {candidate['label'] for candidate in candidates}
            reuse_allowed = self.committed_path_label in attempted_labels
            reused_path = (
                self.reuse_collision_checked_path(ego_state, context)
                if reuse_allowed else []
            )
            if reused_path:
                return reused_path, True, {
                    'selected_candidate': self.selected_candidate,
                    'selected_visibility': 0.0,
                    'valid_candidate_count': 0,
                    'reused_previous_path': True,
                    'candidates': public_candidates,
                }
            refresh_requested = False
            if (
                blockage is not None
                and self.committed_path_label in attempted_labels
                and blockage.get('chosen_side') == self.committed_path_label
            ):
                # The vehicle can deviate slightly from a frozen corridor as
                # it passes the obstacle.  If both the still-checked suffix
                # and a same-side Hybrid-A* recovery fail, request one
                # geometry refresh. Side commitment is preserved: this repairs
                # an obsolete gateway without ever crossing the obstacle
                # corridor or accepting an unchecked path.
                blockage['refresh_requested'] = True
                refresh_requested = True
                refreshed_blockage = self.find_frontal_blockage(
                    ego_state, context
                )
                if (
                    refreshed_blockage is not None
                    and refreshed_blockage is not blockage
                    and refreshed_blockage.get('chosen_side')
                    == self.committed_path_label
                ):
                    refreshed_path = self.collision_checked_corridor_path(
                        ego_state,
                        refreshed_blockage,
                        self.committed_path_label,
                        context,
                    )
                    if len(refreshed_path) >= 2:
                        refreshed_visibility = 1.0
                        if self.use_visibility:
                            refreshed_visibility, _ = self.predict_path_visibility(
                                refreshed_path,
                                ego_state,
                                target_state,
                                desired_speed,
                                context,
                            )
                        self.committed_path = list(refreshed_path)
                        public_candidates.append({
                            'label': self.committed_path_label,
                            'planner_source': 'same_cycle_corridor_refresh',
                            'visibility': refreshed_visibility,
                            'valid': True,
                        })
                        return refreshed_path, True, {
                            'selected_candidate': self.committed_path_label,
                            'selected_visibility': refreshed_visibility,
                            'valid_candidate_count': 1,
                            'corridor_refresh_requested': True,
                            'corridor_refresh_succeeded': True,
                            'candidates': public_candidates,
                        }
            return [], False, {
                'selected_candidate': 'none',
                'selected_visibility': 0.0,
                'valid_candidate_count': 0,
                'corridor_refresh_requested': refresh_requested,
                'corridor_refresh_succeeded': False,
                'candidates': public_candidates,
            }

        best = min(valid_candidates, key=lambda candidate: candidate['score'])
        geometric_preference = valid_candidates[0]
        visibility_advantage_over_geometric = 0.0
        visibility_margin_cleared = False
        if self.use_visibility and len(valid_candidates) >= 2:
            maximum_visibility = max(
                candidate['visibility'] for candidate in valid_candidates
            )
            visibility_advantage_over_geometric = (
                maximum_visibility - geometric_preference['visibility']
            )
            visibility_margin_cleared = (
                visibility_advantage_over_geometric
                + 1e-9
                >= self.candidate_visibility_margin
            )
            # A single sigma-point/ray change is too sensitive to scan-grid
            # quantization to justify changing sides. Keep the corridor
            # preference unless another candidate clears an explicit
            # visibility advantage margin.
            if not visibility_margin_cleared:
                best = geometric_preference
        previous = next(
            (candidate for candidate in valid_candidates
             if candidate['label'] == self.selected_candidate),
            None,
        )
        hysteresis_margin = (
            self.candidate_switch_margin
            if self.selected_candidate == 'center'
            else self.candidate_rejoin_margin
        )
        if (
            previous is not None
            and best['label'] != previous['label']
            and previous['score'] <= best['score'] + hysteresis_margin
        ):
            best = previous
        switched = best['label'] != self.selected_candidate
        self.selected_candidate = best['label']
        self.committed_path = list(best['path'])
        self.committed_path_label = best['label']
        if blockage is not None:
            blockage['chosen_side'] = best['label']

        public_candidates = []
        for candidate in candidates:
            public_candidate = {
                key: value for key, value in candidate.items() if key != 'path'
            }
            public_candidates.append(public_candidate)
        return best['path'], True, {
            'selected_candidate': best['label'],
            'selected_visibility': float(best['visibility']),
            'valid_candidate_count': len(valid_candidates),
            'switched': switched,
            'geometric_preference': geometric_preference['label'],
            'visibility_margin': self.candidate_visibility_margin,
            'visibility_advantage_over_geometric': float(
                visibility_advantage_over_geometric
            ),
            'visibility_margin_cleared': visibility_margin_cleared,
            'visibility_changed_selection': (
                self.use_visibility
                and best['label'] != geometric_preference['label']
            ),
            'candidates': public_candidates,
        }

    def collision_checked_corridor_path(self, ego_state, blockage, label, context):
        """Trim and validate a smooth side-corridor seed from current pose."""
        fixed_path = blockage.get(f'{label}_path', [])
        if len(fixed_path) < 2:
            return []
        suffix = self.trim_forward_path(ego_state, fixed_path)
        if len(suffix) < 2:
            return []
        collision = self.first_path_collision(
            suffix,
            context['occupancy'],
            context['world_to_grid'],
            context['resolution'],
        )
        blockage[f'{label}_current_collision'] = collision
        if collision is not None:
            return []
        return suffix

    def find_frontal_blockage(self, ego_state, context):
        """Return left/right gateways for a compact obstacle in the lane.

        The points are derived from the same target-filtered LiDAR data used by
        Hybrid A*.  No map or opponent ground truth enters this decision.  A
        narrow forward band excludes the two long track boundaries on the
        standardized courses while retaining obstacles that block the direct
        following corridor.
        """
        previous_blockage = self.active_blockage
        if previous_blockage is not None:
            center_x, center_y = previous_blockage['center']
            dx = center_x - ego_state.x
            dy = center_y - ego_state.y
            longitudinal = (
                math.cos(ego_state.yaw) * dx + math.sin(ego_state.yaw) * dy
            )
            if longitudinal < -0.6:
                self.active_blockage = None
                self.pending_blockage = None
                previous_blockage = None
            elif (
                previous_blockage.get('chosen_side') in {'left', 'right'}
                and not previous_blockage.get('refresh_requested', False)
            ):
                # Freeze the LiDAR-derived corridor after side commitment.
                # Re-fitting it from a changing viewpoint can replace the
                # reference midway through the pass even though side-lock is
                # active.  A failed collision check may explicitly request a
                # same-side refresh on the next control cycle.
                return previous_blockage
            else:
                now_seconds = self.get_clock().now().nanoseconds * 1e-9
                refresh_seconds = previous_blockage.get(
                    'geometry_refresh_time_s', -float('inf')
                )
                if (
                    now_seconds - refresh_seconds
                    < self.corridor_geometry_refresh_period
                ):
                    return previous_blockage

        points = context.get('static_points', [])
        if len(points) < 3:
            return None

        cos_yaw = math.cos(ego_state.yaw)
        sin_yaw = math.sin(ego_state.yaw)
        target_longitudinal = 0.0
        if self.target is not None:
            target_dx = self.target.pose.pose.position.x - ego_state.x
            target_dy = self.target.pose.pose.position.y - ego_state.y
            target_longitudinal = cos_yaw * target_dx + sin_yaw * target_dy
        local_points = []
        for beam_index, world_x, world_y in points:
            dx = world_x - ego_state.x
            dy = world_y - ego_state.y
            longitudinal = cos_yaw * dx + sin_yaw * dy
            lateral = -sin_yaw * dx + cos_yaw * dy
            if (
                0.25 <= longitudinal <= 5.0
                and abs(lateral) <= 2.0
            ):
                local_points.append((beam_index, longitudinal, lateral))
        if len(local_points) < 3:
            return None

        beam_clusters = []
        current_cluster = []
        for point in local_points:
            if current_cluster:
                previous = current_cluster[-1]
                spatial_gap = math.hypot(
                    point[1] - previous[1], point[2] - previous[2]
                )
                if point[0] - previous[0] > 2 or spatial_gap > 0.3:
                    beam_clusters.append(current_cluster)
                    current_cluster = []
            current_cluster.append(point)
        if current_cluster:
            beam_clusters.append(current_cluster)

        is_new_blockage = previous_blockage is None
        if previous_blockage is not None:
            center_x, center_y = previous_blockage['center']
            dx = center_x - ego_state.x
            dy = center_y - ego_state.y
            obstacle_longitudinal = cos_yaw * dx + sin_yaw * dy
            obstacle_lateral = -sin_yaw * dx + cos_yaw * dy
            # Exclude the return cluster nearest the remembered obstacle while
            # refreshing road geometry from each LiDAR frame.
            nearby_clusters = []
            for candidate in beam_clusters:
                candidate_longitudinal = float(np.median([
                    point[1] for point in candidate
                ]))
                candidate_lateral = float(np.median([
                    point[2] for point in candidate
                ]))
                distance = math.hypot(
                    candidate_longitudinal - obstacle_longitudinal,
                    candidate_lateral - obstacle_lateral,
                )
                if distance <= 0.6:
                    nearby_clusters.append((distance, candidate))
            cluster = min(nearby_clusters, default=(0.0, None))[1]
            centerline, geometry_source = self.fit_lidar_corridor_centerline(
                beam_clusters, cluster, max(0.35, obstacle_longitudinal + 0.9)
            )
            corridor_residual = obstacle_lateral - centerline(obstacle_longitudinal)
            center = previous_blockage['center']
            chosen_side = previous_blockage.get('chosen_side')
        else:
            compact_clusters = []
            for cluster in beam_clusters:
                if len(cluster) < 3:
                    continue
                longitudinals = [point[1] for point in cluster]
                laterals = [point[2] for point in cluster]
                if min(longitudinals) < max(0.7, target_longitudinal + 0.6):
                    continue
                if (
                    min(longitudinals) <= self.corridor_blockage_max_range
                    and max(longitudinals) - min(longitudinals) <= 0.40
                    and max(laterals) - min(laterals) <= 0.80
                ):
                    candidate_longitudinal = float(np.median(longitudinals))
                    candidate_centerline, geometry_source = (
                        self.fit_lidar_corridor_centerline(
                            beam_clusters,
                            cluster,
                            candidate_longitudinal + 0.9,
                        )
                    )
                    corridor_residual = (
                        float(np.median(laterals))
                        - candidate_centerline(candidate_longitudinal)
                    )
                    # Test compact objects against the curved LiDAR corridor,
                    # not against the ego x-axis.
                    if abs(corridor_residual) <= 0.35:
                        compact_clusters.append((
                            cluster,
                            candidate_centerline,
                            geometry_source,
                            corridor_residual,
                        ))
            if not compact_clusters:
                self.pending_blockage = None
                return None

            cluster, centerline, geometry_source, corridor_residual = min(
                compact_clusters,
                key=lambda candidate: float(np.median([
                    point[1] for point in candidate[0]
                ])),
            )
            obstacle_longitudinal = float(np.median([
                point[1] for point in cluster
            ]))
            obstacle_lateral = float(np.median([
                point[2] for point in cluster
            ]))
            center = (
                ego_state.x + obstacle_longitudinal * cos_yaw - obstacle_lateral * sin_yaw,
                ego_state.y + obstacle_longitudinal * sin_yaw + obstacle_lateral * cos_yaw,
            )
            if (
                self.pending_blockage is not None
                and math.hypot(
                    center[0] - self.pending_blockage['center'][0],
                    center[1] - self.pending_blockage['center'][1],
                ) <= self.corridor_blockage_match_radius
            ):
                confirmation_count = self.pending_blockage['count'] + 1
                center = (
                    0.5 * (center[0] + self.pending_blockage['center'][0]),
                    0.5 * (center[1] + self.pending_blockage['center'][1]),
                )
            else:
                confirmation_count = 1
            self.pending_blockage = {
                'center': center,
                'count': confirmation_count,
            }
            if confirmation_count < self.corridor_blockage_confirmation_scans:
                return None
            self.pending_blockage = None
            chosen_side = None

        gateway_longitudinal = obstacle_longitudinal + 0.9

        origin_x = ego_state.x
        origin_y = ego_state.y

        def local_to_world(longitudinal, lateral):
            return (
                origin_x + longitudinal * cos_yaw - lateral * sin_yaw,
                origin_y + longitudinal * sin_yaw + lateral * cos_yaw,
            )

        centerline_origin = float(np.clip(centerline(0.0), -0.8, 0.8))

        def centerline_lateral(longitudinal):
            raw_lateral = centerline(longitudinal) - centerline_origin
            ratio = float(np.clip(
                longitudinal / self.corridor_curve_blend_distance, 0.0, 1.0
            ))
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            return raw_lateral * smooth_ratio

        def centerline_tangent(longitudinal):
            step = 0.03
            before = max(0.0, longitudinal - step)
            after = min(gateway_longitudinal, longitudinal + step)
            if after - before <= 1e-6:
                return 0.0
            return (
                centerline_lateral(after) - centerline_lateral(before)
            ) / (after - before)

        def curve_point(longitudinal, side_lateral):
            base_lateral = centerline_lateral(longitudinal)
            slope = centerline_tangent(longitudinal)
            normal_scale = math.hypot(1.0, slope)
            offset_x = -side_lateral * slope / normal_scale
            offset_y = side_lateral / normal_scale
            return local_to_world(
                longitudinal + offset_x,
                base_lateral + offset_y,
            )

        def build_side_path(side_lateral):
            # Complete the lane change before the inflated obstacle face, then
            # follow a LiDAR-derived curved corridor through the obstacle. The
            # 1.8 m cap keeps the turn visible inside the one-second MPC
            # horizon; the lower bound keeps the smoothstep curvature within
            # the F1TENTH steering envelope for the configured offsets.
            shift_end = max(
                1.0, min(1.8, obstacle_longitudinal - 1.0)
            )
            sample_count = max(20, int(math.ceil(gateway_longitudinal / 0.08)))
            path = []
            for longitudinal in np.linspace(0.0, gateway_longitudinal, sample_count):
                ratio = float(np.clip(longitudinal / shift_end, 0.0, 1.0))
                smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
                # centerline_lateral is expressed relative to the ego pose so
                # it starts continuously at zero.  Restore centerline_origin
                # as the lane-change completes so the terminal corridor stays
                # offset from the actual road center, not the current ego
                # lateral position.
                path.append(curve_point(
                    float(longitudinal),
                    (centerline_origin + side_lateral) * smooth_ratio,
                ))
            return path

        gateway_slope = centerline_tangent(gateway_longitudinal)
        gateway_yaw = normalize_angle(ego_state.yaw + math.atan(gateway_slope))

        offset_magnitudes = list(np.arange(
            self.corridor_min_lateral_offset,
            self.corridor_max_lateral_offset + 0.5 * self.corridor_lateral_offset_step,
            self.corridor_lateral_offset_step,
        ))
        offset_magnitudes.append(self.corridor_lateral_offset)
        offset_magnitudes = sorted({
            round(float(value), 4) for value in offset_magnitudes
        })

        def path_clearance(path):
            minimum = float('inf')
            for world_x, world_y in path:
                grid_x, grid_y = context['world_to_grid'](world_x, world_y)
                if not (
                    0 <= grid_x < context['clearance'].shape[0]
                    and 0 <= grid_y < context['clearance'].shape[1]
                ):
                    return -float('inf')
                minimum = min(
                    minimum, float(context['clearance'][grid_x, grid_y])
                )
            return minimum

        def select_side_option(side_sign, side_name):
            previous_offset = None
            if previous_blockage is not None:
                previous_offset = previous_blockage.get(f'{side_name}_offset')
            options = []
            for magnitude in offset_magnitudes:
                lateral = side_sign * magnitude
                path = build_side_path(lateral)
                gateway = curve_point(
                    gateway_longitudinal, centerline_origin + lateral
                )
                collision = self.first_path_collision(
                    path, context['occupancy'], context['world_to_grid'],
                    context['resolution']
                )
                valid = collision is None
                options.append({
                    'offset': lateral,
                    'path': path,
                    'gateway': gateway,
                    'valid': valid,
                    'collision': collision,
                    'clearance': path_clearance(path) if valid else -1.0,
                })

            feasible = [option for option in options if option['valid']]
            if feasible:
                def option_score(option):
                    preference_penalty = 0.0
                    if previous_offset is not None:
                        preference_penalty = 0.20 * abs(
                            option['offset'] - previous_offset
                        )
                    return (
                        option['clearance']
                        - 0.15 * abs(
                            abs(option['offset']) - self.corridor_lateral_offset
                        )
                        - preference_penalty
                    )
                selected = max(feasible, key=option_score)
            else:
                # Hybrid A* can still find a non-seed path. Give it the least
                # displaced gateway rather than silently accepting a collision.
                selected = min(
                    options,
                    key=lambda option: abs(
                        abs(option['offset']) - self.corridor_lateral_offset
                    ),
                )
            selected['option_count'] = len(options)
            selected['valid_option_count'] = len(feasible)
            return selected

        left_option = select_side_option(1.0, 'left')
        right_option = select_side_option(-1.0, 'right')
        self.active_blockage = {
            'range_m': obstacle_longitudinal,
            'center': center,
            'yaw': gateway_yaw,
            'geometry_source': geometry_source,
            'corridor_residual_m': corridor_residual,
            'chosen_side': chosen_side,
            'left_gateway': left_option['gateway'],
            'right_gateway': right_option['gateway'],
            'left_path': left_option['path'],
            'right_path': right_option['path'],
            'left_offset': left_option['offset'],
            'right_offset': right_option['offset'],
            'left_valid_option_count': left_option['valid_option_count'],
            'right_valid_option_count': right_option['valid_option_count'],
            'left_collision': left_option['collision'],
            'right_collision': right_option['collision'],
            'refresh_requested': False,
            'geometry_refresh_time_s': (
                self.get_clock().now().nanoseconds * 1e-9
            ),
        }
        if is_new_blockage:
            self.get_logger().info(
                'Activated LiDAR obstacle corridors: range=%.2f m, geometry=%s, '
                'residual=%.2f m, gateway_yaw_delta=%.2f rad, '
                'offsets=(%.2f,%.2f), valid_offsets=(%d,%d), '
                'left=(%.2f, %.2f), right=(%.2f, %.2f)'
                % (
                    obstacle_longitudinal,
                    geometry_source,
                    corridor_residual,
                    normalize_angle(gateway_yaw - ego_state.yaw),
                    left_option['offset'],
                    right_option['offset'],
                    left_option['valid_option_count'],
                    right_option['valid_option_count'],
                    self.active_blockage['left_gateway'][0],
                    self.active_blockage['left_gateway'][1],
                    self.active_blockage['right_gateway'][0],
                    self.active_blockage['right_gateway'][1],
                )
            )
        return self.active_blockage

    def fit_lidar_corridor_centerline(
        self, beam_clusters, obstacle_cluster, gateway_longitudinal
    ):
        """Infer local road curvature from long LiDAR boundary clusters.

        Each accepted boundary is fitted independently in the ego frame.  Two
        visible walls provide both road-center offset and curvature.  With one
        wall, its absolute offset is removed so curvature remains usable
        without assuming a lane width.  Compact obstacle returns are excluded.
        """
        fit_limit = gateway_longitudinal + 0.8
        reference_longitudinal = min(0.7, gateway_longitudinal)
        fits = []
        for candidate in beam_clusters:
            if candidate is obstacle_cluster:
                continue
            filtered = [
                point for point in candidate
                if 0.25 <= point[1] <= fit_limit
            ]
            if len(filtered) < self.corridor_boundary_min_points:
                continue
            longitudinals = np.asarray([point[1] for point in filtered], dtype=float)
            laterals = np.asarray([point[2] for point in filtered], dtype=float)
            span = float(np.ptp(longitudinals))
            if span < self.corridor_boundary_min_span:
                continue
            coefficients, rmse = self.robust_boundary_polynomial(
                longitudinals, laterals
            )
            if coefficients is None or rmse > self.corridor_boundary_fit_max_rmse:
                continue
            near_lateral = float(np.polyval(coefficients, reference_longitudinal))
            # Near the vehicle, track walls should be outside the driveable
            # center corridor.  This rejects other compact clutter without
            # assuming which wall remains on which side farther into a bend.
            if not 0.45 <= abs(near_lateral) <= 1.8:
                continue
            fits.append({
                'coefficients': coefficients,
                'near_lateral': near_lateral,
                'span': span,
                'rmse': rmse,
                'point_count': len(filtered),
            })

        def fit_rank(item):
            return (item['span'], item['point_count'], -item['rmse'])

        left_fits = [item for item in fits if item['near_lateral'] > 0.0]
        right_fits = [item for item in fits if item['near_lateral'] < 0.0]
        selected = []
        if left_fits:
            selected.append(max(left_fits, key=fit_rank))
        if right_fits:
            selected.append(max(right_fits, key=fit_rank))
        if not selected and fits:
            selected.append(max(fits, key=fit_rank))

        if not selected:
            return lambda longitudinal: 0.0, 'ego_straight_fallback'

        if len(selected) == 2:
            def centerline(longitudinal):
                return float(np.mean([
                    np.polyval(item['coefficients'], longitudinal)
                    for item in selected
                ]))
        else:
            coefficients = selected[0]['coefficients']

            def centerline(longitudinal):
                return float(
                    np.polyval(coefficients, longitudinal)
                    - np.polyval(coefficients, 0.0)
                )

        source = 'two_boundary_fit' if len(selected) == 2 else 'single_boundary_fit'
        return centerline, source

    def robust_boundary_polynomial(self, longitudinals, laterals):
        """Fit a quadratic boundary model with small iterative MAD rejection."""
        mask = np.ones(longitudinals.shape, dtype=bool)
        coefficients = None
        for _ in range(3):
            if int(np.count_nonzero(mask)) < self.corridor_boundary_min_points:
                return None, float('inf')
            degree = 2 if float(np.ptp(longitudinals[mask])) >= 0.8 else 1
            coefficients = np.polyfit(
                longitudinals[mask], laterals[mask], degree
            )
            residuals = laterals - np.polyval(coefficients, longitudinals)
            centered = residuals[mask] - np.median(residuals[mask])
            mad = float(np.median(np.abs(centered)))
            rejection_limit = max(0.06, 2.5 * 1.4826 * mad)
            updated_mask = np.abs(residuals) <= rejection_limit
            if np.array_equal(updated_mask, mask):
                break
            mask = updated_mask
        if coefficients is None or int(np.count_nonzero(mask)) == 0:
            return None, float('inf')
        residuals = (
            laterals[mask] - np.polyval(coefficients, longitudinals[mask])
        )
        return coefficients, float(math.sqrt(np.mean(residuals ** 2)))

    def reuse_collision_checked_path(self, ego_state, context):
        """Reuse only the still collision-free suffix of the last accepted path."""
        if len(self.committed_path) < 2:
            return []
        suffix = self.trim_forward_path(ego_state, self.committed_path)
        if len(suffix) < 2:
            return []
        if not self.path_collision_free(
            suffix,
            context['occupancy'],
            context['world_to_grid'],
            context['resolution'],
        ):
            return []
        return suffix

    @staticmethod
    def trim_forward_path(ego_state, path):
        """Trim without jumping to a nearby reverse-facing path branch."""
        if len(path) < 2:
            return []
        cos_yaw = math.cos(ego_state.yaw)
        sin_yaw = math.sin(ego_state.yaw)
        candidates = []
        for index in range(1, len(path)):
            point_x, point_y = path[index]
            dx = point_x - ego_state.x
            dy = point_y - ego_state.y
            if cos_yaw * dx + sin_yaw * dy <= 0.02:
                continue
            bearing_error = abs(normalize_angle(
                math.atan2(dy, dx) - ego_state.yaw
            ))
            segment_dx = point_x - path[index - 1][0]
            segment_dy = point_y - path[index - 1][1]
            if math.hypot(segment_dx, segment_dy) <= 1e-6:
                continue
            tangent_error = abs(normalize_angle(
                math.atan2(segment_dy, segment_dx) - ego_state.yaw
            ))
            if bearing_error > 0.5 * math.pi or tangent_error > 0.5 * math.pi:
                continue
            candidates.append((
                math.hypot(dx, dy) + 0.20 * tangent_error,
                index,
            ))
        if not candidates:
            return []
        _, start_index = min(candidates, key=lambda item: item[0])
        return [(ego_state.x, ego_state.y)] + list(path[start_index:])

    def candidate_cost_components(self, path, lateral_offset, visibility, context):
        length = self.path_length(path)
        headings = [
            math.atan2(y1 - y0, x1 - x0)
            for (x0, y0), (x1, y1) in zip(path[:-1], path[1:])
            if math.hypot(x1 - x0, y1 - y0) > 1e-6
        ]
        curvature = 0.0
        if len(headings) > 1:
            curvature = float(np.mean([
                abs(normalize_angle(next_heading - heading))
                for heading, next_heading in zip(headings[:-1], headings[1:])
            ])) / max(self.config.MAX_STEER, 1e-3)

        occupied = np.argwhere(context['occupancy'])
        clearance_penalty = 0.0
        if occupied.size:
            sampled = path[::max(1, len(path) // 12)]
            clearances = []
            for x, y in sampled:
                gx, gy = context['world_to_grid'](x, y)
                distance_cells = np.min(np.hypot(
                    occupied[:, 0] - gx, occupied[:, 1] - gy
                ))
                clearances.append(distance_cells * context['resolution'])
            clearance_penalty = float(np.mean(np.exp(-np.asarray(clearances) / 0.5)))

        return {
            'length': length / max(self.astar_grid_radius, 1e-3),
            'curvature': curvature,
            'clearance': clearance_penalty,
            'rejoin': abs(lateral_offset) / max(self.corridor_lateral_offset, 1e-3),
            'visibility': 1.0 - float(np.clip(visibility, 0.0, 1.0)),
            'switch': float(self.selected_candidate != (
                'center' if abs(lateral_offset) < 1e-6
                else ('left' if lateral_offset > 0.0 else 'right')
            )),
        }

    def predict_path_visibility(
        self, path, ego_state, target_state, desired_speed, context
    ):
        tx, ty, target_speed, target_yaw = target_state
        position_covariance, velocity_covariance = self.target_covariance_matrices()
        total = 0.0
        weight = 0.0
        profile = []
        # Candidate selection may look beyond the one-second MPC preview: the
        # side paths often have not diverged enough inside that short horizon
        # for a static occluder to produce a meaningful visibility difference.
        # This remains a local, LiDAR-only prediction over the current scan.
        horizon = self.visibility_prediction_horizon
        for sample_index in range(self.visibility_samples):
            fraction = sample_index / max(self.visibility_samples - 1, 1)
            prediction_time = fraction * horizon
            progress = min(
                prediction_time * max(desired_speed, 0.1), self.path_length(path)
            )
            ego_x, ego_y, ego_yaw = self.sample_path(path, progress, ego_state.yaw)
            mean_x, mean_y, _ = self.predict_visibility_target_pose(
                tx,
                ty,
                target_speed,
                target_yaw,
                self.filtered_target_yaw_rate,
                prediction_time,
            )
            covariance = self.sanitize_covariance(
                position_covariance + prediction_time ** 2 * velocity_covariance
            )
            values, vectors = np.linalg.eigh(covariance)
            # Five-point unscented rule for a 2-D Gaussian with lambda=2:
            # the mean carries 0.5 weight and the four +/-2-sigma points
            # carry 0.125 each.  Unlike equal weighting, this reproduces the
            # supplied covariance and prevents extreme points from dominating
            # the expected-visibility estimate.
            sigma_points = [(np.zeros(2), 0.5)]
            for axis in range(2):
                offset = math.sqrt(4.0 * values[axis]) * vectors[:, axis]
                sigma_points.extend([(offset, 0.125), (-offset, 0.125)])
            sample_total = 0.0
            sample_weight = 0.0
            reason_counts = {}
            reason_weights = {}
            for offset, sigma_weight in sigma_points:
                visible, reason = self.line_of_sight_status(
                    ego_x,
                    ego_y,
                    ego_yaw,
                    mean_x + float(offset[0]),
                    mean_y + float(offset[1]),
                    context,
                )
                reason_counts[reason] = reason_counts.get(reason, 0) + 1
                reason_weights[reason] = (
                    reason_weights.get(reason, 0.0) + sigma_weight
                )
                sample_total += sigma_weight * float(visible)
                sample_weight += sigma_weight
                total += sigma_weight * float(visible)
                weight += sigma_weight
            profile.append({
                'time_s': float(prediction_time),
                'ego_x': float(ego_x),
                'ego_y': float(ego_y),
                'target_x': float(mean_x),
                'target_y': float(mean_y),
                'visibility': (
                    sample_total / sample_weight if sample_weight > 0.0 else 0.0
                ),
                'ray_status_counts': reason_counts,
                'ray_status_weights': reason_weights,
            })
        return (total / weight if weight > 0.0 else 0.0), profile

    def predict_visibility_target_pose(
        self, x, y, speed, yaw, yaw_rate, horizon
    ):
        """Use the EKF constant-velocity belief over the visibility window.

        The short MPC anchor can exploit the filtered yaw rate, but extending
        one instantaneous turn rate over a complete pass predicts that an
        S-turning lead leaves the corridor.  Constant velocity is the explicit
        tracker model and makes no unobserved-map assumption.
        """
        del yaw_rate
        horizon = max(horizon, 0.0)
        return (
            x + speed * horizon * math.cos(yaw),
            y + speed * horizon * math.sin(yaw),
            yaw,
        )

    def line_of_sight_status(self, ex, ey, eyaw, tx, ty, context):
        distance = math.hypot(tx - ex, ty - ey)
        bearing = normalize_angle(math.atan2(ty - ey, tx - ex) - eyaw)
        if distance > self.visibility_max_range:
            return False, 'range'
        if abs(bearing) > 0.5 * self.visibility_fov_rad:
            return False, 'fov'
        start = context['world_to_grid'](ex, ey)
        end = context['world_to_grid'](tx, ty)
        occupancy = context.get('visibility_occupancy', context['occupancy'])
        width, height = occupancy.shape
        steps = max(abs(end[0] - start[0]), abs(end[1] - start[1]), 1)
        # Ignore the ego cell and two endpoint cells containing the target belief.
        for index in range(1, max(1, steps - 2)):
            ratio = index / steps
            gx = int(round(start[0] + ratio * (end[0] - start[0])))
            gy = int(round(start[1] + ratio * (end[1] - start[1])))
            if not (0 <= gx < width and 0 <= gy < height):
                return False, 'grid_bounds'
            if occupancy[gx, gy]:
                return False, 'occluded'
        if not (0 <= end[0] < width and 0 <= end[1] < height):
            return False, 'grid_bounds'
        return True, 'visible'

    @staticmethod
    def predict_target_pose(x, y, speed, yaw, yaw_rate, horizon):
        """Constant-turn-rate target prediction for the moving rear anchor."""
        if horizon <= 0.0 or speed <= 1e-6:
            return x, y, yaw
        if abs(yaw_rate) <= 1e-3:
            return (
                x + speed * horizon * math.cos(yaw),
                y + speed * horizon * math.sin(yaw),
                yaw,
            )
        predicted_yaw = yaw + yaw_rate * horizon
        return (
            x + speed / yaw_rate * (math.sin(predicted_yaw) - math.sin(yaw)),
            y - speed / yaw_rate * (math.cos(predicted_yaw) - math.cos(yaw)),
            normalize_angle(predicted_yaw),
        )

    @staticmethod
    def config_horizon_seconds():
        return mpc_config.TK * mpc_config.DTK

    def build_planning_context(self, ego_state: State):
        """Build one target-filtered local occupancy grid per control cycle."""
        if self.scan is None or self.fault_enabled('fault_inject_planner_failure'):
            return None
        resolution = max(self.astar_resolution, 0.05)
        radius = max(self.astar_grid_radius, resolution * 4.0)
        grid_size = int(math.ceil((2.0 * radius) / resolution)) + 1
        center = grid_size // 2
        occupancy = np.zeros((grid_size, grid_size), dtype=bool)
        # Visibility is a sensing property, not a footprint-clearance test.
        # Keep a raw point-occupancy grid for line-of-sight ray casting while
        # the inflated grid remains the conservative collision representation.
        visibility_occupancy = np.zeros((grid_size, grid_size), dtype=bool)
        inflation_cells = max(1, int(math.ceil(self.astar_obstacle_inflation / resolution)))
        # Rasterize the configured Euclidean inflation radius.  Filling the
        # whole enclosing square inflated diagonal cells by sqrt(2) (0.34 m
        # for a requested 0.20 m radius at 0.12 m resolution), which could
        # falsely close a physically passable corridor.  Half a cell is kept
        # as a conservative discretization allowance.
        inflation_limit = self.astar_obstacle_inflation + 0.5 * resolution
        inflation_offsets = [
            (dx, dy)
            for dx in range(-inflation_cells, inflation_cells + 1)
            for dy in range(-inflation_cells, inflation_cells + 1)
            if math.hypot(dx, dy) * resolution <= inflation_limit + 1e-9
        ]

        def world_to_grid(x: float, y: float):
            gx = int(round((x - ego_state.x) / resolution)) + center
            gy = int(round((y - ego_state.y) / resolution)) + center
            return gx, gy

        def grid_to_world(cell):
            gx, gy = cell
            x = ego_state.x + (gx - center) * resolution
            y = ego_state.y + (gy - center) * resolution
            return x, y

        exclude_tracked_target = (
            self.target is not None
            and self.tracker_mode == 'TRACKED'
            and self.effective_measurement_age() <= min(0.15, self.target_timeout)
        )
        target_x = self.target.pose.pose.position.x if self.target is not None else 0.0
        target_y = self.target.pose.pose.position.y if self.target is not None else 0.0
        target_exclusion_radius = self.target_occupancy_exclusion_radius
        if self.method != 'nominal':
            target_exclusion_radius = max(target_exclusion_radius, 0.7)
        scan_points = []
        for idx, dist in enumerate(self.scan.ranges):
            if not math.isfinite(dist) or dist <= self.scan.range_min:
                continue
            if dist > min(self.scan.range_max, self.astar_max_scan_range):
                continue
            angle = ego_state.yaw + self.scan.angle_min + idx * self.scan.angle_increment
            ox = ego_state.x + dist * math.cos(angle)
            oy = ego_state.y + dist * math.sin(angle)
            scan_points.append((idx, dist, ox, oy))

        # Remove exactly one compact LiDAR cluster associated with the fresh
        # EKF target.  The previous radius mask could leave vehicle-edge
        # returns behind when the EKF lagged in a turn, while increasing that
        # radius risked erasing a nearby real obstacle.
        target_beam_indices = set()
        target_cluster_match_error = None
        target_cluster_width = None
        target_cluster_point_count = 0
        if exclude_tracked_target and scan_points:
            clusters = []
            current = []
            for point in scan_points:
                if current:
                    previous = current[-1]
                    spatial_gap = math.hypot(
                        point[2] - previous[2], point[3] - previous[3]
                    )
                    if point[0] - previous[0] > 2 or spatial_gap > 0.30:
                        if current:
                            clusters.append(current)
                        current = []
                current.append(point)
            if current:
                clusters.append(current)

            target_candidates = []
            for cluster in clusters:
                if not 3 <= len(cluster) <= 120:
                    continue
                width = math.hypot(
                    cluster[-1][2] - cluster[0][2],
                    cluster[-1][3] - cluster[0][3],
                )
                if not 0.04 <= width <= 0.90:
                    continue
                centroid_x = float(np.mean([point[2] for point in cluster]))
                centroid_y = float(np.mean([point[3] for point in cluster]))
                dx = centroid_x - ego_state.x
                dy = centroid_y - ego_state.y
                distance = math.hypot(dx, dy)
                if distance > 1e-6:
                    centroid_x += self.target_center_offset * dx / distance
                    centroid_y += self.target_center_offset * dy / distance
                error = math.hypot(
                    centroid_x - target_x, centroid_y - target_y
                )
                if error <= target_exclusion_radius:
                    target_candidates.append((error, cluster))
            if target_candidates:
                target_cluster_match_error, matched_cluster = min(
                    target_candidates, key=lambda candidate: candidate[0]
                )
                target_cluster_width = math.hypot(
                    matched_cluster[-1][2] - matched_cluster[0][2],
                    matched_cluster[-1][3] - matched_cluster[0][3],
                )
                target_cluster_point_count = len(matched_cluster)
                target_beam_indices = {
                    point[0] for point in matched_cluster
                }

        static_points = []
        for idx, _dist, ox, oy in scan_points:
            # A box-shaped car can produce separate rear and side clusters at
            # its corners.  Remove the precisely matched cluster plus the
            # configured vehicle footprint around the same fresh EKF state;
            # do not enlarge that footprint to solve planner clearance.
            if (
                idx in target_beam_indices
                or (
                    exclude_tracked_target
                    and math.hypot(ox - target_x, oy - target_y)
                    <= target_exclusion_radius
                )
            ):
                continue
            static_points.append((idx, ox, oy))
            gx, gy = world_to_grid(ox, oy)
            if not (0 <= gx < grid_size and 0 <= gy < grid_size):
                continue
            if self.use_visibility:
                visibility_occupancy[gx, gy] = True
            for dx, dy in inflation_offsets:
                inflated_x = gx + dx
                inflated_y = gy + dy
                if (
                    0 <= inflated_x < grid_size
                    and 0 <= inflated_y < grid_size
                ):
                    occupancy[inflated_x, inflated_y] = True

        # A scan represents observed surfaces, not isolated point obstacles.
        # Connect adjacent returns from the same surface in the raw visibility
        # grid so an LOS ray cannot tunnel through angular/grid sampling gaps.
        # Do not bridge removed target returns or distinct objects.
        for previous, current in (
            zip(static_points, static_points[1:]) if self.use_visibility else []
        ):
            previous_idx, previous_x, previous_y = previous
            current_idx, current_x, current_y = current
            if (
                current_idx - previous_idx > 2
                or math.hypot(
                    current_x - previous_x, current_y - previous_y
                ) > 0.30
            ):
                continue
            start_x, start_y = world_to_grid(previous_x, previous_y)
            end_x, end_y = world_to_grid(current_x, current_y)
            steps = max(abs(end_x - start_x), abs(end_y - start_y), 1)
            for step in range(steps + 1):
                ratio = step / steps
                grid_x = int(round(start_x + ratio * (end_x - start_x)))
                grid_y = int(round(start_y + ratio * (end_y - start_y)))
                if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                    visibility_occupancy[grid_x, grid_y] = True

        # Inflated obstacles make the car a point in configuration space.
        # Clear only the tiny region already occupied by the ego footprint so
        # a wall-adjacent pose can plan out of that region. This is disabled
        # for the frozen nominal baseline. Raw visibility occupancy and the
        # independent scan-based safety filter retain every obstacle return.
        start_clearance_cells = int(math.ceil(
            self.planning_start_clearance_radius / resolution
        ))
        for dx in range(-start_clearance_cells, start_clearance_cells + 1):
            for dy in range(-start_clearance_cells, start_clearance_cells + 1):
                if (
                    math.hypot(dx, dy) * resolution
                    <= self.planning_start_clearance_radius
                ):
                    grid_x = center + dx
                    grid_y = center + dy
                    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                        occupancy[grid_x, grid_y] = False
        visibility_occupancy[center, center] = False
        return {
            'occupancy': occupancy,
            'visibility_occupancy': visibility_occupancy,
            'clearance': distance_transform_edt(~occupancy) * resolution,
            'world_to_grid': world_to_grid,
            'grid_to_world': grid_to_world,
            'resolution': resolution,
            'center': center,
            'static_points': static_points,
            'target_cluster_match_error_m': target_cluster_match_error,
            'target_cluster_width_m': target_cluster_width,
            'target_cluster_point_count': target_cluster_point_count,
        }

    def plan_astar_path(
        self,
        ego_state: State,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        context=None,
        time_budget_ms=None,
    ):
        if context is None:
            context = self.build_planning_context(ego_state)
        if context is None:
            return [], False

        occupancy = context['occupancy']
        world_to_grid = context['world_to_grid']
        grid_to_world = context['grid_to_world']
        resolution = context['resolution']
        center = context['center']
        grid_size = occupancy.shape[0]
        goal = world_to_grid(goal_x, goal_y)
        goal = (min(max(goal[0], 0), grid_size - 1), min(max(goal[1], 0), grid_size - 1))
        goal = self.nearest_free_cell(goal, occupancy)
        if goal is None:
            return [], False

        path = self.hybrid_astar_search(
            ego_state,
            goal,
            occupancy,
            world_to_grid,
            grid_to_world,
            resolution,
            goal_yaw,
            clearance=context['clearance'],
            time_budget_ms=time_budget_ms,
        )
        if not path:
            return [], False

        path[0] = (ego_state.x, ego_state.y)
        if len(path) == 1:
            # Reaching the anchor tolerance is a valid zero-length planning
            # result, especially while the lead is stopped.  Append the exact
            # free goal so downstream trajectory sampling has two points.
            path.append(grid_to_world(goal))
        if not self.path_collision_free(path, occupancy, world_to_grid, resolution):
            return [], False

        smoothed = self.smooth_grid_path(path)
        if not self.path_collision_free(smoothed, occupancy, world_to_grid, resolution):
            smoothed = path
        splined = self.spline_smooth_path(smoothed)
        if (
            self.path_exceeds_steering_limit(splined)
            or not self.path_collision_free(splined, occupancy, world_to_grid, resolution)
        ):
            return smoothed, True
        return splined, True

    def path_collision_free(self, path, occupancy, world_to_grid, resolution) -> bool:
        return self.first_path_collision(
            path, occupancy, world_to_grid, resolution
        ) is None

    @staticmethod
    def first_path_collision(path, occupancy, world_to_grid, resolution):
        if not path:
            return ('empty',)
        width, height = occupancy.shape
        sample_spacing = max(0.5 * resolution, 0.02)
        for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
            distance = math.hypot(x1 - x0, y1 - y0)
            sample_count = max(1, int(math.ceil(distance / sample_spacing)))
            for i in range(sample_count + 1):
                ratio = i / sample_count
                x = x0 + ratio * (x1 - x0)
                y = y0 + ratio * (y1 - y0)
                gx, gy = world_to_grid(x, y)
                if not (0 <= gx < width and 0 <= gy < height):
                    return ('outside', round(x, 2), round(y, 2))
                if occupancy[gx, gy]:
                    return ('occupied', round(x, 2), round(y, 2))
        return None

    def nearest_free_cell(self, cell, occupancy):
        if not occupancy[cell]:
            return cell

        width, height = occupancy.shape
        for radius in range(1, max(width, height)):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if 0 <= candidate[0] < width and 0 <= candidate[1] < height and not occupancy[candidate]:
                        return candidate
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if 0 <= candidate[0] < width and 0 <= candidate[1] < height and not occupancy[candidate]:
                        return candidate
        return None

    def hybrid_astar_search(
        self,
        start_state,
        goal,
        occupancy,
        world_to_grid,
        grid_to_world,
        resolution,
        goal_yaw,
        clearance=None,
        time_budget_ms=None,
    ):
        width, height = occupancy.shape
        yaw_bins = max(self.hybrid_astar_yaw_bins, 8)
        step_distance = max(self.hybrid_astar_step_distance, resolution)
        goal_xy = grid_to_world(goal)
        goal_tolerance = max(1.5 * resolution, 0.22)
        steer_set = [
            self.config.MIN_STEER,
            0.5 * self.config.MIN_STEER,
            0.0,
            0.5 * self.config.MAX_STEER,
            self.config.MAX_STEER,
        ]

        def yaw_to_bin(yaw):
            wrapped = normalize_angle(yaw)
            return int(round((wrapped + math.pi) / (2.0 * math.pi) * yaw_bins)) % yaw_bins

        def pose_to_key(x, y, yaw):
            gx, gy = world_to_grid(x, y)
            return gx, gy, yaw_to_bin(yaw)

        def is_free(x, y):
            gx, gy = world_to_grid(x, y)
            if not (0 <= gx < width and 0 <= gy < height):
                return False
            return not occupancy[gx, gy]

        def rollout(x, y, yaw, steer):
            samples = max(3, int(math.ceil(step_distance / max(resolution, 1e-3))))
            nx, ny, nyaw = x, y, yaw
            for _ in range(samples):
                ds = step_distance / samples
                nx += ds * math.cos(nyaw)
                ny += ds * math.sin(nyaw)
                nyaw = normalize_angle(nyaw + (ds / self.config.WB) * math.tan(steer))
                if not is_free(nx, ny):
                    return None
            return nx, ny, nyaw

        start_key = pose_to_key(start_state.x, start_state.y, start_state.yaw)
        start_pose = (start_state.x, start_state.y, start_state.yaw)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start_key))
        came_from = {}
        pose_by_key = {start_key: start_pose}
        g_score = {start_key: 0.0}

        def position_error(x, y):
            return math.hypot(x - goal_xy[0], y - goal_xy[1])

        def heuristic(x, y, yaw):
            yaw_error = abs(normalize_angle(yaw - goal_yaw))
            return position_error(x, y) + self.goal_yaw_weight * yaw_error

        expansions = 0
        budget_ms = (
            self.hybrid_astar_time_budget_ms
            if time_budget_ms is None
            else max(1.0, float(time_budget_ms))
        )
        deadline = time.perf_counter() + budget_ms * 1e-3
        while open_set:
            if time.perf_counter() >= deadline:
                return []
            _, _, current = heapq.heappop(open_set)
            x, y, yaw = pose_by_key[current]
            if position_error(x, y) <= goal_tolerance:
                path = [(x, y)]
                while current in came_from:
                    current = came_from[current]
                    px, py, _ = pose_by_key[current]
                    path.append((px, py))
                path.reverse()
                return path

            expansions += 1
            if expansions >= self.hybrid_astar_max_expansions:
                break

            for steer in steer_set:
                next_pose = rollout(x, y, yaw, steer)
                if next_pose is None:
                    continue
                nx, ny, nyaw = next_pose
                next_key = pose_to_key(nx, ny, nyaw)
                if not (0 <= next_key[0] < width and 0 <= next_key[1] < height):
                    continue
                steer_penalty = 0.15 * abs(steer) / max(self.config.MAX_STEER, 1e-3)
                tentative = g_score[current] + step_distance * (1.0 + steer_penalty)
                if clearance is not None and self.astar_clearance_cost_gain > 0.0:
                    local_clearance = float(clearance[next_key[0], next_key[1]])
                    tentative += (
                        self.astar_clearance_cost_gain
                        * step_distance
                        * math.exp(-local_clearance / self.astar_clearance_cost_scale)
                    )
                if steer != 0.0:
                    tentative += 0.05
                if tentative >= g_score.get(next_key, float('inf')):
                    continue
                came_from[next_key] = current
                pose_by_key[next_key] = next_pose
                g_score[next_key] = tentative
                counter += 1
                priority = tentative + heuristic(nx, ny, nyaw)
                heapq.heappush(open_set, (priority, counter, next_key))

        return []

    def smooth_grid_path(self, path):
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev_x, prev_y = smoothed[-1]
            cur_x, cur_y = path[i]
            next_x, next_y = path[i + 1]
            prev_heading = math.atan2(cur_y - prev_y, cur_x - prev_x)
            next_heading = math.atan2(next_y - cur_y, next_x - cur_x)
            if abs(normalize_angle(next_heading - prev_heading)) > 0.1:
                smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

    def spline_smooth_path(self, path):
        if len(path) < 4:
            return path

        distances = [0.0]
        filtered = [path[0]]
        for point in path[1:]:
            if math.hypot(point[0] - filtered[-1][0], point[1] - filtered[-1][1]) < 1e-4:
                continue
            filtered.append(point)
            distances.append(
                distances[-1] + math.hypot(filtered[-1][0] - filtered[-2][0], filtered[-1][1] - filtered[-2][1])
            )

        if len(filtered) < 4 or distances[-1] < 1e-3:
            return filtered

        s = np.array(distances)
        pts = np.array(filtered)
        spline_x = CubicSpline(s, pts[:, 0], bc_type='natural')
        spline_y = CubicSpline(s, pts[:, 1], bc_type='natural')
        sample_count = max(len(filtered) * 3, int(distances[-1] / 0.12))
        sample_s = np.linspace(0.0, distances[-1], sample_count)
        return [(float(spline_x(si)), float(spline_y(si))) for si in sample_s]

    def path_exceeds_steering_limit(self, path) -> bool:
        if len(path) < 3:
            return False
        max_curvature = math.tan(self.config.MAX_STEER) / max(self.config.WB, 1e-6)
        for p0, p1, p2 in zip(path[:-2], path[1:-1], path[2:]):
            a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
            area2 = abs(
                (p1[0] - p0[0]) * (p2[1] - p0[1])
                - (p1[1] - p0[1]) * (p2[0] - p0[0])
            )
            denom = max(a * b * c, 1e-9)
            curvature = 2.0 * area2 / denom
            if curvature > 1.08 * max_curvature:
                return True
        return False

    def path_length(self, path) -> float:
        return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]))

    def sample_path(self, path, distance: float, fallback_yaw: float):
        remaining = max(distance, 0.0)
        for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-6:
                continue
            if remaining <= seg_len:
                ratio = remaining / seg_len
                x = x0 + ratio * (x1 - x0)
                y = y0 + ratio * (y1 - y0)
                yaw = math.atan2(y1 - y0, x1 - x0)
                return x, y, yaw
            remaining -= seg_len

        if len(path) >= 2:
            x0, y0 = path[-2]
            x1, y1 = path[-1]
            yaw = math.atan2(y1 - y0, x1 - x0)
            return x1, y1, yaw
        return path[0][0], path[0][1], fallback_yaw

    def mpc_prob_init(self):
        self.xk = cvxpy.Variable((self.config.NXK, self.config.TK + 1))
        self.uk = cvxpy.Variable((self.config.NU, self.config.TK))
        objective = 0.0
        constraints = []

        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))
        self.h_front_k = cvxpy.Parameter(nonneg=True, value=5.0)

        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))
        Q_block = [self.config.Qk] * self.config.TK
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        objective += cvxpy.quad_form(cvxpy.reshape(self.uk, (self.config.NU * self.config.TK, 1), order='F'), R_block)
        objective += cvxpy.quad_form(
            cvxpy.reshape(self.ref_traj_k - self.xk, (self.config.NXK * (self.config.TK + 1), 1), order='F'),
            Q_block,
        )
        objective += cvxpy.quad_form(
            cvxpy.reshape(self.uk[:, 1:] - self.uk[:, :-1], (self.config.NU * (self.config.TK - 1), 1), order='F'),
            Rd_block,
        )

        A_block = []
        B_block = []
        C_block = []
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))
        self.Annz_k.value = A_block.data
        self.Ak_ = cvxpy.reshape(indexer @ self.Annz_k, (m, n), order='C')

        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bnnz_k.value = B_block.data
        self.Bk_ = cvxpy.reshape(indexer @ self.Bnnz_k, (m, n), order='C')

        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        constraints += [
            cvxpy.reshape(self.xk[:, 1:], (self.config.NXK * self.config.TK, 1), order='F')
            == self.Ak_ @ cvxpy.reshape(self.xk[:, :-1], (self.config.NXK * self.config.TK, 1), order='F')
            + self.Bk_ @ cvxpy.reshape(self.uk, (self.config.NU * self.config.TK, 1), order='F')
            + cvxpy.reshape(self.Ck_, (self.config.NXK * self.config.TK, 1)),
            self.xk[2, :] <= self.config.MAX_SPEED,
            self.xk[2, :] >= self.config.MIN_SPEED,
            self.xk[:, 0] == self.x0k,
            self.uk[0, :] <= self.config.MAX_ACCEL,
            self.uk[0, :] >= -self.config.MAX_ACCEL,
            self.uk[1, :] <= self.config.MAX_STEER,
            self.uk[1, :] >= self.config.MIN_STEER,
        ]

        # Keep only the forward clearance constraint in the MPC.
        constraints += [
            self.config.DTK * self.xk[2, :-1] + (self.config.DTK ** 2) * self.uk[0, :]
            <= self.cbf_front_gamma * self.h_front_k,
        ]

        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], v=x0[2], yaw=x0[3])
        for ai, di, i in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):
        delta = max(min(delta, self.config.MAX_STEER), self.config.MIN_STEER)
        state.x += state.v * math.cos(state.yaw) * self.config.DTK
        state.y += state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw += (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        state.v += a * self.config.DTK
        state.v = max(min(state.v, self.config.MAX_SPEED), self.config.MIN_SPEED)
        return state

    def get_model_matrix(self, v, phi, delta):
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        if self.fault_enabled('fault_inject_solver_failure'):
            return None, None, None, None, None, None

        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block
        self.ref_traj_k.value = ref_traj

        try:
            self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)
        except Exception as exc:
            self.get_logger().error(f'Ego MPC solver exception: {exc}')
            return None, None, None, None, None, None

        if self.MPC_prob.status in (cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE):
            if self.xk.value is None or self.uk.value is None:
                self.get_logger().error('Ego MPC returned an empty solution')
                return None, None, None, None, None, None
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()
            solution_arrays = (ox, oy, ov, oyaw, oa, odelta)
            if not all(np.all(np.isfinite(values)) for values in solution_arrays):
                self.get_logger().error('Ego MPC returned NaN or Inf')
                return None, None, None, None, None, None
            if len(oa) == 0 or len(odelta) == 0:
                self.get_logger().error('Ego MPC returned an empty control sequence')
                return None, None, None, None, None, None
            return oa, odelta, ox, oy, ov, oyaw

        self.get_logger().warn('Ego MPC solve failed')
        return None, None, None, None, None, None

    def linear_mpc_control(self, ref_path, x0, oa, od):
        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        path_predict = self.predict_motion(x0, oa, od, ref_path)
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_v, mpc_yaw = self.mpc_prob_solve(ref_path, path_predict, x0)
        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_v, mpc_yaw, path_predict

    def update_safety_margin(self, front_dist: float) -> None:
        def finite_margin(dist: float, threshold: float) -> float:
            if not math.isfinite(dist):
                return 5.0
            return max(dist - threshold, 0.0)

        self.h_front_k.value = finite_margin(front_dist, self.wall_stop_distance)

    def visualize_path(self, ref_traj, publisher, r, g, b, namespace):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.color.a = 0.8
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for i in range(ref_traj.shape[1]):
            marker.points.append(Point(x=float(ref_traj[0, i]), y=float(ref_traj[1, i]), z=0.0))
        publisher.publish(marker)

    def visualize_xy_path(self, ox, oy, publisher, r, g, b, namespace):
        if ox is None or oy is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a = 0.9
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for x, y in zip(ox, oy):
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        publisher.publish(marker)

    def visualize_point_path(self, path, publisher, r, g, b, namespace):
        if not path:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.16
        marker.color.a = 0.95
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        for x, y in path:
            marker.points.append(Point(x=float(x), y=float(y), z=0.04))
        publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EgoMPCFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 launch invalidates the shared context while timer callbacks may
        # still be returning.  Suppress only that shutdown race; real runtime
        # failures must remain visible.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
