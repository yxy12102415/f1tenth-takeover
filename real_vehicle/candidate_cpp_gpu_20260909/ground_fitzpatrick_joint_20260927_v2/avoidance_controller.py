#!/usr/bin/env python3
"""Fail-closed Frenet planner with a four-state dynamic MPC local-path tracker."""
import json
import math
import os
from collections import deque
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
PLANNER = WORKSPACE / 'candidate_cpp_gpu_20260909' / 'open_route_frenet_v1_20260923'
sys.path.insert(0, str(PLANNER))
from return_lattice_planner import ClosedLoopLatticePlanner
from obstacle_tracker import (
    PersistentObstacleTracker, cluster_scan_points, confirmed_points,
)
from local_path_tracker import (
    CONFIG as TRACKER_CONFIG, CONTROLLER_NAME, LocalPathTracker,
)

TIMEOUT = 0.50
# The Hokuyo source stamp can cross the 0.50 s boundary by a few milliseconds
# between the relay and controller callbacks even though receipt is current.
# Keep receipt freshness and the independent final guard at 0.50 s; this small
# source-stamp allowance prevents a recovered 0.502 s sample from latching the
# planner invalid before the next scan arrives.
SOURCE_STAMP_TOLERANCE = 0.005
MAX_SPEED = 1.50
MIN_MOVING_SPEED = 0.60
AVOIDANCE_SPEED_MAX = 1.20
PLANNER_TUNING = {
    # Keep the original escape candidates; add a finer 15 cm option.
    'offsets': (-.40, -.30, -.20, -.15, -.10, 0., .10, .15, .20, .30, .40),
    'minimum_initial_avoidance_offset_m': .15,
}
BLOCKAGE_CONFIRM_SPEED = 0.60
# Preview lets the vehicle leave the required zero-speed start.  The query is
# capped before the terminal zero so endpoint_speed_limit owns the final stop.
SPEED_PROFILE_LOOKAHEAD_M = 0.20
BLOCKAGE_CONFIRM_SECONDS = 0.75
NEAR_BLOCK_DISTANCE_M = 0.50
NEAR_BLOCK_CONFIRM_SECONDS = 0.20
STATIC_ECHO_TOLERANCE = 0.25
WALL_ECHO_FORWARD_ASSOCIATION_M = 0.45
# AckermannDrive.steering_angle is float32.  Publishing the exact double
# boundary 0.27 encodes as 0.2700000107 and trips the downstream double
# precision 0.27 guard.  Keep a small serialization margin while retaining
# the chassis/planner safety limit of 0.27 rad.
MAX_STEERING = 0.269
MAX_STEERING_RATE = 1.0
LASER_OFFSET_X = 0.27
ENDPOINT_STOP_REMAINING_M = 0.05
ENDPOINT_DECELERATION_MPS2 = 0.25
TERMINAL_TRACKER_TOLERANCE_M = 0.15
ENDPOINT_POSITION_TOLERANCE_M = 0.10
ENDPOINT_HEADING_TOLERANCE_RAD = 0.20
PLAN_INTERVAL_S = 0.20
CLUSTER_MAX_POINT_GAP_M = 0.18
CLUSTER_MAX_BEAM_GAP = 2
CLUSTER_MIN_POINTS = 3
CLUSTER_MIN_EXTENT_M = 0.025
CLUSTER_MAX_EXTENT_M = 0.80
OBSTACLE_CONFIRM_SCANS = 2
OBSTACLE_ASSOCIATION_M = 0.22
# Planning consumes a new scan at 5 Hz.  A 0.30 s age bound lets a confirmed
# obstacle bridge exactly one missed planning scan; maximum_confirmed_misses=1
# still removes it on the second miss.
OBSTACLE_HOLD_S = 0.30


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw_checked(q):
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.02 or abs(q.x) > 0.01 or abs(q.y) > 0.01:
        raise ValueError('invalid quaternion')
    return 2.0 * math.atan2(q.z, q.w)


def endpoint_speed_limit(remaining_m):
    """Low-speed approach that leaves only a short coast at lap completion."""
    remaining_m = max(0.0, float(remaining_m))
    if remaining_m <= ENDPOINT_STOP_REMAINING_M:
        return 0.0
    return math.sqrt(2.0 * ENDPOINT_DECELERATION_MPS2 * remaining_m)


def endpoint_pose_matches(distance_m, heading_error_rad):
    return (math.isfinite(distance_m) and math.isfinite(heading_error_rad)
            and 0. <= distance_m <= ENDPOINT_POSITION_TOLERANCE_M
            and abs(heading_error_rad) <= ENDPOINT_HEADING_TOLERANCE_RAD)


def endpoint_pose_error(route, x, y, heading):
    return (float(np.hypot(route[-1,0]-x, route[-1,1]-y)),
            math.atan2(math.sin(heading-route[-1,2]), math.cos(heading-route[-1,2])))


def open_endpoint_reached(closed_loop, route_length_m, progress_m,
                          distance_m=math.inf, heading_error_rad=math.inf):
    """Recognize an open endpoint before a one-point terminal plan is rejected."""
    return (not bool(closed_loop)
            and endpoint_speed_limit(float(route_length_m) - float(progress_m)) == 0.0
            and endpoint_pose_matches(distance_m, heading_error_rad))


def terminal_tracker_stop_allowed(reason, remaining_m, endpoint_distance_m,
                                  heading_error_rad=math.inf):
    """Accept bounded MPC feasibility errors only inside the terminal box."""
    return (reason in {
                'mpc_cross_track_error', 'mpc_heading_error', 'mpc_infeasible'}
            and 0.0 <= float(remaining_m) <= TERMINAL_TRACKER_TOLERANCE_M
            and endpoint_pose_matches(endpoint_distance_m, heading_error_rad))


def load_speed_profile(path, route):
    """Load a profile whose geometry exactly matches the controller route."""
    path = Path(path)
    expected = ('x', 'y', 'yaw', 'distance_m', 'curvature_1pm', 'speed_mps')
    header = tuple(path.read_text().splitlines()[0].split(','))
    if header != expected:
        raise ValueError('speed_profile_schema')
    values = np.loadtxt(path, delimiter=',', skiprows=1)
    values = np.atleast_2d(values)
    route = np.asarray(route, dtype=float)
    if values.shape != (len(route), len(expected)):
        raise ValueError('speed_profile_length')
    if not np.isfinite(values).all():
        raise ValueError('speed_profile_nonfinite')
    heading_error = np.abs(np.arctan2(
        np.sin(values[:, 2] - route[:, 2]),
        np.cos(values[:, 2] - route[:, 2]),
    ))
    if (np.max(np.linalg.norm(values[:, :2] - route[:, :2], axis=1)) > 1e-6
            or np.max(heading_error) > 1e-6):
        raise ValueError('speed_profile_route_mismatch')
    distance = values[:, 3]
    speed = values[:, 5]
    if (abs(distance[0]) > 1e-9 or np.any(np.diff(distance) <= 0.0)
            or abs(speed[0]) > 1e-9 or abs(speed[-1]) > 1e-9
            or np.any(speed < 0.0) or np.any(speed > MAX_SPEED + 1e-9)):
        raise ValueError('speed_profile_limits')
    return distance, speed


def command_heartbeat_allowed(enabled, has_safe_path, command_age_s):
    """Replay only a recent command while all live inputs remain healthy."""
    return (bool(enabled) and bool(has_safe_path)
            and 0.0 <= float(command_age_s) <= TIMEOUT)


def plan_due(last_plan_finished, now, has_new_scan, enabled=True,
             has_safe_path=False):
    """Bound active search to 5 Hz and avoid idle pre-arm replanning."""
    return ((bool(enabled) or not bool(has_safe_path))
            and bool(has_new_scan) and math.isfinite(float(now))
            and float(now) - float(last_plan_finished) + 1e-9
            >= PLAN_INTERVAL_S)


class LapProgress:
    """Accumulate plausible forward progress on closed or open routes."""
    def __init__(self, segment_m, closed_loop=True):
        self.segment_m = np.asarray(segment_m, dtype=float)
        self.closed_loop = bool(closed_loop)
        self.route_length_m = float(np.sum(self.segment_m))
        self.previous_index = None
        self.distance_m = 0.0

    def update(self, index):
        if self.closed_loop:
            index = int(index) % len(self.segment_m)
        else:
            index = int(np.clip(int(index), 0, len(self.segment_m) - 1))
        if self.previous_index is None:
            self.previous_index = index
            return self.distance_m
        if self.closed_loop:
            forward = (index - self.previous_index) % len(self.segment_m)
            backward = (self.previous_index - index) % len(self.segment_m)
            if backward <= 8 and backward < forward:
                return self.distance_m
        else:
            forward = index - self.previous_index
            if forward < 0:
                if -forward <= 8:
                    return self.distance_m
                raise ValueError('progress_jump')
        if forward == 0:
            return self.distance_m
        if forward > 80:
            raise ValueError('progress_jump')
        for step in range(forward):
            self.distance_m += self.segment_m[(self.previous_index + step) % len(self.segment_m)]
        self.previous_index = index
        return self.distance_m


class BlockageConfirmation:
    """Require a stable blockage duration, shortened for close-range returns."""
    def __init__(self, required_seconds=BLOCKAGE_CONFIRM_SECONDS,
                 immediate_distance_m=NEAR_BLOCK_DISTANCE_M,
                 near_required_seconds=NEAR_BLOCK_CONFIRM_SECONDS):
        self.required_seconds = float(required_seconds)
        self.immediate_distance_m = float(immediate_distance_m)
        self.near_required_seconds = float(near_required_seconds)
        if (self.required_seconds <= 0.0 or self.immediate_distance_m <= 0.0
                or self.near_required_seconds <= 0.0
                or self.near_required_seconds > self.required_seconds):
            raise ValueError('blockage confirmation limits must be positive')
        self.count = 0
        self.started = None
        self.active_required_seconds = self.required_seconds

    def reset(self):
        self.count = 0
        self.started = None
        self.active_required_seconds = self.required_seconds

    def elapsed(self, now=None):
        now = time.monotonic() if now is None else float(now)
        return 0.0 if self.started is None else max(0.0, now - self.started)

    def update(self, valid, reason, now=None, nearest_obstacle_m=None):
        now = time.monotonic() if now is None else float(now)
        if valid:
            self.reset()
            return 'valid'
        if reason != 'all_candidates_blocked':
            self.reset()
            return 'blocked'
        near = (nearest_obstacle_m is not None and math.isfinite(nearest_obstacle_m)
                and nearest_obstacle_m <= self.immediate_distance_m)
        self.active_required_seconds = (
            self.near_required_seconds if near else self.required_seconds
        )
        if self.started is None:
            self.started = now
        self.count += 1
        return ('blocked' if self.elapsed(now) + 1e-9 >= self.active_required_seconds
                else 'confirming')


class PoseHistory:
    """Interpolate the map pose at a sensor source timestamp."""
    def __init__(self, max_samples=80, maximum_bracket_s=0.15):
        self.samples = deque(maxlen=int(max_samples))
        self.maximum_bracket_s = float(maximum_bracket_s)

    def add(self, source, x, y, heading):
        values = (source, x, y, heading)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('pose_history_nonfinite')
        if self.samples and source <= self.samples[-1][0]:
            raise ValueError('pose_history_nonincreasing')
        self.samples.append(tuple(float(value) for value in values))

    def at(self, source):
        if not math.isfinite(source) or not self.samples:
            raise ValueError('scan_pose_unsynchronized')
        samples = list(self.samples)
        for index, right in enumerate(samples):
            if right[0] < source:
                continue
            if index == 0:
                if right[0] - source <= self.maximum_bracket_s / 2.0:
                    return right[1], right[2], right[3], right[0] - source
                raise ValueError('scan_pose_unsynchronized')
            left = samples[index - 1]
            span = right[0] - left[0]
            if span <= 0.0 or span > self.maximum_bracket_s:
                raise ValueError('scan_pose_unsynchronized')
            fraction = (source - left[0]) / span
            yaw_delta = math.atan2(math.sin(right[3] - left[3]),
                                   math.cos(right[3] - left[3]))
            return (left[1] + fraction * (right[1] - left[1]),
                    left[2] + fraction * (right[2] - left[2]),
                    left[3] + fraction * yaw_delta,
                    span)
        last = samples[-1]
        if source - last[0] <= self.maximum_bracket_s / 2.0:
            return last[1], last[2], last[3], source - last[0]
        raise ValueError('scan_pose_unsynchronized')


def remove_map_wall_returns(static_map, points, directions,
                            endpoint_tolerance=STATIC_ECHO_TOLERANCE,
                            forward_association_m=WALL_ECHO_FORWARD_ASSOCIATION_M,
                            return_mask=False):
    """Keep free-space returns unless a mapped wall is nearby on the same ray.

    PF/map registration error can place a wall return on either side of the
    frozen occupancy boundary.  Looking in both directions along the laser ray
    associates both short and overshooting returns without erasing objects that
    are farther than the bounded association distance from a mapped wall.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    directions = np.asarray(directions, dtype=float).reshape(-1, 2)
    if len(points) != len(directions):
        raise ValueError('scan_point_direction_mismatch')
    if not len(points):
        return (points, np.zeros(0, dtype=bool)) if return_mask else points
    endpoint_clearance = static_map.clearance_many(points)
    candidates = endpoint_clearance > float(endpoint_tolerance)
    if not np.any(candidates):
        empty = np.empty((0, 2), dtype=float)
        return (empty, np.zeros(len(points), dtype=bool)) if return_mask else empty
    candidate_points = points[candidates]
    candidate_directions = directions[candidates]
    steps = np.arange(0.05, float(forward_association_m) + 1e-9, 0.05)
    signed_steps = np.r_[-steps[::-1], steps]
    along_ray = (candidate_points[:, None, :]
                 + candidate_directions[:, None, :] * signed_steps[None, :, None])
    clearances = static_map.clearance_many(along_ray.reshape(-1, 2)).reshape(
        len(candidate_points), len(signed_steps)
    )
    wall_on_ray = np.any(clearances <= 0.05, axis=1)
    keep = np.zeros(len(points), dtype=bool)
    keep[np.flatnonzero(candidates)[~wall_on_ray]] = True
    dynamic = points[keep]
    return (dynamic, keep) if return_mask else dynamic


class AvoidanceController(Node):
    def __init__(self):
        super().__init__(CONTROLLER_NAME, start_parameter_services=False)
        route = str(self.declare_parameter('route_file', '').value)
        map_file = str(self.declare_parameter('map_file', '').value)
        speed_profile = str(self.declare_parameter('speed_profile_file', '').value)
        if not route or not map_file or not speed_profile:
            raise RuntimeError(
                'route_file, map_file and speed_profile_file are required')
        self.planner = ClosedLoopLatticePlanner(
            route, map_file,
            **PLANNER_TUNING,
            footprint_radius_m=0.25,
            vehicle_half_width_m=0.15,
            obstacle_margin_m=0.0,
            proactive_avoidance_clearance_m=0.22,
            # Live no-obstacle laps showed map-wall returns displaced up to
            # 0.25 m into free space by localization/scan registration error.
            # The earlier ground avoidance controller uses the same 0.25 m
            # wall-echo band. Objects farther into mapped free space remain.
            static_echo_tolerance_m=STATIC_ECHO_TOLERANCE,
        )
        self.lap = LapProgress(
            self.planner.segment_m, closed_loop=self.planner.closed_loop)
        self.profile_distance, self.profile_speed = load_speed_profile(
            speed_profile, self.planner.route)
        self.reference_profile_speed = 0.0
        self.blockage_confirmation = BlockageConfirmation()
        self.pose_history = PoseHistory()
        self.input_lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.tracker_lock = threading.RLock()
        self.tracker = LocalPathTracker(steering_limit=MAX_STEERING)
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.planning_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.watchdog_group = MutuallyExclusiveCallbackGroup()
        self.lattice_state = 'FOLLOWING'
        self.obstacle_tracker = PersistentObstacleTracker(
            required_hits=OBSTACLE_CONFIRM_SCANS,
            association_distance_m=OBSTACLE_ASSOCIATION_M,
            confirmed_hold_s=OBSTACLE_HOLD_S,
            maximum_confirmed_misses=1,
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/baseline/drive_nominal', 1)
        self.valid_pub = self.create_publisher(Bool, '/baseline/planner_valid', 1)
        self.status_pub = self.create_publisher(String, '/baseline/controller_status', 1)
        self.path_pub = self.create_publisher(PathMessage, '/baseline/lattice_selected', 1)
        self.obstacle_pub = self.create_publisher(
            MarkerArray, '/baseline/confirmed_obstacles', 1)
        self.scan = None
        self.scan_stamp = self.scan_receipt = -math.inf
        self.odom_stamp = self.odom_receipt = -math.inf
        self.pose_stamp = self.pose_receipt = -math.inf
        self.pose_input_stream_stamp = -math.inf
        self.pose_control_stream_stamp = -math.inf
        self.latest_pose = None
        self.quality = False
        self.quality_receipt = -math.inf
        self.enabled = False
        self.enable_receipt = -math.inf
        self.last_planned_scan = -math.inf
        self.last_plan_finished = -math.inf
        self.plan_started = None
        self.last_plan_duration_s = None
        self.planning_phase = 'idle'
        self.last_plan_stages = {}
        self.current_plan_stages = {}
        self.last_result = None
        self.last_safe_path = None
        self.last_angle = 0.0
        self.last_command_speed = 0.0
        self.last_command_angle = 0.0
        self.last_control_receipt = -math.inf
        self.last_pose_tick = None
        self.last_tracker_tick = None
        self.sequence = 0
        self.complete = False
        self.measured_speed = 0.0
        self.measured_lateral_speed = 0.0
        self.measured_yaw_rate = 0.0
        self.scan_pose_alignment_span_s = None
        self.last_candidate_reasons = {}
        self.raw_scan_point_count = 0
        self.dynamic_scan_point_count = 0
        self.confirmed_dynamic_point_count = 0
        self.detected_cluster_count = 0
        self.confirmed_cluster_count = 0
        self.confirmed_tracks = []
        self.nearest_dynamic_point_m = None
        self.create_subscription(
            LaserScan, '/baseline/scan', self.scan_cb, qos_profile_sensor_data,
            callback_group=self.sensor_group)
        self.create_subscription(
            Odometry, '/baseline/odom', self.odom_cb, 1,
            callback_group=self.sensor_group)
        self.create_subscription(
            Odometry, '/baseline/localization_pose', self.pose_input_cb, 1,
            callback_group=self.sensor_group)
        self.create_subscription(
            Odometry, '/baseline/localization_pose', self.pose_cb, 1,
            callback_group=self.planning_group)
        self.create_subscription(
            Bool, '/baseline/localization_valid', self.quality_cb, 1,
            callback_group=self.sensor_group)
        self.create_subscription(
            Bool, '/baseline/enable', self.enable_cb, 1,
            callback_group=self.sensor_group)
        self.create_timer(0.02, self.watchdog,
                          callback_group=self.watchdog_group)
        self.create_timer(0.05, self.control_tick,
                          callback_group=self.control_group)

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def source_fresh(self, source, receipt):
        now = self.ros_now(); mono = time.monotonic()
        return (-0.02 <= now - source <= TIMEOUT + SOURCE_STAMP_TOLERANCE
                and 0.0 <= mono - receipt <= TIMEOUT)

    def base_ready(self):
        with self.input_lock:
            mono = time.monotonic()
            return (self.source_fresh(self.pose_stamp, self.pose_receipt)
                    and self.source_fresh(self.scan_stamp, self.scan_receipt)
                    and self.source_fresh(self.odom_stamp, self.odom_receipt)
                    and self.quality
                    and 0.0 <= mono - self.quality_receipt <= TIMEOUT)

    def scan_cb(self, message):
        try:
            source = stamp(message)
            if (message.header.frame_id != 'laser' or not math.isfinite(source)
                    or source <= self.scan_stamp or not math.isfinite(message.angle_min)
                    or not math.isfinite(message.angle_increment) or message.angle_increment <= 0.0
                    or not 0.0 <= message.range_min < message.range_max
                    or not math.isfinite(message.range_max)):
                raise ValueError
            with self.input_lock:
                self.scan = message
                self.scan_stamp = source
                self.scan_receipt = time.monotonic()
        except ValueError:
            with self.input_lock:
                self.scan = None
            self.publish_stop('scan_invalid')

    def odom_cb(self, message):
        try:
            source = stamp(message)
            speed = float(message.twist.twist.linear.x)
            lateral_speed = float(message.twist.twist.linear.y)
            yaw_rate = float(message.twist.twist.angular.z)
            yaw_checked(message.pose.pose.orientation)
            if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link'
                    or not math.isfinite(source) or source <= self.odom_stamp
                    or not all(math.isfinite(v) for v in (speed, lateral_speed, yaw_rate))
                    or abs(lateral_speed) > TRACKER_CONFIG['maximum_lateral_speed_mps']
                    or abs(yaw_rate) > TRACKER_CONFIG['maximum_yaw_rate_rad_s']):
                raise ValueError
            with self.input_lock:
                self.odom_stamp = source
                self.odom_receipt = time.monotonic()
                self.measured_speed = abs(speed)
                self.measured_lateral_speed = lateral_speed
                self.measured_yaw_rate = yaw_rate
        except ValueError:
            with self.input_lock:
                self.odom_receipt = -math.inf
            self.publish_stop('odom_invalid')

    def quality_cb(self, message):
        with self.input_lock:
            self.quality = bool(message.data)
            self.quality_receipt = time.monotonic()
            quality = self.quality
        if not quality:
            self.publish_stop('localization_invalid')

    def pose_values(self, message):
        source = stamp(message)
        heading = yaw_checked(message.pose.pose.orientation)
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if (message.header.frame_id != 'map'
                or message.child_frame_id != 'base_link'
                or not all(math.isfinite(v) for v in
                           (source, x, y, heading))):
            raise ValueError('pose_invalid')
        return source, x, y, heading

    def record_latest_pose(self, source, x, y, heading, receipt):
        """Record one pose once even though two callback groups receive it."""
        with self.input_lock:
            if source <= self.pose_stamp:
                return False
            self.pose_stamp = source
            self.pose_receipt = receipt
            self.latest_pose = (source, x, y, heading)
            self.pose_history.add(source, x, y, heading)
            return True

    def pose_input_cb(self, message):
        """Keep localization freshness live while the planner is computing."""
        receipt = time.monotonic()
        try:
            source, x, y, heading = self.pose_values(message)
            if source <= self.pose_input_stream_stamp:
                raise ValueError('pose_invalid')
            self.pose_input_stream_stamp = source
            self.record_latest_pose(source, x, y, heading, receipt)
        except ValueError:
            with self.input_lock:
                self.pose_receipt = -math.inf
            self.publish_stop('pose_invalid')

    def enable_cb(self, message):
        with self.input_lock:
            self.enabled = bool(message.data)
            self.enable_receipt = time.monotonic()
            enabled = self.enabled
        if not enabled:
            with self.command_lock:
                self.reset_tracker()
                self.last_command_speed = 0.0
                self.last_command_angle = 0.0
                self.last_control_receipt = -math.inf
            if self.last_safe_path is not None and self.base_ready():
                self.publish(0.0, 0.0, True, 'waiting_for_operator', self.last_result)
            else:
                self.publish_stop('waiting_for_inputs')

    def scan_points(self, scan_message, x, y, heading):
        lx = x + LASER_OFFSET_X * math.cos(heading)
        ly = y + LASER_OFFSET_X * math.sin(heading)
        points = []
        directions = []
        beam_indices = []
        for index, raw in enumerate(scan_message.ranges):
            distance = float(raw)
            if (not math.isfinite(distance) or distance < scan_message.range_min
                    or distance > min(scan_message.range_max, 7.0)):
                continue
            angle = (heading + scan_message.angle_min
                     + index * scan_message.angle_increment)
            points.append((lx + distance * math.cos(angle), ly + distance * math.sin(angle)))
            directions.append((math.cos(angle), math.sin(angle)))
            beam_indices.append(index)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        directions = np.asarray(directions, dtype=float).reshape(-1, 2)
        beam_indices = np.asarray(beam_indices, dtype=int)
        dynamic, keep = remove_map_wall_returns(
            self.planner.static_map, points, directions, return_mask=True)
        clusters = cluster_scan_points(
            dynamic, beam_indices[keep],
            maximum_point_gap_m=CLUSTER_MAX_POINT_GAP_M,
            maximum_beam_gap=CLUSTER_MAX_BEAM_GAP,
            minimum_points=CLUSTER_MIN_POINTS,
            minimum_extent_m=CLUSTER_MIN_EXTENT_M,
            maximum_extent_m=CLUSTER_MAX_EXTENT_M,
        )
        tracks = self.obstacle_tracker.update(clusters, stamp(scan_message))
        confirmed = confirmed_points(tracks)
        self.raw_scan_point_count = len(points)
        self.dynamic_scan_point_count = len(dynamic)
        self.confirmed_dynamic_point_count = len(confirmed)
        self.detected_cluster_count = len(clusters)
        self.confirmed_cluster_count = len(tracks)
        self.confirmed_tracks = tracks
        self.nearest_dynamic_point_m = (
            float(np.min(np.linalg.norm(confirmed - np.array([x, y]), axis=1)))
            if len(confirmed) else None
        )
        self.publish_obstacle_markers(tracks)
        return confirmed

    def publish_obstacle_markers(self, tracks):
        message = MarkerArray()
        clear = Marker()
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        for track in tracks:
            marker = Marker()
            marker.header = clear.header
            marker.ns = 'confirmed_obstacles'
            marker.id = int(track.track_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(track.centroid[0])
            marker.pose.position.y = float(track.centroid[1])
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            span = np.ptp(track.points, axis=0) if len(track.points) else np.zeros(2)
            marker.scale.x = float(max(0.05, span[0] + 0.04))
            marker.scale.y = float(max(0.05, span[1] + 0.04))
            marker.scale.z = 0.20
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.80
            message.markers.append(marker)
        self.obstacle_pub.publish(message)

    def path_message(self, path):
        message = PathMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        for px, py, heading in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.orientation.z = math.sin(float(heading) / 2.0)
            pose.pose.orientation.w = math.cos(float(heading) / 2.0)
            message.poses.append(pose)
        return message

    def control(self, path, x, y, heading, now):
        remaining = max(0.0, self.lap.route_length_m - self.lap.distance_m)
        terminal_error = (endpoint_pose_error(self.planner.route,x,y,heading)
                          if remaining <= TERMINAL_TRACKER_TOLERANCE_M else (math.inf,math.inf))
        if endpoint_speed_limit(remaining) == 0.0:
            if not endpoint_pose_matches(*terminal_error):
                raise ValueError('endpoint_pose_mismatch')
            self.complete = True
            self.reset_tracker()
            return 0.0, 0.0
        with self.input_lock:
            speed_state = self.measured_speed
            lateral_state = self.measured_lateral_speed
            yaw_rate_state = self.measured_yaw_rate
        try:
            with self.tracker_lock:
                # Solve-start intervals include the previous solve time. The
                # publication timestamp belongs only to the command lease.
                dt = (0.05 if self.last_tracker_tick is None else
                      max(0.0, min(0.05, now-self.last_tracker_tick)))
                self.last_angle = self.tracker.compute(
                    path, x, y, heading, dt,
                    speed_mps=speed_state, lateral_speed_mps=lateral_state,
                    yaw_rate_rad_s=yaw_rate_state,
                )
                self.last_tracker_tick = now
        except ValueError as exc:
            endpoint_distance = float(np.hypot(
                self.planner.route[-1, 0] - x,
                self.planner.route[-1, 1] - y,
            ))
            if terminal_tracker_stop_allowed(
                    str(exc), remaining, endpoint_distance, terminal_error[1]):
                self.complete = True
                self.reset_tracker()
                return 0.0, 0.0
            raise
        profile_query = min(
            self.lap.route_length_m - ENDPOINT_STOP_REMAINING_M,
            self.lap.distance_m + SPEED_PROFILE_LOOKAHEAD_M,
        )
        self.reference_profile_speed = float(np.interp(
            profile_query, self.profile_distance, self.profile_speed))
        curve_fraction = min(1.0, abs(self.last_angle) / MAX_STEERING)
        steering_limit = (
            MAX_SPEED - (MAX_SPEED-MIN_MOVING_SPEED)*curve_fraction
        )
        speed = min(self.reference_profile_speed, steering_limit)
        if self.lattice_state in ('AVOIDING', 'RETURNING'):
            speed = min(speed, AVOIDANCE_SPEED_MAX)
        elif self.lattice_state in ('CONFIRM_BLOCKAGE', 'WAIT_SCAN_POSE'):
            speed = min(speed, BLOCKAGE_CONFIRM_SPEED)
        endpoint_limit = endpoint_speed_limit(remaining)
        if endpoint_limit == 0.0:
            raise ValueError('endpoint_pose_mismatch')
        speed = min(speed, endpoint_limit)
        return max(0.0, speed), self.last_angle

    def reset_tracker(self):
        with self.tracker_lock:
            self.tracker.reset()
            self.last_angle = 0.0
            self.last_tracker_tick = None

    def plan_fresh(self, now):
        # Tracking cannot extend the lifetime of an unrefreshed plan. Each
        # in-flight search has its own 500 ms deadline, independent of commands.
        if self.plan_started is not None and not (
                math.isfinite(self.last_plan_finished)
                and 0.0 <= self.plan_started-self.last_plan_finished <= TIMEOUT):
            return False
        anchor = self.plan_started if self.plan_started is not None else self.last_plan_finished
        return math.isfinite(anchor) and 0.0 <= now-anchor <= TIMEOUT

    def control_tick(self):
        # Only this callback computes moving commands. The expensive lattice
        # search runs on a separate callback group and never takes this lock.
        with self.command_lock:
            try:
                now = time.monotonic()
                if not self.base_ready():
                    raise ValueError('inputs_not_ready')
                with self.input_lock:
                    enabled = self.enabled and now-self.enable_receipt <= TIMEOUT
                    pose = self.latest_pose
                if self.complete:
                    self.reset_tracker()
                    speed, angle, reason = 0.0, 0.0, 'route_complete'
                elif self.last_safe_path is None:
                    raise ValueError('waiting_for_plan')
                elif not enabled:
                    self.reset_tracker()
                    speed, angle, reason = 0.0, 0.0, 'waiting_for_operator'
                else:
                    if not self.plan_fresh(now):
                        raise ValueError('planning_timeout')
                    if pose is None:
                        raise ValueError('inputs_not_ready')
                    _, x, y, heading = pose
                    speed, angle = self.control(self.last_safe_path, x, y, heading, now)
                    # A slow solve must not publish after losing live inputs,
                    # operator enable, or the independent planning deadline.
                    if not self.base_ready():
                        raise ValueError('inputs_not_ready')
                    if not self.plan_fresh(time.monotonic()):
                        raise ValueError('planning_timeout')
                    with self.input_lock:
                        still_enabled = self.enabled and time.monotonic()-self.enable_receipt <= TIMEOUT
                    if not still_enabled:
                        self.reset_tracker()
                        speed, angle, reason = 0.0, 0.0, 'waiting_for_operator'
                    else:
                        reason = ('route_complete' if self.complete else
                                  'confirming_blockage' if self.lattice_state == 'CONFIRM_BLOCKAGE' else
                                  'waiting_for_scan_pose' if self.lattice_state == 'WAIT_SCAN_POSE' else
                                  'tracking_during_replan' if self.plan_started is not None else 'ok')
                self.last_command_speed = speed
                self.last_command_angle = angle
                self.last_control_receipt = time.monotonic()
                self.publish(speed, angle, True, reason, self.last_result)
            except ValueError as error:
                self.publish_stop(str(error))

    def pose_cb(self, message):
        now = time.monotonic()
        try:
            source, x, y, heading = self.pose_values(message)
            if source <= self.pose_control_stream_stamp:
                raise ValueError('pose_invalid')
            self.pose_control_stream_stamp = source
            with self.input_lock:
                self.record_latest_pose(source, x, y, heading, now)
                if self.latest_pose is None:
                    raise ValueError('inputs_not_ready')
                _, x, y, heading = self.latest_pose
                scan_message = self.scan
                scan_stamp = self.scan_stamp
            if not self.base_ready() or scan_message is None:
                raise ValueError('inputs_not_ready')
            if self.complete or not plan_due(
                    self.last_plan_finished, now, scan_stamp > self.last_planned_scan,
                    enabled=self.enabled, has_safe_path=self.last_safe_path is not None):
                return
            with self.command_lock:
                self.plan_started = time.monotonic()
                self.planning_phase = 'scan_alignment'
                self.current_plan_stages = {}
            plan_cpu_started = time.thread_time()
            try:
                with self.input_lock:
                    scan_x, scan_y, scan_heading, alignment_span = self.pose_history.at(scan_stamp)
            except ValueError as error:
                if str(error) == 'scan_pose_unsynchronized' and self.last_safe_path is not None:
                    # Waiting for interpolation never refreshes plan freshness.
                    with self.command_lock:
                        self.lattice_state = 'WAIT_SCAN_POSE'
                    return
                raise
            self.planning_phase = 'scan_processing'
            phase_started = time.monotonic()
            obstacles = self.scan_points(scan_message, scan_x, scan_y, scan_heading)
            self.current_plan_stages['scan_processing_s'] = time.monotonic()-phase_started
            self.planning_phase = 'lattice_search'
            phase_started = time.monotonic()
            result, candidates = self.planner.plan(
                x, y, obstacles,
                horizon_m=max(0.0, self.lap.route_length_m-self.lap.distance_m), heading=heading)
            self.current_plan_stages['lattice_search_s'] = time.monotonic()-phase_started
            self.current_plan_stages['compute_thread_cpu_s'] = time.thread_time()-plan_cpu_started
            duration = time.monotonic()-self.plan_started
            self.current_plan_stages['compute_wall_s'] = duration
            if duration > TIMEOUT:
                raise ValueError('planning_timeout')
            # Atomically install the new result between tracker ticks. The
            # expensive computation above leaves the controller lock free.
            self.planning_phase = 'install_wait'
            install_wait_started = time.monotonic()
            with self.command_lock:
                self.current_plan_stages['install_wait_s'] = time.monotonic()-install_wait_started
                self.planning_phase = 'install'
                self.last_plan_duration_s = duration
                self.scan_pose_alignment_span_s = alignment_span
                self.last_candidate_reasons = {}
                for candidate in candidates:
                    self.last_candidate_reasons[candidate.reason] = self.last_candidate_reasons.get(candidate.reason, 0)+1
                self.last_planned_scan = scan_stamp
                previous_offset = self.last_result.offset_m if self.last_result is not None else 0.0
                self.lap.update(self.planner.progress_index)
                terminal_error = endpoint_pose_error(self.planner.route,x,y,heading)
                if (endpoint_speed_limit(self.lap.route_length_m-self.lap.distance_m) == 0.0
                        and not endpoint_pose_matches(*terminal_error)):
                    raise ValueError('endpoint_pose_mismatch')
                if self.complete or open_endpoint_reached(
                        self.lap.closed_loop, self.lap.route_length_m, self.lap.distance_m,
                        *terminal_error):
                    self.complete = True
                    self.last_result = result
                    self.lattice_state = 'FOLLOWING'
                    self.reset_tracker()
                    self.last_command_speed = self.last_command_angle = 0.0
                    self.last_control_receipt = time.monotonic()
                    self.publish(0.0, 0.0, True, 'route_complete', result)
                else:
                    decision = self.blockage_confirmation.update(
                        result.valid, result.reason, time.monotonic(),
                        nearest_obstacle_m=self.nearest_dynamic_point_m)
                    if result.valid and len(result.path) >= 2:
                        self.last_result = result
                        self.last_safe_path = result.path.copy()
                        self.path_pub.publish(self.path_message(self.last_safe_path))
                        if self.planner.mode == 'AVOIDING':
                            self.lattice_state = 'AVOIDING'
                        elif self.planner.mode == 'RETURNING' or abs(previous_offset) > 1e-9:
                            self.lattice_state = 'RETURNING'
                        else:
                            self.lattice_state = 'FOLLOWING'
                    elif decision == 'confirming' and self.last_safe_path is not None:
                        self.lattice_state = 'CONFIRM_BLOCKAGE'
                    else:
                        self.last_result = result
                        self.lattice_state = 'BLOCKED'
                        self.last_safe_path = None
                        self.publish_stop(result.reason)
                self.last_plan_finished = time.monotonic()
                self.current_plan_stages['total_wall_s'] = self.last_plan_finished-self.plan_started
                self.last_plan_stages = dict(self.current_plan_stages)
                self.plan_started = None
        except ValueError as error:
            with self.command_lock:
                self.last_safe_path = None
                self.publish_stop(str(error))
        finally:
            with self.command_lock:
                self.plan_started = None
                self.planning_phase = 'idle'

    def ages(self):
        n = time.monotonic(); r = self.ros_now()
        def one(source, receipt):
            return {'source_age_s': r-source if math.isfinite(source) else 1e9,
                    'receipt_age_s': n-receipt if math.isfinite(receipt) else 1e9,
                    'timeout_s': TIMEOUT}
        return {'pose': one(self.pose_stamp,self.pose_receipt),
                'scan': one(self.scan_stamp,self.scan_receipt),
                'odom': one(self.odom_stamp,self.odom_receipt)}

    def publish(self, speed, angle, valid, reason, result=None):
        with self.command_lock:
            if not all(math.isfinite(v) for v in (speed,angle)) or not 0.0 <= speed <= MAX_SPEED or abs(angle)>MAX_STEERING:
                speed=angle=0.0;valid=False;reason='output_invalid'
            self.publish_command(speed, angle, valid)
            self.sequence += 1
            latest = self.latest_pose
            terminal_error = endpoint_pose_error(self.planner.route,*latest[1:]) if latest else (None,None)
            data={'valid':bool(valid),'complete':bool(self.complete),'reason':reason,
              'controller':CONTROLLER_NAME,'tracker_config':TRACKER_CONFIG,
              'planner_tuning':PLANNER_TUNING,
              'tracker_diagnostics':dict(self.tracker.last),
              'speed_command':float(speed),
              'steering_command':float(angle),'progress_index':int(self.planner.progress_index),
              'reference_profile_speed_mps':float(self.reference_profile_speed),
              'event':'pose_callback' if speed or valid else 'stop_watchdog',
              'state':self.lattice_state,
              'side':('left' if result and result.offset_m > 0.0 else
                      'right' if result and result.offset_m < 0.0 else 'center'),
              'offset_m':float(result.offset_m) if result else 0.0,
              'transition_m':float(result.transition_m) if result else 0.0,
              'return_m':float(result.return_m) if result else 0.0,
              'lateral_jerk_cost':float(result.lateral_jerk_cost) if result else 0.0,
              'candidate_score':float(result.score) if result and math.isfinite(result.score) else None,
              'candidate_score_terms':result.score_terms if result and result.score_terms else {},
              'candidate_switch_count':int(self.planner.candidate_switch_count),
              'commit_age_scans':int(self.planner.commit_age_scans),
              'switch_reason':self.planner.last_switch_reason,
              'center_path_reused':bool(self.planner.center_path_reused),
              'endpoint_distance_m':terminal_error[0],
              'endpoint_heading_error_rad':terminal_error[1],
              'local_path_endpoint_xy':result.path[-1,:2].tolist() if result and len(result.path) else None,
              'footprint_collision':bool(result.footprint_collision) if result else False,
              'minimum_obstacle_clearance_m':(
                  float(result.minimum_obstacle_clearance_m)
                  if result and math.isfinite(result.minimum_obstacle_clearance_m) else None),
              'minimum_static_clearance_m':(
                  float(result.minimum_static_clearance_m)
                  if result and math.isfinite(result.minimum_static_clearance_m) else None),
              'route_mode':self.planner.route_mode,
              'lap_progress_m':float(self.lap.distance_m),
              'lap_length_m':float(self.lap.route_length_m),
              'blocked_confirmation_scans':int(self.blockage_confirmation.count),
              'blocked_confirmation_elapsed_s':float(
                  self.blockage_confirmation.elapsed(time.monotonic())),
              'blocked_confirmation_required_s':float(
                  self.blockage_confirmation.active_required_seconds),
              'near_block_distance_m':float(
                  self.blockage_confirmation.immediate_distance_m),
              'scan_pose_alignment_span_s':self.scan_pose_alignment_span_s,
              'candidate_reason_counts':self.last_candidate_reasons,
              'raw_scan_point_count':int(self.raw_scan_point_count),
              'dynamic_scan_point_count':int(self.dynamic_scan_point_count),
              'confirmed_dynamic_point_count':int(self.confirmed_dynamic_point_count),
              'detected_cluster_count':int(self.detected_cluster_count),
              'confirmed_cluster_count':int(self.confirmed_cluster_count),
              'confirmed_obstacle_tracks':[
                  {'id':int(track.track_id),'hits':int(track.hits),
                   'misses':int(track.misses),
                   'x':float(track.centroid[0]),'y':float(track.centroid[1]),
                   'extent_m':float(track.extent_m)}
                  for track in self.confirmed_tracks],
              'nearest_dynamic_point_m':self.nearest_dynamic_point_m,
              'plan_interval_s':PLAN_INTERVAL_S,
              'control_period_s':0.05,
              'planning_active':self.plan_started is not None,
              'planning_age_s':(time.monotonic()-self.plan_started if self.plan_started is not None else None),
              'last_plan_duration_s':self.last_plan_duration_s,
              'planning_phase':self.planning_phase,
              'last_plan_stages':dict(self.last_plan_stages),
              'current_plan_stages':dict(self.current_plan_stages),
              'runtime_timing':{'sequence':self.sequence,'tick_start_mono':time.monotonic(),'publish_mono':time.monotonic()},
              'input_age':self.ages()}
            self.status_pub.publish(String(data=json.dumps(data,allow_nan=False,separators=(',',':'))))

    def publish_command(self, speed, angle, valid):
        drive=AckermannDriveStamped();drive.header.stamp=self.get_clock().now().to_msg();drive.header.frame_id='base_link'
        drive.drive.speed=float(speed);drive.drive.steering_angle=float(angle);self.drive_pub.publish(drive)
        self.valid_pub.publish(Bool(data=bool(valid)))

    def publish_stop(self, reason):
        with self.command_lock:
            self.reset_tracker()
            self.last_command_speed=0.0
            self.last_command_angle=0.0
            self.last_control_receipt=-math.inf
            self.publish(0.0,0.0,False,reason,self.last_result)

    def watchdog(self):
        if not self.base_ready():
            self.publish_stop('input_stale')
            return
        with self.input_lock:
            enabled = self.enabled
            enable_age = time.monotonic() - self.enable_receipt
        if not enabled or enable_age > TIMEOUT:
            if self.last_safe_path is not None:
                self.publish(0.0,0.0,True,'waiting_for_operator',self.last_result)
            else:
                self.publish_stop('waiting_for_plan')
        else:
            with self.command_lock:
                if not self.complete and not self.plan_fresh(time.monotonic()):
                    self.publish_stop('planning_timeout')
                    return
                command_age = time.monotonic() - self.last_control_receipt
                if command_heartbeat_allowed(
                        enabled, self.last_safe_path is not None, command_age):
                    # The tracker refreshes commands independently of planning.
                    # Both plan and command deadlines must remain satisfied.
                    self.publish_command(self.last_command_speed,
                                         self.last_command_angle, True)
                else:
                    self.publish_stop('waiting_for_plan')


def main():
    domain=os.environ.get('ROS_DOMAIN_ID'); local=os.environ.get('ROS_LOCALHOST_ONLY')
    offline=domain=='83'
    ground=(domain=='85' and os.environ.get('F1TENTH_GROUND_AVOIDANCE_TEST')=='bounded_25s_L1')
    if local!='1' or (not offline and not ground):
        raise SystemExit('REFUSED: domain 83 offline or explicit domain 85 avoidance permit required')
    rclpy.init()
    node=AvoidanceController()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.publish_stop('shutdown');executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
