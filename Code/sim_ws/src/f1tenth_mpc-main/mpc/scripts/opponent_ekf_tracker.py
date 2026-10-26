#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class OpponentEKFTracker(Node):
    def __init__(self):
        super().__init__('opp_ekf_tracker')

        self.declare_parameter('measurement_source', 'lidar')
        self.declare_parameter('measurement_topic', '/ego_racecar/opp_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ego_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_topic', '/ego_racecar/opp_odom_ekf')
        self.declare_parameter('output_pose_topic', '/ego_racecar/opp_odom_ekf_pose')
        self.declare_parameter('status_topic', '/ego_tracker/status')
        self.declare_parameter('process_noise_pos', 0.05)
        self.declare_parameter('process_noise_vel', 0.5)
        self.declare_parameter('measurement_noise_pos', 0.08)
        self.declare_parameter('measurement_noise_vel', 0.8)
        self.declare_parameter('lidar_fov_deg', 120.0)
        self.declare_parameter('tracked_lidar_fov_deg', 0.0)
        self.declare_parameter('tracked_min_forward_x', -1.5)
        self.declare_parameter('tracked_max_cluster_points', 300)
        self.declare_parameter('tracked_geometry_timeout', 0.3)
        self.declare_parameter('lidar_min_range', 0.2)
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('cluster_gap', 0.35)
        self.declare_parameter('min_cluster_points', 9)
        self.declare_parameter('max_cluster_points', 100)
        self.declare_parameter('min_cluster_width', 0.05)
        self.declare_parameter('max_cluster_width', 0.75)
        self.declare_parameter('association_gate', 1.2)
        self.declare_parameter('reacquisition_gate', 1.5)
        self.declare_parameter('reacquisition_after_missed_scans', 3)
        self.declare_parameter('association_sigma', 3.0)
        # Prediction output is opt-in per launch so older consumers that only
        # check message reception time do not silently weaken fail-stop timing.
        self.declare_parameter('prediction_timeout', 0.0)
        self.declare_parameter('global_reacquisition_after', 1.5)
        self.declare_parameter('global_reacquisition_confirmation_scans', 4)
        self.declare_parameter('global_reacquisition_track_radius', 0.35)
        self.declare_parameter('global_reacquisition_min_speed', 0.18)
        self.declare_parameter('global_reacquisition_min_observation_time', 0.35)
        self.declare_parameter('global_reacquisition_max_motion_rmse', 0.08)
        self.declare_parameter('global_reacquisition_existence_threshold', 0.60)
        self.declare_parameter('target_center_offset', 0.29)
        self.declare_parameter('max_target_speed', 3.0)
        self.declare_parameter('min_forward_x', -0.5)
        self.declare_parameter('static_map_assets', '')
        self.declare_parameter('scan_origin_x', 0.275)
        # Disabled by default. These deterministic simulation-only hooks act
        # on the LiDAR measurement path; they never substitute opponent truth.
        self.declare_parameter('simulation_seed', 0)
        self.declare_parameter('simulation_measurement_noise_std', 0.0)
        self.declare_parameter('simulation_dropout_start_s', -1.0)
        self.declare_parameter('simulation_dropout_duration_s', 0.0)

        measurement_source = self.get_parameter('measurement_source').value
        measurement_topic = self.get_parameter('measurement_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        ego_odom_topic = self.get_parameter('ego_odom_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, output_pose_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        if measurement_source == 'lidar':
            self.scan_sub = self.create_subscription(
                LaserScan,
                scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )
            self.ego_odom_sub = self.create_subscription(Odometry, ego_odom_topic, self.ego_odom_callback, 10)
        else:
            self.sub = self.create_subscription(Odometry, measurement_topic, self.odom_callback, 10)

        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 0.5
        self.last_stamp = None
        self.initialized = False
        self.ego_odom = None
        self.last_detection_stamp = None
        self.lidar_fov = math.radians(float(self.get_parameter('lidar_fov_deg').value))
        self.tracked_lidar_fov = math.radians(float(self.get_parameter('tracked_lidar_fov_deg').value))
        self.tracked_min_forward_x = float(self.get_parameter('tracked_min_forward_x').value)
        self.tracked_max_cluster_points = int(self.get_parameter('tracked_max_cluster_points').value)
        self.tracked_geometry_timeout = float(self.get_parameter('tracked_geometry_timeout').value)
        self.lidar_min_range = float(self.get_parameter('lidar_min_range').value)
        self.lidar_max_range = float(self.get_parameter('lidar_max_range').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.min_cluster_width = float(self.get_parameter('min_cluster_width').value)
        self.max_cluster_width = float(self.get_parameter('max_cluster_width').value)
        self.association_gate = float(self.get_parameter('association_gate').value)
        self.reacquisition_gate = max(
            self.association_gate,
            float(self.get_parameter('reacquisition_gate').value),
        )
        self.reacquisition_after_missed_scans = max(
            1, int(self.get_parameter('reacquisition_after_missed_scans').value)
        )
        self.association_sigma = max(
            1.0, float(self.get_parameter('association_sigma').value)
        )
        self.prediction_timeout = max(
            0.0, float(self.get_parameter('prediction_timeout').value)
        )
        self.global_reacquisition_after = max(
            self.prediction_timeout,
            float(self.get_parameter('global_reacquisition_after').value),
        )
        self.global_reacquisition_confirmation_scans = max(
            2,
            int(self.get_parameter(
                'global_reacquisition_confirmation_scans'
            ).value),
        )
        self.global_reacquisition_track_radius = max(
            0.1,
            float(self.get_parameter('global_reacquisition_track_radius').value),
        )
        self.global_reacquisition_min_speed = max(
            0.0,
            float(self.get_parameter('global_reacquisition_min_speed').value),
        )
        self.global_reacquisition_min_observation_time = max(
            0.1,
            float(self.get_parameter(
                'global_reacquisition_min_observation_time'
            ).value),
        )
        self.global_reacquisition_max_motion_rmse = max(
            0.01,
            float(self.get_parameter(
                'global_reacquisition_max_motion_rmse'
            ).value),
        )
        self.global_reacquisition_existence_threshold = float(np.clip(
            self.get_parameter(
                'global_reacquisition_existence_threshold'
            ).value,
            0.0,
            1.0,
        ))
        self.target_center_offset = max(
            0.0, float(self.get_parameter('target_center_offset').value)
        )
        self.max_target_speed = max(
            0.1, float(self.get_parameter('max_target_speed').value)
        )
        self.min_forward_x = float(self.get_parameter('min_forward_x').value)
        self.static_map = None
        assets_path = str(self.get_parameter('static_map_assets').value)
        if assets_path:
            with open(assets_path, encoding='utf-8') as handle:
                self.static_map = json.load(handle)
            self.static_map['clearance'] = np.asarray(self.static_map['clearance'])
            self.static_map['route_xy'] = np.asarray(self.static_map['route'])[:, :2]
        self.scan_origin_x = float(self.get_parameter('scan_origin_x').value)
        self.missed_lidar_scans = 0
        self.reacquisition_tracks = []
        self.track_existence_probability = 0.0
        self.simulation_rng = np.random.default_rng(
            int(self.get_parameter('simulation_seed').value)
        )
        self.simulation_measurement_noise_std = max(
            0.0,
            float(self.get_parameter('simulation_measurement_noise_std').value),
        )
        self.simulation_dropout_start_s = float(
            self.get_parameter('simulation_dropout_start_s').value
        )
        self.simulation_dropout_duration_s = max(
            0.0,
            float(self.get_parameter('simulation_dropout_duration_s').value),
        )
        self.simulation_start_stamp = None

        q_pos = float(self.get_parameter('process_noise_pos').value)
        q_vel = float(self.get_parameter('process_noise_vel').value)
        r_pos = float(self.get_parameter('measurement_noise_pos').value)
        r_vel = float(self.get_parameter('measurement_noise_vel').value)
        self.Q_base = np.diag([q_pos, q_pos, q_vel, q_vel])
        self.R_position = np.diag([r_pos, r_pos])
        self.H_position = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ]
        )
        # Retained for the optional odometry measurement mode only. The pure
        # LiDAR baseline estimates velocity through the position-state EKF.
        self.R_full = np.diag([r_pos, r_pos, r_vel, r_vel])

    def odom_callback(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        z = np.array(
            [
                [msg.pose.pose.position.x],
                [msg.pose.pose.position.y],
                [msg.twist.twist.linear.x],
                [msg.twist.twist.linear.y],
            ]
        )
        self.update_filter(z, stamp)
        self.last_detection_stamp = stamp
        self.publish_estimate(msg.header.frame_id, msg.child_frame_id or 'opp_racecar/base_link', msg.header.stamp)
        self.publish_status('TRACKED', stamp, {'miss_reason': 'none'})

    def ego_odom_callback(self, msg: Odometry) -> None:
        self.ego_odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        if self.ego_odom is None:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.simulation_start_stamp is None:
            self.simulation_start_stamp = stamp
        if self.initialized:
            self.predict_to(stamp)

        elapsed = stamp - self.simulation_start_stamp
        injected_dropout = (
            self.simulation_dropout_start_s >= 0.0
            and self.simulation_dropout_start_s <= elapsed
            < self.simulation_dropout_start_s + self.simulation_dropout_duration_s
        )
        if injected_dropout:
            detection = None
            diagnostics = {
                'miss_reason': 'simulation_injected_dropout',
                'raw_cluster_count': 0,
                'candidate_count': 0,
                'association_error_m': None,
                'association_gate_m': None,
                'reacquisition_confirmation_count': 0,
                'reacquisition_motion_mps': None,
            }
        else:
            detection, diagnostics = self.extract_lidar_target(msg, stamp)
            if detection is not None and self.simulation_measurement_noise_std > 0.0:
                noise = self.simulation_rng.normal(
                    0.0, self.simulation_measurement_noise_std, size=2
                )
                detection = (detection[0] + noise[0], detection[1] + noise[1])
                diagnostics['simulation_noise_std_m'] = (
                    self.simulation_measurement_noise_std
                )
        if detection is None:
            self.missed_lidar_scans += 1
            # Gym can publish scans far faster than a physical LiDAR. Keep the
            # existence decay conservative per callback so it is not tied to a
            # particular scan rate.
            self.track_existence_probability *= 0.999
            measurement_age = self.measurement_age(stamp)
            if self.initialized and measurement_age <= self.prediction_timeout:
                self.publish_estimate('map', 'opp_racecar/base_link', msg.header.stamp)
                mode = 'COASTING'
            else:
                mode = 'LOST' if self.initialized else 'UNINITIALIZED'
            self.publish_status(mode, stamp, diagnostics)
            return
        self.missed_lidar_scans = 0

        rel_x, rel_y = detection
        ego_yaw = self.yaw_from_odom(self.ego_odom)
        ego_x = self.ego_odom.pose.pose.position.x
        ego_y = self.ego_odom.pose.pose.position.y
        global_x = ego_x + math.cos(ego_yaw) * rel_x - math.sin(ego_yaw) * rel_y
        global_y = ego_y + math.sin(ego_yaw) * rel_x + math.cos(ego_yaw) * rel_y

        global_reacquisition = (
            diagnostics.get('miss_reason') == 'global_reacquisition'
        )
        if not self.initialized or global_reacquisition:
            self.x = np.array([[global_x], [global_y], [0.0], [0.0]])
            if global_reacquisition:
                self.x[2, 0] = float(diagnostics.get('reacquisition_vx_mps', 0.0))
                self.x[3, 0] = float(diagnostics.get('reacquisition_vy_mps', 0.0))
            self.P = np.diag([0.1, 0.1, 0.5, 0.5])
            self.last_stamp = stamp
            self.initialized = True
        else:
            self.correct_position(np.array([[global_x], [global_y]]))
        self.track_existence_probability = min(
            0.995,
            max(self.track_existence_probability, 0.65)
            + 0.12 * (1.0 - max(self.track_existence_probability, 0.65)),
        )
        self.reacquisition_tracks = []
        self.last_detection_stamp = stamp
        self.publish_estimate('map', 'opp_racecar/base_link', msg.header.stamp)
        if diagnostics.get('miss_reason') != 'global_reacquisition':
            diagnostics['miss_reason'] = 'none'
        self.publish_status('TRACKED', stamp, diagnostics)

    def extract_lidar_target(self, msg: LaserScan, stamp: float):
        diagnostics = {
            'miss_reason': 'none',
            'raw_cluster_count': 0,
            'candidate_count': 0,
            'association_error_m': None,
            'association_gate_m': None,
            'reacquisition_confirmation_count': 0,
            'reacquisition_motion_mps': None,
        }
        points = []
        # Rear/side association is opt-in and requires a recently measured
        # identity. Initial/global acquisition retains the original front mask.
        # A fully visible side panel also spans more beams than a rear face.
        established = (self.tracked_lidar_fov > 0.0 and self.initialized and
                       self.last_detection_stamp is not None and
                       0.0 <= stamp-self.last_detection_stamp <= self.tracked_geometry_timeout)
        half_fov = (max(self.lidar_fov,self.tracked_lidar_fov) if established else self.lidar_fov)*.5
        max_points = max(self.max_cluster_points,self.tracked_max_cluster_points) if established else self.max_cluster_points
        min_x = min(self.min_forward_x,self.tracked_min_forward_x) if established else self.min_forward_x
        diagnostics['established_track_geometry'] = established
        diagnostics['active_fov_deg'] = math.degrees(2*half_fov)
        diagnostics['active_max_cluster_points'] = max_points
        for idx, dist in enumerate(msg.ranges):
            if not math.isfinite(dist):
                continue
            if dist < self.lidar_min_range or dist > self.lidar_max_range:
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            if abs(angle) > half_fov:
                continue
            points.append((dist * math.cos(angle), dist * math.sin(angle)))

        if len(points) < self.min_cluster_points:
            diagnostics['miss_reason'] = 'insufficient_points'
            return None, diagnostics

        clusters = []
        current = [points[0]]
        for point in points[1:]:
            prev = current[-1]
            if math.hypot(point[0] - prev[0], point[1] - prev[1]) <= self.cluster_gap:
                current.append(point)
            else:
                if len(current) >= self.min_cluster_points:
                    clusters.append(current)
                current = [point]
        if len(current) >= self.min_cluster_points:
            clusters.append(current)

        if not clusters:
            diagnostics['miss_reason'] = 'no_clusters'
            return None, diagnostics
        diagnostics['raw_cluster_count'] = len(clusters)

        candidates = []
        for cluster in clusters:
            if len(cluster) > max_points:
                continue
            if self.static_map is not None:
                # Reject returns belonging to the known static map, before
                # the target-centre correction. Map data is independent of
                # opponent ground truth; sensor extrinsics apply to raw hits.
                clear = self.static_map
                hits = 0
                for px, py in cluster:
                    gx, gy = self.ego_to_global_frame(px + self.scan_origin_x, py)
                    col = int(math.floor((gx - clear['origin'][0]) / clear['resolution']))
                    row = clear['height'] - 1 - int(math.floor((gy - clear['origin'][1]) / clear['resolution']))
                    if not (0 <= col < clear['width'] and 0 <= row < clear['height']):
                        hits += 1
                    elif clear['clearance'][row * clear['width'] + col] < 0.15:
                        hits += 1
                if hits / len(cluster) >= 0.6:
                    continue
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            distance = math.hypot(cx, cy)
            width = self.cluster_width(cluster)
            if width < self.min_cluster_width or width > self.max_cluster_width:
                continue
            if cx <= min_x:
                continue
            if distance > 1e-6:
                cx += self.target_center_offset * cx / distance
                cy += self.target_center_offset * cy / distance
                distance += self.target_center_offset
            if self.static_map is not None:
                gx, gy = self.ego_to_global_frame(cx, cy)
                route_distance = np.min(np.linalg.norm(
                    self.static_map['route_xy'] - np.array([gx, gy]), axis=1))
                # The route-following experiment concerns the local driving
                # corridor, not segmented furniture across the open hall.
                if route_distance > 1.5:
                    continue
            candidates.append((distance, cx, cy))

        if not candidates:
            diagnostics['miss_reason'] = 'shape_or_forward_filter'
            return None, diagnostics
        diagnostics['candidate_count'] = len(candidates)

        if self.initialized and self.ego_odom is not None:
            pred_rel_x, pred_rel_y = self.global_to_ego_frame(float(self.x[0, 0]), float(self.x[1, 0]))
            gated = [
                (math.hypot(cx - pred_rel_x, cy - pred_rel_y), cx, cy)
                for _distance, cx, cy in candidates
            ]
            best_error, cx, cy = min(gated, key=lambda item: item[0])
            position_sigma = math.sqrt(
                max(float(np.max(np.linalg.eigvalsh(self.P[:2, :2]))), 0.0)
            )
            gate = max(self.association_gate, self.association_sigma * position_sigma)
            if self.missed_lidar_scans >= self.reacquisition_after_missed_scans:
                gate = max(gate, self.association_gate + 0.1 * self.missed_lidar_scans)
            gate = min(gate, self.reacquisition_gate)
            diagnostics['association_error_m'] = best_error
            diagnostics['association_gate_m'] = gate
            if best_error <= gate:
                self.reacquisition_tracks = []
                return (cx, cy), diagnostics

            reacquired = self.motion_confirmed_reacquisition(
                candidates, stamp, diagnostics
            )
            if (
                reacquired is not None
                and self.measurement_age(stamp) >= self.global_reacquisition_after
            ):
                return reacquired, diagnostics

            diagnostics['miss_reason'] = 'association_gate'
            return None, diagnostics

        _distance, cx, cy = min(candidates, key=lambda item: item[0])
        return (cx, cy), diagnostics

    def motion_confirmed_reacquisition(self, candidates, stamp, diagnostics):
        """Track rejected clusters in the world frame before reinitializing.

        A static compact obstacle can satisfy the LiDAR shape filter indefinitely.
        Reacquisition therefore requires a persistent candidate with measurable
        world-frame motion and a bounded existence score.  If identity cannot be
        established from LiDAR motion, the tracker remains LOST instead of
        selecting the nearest forward object.
        """
        world_candidates = []
        for distance, rel_x, rel_y in candidates:
            if rel_x <= 0.0:
                continue
            world_x, world_y = self.ego_to_global_frame(rel_x, rel_y)
            world_candidates.append({
                'distance': distance,
                'rel_x': rel_x,
                'rel_y': rel_y,
                'x': world_x,
                'y': world_y,
            })

        previous_tracks = [
            track for track in self.reacquisition_tracks
            if stamp - track['last_stamp'] <= 0.30
        ]
        matched_track_ids = set()
        updated_tracks = []
        for candidate in world_candidates:
            best_index = None
            best_distance = float('inf')
            for index, track in enumerate(previous_tracks):
                if index in matched_track_ids:
                    continue
                dt = max(stamp - track['last_stamp'], 1e-3)
                predicted_x = track['x'] + track['vx'] * dt
                predicted_y = track['y'] + track['vy'] * dt
                error = math.hypot(
                    candidate['x'] - predicted_x,
                    candidate['y'] - predicted_y,
                )
                match_radius = max(
                    self.global_reacquisition_track_radius,
                    1.25 * self.max_target_speed * dt,
                )
                if error <= match_radius and error < best_distance:
                    best_index = index
                    best_distance = error

            if best_index is None:
                updated_tracks.append({
                    **candidate,
                    'first_x': candidate['x'],
                    'first_y': candidate['y'],
                    'first_stamp': stamp,
                    'last_stamp': stamp,
                    'vx': 0.0,
                    'vy': 0.0,
                    'count': 1,
                    'existence_probability': 0.25,
                    'samples': [(stamp, candidate['x'], candidate['y'])],
                })
                continue

            matched_track_ids.add(best_index)
            track = previous_tracks[best_index]
            dt = max(stamp - track['last_stamp'], 1e-3)
            instant_vx = (candidate['x'] - track['x']) / dt
            instant_vy = (candidate['y'] - track['y']) / dt
            velocity_alpha = 0.45
            samples = list(track.get('samples', []))
            samples.append((stamp, candidate['x'], candidate['y']))
            # Retain a time window rather than a fixed sample count: Gym can
            # publish around 250 Hz while the physical LiDAR is much slower.
            samples = [
                sample for sample in samples
                if stamp - sample[0] <= 0.60
            ]
            updated_tracks.append({
                **candidate,
                'first_x': track['first_x'],
                'first_y': track['first_y'],
                'first_stamp': track['first_stamp'],
                'last_stamp': stamp,
                'vx': (
                    (1.0 - velocity_alpha) * track['vx']
                    + velocity_alpha * instant_vx
                ),
                'vy': (
                    (1.0 - velocity_alpha) * track['vy']
                    + velocity_alpha * instant_vy
                ),
                'count': track['count'] + 1,
                'existence_probability': min(
                    0.995,
                    track['existence_probability']
                    + 0.22 * (1.0 - track['existence_probability']),
                ),
                'samples': samples,
            })

        for index, track in enumerate(previous_tracks):
            if index in matched_track_ids:
                continue
            stale_track = dict(track)
            stale_track['existence_probability'] *= 0.75
            if stale_track['existence_probability'] >= 0.15:
                updated_tracks.append(stale_track)
        self.reacquisition_tracks = updated_tracks

        confirmed = []
        for track in updated_tracks:
            motion = self.fit_candidate_motion(track.get('samples', []))
            motion_speed = motion['speed']
            if (
                track['count'] >= self.global_reacquisition_confirmation_scans
                and track['existence_probability']
                >= self.global_reacquisition_existence_threshold
                and motion['duration']
                >= self.global_reacquisition_min_observation_time
                and motion_speed >= self.global_reacquisition_min_speed
                and motion['rmse']
                <= self.global_reacquisition_max_motion_rmse
            ):
                confirmed.append((track, motion))

        if not confirmed:
            if updated_tracks:
                strongest = max(
                    updated_tracks,
                    key=lambda item: (
                        item['existence_probability'], item['count']
                    ),
                )
                strongest_motion = self.fit_candidate_motion(
                    strongest.get('samples', [])
                )
                diagnostics['reacquisition_confirmation_count'] = strongest['count']
                diagnostics['reacquisition_motion_mps'] = strongest_motion['speed']
                diagnostics['reacquisition_motion_rmse_m'] = strongest_motion['rmse']
            return None

        predicted_x = float(self.x[0, 0])
        predicted_y = float(self.x[1, 0])
        predicted_vx = float(self.x[2, 0])
        predicted_vy = float(self.x[3, 0])

        def reacquisition_score(item):
            track, motion = item
            position_error = math.hypot(
                track['x'] - predicted_x, track['y'] - predicted_y
            )
            velocity_error = math.hypot(
                motion['vx'] - predicted_vx, motion['vy'] - predicted_vy
            )
            return position_error + 0.25 * velocity_error

        track, motion = min(confirmed, key=reacquisition_score)
        diagnostics['miss_reason'] = 'global_reacquisition'
        diagnostics['reacquisition_confirmation_count'] = track['count']
        diagnostics['reacquisition_motion_mps'] = motion['speed']
        diagnostics['reacquisition_motion_rmse_m'] = motion['rmse']
        diagnostics['reacquisition_vx_mps'] = motion['vx']
        diagnostics['reacquisition_vy_mps'] = motion['vy']
        diagnostics['track_existence_probability'] = track[
            'existence_probability'
        ]
        return track['rel_x'], track['rel_y']

    @staticmethod
    def fit_candidate_motion(samples):
        """Least-squares constant-velocity fit over a short world-frame track."""
        if len(samples) < 2:
            return {
                'vx': 0.0, 'vy': 0.0, 'speed': 0.0,
                'duration': 0.0, 'rmse': float('inf'),
            }
        values = np.asarray(samples, dtype=float)
        times = values[:, 0]
        duration = float(times[-1] - times[0])
        centered_times = times - float(np.mean(times))
        denominator = float(centered_times @ centered_times)
        if duration <= 0.0 or denominator <= 1e-9:
            return {
                'vx': 0.0, 'vy': 0.0, 'speed': 0.0,
                'duration': max(duration, 0.0), 'rmse': float('inf'),
            }
        mean_x = float(np.mean(values[:, 1]))
        mean_y = float(np.mean(values[:, 2]))
        vx = float(centered_times @ (values[:, 1] - mean_x) / denominator)
        vy = float(centered_times @ (values[:, 2] - mean_y) / denominator)
        predicted_x = mean_x + vx * centered_times
        predicted_y = mean_y + vy * centered_times
        residual_squared = (
            (values[:, 1] - predicted_x) ** 2
            + (values[:, 2] - predicted_y) ** 2
        )
        return {
            'vx': vx,
            'vy': vy,
            'speed': math.hypot(vx, vy),
            'duration': duration,
            'rmse': float(math.sqrt(np.mean(residual_squared))),
        }

    def cluster_width(self, cluster) -> float:
        max_width = 0.0
        for i, p0 in enumerate(cluster):
            for p1 in cluster[i + 1:]:
                max_width = max(max_width, math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        return max_width

    def global_to_ego_frame(self, global_x: float, global_y: float):
        ego_yaw = self.yaw_from_odom(self.ego_odom)
        ego_x = self.ego_odom.pose.pose.position.x
        ego_y = self.ego_odom.pose.pose.position.y
        dx = global_x - ego_x
        dy = global_y - ego_y
        rel_x = math.cos(ego_yaw) * dx + math.sin(ego_yaw) * dy
        rel_y = -math.sin(ego_yaw) * dx + math.cos(ego_yaw) * dy
        return rel_x, rel_y

    def ego_to_global_frame(self, rel_x: float, rel_y: float):
        ego_yaw = self.yaw_from_odom(self.ego_odom)
        ego_x = self.ego_odom.pose.pose.position.x
        ego_y = self.ego_odom.pose.pose.position.y
        return (
            ego_x + math.cos(ego_yaw) * rel_x - math.sin(ego_yaw) * rel_y,
            ego_y + math.sin(ego_yaw) * rel_x + math.cos(ego_yaw) * rel_y,
        )

    def predict_to(self, stamp: float) -> None:
        if not self.initialized:
            return
        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        dt = stamp - self.last_stamp
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)
        self.last_stamp = stamp
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q_base * max(dt, 0.05)
        self.limit_speed()

    def correct_position(self, z_position) -> None:
        innovation = z_position - self.H_position @ self.x
        innovation_covariance = (
            self.H_position @ self.P @ self.H_position.T + self.R_position
        )
        gain = self.P @ self.H_position.T @ np.linalg.inv(innovation_covariance)
        self.x = self.x + gain @ innovation
        self.P = (np.eye(4) - gain @ self.H_position) @ self.P
        self.limit_speed()

    def update_filter(self, z, stamp) -> None:
        if not self.initialized:
            self.x = z.copy()
            self.last_stamp = stamp
            self.initialized = True
            return
        self.predict_to(stamp)
        innovation = z - self.x
        innovation_covariance = self.P + self.R_full
        gain = self.P @ np.linalg.inv(innovation_covariance)
        self.x = self.x + gain @ innovation
        self.P = (np.eye(4) - gain) @ self.P
        self.limit_speed()

    def limit_speed(self) -> None:
        velocity = self.x[2:4, 0]
        speed = float(np.linalg.norm(velocity))
        if speed > self.max_target_speed:
            self.x[2:4, 0] = velocity * (self.max_target_speed / speed)

    def measurement_age(self, stamp: float) -> float:
        if self.last_detection_stamp is None:
            return float('inf')
        return max(0.0, stamp - self.last_detection_stamp)

    @staticmethod
    def json_number(value):
        if value is None or not math.isfinite(float(value)):
            return None
        return float(value)

    def publish_status(self, mode: str, stamp: float, diagnostics) -> None:
        status = {
            'mode': mode,
            'measurement_age_s': self.json_number(self.measurement_age(stamp)),
            'missed_scans': self.missed_lidar_scans,
            'miss_reason': diagnostics.get('miss_reason', 'none'),
            'raw_cluster_count': int(diagnostics.get('raw_cluster_count', 0)),
            'candidate_count': int(diagnostics.get('candidate_count', 0)),
            'association_error_m': self.json_number(
                diagnostics.get('association_error_m')
            ),
            'association_gate_m': self.json_number(
                diagnostics.get('association_gate_m')
            ),
            'position_covariance_trace': (
                float(np.trace(self.P[:2, :2])) if self.initialized else None
            ),
            'existence_probability': float(self.track_existence_probability),
            'reacquisition_confirmation_count': int(
                diagnostics.get('reacquisition_confirmation_count', 0)
            ),
            'reacquisition_motion_mps': self.json_number(
                diagnostics.get('reacquisition_motion_mps')
            ),
            'reacquisition_motion_rmse_m': self.json_number(
                diagnostics.get('reacquisition_motion_rmse_m')
            ),
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def publish_estimate(self, frame_id, child_frame_id, stamp) -> None:
        vx = float(self.x[2, 0])
        vy = float(self.x[3, 0])
        yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-4 else 0.0
        quat = Rotation.from_euler('z', yaw).as_quat()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = float(self.x[0, 0])
        odom.pose.pose.position.y = float(self.x[1, 0])
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.pose.covariance[0] = float(self.P[0, 0])
        odom.pose.covariance[1] = float(self.P[0, 1])
        odom.pose.covariance[6] = float(self.P[1, 0])
        odom.pose.covariance[7] = float(self.P[1, 1])
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.covariance[0] = float(self.P[2, 2])
        odom.twist.covariance[1] = float(self.P[2, 3])
        odom.twist.covariance[6] = float(self.P[3, 2])
        odom.twist.covariance[7] = float(self.P[3, 3])
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

    def yaw_from_odom(self, odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return quat.as_euler('zxy', degrees=False)[0]


def main(args=None):
    rclpy.init(args=args)
    node = OpponentEKFTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 launch may invalidate the context while a scan callback is
        # publishing.  Ignore only that teardown race.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
