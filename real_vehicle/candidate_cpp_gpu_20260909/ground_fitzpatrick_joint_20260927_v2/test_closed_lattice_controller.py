import sys
from pathlib import Path
import unittest
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
sys.path.insert(0, str(WORKSPACE / "candidate_cpp_gpu_20260909" / "open_route_frenet_v1_20260923"))

from avoidance_controller import (
    AVOIDANCE_SPEED_MAX, PLANNER_TUNING, AvoidanceController, BlockageConfirmation, LapProgress, PoseHistory, STATIC_ECHO_TOLERANCE,
    command_heartbeat_allowed, endpoint_speed_limit, load_speed_profile, plan_due,
    open_endpoint_reached, remove_map_wall_returns,
    terminal_tracker_stop_allowed,
)
from return_lattice_planner import ClosedLoopLatticePlanner
from obstacle_tracker import (
    PersistentObstacleTracker, cluster_scan_points, confirmed_points,
)


ROUTE = WORKSPACE / "routes/manual_route_20260923_172359_controller_ready.csv"
MAP = WORKSPACE / "maps/381_2_0.yaml"
SPEED_PROFILE = WORKSPACE / "routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv"
STATIC_TEST_SITES = {
    "A": (1.979, -0.217),
    "B": (2.972, -0.314),
    "C": (3.926, -0.044),
}


def square_box_points(center, half_size=0.08, samples=17):
    center = np.asarray(center, dtype=float)
    values = np.linspace(-half_size, half_size, samples)
    return np.vstack([
        np.c_[center[0] + values,
              np.full_like(values, center[1] - half_size)],
        np.c_[center[0] + values,
              np.full_like(values, center[1] + half_size)],
        np.c_[np.full_like(values, center[0] - half_size),
              center[1] + values],
        np.c_[np.full_like(values, center[0] + half_size),
              center[1] + values],
    ])


class ClosedLatticeGroundControllerTest(unittest.TestCase):
    def test_compact_bypass_keeps_footprint_clearance_and_escape_options(self):
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, static_echo_tolerance_m=.25,
            proactive_avoidance_clearance_m=.22, **PLANNER_TUNING)
        x, y, heading = planner.route[18]
        selected, candidates = planner.plan(
            x, y, square_box_points((2.8, -.19), half_size=.064), heading=heading)
        self.assertTrue(selected.valid)
        self.assertAlmostEqual(selected.offset_m, -.15)
        self.assertGreater(selected.minimum_obstacle_clearance_m, .15)
        self.assertFalse(selected.footprint_collision)
        self.assertTrue(any(abs(c.offset_m) >= .4 for c in candidates))
        self.assertAlmostEqual(planner.dynamic_clearance_m, .15)

    def test_tracker_interval_includes_previous_solve_time(self):
        tracker = Mock()
        tracker.compute.return_value = 0.0
        node = SimpleNamespace(
            last_pose_tick=None, last_tracker_tick=None,
            tracker_lock=threading.RLock(), input_lock=threading.RLock(), tracker=tracker,
            lap=SimpleNamespace(route_length_m=6.0, distance_m=1.0),
            measured_speed=1.0, measured_lateral_speed=0., measured_yaw_rate=0.,
            profile_distance=np.array([0., 6.]),
            profile_speed=np.array([1., 1.]), lattice_state='AVOIDING')
        path = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
        AvoidanceController.control(node, path, 0., 0., 0., 10.)
        # Publication occurs 35 ms after the preceding solve started.
        node.last_pose_tick = 10.035
        AvoidanceController.control(node, path, 0., 0., 0., 10.05)
        self.assertAlmostEqual(tracker.compute.call_args.args[4], .05)

    def test_mpc_avoidance_speed_supports_future_overtaking(self):
        self.assertEqual(AVOIDANCE_SPEED_MAX, 1.2)

    def test_mpc_failure_is_completion_only_in_terminal_box(self):
        for reason in ('mpc_cross_track_error', 'mpc_heading_error',
                       'mpc_infeasible'):
            self.assertTrue(terminal_tracker_stop_allowed(reason, .10, .08, .10))
        self.assertFalse(terminal_tracker_stop_allowed(
            'mpc_infeasible', .16, .08, .10))
        self.assertFalse(terminal_tracker_stop_allowed(
            'mpc_infeasible', .10, .16, .10))
        self.assertFalse(terminal_tracker_stop_allowed(
            'mpc_non_finite', .10, .08, .10))

    def test_full_candidate_search_is_bounded_to_five_hertz(self):
        self.assertTrue(plan_due(-math.inf, 10.0, True))
        self.assertFalse(plan_due(10.0, 10.199, True))
        self.assertTrue(plan_due(10.0, 10.20, True))
        self.assertFalse(plan_due(10.0, 11.0, False))
        self.assertTrue(plan_due(10.0, 11.0, True, enabled=False,
                                 has_safe_path=False))
        self.assertFalse(plan_due(10.0, 11.0, True, enabled=False,
                                  has_safe_path=True))
        self.assertTrue(plan_due(10.0, 11.0, True, enabled=True,
                                 has_safe_path=True))

    def test_command_heartbeat_is_bounded_by_the_same_freshness_budget(self):
        self.assertTrue(command_heartbeat_allowed(True, True, 0.0))
        self.assertTrue(command_heartbeat_allowed(True, True, 0.5))
        self.assertFalse(command_heartbeat_allowed(True, True, 0.500001))
        self.assertFalse(command_heartbeat_allowed(False, True, 0.1))
        self.assertFalse(command_heartbeat_allowed(True, False, 0.1))
        self.assertFalse(command_heartbeat_allowed(True, True, -0.1))

    def test_endpoint_speed_reduces_early_and_stops_at_five_centimeters(self):
        self.assertAlmostEqual(endpoint_speed_limit(1.0), math.sqrt(0.5))
        self.assertAlmostEqual(endpoint_speed_limit(0.20), math.sqrt(0.10))
        self.assertEqual(endpoint_speed_limit(0.05), 0.0)
        self.assertEqual(endpoint_speed_limit(0.0), 0.0)

    def test_open_terminal_singleton_is_completion_not_planner_failure(self):
        self.assertFalse(open_endpoint_reached(False, 4.411, 4.360, .04, .05))
        self.assertTrue(open_endpoint_reached(False, 4.411, 4.361, .04, .05))
        self.assertTrue(open_endpoint_reached(False, 4.411, 4.411, .04, .05))
        self.assertFalse(open_endpoint_reached(True, 4.411, 4.411, .04, .05))

    def test_wall_return_is_associated_on_both_sides_along_laser_ray(self):
        class WallAtX:
            @staticmethod
            def clearance_many(points):
                return np.abs(np.asarray(points)[:, 0] - 1.0)
        points = np.array([[0.70, 0.0], [1.30, 1.0], [0.50, 2.0]])
        directions = np.array([[1.0, 0.0], [1.0, 0.0], [-1.0, 0.0]])
        dynamic = remove_map_wall_returns(WallAtX(), points, directions)
        self.assertEqual(dynamic.shape, (1, 2))
        np.testing.assert_allclose(dynamic[0], points[2])

    def test_wall_return_filter_can_return_original_beam_mask(self):
        class WallAtX:
            @staticmethod
            def clearance_many(points):
                return np.abs(np.asarray(points)[:, 0] - 1.0)
        points = np.array([[0.70, 0.0], [1.30, 1.0], [0.50, 2.0]])
        directions = np.array([[1.0, 0.0], [1.0, 0.0], [-1.0, 0.0]])
        dynamic, keep = remove_map_wall_returns(
            WallAtX(), points, directions, return_mask=True)
        np.testing.assert_array_equal(keep, [False, False, True])
        np.testing.assert_allclose(dynamic[0], points[2])

    def test_pose_history_interpolates_scan_time(self):
        history = PoseHistory()
        history.add(10.00, 1.0, 2.0, 0.10)
        history.add(10.05, 1.1, 2.2, 0.20)
        x, y, heading, span = history.at(10.025)
        self.assertAlmostEqual(x, 1.05)
        self.assertAlmostEqual(y, 2.10)
        self.assertAlmostEqual(heading, 0.15)
        self.assertAlmostEqual(span, 0.05)

    def test_pose_history_wraps_heading_interpolation(self):
        history = PoseHistory()
        history.add(20.00, 0.0, 0.0, math.pi - 0.10)
        history.add(20.05, 0.0, 0.0, -math.pi + 0.10)
        _, _, heading, _ = history.at(20.025)
        self.assertAlmostEqual(abs(heading), math.pi)

    def test_pose_history_rejects_unbracketed_old_scan(self):
        history = PoseHistory()
        history.add(30.0, 0.0, 0.0, 0.0)
        with self.assertRaisesRegex(ValueError, 'scan_pose_unsynchronized'):
            history.at(29.8)

    def test_lightweight_pose_stream_can_advance_during_slow_planning(self):
        controller = object.__new__(AvoidanceController)
        controller.input_lock = threading.RLock()
        controller.pose_stamp = -math.inf
        controller.pose_receipt = -math.inf
        controller.latest_pose = None
        controller.pose_history = PoseHistory()
        self.assertTrue(controller.record_latest_pose(
            10.00, 1.0, 2.0, 0.10, 100.00))
        # The second subscription receives the same sample and must not corrupt
        # history; a newer sample still advances freshness while planning runs.
        self.assertFalse(controller.record_latest_pose(
            10.00, 9.0, 9.0, 0.90, 100.01))
        self.assertTrue(controller.record_latest_pose(
            10.05, 1.1, 2.1, 0.12, 100.05))
        self.assertEqual(controller.latest_pose,
                         (10.05, 1.1, 2.1, 0.12))
        self.assertEqual(controller.pose_receipt, 100.05)
        self.assertEqual(len(controller.pose_history.samples), 2)

    def test_remote_blockage_requires_three_quarter_second(self):
        confirmation = BlockageConfirmation(required_seconds=0.75)
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.0,
                                             nearest_obstacle_m=2.0), 'confirming')
        self.assertEqual(confirmation.count, 1)
        for index in range(1, 15):
            self.assertEqual(confirmation.update(
                False, 'all_candidates_blocked', now=10.0 + index * 0.05,
                nearest_obstacle_m=2.0), 'confirming')
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.75,
                                             nearest_obstacle_m=2.0), 'blocked')

    def test_close_blockage_requires_two_tenths_second(self):
        confirmation = BlockageConfirmation(required_seconds=0.75,
                                             immediate_distance_m=0.50,
                                             near_required_seconds=0.20)
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.0,
                                             nearest_obstacle_m=0.40), 'confirming')
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.10,
                                             nearest_obstacle_m=0.40), 'confirming')
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.20,
                                             nearest_obstacle_m=0.40), 'blocked')
        self.assertEqual(confirmation.count, 3)

    def test_valid_scan_resets_blockage_confirmation(self):
        confirmation = BlockageConfirmation(required_seconds=0.75)
        confirmation.update(False, 'all_candidates_blocked', now=10.0)
        self.assertEqual(confirmation.update(True, 'ok'), 'valid')
        self.assertEqual(confirmation.count, 0)
        self.assertEqual(confirmation.elapsed(now=10.4), 0.0)
        self.assertEqual(confirmation.update(False, 'all_candidates_blocked', now=10.5),
                         'confirming')

    def test_non_candidate_failure_stops_immediately(self):
        confirmation = BlockageConfirmation(required_seconds=0.75)
        self.assertEqual(confirmation.update(False, 'static_clearance_invalid'), 'blocked')
        self.assertEqual(confirmation.count, 0)

    def test_lap_progress_wraps_once_and_ignores_small_backward_jitter(self):
        tracker = LapProgress([1.0, 1.0, 1.0, 1.0])
        self.assertEqual(tracker.update(0), 0.0)
        self.assertEqual(tracker.update(1), 1.0)
        self.assertEqual(tracker.update(0), 1.0)
        self.assertEqual(tracker.update(2), 2.0)
        self.assertEqual(tracker.update(3), 3.0)
        self.assertEqual(tracker.update(0), 4.0)

    def test_open_progress_never_wraps_to_start(self):
        tracker = LapProgress([1.0] * 9 + [0.0], closed_loop=False)
        self.assertEqual(tracker.update(0), 0.0)
        self.assertEqual(tracker.update(4), 4.0)
        self.assertEqual(tracker.update(3), 4.0)
        self.assertEqual(tracker.update(9), 9.0)
        with self.assertRaisesRegex(ValueError, 'progress_jump'):
            tracker.update(0)

    def test_every_clear_route_window_selects_center(self):
        planner = ClosedLoopLatticePlanner(ROUTE, MAP)
        for index, pose in enumerate(planner.route):
            planner.progress_index = index
            planner.clear_commit()
            selected, _ = planner.plan(pose[0], pose[1], np.empty((0, 2)))
            self.assertTrue(selected.valid, index)
            self.assertAlmostEqual(selected.offset_m, 0.0, msg=str(index))

    def test_open_route_remaining_distance_never_wraps_to_start(self):
        planner = ClosedLoopLatticePlanner(ROUTE, MAP)
        start = len(planner.route) - 10
        planner.progress_index = start
        remaining = float(np.sum(planner.segment_m[start:]))
        obstacle = planner.route[5, :2]
        pose = planner.route[start]
        selected, candidates = planner.plan(
            pose[0], pose[1], obstacle[None, :], horizon_m=remaining
        )
        self.assertTrue(selected.valid)
        self.assertNotIn(0, planner.window_indices(
            start, horizon_m=remaining))
        self.assertTrue(all(
            np.min(np.linalg.norm(candidate.path[:, :2] - obstacle, axis=1)) >= 0.15
            for candidate in candidates if candidate.valid
        ))

    def test_static_box_sites_A_B_C_have_a_safe_frenet_solution(self):
        for name, center in STATIC_TEST_SITES.items():
            with self.subTest(site=name):
                planner = ClosedLoopLatticePlanner(
                    ROUTE, MAP, footprint_radius_m=0.25,
                    vehicle_half_width_m=0.15, obstacle_margin_m=0.0,
                    static_echo_tolerance_m=STATIC_ECHO_TOLERANCE,
                )
                center = np.asarray(center, dtype=float)
                obstacle_index = int(np.argmin(np.linalg.norm(
                    planner.route[:, :2] - center, axis=1)))
                start = obstacle_index
                distance_before = 0.0
                while distance_before < 1.5:
                    start = max(0, start - 1)
                    distance_before += planner.segment_m[start]
                    if start == 0:
                        break
                planner.progress_index = start
                pose = planner.route[start]
                selected, candidates = planner.plan(
                    pose[0], pose[1], square_box_points(center),
                    heading=pose[2],
                )
                self.assertEqual(len(candidates), 125)
                self.assertTrue(selected.valid)
                self.assertGreater(abs(selected.offset_m), 0.0)
                self.assertGreaterEqual(
                    selected.minimum_static_clearance_m, 0.25)
                self.assertGreaterEqual(
                    selected.minimum_obstacle_clearance_m, 0.15)
                self.assertGreaterEqual(
                    sum(candidate.valid for candidate in candidates), 1)

    def test_A_B_C_boxes_pass_persistent_perception_before_planning(self):
        for name, center in STATIC_TEST_SITES.items():
            with self.subTest(site=name):
                planner = ClosedLoopLatticePlanner(
                    ROUTE, MAP, footprint_radius_m=0.25,
                    vehicle_half_width_m=0.15, obstacle_margin_m=0.0,
                    static_echo_tolerance_m=STATIC_ECHO_TOLERANCE,
                )
                center = np.asarray(center, dtype=float)
                obstacle_index = int(np.argmin(np.linalg.norm(
                    planner.route[:, :2] - center, axis=1)))
                start = obstacle_index
                distance_before = 0.0
                while distance_before < 1.5:
                    start = max(0, start - 1)
                    distance_before += planner.segment_m[start]
                    if start == 0:
                        break
                planner.progress_index = start
                pose = planner.route[start]
                line_of_sight = center - pose[:2]
                line_of_sight /= np.linalg.norm(line_of_sight)
                tangent = np.array([-line_of_sight[1], line_of_sight[0]])
                face = center + np.linspace(-0.08, 0.08, 13)[:, None] * tangent
                clusters = cluster_scan_points(face, np.arange(100, 113))
                tracker = PersistentObstacleTracker(required_hits=2)
                self.assertEqual(tracker.update(clusters, 1.00), [])
                tracks = tracker.update(clusters, 1.05)
                self.assertEqual(len(tracks), 1)
                selected, candidates = planner.plan(
                    pose[0], pose[1], confirmed_points(tracks),
                    heading=pose[2],
                )
                self.assertEqual(len(candidates), 125)
                self.assertTrue(selected.valid)
                self.assertGreater(abs(selected.offset_m), 0.0)

    def test_new_route_approach_selects_bypass_while_center_is_still_safe(self):
        """A nearby box triggers an early offset before center becomes invalid."""
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, footprint_radius_m=0.25,
            vehicle_half_width_m=0.15, obstacle_margin_m=0.0,
            static_echo_tolerance_m=STATIC_ECHO_TOLERANCE,
            proactive_avoidance_clearance_m=0.22,
        )
        planner.progress_index = 0
        recorded_pose = planner.route[0]
        observed_box_center = (1.2081769272248628, -0.37682462879917183)
        selected, candidates = planner.plan(
            recorded_pose[0], recorded_pose[1],
            square_box_points(observed_box_center, half_size=0.04),
            heading=recorded_pose[2],
        )
        valid_centers = [candidate for candidate in candidates
                         if candidate.valid
                         and abs(candidate.offset_m) < 1e-9]
        self.assertTrue(valid_centers)
        self.assertLessEqual(
            min(candidate.minimum_obstacle_clearance_m
                for candidate in valid_centers), 0.22)
        self.assertTrue(selected.valid)
        self.assertGreaterEqual(abs(selected.offset_m), 0.10 - 1e-9)
        self.assertGreaterEqual(selected.minimum_obstacle_clearance_m, 0.22)
        self.assertEqual(
            planner.last_switch_reason,
            'proactive_clearance_select_avoidance')

    def test_live_clearance_policy_uses_vehicle_half_width_physical_limit(self):
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, static_echo_tolerance_m=STATIC_ECHO_TOLERANCE
        )
        self.assertAlmostEqual(planner.vehicle_half_width_m, 0.15)
        self.assertAlmostEqual(planner.obstacle_margin_m, 0.0)
        self.assertAlmostEqual(planner.dynamic_clearance_m, 0.15)
        self.assertAlmostEqual(planner.proactive_avoidance_clearance_m, 0.22)
        self.assertAlmostEqual(planner.static_echo_tolerance_m, 0.25)


if __name__ == "__main__":
    unittest.main()
