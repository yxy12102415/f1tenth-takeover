import math
import sys
from pathlib import Path
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
sys.path.insert(0, str(WORKSPACE / 'candidate_cpp_gpu_20260909' / 'open_route_frenet_v1_20260923'))

from return_lattice_planner import ClosedLoopLatticePlanner
from local_path_tracker import CONFIG, LocalPathTracker

ROUTE = WORKSPACE / 'routes/manual_route_20260923_172359_controller_ready.csv'
MAP = WORKSPACE / 'maps/381_2_0.yaml'


def square(center, half=0.04):
    center = np.asarray(center, dtype=float)
    value = np.linspace(-half, half, 17)
    return np.vstack([
        np.c_[center[0] + value, np.full_like(value, center[1] - half)],
        np.c_[center[0] + value, np.full_like(value, center[1] + half)],
        np.c_[np.full_like(value, center[0] - half), center[1] + value],
        np.c_[np.full_like(value, center[0] + half), center[1] + value],
    ])


class LocalTrackerTest(unittest.TestCase):
    def test_crossing_waypoint_midpoint_does_not_jump_heading_error(self):
        theta = np.arange(0., 1.51, .05)
        path = np.c_[np.sin(theta), 1.-np.cos(theta), theta]
        errors = []
        for position in (.0249, .0251):
            tracker = LocalPathTracker()
            tracker.compute(path, math.sin(position), 1.-math.cos(position),
                            position, .05, .75)
            errors.append(tracker.last['heading_error_rad'])
        self.assertLess(abs(errors[1]-errors[0]), .001)

    def test_terminal_two_point_path_still_has_valid_geometry(self):
        from avoidance_controller import PLANNER_TUNING
        planner = ClosedLoopLatticePlanner(ROUTE, MAP, **PLANNER_TUNING)
        planner.progress_index = 116
        selected, _ = planner.plan(
            5.242047, 1.123925, np.empty((0, 2)), heading=.8887403,
            horizon_m=5.845164147568343-5.745247555567141)
        self.assertTrue(selected.valid)
        self.assertEqual(len(selected.path), 2)
        tracker = LocalPathTracker()
        angle = tracker.compute(selected.path, 5.242047, 1.123925,
                                .8887403, .05, .23)
        self.assertTrue(math.isfinite(angle))
        self.assertLessEqual(abs(angle), .05 + 1e-9)
        with self.assertRaisesRegex(ValueError, 'tracker_path_invalid'):
            tracker.compute(selected.path[:1], 5.242047, 1.123925,
                            .8887403, .05, .23)

    def test_prediction_does_not_depend_on_path_sample_spacing(self):
        angles = []
        for spacing in (0.01, 0.05, 0.10):
            x = np.arange(0.0, 4.001, spacing)
            path = np.c_[x, .1*np.sin(2*x), np.arctan(.2*np.cos(2*x))]
            tracker = LocalPathTracker()
            for _ in range(20):
                angle = tracker.compute(
                    path, .5, .1*math.sin(1), math.atan(.2*math.cos(1)),
                    .05, .60)
            angles.append(angle)
        self.assertLess(max(angles)-min(angles), .002)

    def test_optimizer_first_move_matches_actual_rate_budget(self):
        x = np.linspace(0, 3, 61)
        tracker = LocalPathTracker()
        angle = tracker.compute(np.c_[x, x*0, x*0], 0, .08, 0, .008, .75)
        self.assertLessEqual(abs(angle), .008 + 1e-9)
        self.assertAlmostEqual(tracker.previous[0], angle)

    def test_one_meter_per_second_bypass_with_periodic_replanning(self):
        # Ideal bicycle-model regression using the last run's tracked box.
        # This verifies the software chain, not real-world tracking accuracy.
        from avoidance_controller import PLANNER_TUNING
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, static_echo_tolerance_m=.25,
            proactive_avoidance_clearance_m=.22, **PLANNER_TUNING)
        box = square((2.8005, -.1901), .064)
        arc = np.r_[0., np.cumsum(np.linalg.norm(
            np.diff(planner.route[:, :2], axis=0), axis=1))]
        x, y, heading = planner.route[0]
        tracker = LocalPathTracker()
        speed = 0.; steering = 0.
        errors, clearances = [], []
        for tick in range(110):
            if tick % 8 == 0:
                nearest = int(np.argmin(np.hypot(
                    planner.route[:, 0]-x, planner.route[:, 1]-y)))
                selected, _ = planner.plan(
                    x, y, box, heading=heading,
                    horizon_m=float(arc[-1]-arc[nearest]))
                self.assertTrue(selected.valid, selected.reason)
            speed = min(1., speed + .05)
            steering = tracker.compute(
                selected.path, x, y, heading, .05, speed,
                yaw_rate_rad_s=speed/.25*math.tan(steering))
            x += speed * math.cos(heading) * .05
            y += speed * math.sin(heading) * .05
            heading += speed / .25 * math.tan(steering) * .05
            errors.append(abs(tracker.last['cross_track_error_m']))
            clearances.append(float(np.min(np.hypot(box[:, 0]-x, box[:, 1]-y))))
        self.assertGreater(x, 4.)  # The rollout passed the obstacle.
        self.assertLess(max(errors), .05)
        self.assertGreater(min(clearances), .15)

    def test_noisy_odometry_feedback_stays_within_tracking_budget(self):
        # The old 15% comparison between linear-MPC weights is not a dynamic
        # model acceptance criterion. Exercise the deployed weights with the
        # same 4 mm / 8 mrad noise and VESC-style yaw-rate feedback instead.
        x_axis = np.linspace(0., 15., 301)
        path = np.c_[x_axis, x_axis*0., x_axis*0.]
        tracker = LocalPathTracker()
        x, y, heading = 0., .05, .03
        previous_steering = 0.; angles = []; errors = []
        for tick in range(180):
            steering = tracker.compute(
                path, x, y+.004*math.sin(tick*1.1),
                heading+.008*math.sin(tick*.8), .05, 1.,
                yaw_rate_rad_s=math.tan(previous_steering)/.25)
            previous_steering = steering
            x += math.cos(heading)*.05; y += math.sin(heading)*.05
            heading += math.tan(steering)/.25*.05
            angles.append(steering); errors.append(y)
        self.assertLess(np.max(np.abs(errors)), .06)
        self.assertLess(np.max(np.abs(errors[60:])), .01)
        self.assertLess(np.quantile(np.abs(np.diff(angles[60:])), .95), .02)

    def test_straight_path_command_is_finite_bounded_and_toward_path(self):
        x = np.linspace(0.0, 3.0, 61)
        path = np.c_[x, np.zeros_like(x), np.zeros_like(x)]
        tracker = LocalPathTracker()
        angle = tracker.compute(path, 0.0, 0.05, 0.0, 0.05, 0.75)
        self.assertTrue(math.isfinite(angle))
        self.assertLess(angle, 0.0)
        self.assertLessEqual(abs(angle), 0.269)
        self.assertLessEqual(abs(angle), CONFIG['steering_rate_limit_rad_s'] * 0.05 + 1e-9)

    def test_recorded_obstacle_bypass_path_remains_trackable(self):
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, static_echo_tolerance_m=0.25,
            proactive_avoidance_clearance_m=0.22)
        planner.progress_index = 0
        pose = planner.route[0]
        selected, _ = planner.plan(
            pose[0], pose[1],
            square((1.2081769272248628, -0.37682462879917183)),
            heading=pose[2])
        self.assertTrue(selected.valid)
        self.assertGreater(abs(selected.offset_m), 0.0)
        tracker = LocalPathTracker()
        commands = []
        for point in selected.path[:min(30, len(selected.path)):3]:
            commands.append(tracker.compute(
                selected.path, point[0], point[1], point[2], 0.05, 0.75))
        self.assertTrue(all(math.isfinite(value) for value in commands))
        self.assertLessEqual(max(abs(value) for value in commands), 0.269)
        if CONFIG['type'] in ('linear_mpc', 'dynamic_mpc'):
            self.assertLess(tracker.last['mpc_solve_ms'], 50.0)
            self.assertLessEqual(
                tracker.last['mpc_predicted_max_cross_track_m'], 0.25)

    def test_open_route_bypass_rollout_stays_on_path(self):
        planner = ClosedLoopLatticePlanner(
            ROUTE, MAP, static_echo_tolerance_m=0.25,
            proactive_avoidance_clearance_m=0.22)
        planner.progress_index = 0
        x, y, heading = planner.route[0]
        selected, _ = planner.plan(
            x, y, square((1.2081769272248628, -0.37682462879917183)),
            heading=heading)
        self.assertTrue(selected.valid)
        self.assertGreaterEqual(abs(selected.offset_m), 0.10 - 1e-9)
        self.assertGreaterEqual(selected.minimum_obstacle_clearance_m, 0.22)
        tracker = LocalPathTracker()
        errors = []; steering = 0.
        for _ in range(90):
            steering = tracker.compute(
                selected.path, x, y, heading, 0.05, 0.50,
                yaw_rate_rad_s=.50/.25*math.tan(steering))
            heading = math.atan2(
                math.sin(heading + 0.50 / 0.25 * math.tan(steering) * 0.05),
                math.cos(heading + 0.50 / 0.25 * math.tan(steering) * 0.05))
            x += 0.50 * math.cos(heading) * 0.05
            y += 0.50 * math.sin(heading) * 0.05
            errors.append(float(np.min(np.hypot(
                selected.path[:, 0] - x, selected.path[:, 1] - y))))
        self.assertLess(max(errors), 0.04)


if __name__ == '__main__':
    unittest.main()
