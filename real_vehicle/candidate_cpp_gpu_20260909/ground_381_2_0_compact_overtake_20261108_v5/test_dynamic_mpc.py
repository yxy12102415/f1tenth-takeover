import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from nav_msgs.msg import Odometry

from avoidance_controller import AvoidanceController
from local_path_tracker import CONFIG, LocalPathTracker, model_parameters
from native_dynamic_mpc import discrete_model, solve
from test_native_mpc import python_reference


def reference_continuous(speed, model):
    """Independent transcription of the old, ground-tested bicycle equations."""
    m, iz, lf, lr, cf, cr = model
    a = np.array([[0., speed, 1., 0.], [0., 0., 0., 1.],
                  [0., 0., -(cf+cr)/(m*speed), -speed+(cr*lr-cf*lf)/(m*speed)],
                  [0., 0., (cr*lr-cf*lf)/(iz*speed), -(cf*lf**2+cr*lr**2)/(iz*speed)]])
    b = np.array([0., 0., cf/m, cf*lf/iz])
    c = np.array([0., -speed, 0., 0.])
    return a, b, c


def reference_solve(initial, speeds, curvature, model, weights, previous,
                    angle, limit, first_step, future_step, dt, iterations):
    n = 16; dim = 4
    states = np.zeros(((n+1)*dim, dim)); states[:dim] = np.eye(dim)
    inputs = np.zeros(((n+1)*dim, n)); affine = np.zeros((n+1)*dim)
    for i in range(n):
        ac, bc, cc = reference_continuous(speeds[i], model)
        implicit = np.eye(dim)-dt*ac
        a = np.linalg.solve(implicit, np.eye(dim))
        b = np.linalg.solve(implicit, dt*bc)
        c = np.linalg.solve(implicit, dt*cc*curvature[i])
        old = slice(i*dim, (i+1)*dim); new = slice((i+1)*dim, (i+2)*dim)
        states[new] = a@states[old]; inputs[new] = a@inputs[old]
        inputs[new, i] += b; affine[new] = a@affine[old]+c
    q = np.r_[np.tile(weights[:4], n), weights[4:8]]
    difference = np.eye(n); difference[np.arange(1, n), np.arange(n-1)] = -1.
    anchor = np.zeros(n); anchor[0] = angle
    weighted = inputs.T*q
    h = weighted@inputs+weights[8]*np.eye(n)+weights[9]*difference.T@difference
    linear = weighted@(states@initial+affine)-weights[9]*difference.T@anchor
    lipschitz = 2.*max(1e-6, np.max(np.sum(np.abs(h), axis=1)))
    controls, count = python_reference(h, linear, np.r_[previous[1:], previous[-1]],
                                        angle, limit, first_step, future_step, lipschitz, iterations)
    predicted = (states@initial+inputs@controls+affine).reshape(-1, dim)
    return controls, np.max(np.abs(predicted), axis=0), count


class DynamicMPCTest(unittest.TestCase):
    def test_native_discretization_matches_original_equations_and_is_stable(self):
        for speed in (.5, .6, .75, 1., 1.5):
            dt = .05; ac, bc, cc = reference_continuous(speed, model_parameters())
            a, b, c = discrete_model(speed, dt, model_parameters())
            implicit = np.eye(4)-dt*ac
            np.testing.assert_allclose(implicit@a, np.eye(4), atol=1e-12)
            np.testing.assert_allclose(implicit@b, dt*bc, atol=1e-12)
            np.testing.assert_allclose(implicit@c, dt*cc, atol=1e-12)
            self.assertLess(np.max(np.abs(np.linalg.eigvals(a[2:, 2:]))), 1.)

    def test_native_full_horizon_matches_independent_python_reference(self):
        rng = np.random.RandomState(923)
        for _ in range(32):
            initial = rng.uniform(-.15, .15, 4)
            speeds = rng.uniform(.5, 1.5, 16)  # exercises LTV, not a single A matrix
            curvature = rng.uniform(-.8, .8, 16)
            previous = rng.uniform(-.2, .2, 16)
            angle = rng.uniform(-.2, .2)
            weights = [140., 70., 1., 3., 210., 105., 1.5, 4.5, 1.2, 16.]
            args = (initial, speeds, curvature, model_parameters(), weights,
                    previous, angle, .269, .017, .05, .05, 80)
            actual, maxima, count = solve(*args)
            expected, expected_maxima, expected_count = reference_solve(*args)
            np.testing.assert_allclose(actual, expected, atol=1e-11, rtol=1e-11)
            np.testing.assert_allclose(maxima, expected_maxima, atol=1e-11, rtol=1e-11)
            self.assertEqual(count, expected_count)
            self.assertLessEqual(abs(actual[0]-angle), .017+1e-12)

    def test_yaw_and_lateral_speed_states_affect_steering(self):
        x = np.linspace(0., 4., 81); path = np.c_[x, x*0., x*0.]
        zero = LocalPathTracker().compute(path, .2, 0., 0., .05, 1.)
        self.assertAlmostEqual(zero, 0., places=10)
        for lateral, yaw in ((.15, 0.), (0., .5)):
            left = LocalPathTracker().compute(path, .2, 0., 0., .05, 1., lateral, yaw)
            right = LocalPathTracker().compute(path, .2, 0., 0., .05, 1., -lateral, -yaw)
            self.assertLess(left, -1e-4)
            self.assertAlmostEqual(left, -right, places=10)

    def test_tire_parameters_affect_prediction_and_low_speed_is_regularized(self):
        a, b, _ = discrete_model(1., .05, model_parameters())
        changed = model_parameters(); changed[4] *= .7
        aa, bb, _ = discrete_model(1., .05, changed)
        self.assertGreater(np.linalg.norm(a-aa)+np.linalg.norm(b-bb), .01)
        x = np.linspace(0., 2., 41); tracker = LocalPathTracker()
        tracker.compute(np.c_[x, x*0., x*0.], 0., .01, 0., .05, 0.)
        self.assertEqual(tracker.last['prediction_speed_mps'], .5)
        self.assertEqual(tracker.last['state_dimension'], 4)

    def test_invalid_dynamic_state_cannot_update_solution(self):
        x = np.linspace(0., 3., 61); path = np.c_[x, x*0., x*0.]
        for vy, rate in ((math.nan, 0.), (0., math.inf), (2.6, 0.), (0., 8.1)):
            tracker = LocalPathTracker()
            with self.assertRaises(ValueError):
                tracker.compute(path, 0., 0., 0., .05, 1., vy, rate)
            np.testing.assert_array_equal(tracker.previous, 0.)

    def test_odometry_forwards_all_three_velocity_components_or_stops(self):
        node = SimpleNamespace(input_lock=threading.RLock(), odom_stamp=-math.inf,
                               odom_receipt=-math.inf, publish_stop=Mock())
        message = Odometry(); message.header.stamp.sec = 10
        message.header.frame_id = 'odom'; message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = .9; message.twist.twist.linear.y = .07
        message.twist.twist.angular.z = -.3
        AvoidanceController.odom_cb(node, message)
        self.assertEqual((node.measured_speed, node.measured_lateral_speed, node.measured_yaw_rate),
                         (.9, .07, -.3))
        node.publish_stop.assert_not_called()
        message.header.stamp.sec = 11; message.twist.twist.angular.z = math.nan
        AvoidanceController.odom_cb(node, message)
        node.publish_stop.assert_called_with('odom_invalid')
        self.assertEqual(node.odom_receipt, -math.inf)

    def test_pose_control_passes_dynamic_states_to_native_tracker(self):
        from test_continuous_tracking import node_fixture
        node = node_fixture(); node.measured_lateral_speed = .07; node.measured_yaw_rate = -.3
        node.control(node.last_safe_path, .2, 0., 0., 10.)
        self.assertEqual(node.tracker.last['lateral_speed_mps'], .07)
        self.assertEqual(node.tracker.last['yaw_rate_rad_s'], -.3)


if __name__ == '__main__':
    unittest.main()
