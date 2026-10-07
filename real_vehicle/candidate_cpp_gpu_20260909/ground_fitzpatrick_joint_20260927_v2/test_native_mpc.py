import math
import unittest

import numpy as np

from native_mpc import projected_iterations


def python_reference(hessian, linear, control, angle, limit, first, step,
                     lipschitz, iterations):
    """Previous iteration rule, retained only as a numerical test oracle."""
    control = control.copy()
    for iteration in range(1, iterations+1):
        updated = np.clip(control-2.*(hessian@control+linear)/lipschitz, -limit, limit)
        updated[0] = min(angle+first, max(angle-first, float(updated[0])))
        for index in range(1, len(control)):
            updated[index] = min(float(updated[index-1]+step),
                                 max(float(updated[index-1]-step), float(updated[index])))
        if np.max(np.abs(updated-control)) < 1e-6:
            return updated, iteration
        control = updated
    return control, iteration


class NativeMPCTest(unittest.TestCase):
    def test_matches_python_under_active_rate_and_angle_constraints(self):
        rng = np.random.RandomState(923)
        for n in (2, 8, 16, 32):
            for _ in range(40):
                matrix = rng.normal(size=(n, n))
                hessian = matrix.T@matrix + np.eye(n)
                linear = rng.normal(size=n)*10.
                warm = rng.uniform(-.269, .269, n)
                angle = rng.uniform(-.269, .269)
                first = rng.uniform(0., .05)
                lipschitz = 2.*np.max(np.sum(np.abs(hessian), axis=1))
                args = (hessian, linear, warm, angle, .269, first, .05, lipschitz, 40)
                actual, count = projected_iterations(*args)
                expected, old_count = python_reference(*args)
                np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)
                self.assertEqual(count, old_count)
                self.assertLessEqual(np.max(np.abs(actual)), .269+1e-12)
                self.assertLessEqual(abs(actual[0]-angle), first+1e-12)
                self.assertLessEqual(np.max(np.abs(np.diff(actual))), .05+1e-12)

    def test_known_unconstrained_optimum(self):
        target = np.array([.01, .02, .03])
        actual, _ = projected_iterations(
            np.eye(3), -target, np.zeros(3), 0., .269, .05, .05, 2., 40)
        np.testing.assert_allclose(actual, target, atol=1e-14)

    def test_zero_elapsed_time_preserves_first_steering(self):
        actual, _ = projected_iterations(
            np.eye(3), np.ones(3), np.zeros(3), .1, .269, 0., .05, 2., 40)
        self.assertEqual(actual[0], .1)

    def test_invalid_inputs_and_shapes_fail_closed(self):
        for bad in (math.nan, math.inf):
            matrix = np.eye(3); matrix[0, 0] = bad
            with self.assertRaises(ValueError):
                projected_iterations(matrix, np.zeros(3), np.zeros(3),
                                     0., .269, .05, .05, 2., 40)
        for n in (0, 65):
            with self.assertRaises(ValueError):
                projected_iterations(np.eye(n), np.zeros(n), np.zeros(n),
                                     0., .269, .05, .05, 2., 40)
        with self.assertRaisesRegex(ValueError, 'mpc_native_shape'):
            projected_iterations(np.eye(3), np.zeros(2), np.zeros(3),
                                 0., .269, .05, .05, 2., 40)
        with self.assertRaises(ValueError):
            projected_iterations(np.eye(3), np.zeros(3), np.zeros(3),
                                 0., .269, .05, .05, 0., 40)


if __name__ == '__main__':
    unittest.main()
