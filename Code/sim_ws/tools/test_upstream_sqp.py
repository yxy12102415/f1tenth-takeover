#!/usr/bin/env python3
"""Unit checks for external-source provenance and the bounded SQP interface."""
import unittest
from pathlib import Path

import numpy as np
import upstream_sqp_worker as worker


class SqpChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[3] / '.deps/predictive-spliner-upstream/src/sqp_avoidance_node.py'
        cls.core = worker.load_core(source)

    def fixture(self, side=1):
        n = 21
        x = np.linspace(0, 4.5, n)
        d = side*.82*(3*(x/4.5)**2-2*(x/4.5)**3)
        bounds = [[-1.25, 1.25] for _ in x]
        for i in range(12, n):
            bounds[i] = [.80, 1.25] if side > 0 else [-1.25, -.80]
        return dict(s=x.tolist(), initial=d.tolist(), bounds=bounds,
                    initial_slope=0., max_curvature=.60, curvature=[0.]*(n-2),
                    obstacle_indices=list(range(12, n)), obstacle_d=[0.]*(n-12),
                    obstacle_clearance=[.80]*(n-12))

    def test_actual_upstream_core_both_sides(self):
        for side in (1, -1):
            result = worker.solve(self.core, self.fixture(side))
            self.assertTrue(result['ok'], result)
            self.assertEqual(result['upstream_commit'], worker.COMMIT)
            self.assertGreaterEqual(side*result['d'][-1], .80-1e-6)
            self.assertAlmostEqual(result['d'][0], 0., places=5)
            coeff = np.asarray(result['coefficients'])
            self.assertAlmostEqual(2*coeff[0, 1], 0., places=4)
            self.assertAlmostEqual(coeff[0, 2], 0., places=4)
            ds = result['s'][1]-result['s'][0]
            self.assertAlmostEqual(6*coeff[-1, 0]*ds+2*coeff[-1, 1], 0., places=4)
            self.assertLess(result['solve_ms'], 200.)  # offline sanity, not controller deadline

    def test_infeasible_start_not_accepted(self):
        request = self.fixture()
        request['bounds'][0] = [.4, .5]
        self.assertFalse(worker.solve(self.core, request)['ok'])

    def test_continuity_cost_cannot_override_clearance(self):
        request = self.fixture()
        request['lateral_reference'] = [0.]*21  # deliberately favors collision
        result = worker.solve(self.core, request)
        self.assertTrue(result['ok'], result)
        self.assertGreaterEqual(min(result['d'][12:]), .8-1e-6)

    def test_empty_corridor_rejected(self):
        request = self.fixture()
        request['bounds'][5] = [.9, .1]
        with self.assertRaisesRegex(ValueError, 'empty_corridor'):
            worker.solve(self.core, request)

    def test_incompatible_endpoint_curvature_rejected_before_sqp(self):
        request = self.fixture()
        request['initial_second_derivative'] = 3.
        result = worker.solve(self.core, request)
        self.assertFalse(result['ok'])
        self.assertEqual(result['message'], 'linear_corridor_infeasible')
        self.assertEqual(result['iterations'], 0)

    def test_nonfinite_rejected(self):
        request = self.fixture()
        request['initial'][1] = float('nan')
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            worker.solve(self.core, request)

    def test_analytic_derivatives_against_original_functions(self):
        solver = self.core()
        solver.down_sampled_delta_s = .3
        solver.max_kappa = .6
        solver.global_traj_kappas = np.linspace(.1, .2, 19)
        solver.obs_downsampled_indices = np.arange(12, 21)
        solver.obs_downsampled_center_d = np.zeros(9)
        solver.obs_downsampled_min_dist = np.full(9, .8)
        d = np.linspace(.05, 1., 21)**2 + .01*np.sin(np.arange(21))
        delta = np.eye(21)*1e-6
        numeric_g = np.array([(solver.objective_function(d+e)-solver.objective_function(d-e))/2e-6 for e in delta])
        numeric_j = np.array([(solver.combined_inequality_constraints(d+e)-solver.combined_inequality_constraints(d-e))/2e-6 for e in delta]).T
        np.testing.assert_allclose(worker.objective_jac(d), numeric_g, atol=1e-6)
        np.testing.assert_allclose(worker.inequality_jac(solver, d), numeric_j, atol=1e-6)
        solver.lateral_reference=np.linspace(0,.8,21)
        solver.spatial_length=6.
        solver.dense_second=np.diff(np.eye(21),n=2,axis=0)/.3**2
        solver.dense_bias=np.linspace(-.01,.01,19)
        adapted=np.array([(worker.adapter_cost(solver,d+e)-worker.adapter_cost(solver,d-e))/2e-6 for e in delta])
        np.testing.assert_allclose(worker.adapter_cost_jac(solver,d),adapted,atol=1e-6)


if __name__ == '__main__':
    unittest.main()
