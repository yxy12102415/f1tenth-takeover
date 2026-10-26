#!/usr/bin/env python3
"""Offline evaluator geometry regressions; never initializes a ROS node."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'src/f1tenth_mpc-main/mpc/scripts/takeover_evaluator.py'
spec = importlib.util.spec_from_file_location('takeover_evaluator', SCRIPT)
evaluator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluator)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.map = SimpleNamespace(map_image=np.full((100, 100), 255, dtype=np.uint8),
                                   map_resolution=0.05, map_origin=np.array([-2.5, -2.5]))

    def occupied(self, x, y):
        col = math.floor((x + 2.5) / 0.05)
        row = 99 - math.floor((y + 2.5) / 0.05)
        self.map.map_image[row, col] = 0

    def boundary(self, body):
        return evaluator.TakeoverEvaluator.boundary_violation(self.map, body)

    def test_clear_body(self):
        self.assertFalse(self.boundary(evaluator.rectangle(0, 0, 0)))

    def test_obstacle_inside_body_not_at_corners(self):
        self.occupied(0, 0)
        self.assertTrue(self.boundary(evaluator.rectangle(0, 0, 0)))

    def test_obstacle_inside_rotated_body(self):
        self.occupied(0.10, 0.10)
        self.assertTrue(self.boundary(evaluator.rectangle(0, 0, math.pi / 4)))

    def test_bbox_is_not_body(self):
        self.occupied(0.28, -0.28)
        self.assertFalse(self.boundary(evaluator.rectangle(0, 0, math.pi / 4)))

    def test_outside_map(self):
        self.assertTrue(self.boundary(evaluator.rectangle(2.49, 0, 0)))

    def test_touching_counts_as_collision(self):
        a = evaluator.rectangle(0, 0, 0)
        b = evaluator.rectangle(0.58, 0, 0)
        self.assertTrue(evaluator.overlap(a, b))
        self.assertEqual(evaluator.polygon_distance(a, b), 0)

    def test_clearance_is_body_not_center_distance(self):
        a = evaluator.rectangle(0, 0, 0)
        b = evaluator.rectangle(1, 0, 0)
        self.assertAlmostEqual(evaluator.polygon_distance(a, b), 0.42)


class PassCounterTests(unittest.TestCase):
    def test_two_independent_lap_encounters(self):
        counter = evaluator.TruthPassCounter(45.)
        for lap in range(2):
            base = lap*10.
            counter.update(base, lap*45.-3., 0., 0., 'FOLLOW')
            counter.update(base+1, lap*45.-2., .6, 0., 'OVERTAKE')
            counter.update(base+2, lap*45.+1.5, 0., 0., 'CRUISE')
            counter.update(base+2.6, lap*45.+1.8, 0., 0., 'CRUISE')
            for j in range(5):
                counter.update(base+3+j, lap*45.+2., 0., 0., 'CRUISE')
            self.assertEqual(len(counter.events), lap+1)

    def test_spawn_ahead_does_not_count(self):
        counter = evaluator.TruthPassCounter(45.)
        for i in range(10):
            counter.update(i, 2., 0., 0., 'CRUISE')
        self.assertEqual(counter.events, [])

    def test_collision_or_failed_rejoin_does_not_count(self):
        for unsafe, offset in ((True, 0.), (False, .3)):
            counter = evaluator.TruthPassCounter(45.)
            counter.update(0., -2., 0., 0., 'OVERTAKE')
            for i in range(1, 10):
                counter.update(i, 2., offset, 0., 'CRUISE', not unsafe)
            self.assertEqual(counter.events, [])


if __name__ == '__main__':
    unittest.main()
