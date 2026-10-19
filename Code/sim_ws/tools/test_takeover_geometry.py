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


if __name__ == '__main__':
    unittest.main()
