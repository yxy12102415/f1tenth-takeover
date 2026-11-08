import unittest

import numpy as np

from obstacle_tracker import (
    PersistentObstacleTracker, cluster_scan_points, confirmed_points,
)


class ObstacleTrackerTest(unittest.TestCase):
    def test_separates_clusters_and_rejects_isolated_returns(self):
        first = np.c_[np.linspace(0.0, 0.12, 5), np.zeros(5)]
        isolated = np.array([[0.5, 0.5]])
        second = np.c_[np.linspace(1.0, 1.15, 6), np.ones(6)]
        points = np.vstack((first, isolated, second))
        beams = np.r_[np.arange(5), 20, np.arange(40, 46)]
        clusters = cluster_scan_points(points, beams)
        self.assertEqual(len(clusters), 2)
        self.assertEqual([len(cluster.points) for cluster in clusters], [5, 6])

    def test_rejects_tiny_and_wall_sized_groups(self):
        tiny = np.c_[np.linspace(0.0, 0.01, 4), np.zeros(4)]
        wall = np.c_[np.linspace(0.0, 1.0, 20), np.ones(20)]
        points = np.vstack((tiny, wall))
        beams = np.r_[np.arange(4), np.arange(20, 40)]
        self.assertEqual(cluster_scan_points(points, beams), [])

    def test_requires_two_consecutive_observations(self):
        points = np.c_[np.linspace(1.0, 1.12, 5), np.zeros(5)]
        clusters = cluster_scan_points(points, np.arange(5))
        tracker = PersistentObstacleTracker(required_hits=2)
        self.assertEqual(tracker.update(clusters, 1.0), [])
        confirmed = tracker.update(clusters, 1.05)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].hits, 2)
        np.testing.assert_allclose(confirmed_points(confirmed), points)

    def test_single_frame_transient_is_not_confirmed_later(self):
        points = np.c_[np.linspace(1.0, 1.12, 5), np.zeros(5)]
        clusters = cluster_scan_points(points, np.arange(5))
        tracker = PersistentObstacleTracker(required_hits=2)
        tracker.update(clusters, 1.0)
        tracker.update([], 1.05)
        self.assertEqual(tracker.update(clusters, 1.10), [])

    def test_confirmed_track_bridges_one_missed_scan(self):
        points = np.c_[np.linspace(1.0, 1.12, 5), np.zeros(5)]
        clusters = cluster_scan_points(points, np.arange(5))
        tracker = PersistentObstacleTracker(
            required_hits=2, confirmed_hold_s=0.30)
        tracker.update(clusters, 1.0)
        tracker.update(clusters, 1.20)
        held = tracker.update([], 1.40)
        self.assertEqual(len(held), 1)
        self.assertEqual(held[0].misses, 1)
        self.assertEqual(tracker.update([], 1.60), [])


if __name__ == "__main__":
    unittest.main()
