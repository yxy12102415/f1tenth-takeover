"""ROS-independent LiDAR clustering and temporal obstacle confirmation."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ObstacleCluster:
    points: np.ndarray
    centroid: np.ndarray
    extent_m: float
    first_beam: int
    last_beam: int


@dataclass
class ObstacleTrack:
    track_id: int
    points: np.ndarray
    centroid: np.ndarray
    extent_m: float
    hits: int
    misses: int
    last_stamp: float
    confirmed: bool


def cluster_scan_points(points, beam_indices, *, maximum_point_gap_m=0.18,
                        maximum_beam_gap=2, minimum_points=3,
                        minimum_extent_m=0.025, maximum_extent_m=0.80):
    """Group adjacent scan returns and reject isolated or implausible groups.

    Input order must match laser beam order.  Both a beam-index gap and a
    Cartesian gap are enforced, so points on separate objects do not merge
    merely because their world coordinates happen to be close.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    beam_indices = np.asarray(beam_indices, dtype=int).reshape(-1)
    if len(points) != len(beam_indices):
        raise ValueError("scan_point_index_mismatch")
    limits = (maximum_point_gap_m, minimum_extent_m, maximum_extent_m)
    if (not all(math.isfinite(float(value)) for value in limits)
            or maximum_point_gap_m <= 0.0 or maximum_beam_gap < 1
            or minimum_points < 2 or minimum_extent_m < 0.0
            or maximum_extent_m <= minimum_extent_m):
        raise ValueError("invalid_cluster_limits")
    if not len(points):
        return []
    if not np.all(np.isfinite(points)):
        raise ValueError("scan_points_nonfinite")

    groups = []
    start = 0
    for index in range(1, len(points)):
        beam_gap = int(beam_indices[index] - beam_indices[index - 1])
        point_gap = float(np.linalg.norm(points[index] - points[index - 1]))
        if (beam_gap <= 0 or beam_gap > int(maximum_beam_gap)
                or point_gap > float(maximum_point_gap_m)):
            groups.append((start, index))
            start = index
    groups.append((start, len(points)))

    clusters = []
    for start, finish in groups:
        group = points[start:finish]
        if len(group) < int(minimum_points):
            continue
        span = np.ptp(group, axis=0)
        extent = float(np.linalg.norm(span))
        if not float(minimum_extent_m) <= extent <= float(maximum_extent_m):
            continue
        clusters.append(ObstacleCluster(
            points=group.copy(),
            centroid=np.mean(group, axis=0),
            extent_m=extent,
            first_beam=int(beam_indices[start]),
            last_beam=int(beam_indices[finish - 1]),
        ))
    return clusters


class PersistentObstacleTracker:
    """Confirm a cluster on two consecutive scans and bridge one missed scan."""

    def __init__(self, required_hits=2, association_distance_m=0.22,
                 confirmed_hold_s=0.15, maximum_confirmed_misses=1):
        self.required_hits = int(required_hits)
        self.association_distance_m = float(association_distance_m)
        self.confirmed_hold_s = float(confirmed_hold_s)
        self.maximum_confirmed_misses = int(maximum_confirmed_misses)
        if (self.required_hits < 1 or self.association_distance_m <= 0.0
                or self.confirmed_hold_s < 0.0
                or self.maximum_confirmed_misses < 0):
            raise ValueError("invalid_tracker_limits")
        self.tracks = []
        self.next_track_id = 1
        self.last_stamp = -math.inf

    def reset(self):
        self.tracks = []
        self.next_track_id = 1
        self.last_stamp = -math.inf

    def update(self, clusters, stamp):
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            raise ValueError("obstacle_tracker_stamp_nonincreasing")
        self.last_stamp = stamp
        clusters = list(clusters)

        pairs = []
        for track_index, track in enumerate(self.tracks):
            for cluster_index, cluster in enumerate(clusters):
                distance = float(np.linalg.norm(track.centroid - cluster.centroid))
                if distance <= self.association_distance_m:
                    pairs.append((distance, track_index, cluster_index))
        pairs.sort()
        matched_tracks = set()
        matched_clusters = set()
        for _, track_index, cluster_index in pairs:
            if track_index in matched_tracks or cluster_index in matched_clusters:
                continue
            track = self.tracks[track_index]
            cluster = clusters[cluster_index]
            track.points = cluster.points.copy()
            track.centroid = cluster.centroid.copy()
            track.extent_m = float(cluster.extent_m)
            track.hits += 1
            track.misses = 0
            track.last_stamp = stamp
            track.confirmed = track.confirmed or track.hits >= self.required_hits
            matched_tracks.add(track_index)
            matched_clusters.add(cluster_index)

        survivors = []
        for index, track in enumerate(self.tracks):
            if index not in matched_tracks:
                track.misses += 1
            if track.confirmed:
                if (track.misses <= self.maximum_confirmed_misses
                        and stamp - track.last_stamp <= self.confirmed_hold_s + 1e-9):
                    survivors.append(track)
            elif track.misses == 0:
                survivors.append(track)
        self.tracks = survivors

        for index, cluster in enumerate(clusters):
            if index in matched_clusters:
                continue
            confirmed = self.required_hits == 1
            self.tracks.append(ObstacleTrack(
                track_id=self.next_track_id,
                points=cluster.points.copy(),
                centroid=cluster.centroid.copy(),
                extent_m=float(cluster.extent_m),
                hits=1,
                misses=0,
                last_stamp=stamp,
                confirmed=confirmed,
            ))
            self.next_track_id += 1

        return [track for track in self.tracks if track.confirmed]


def confirmed_points(tracks):
    tracks = list(tracks)
    if not tracks:
        return np.empty((0, 2), dtype=float)
    return np.vstack([track.points for track in tracks])
