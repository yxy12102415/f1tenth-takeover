#!/usr/bin/env python3
"""Compile a non-actuating C++ probe and replay same-capture LiDAR holdouts.

No ROS, serial, Bluetooth or actuator access. Builds in a temporary directory.
This checks export reproducibility, not independent recognition accuracy.
"""
import gzip
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

REPO = Path(__file__).resolve().parents[1]
VEHICLE = REPO / 'real_vehicle'
RELEASE = VEHICLE / 'candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2'
CAPTURE = VEHICLE / 'analysis/lead_shell_multiview_20261117'
sys.path.insert(0, str(VEHICLE / 'lead_route_identity_20261117'))
from lead_route_tracker import principal_support


def rows(name):
    with gzip.open(CAPTURE / name / 'scans.jsonl.gz', 'rt') as stream:
        return [json.loads(line) for line in stream]


def run():
    background = np.nanmedian([
        [np.nan if value is None else value for value in row['ranges']]
        for row in rows('background')
    ], axis=0)
    checks = []
    with tempfile.TemporaryDirectory(prefix='381-lead-check-') as temporary:
        directory = Path(temporary)
        binary = directory / 'lead_identity_probe'
        subprocess.run([
            'g++', '-std=c++14', '-O3', '-ffp-contract=off',
            '-I/usr/include/eigen3', '-I' + str(RELEASE / 'src/cpp_frenet'),
            str(RELEASE / 'review/lead_identity_probe.cpp'), '-ljsoncpp',
            '-o', str(binary),
        ], check=True)
        # Straight synthetic reference isolates recognition from field localization.
        assets = directory / 'assets.json'
        assets.write_text(json.dumps(dict(
            route=[[i * .1, 0, 0] for i in range(101)],
            speed=[0] + [.7] * 99 + [0], resolution=1., origin=[0, 0],
            width=1, height=1, clearance=[10.],
        )))
        for view in ['rear', 'rear_left', 'left_side', 'front_left',
                     'front', 'front_right', 'right_side', 'rear_right']:
            requests = []
            # Mirrors the recorded regression's held-out frame selection.
            for index, row in enumerate(rows(view)[10:191:20]):
                ranges = np.array([np.nan if x is None else x for x in row['ranges']])
                angle = row['angle_min'] + np.arange(len(ranges)) * row['angle_increment']
                points = np.column_stack((ranges * np.cos(angle), ranges * np.sin(angle)))
                mask = (np.isfinite(ranges) & (ranges >= row['range_min']) & (ranges < 10)
                        & (points[:, 0] >= .7) & (points[:, 0] <= 2.3)
                        & (abs(points[:, 1]) <= .65) & (background - ranges > .08))
                points = principal_support(points[mask])
                assert len(points) >= 6, (view, 'insufficient support')
                low, high = points.min(axis=0), points.max(axis=0)
                stamp = 100 + index * .1
                obstacle = dict(
                    id=index + 1, hits=20, stamp=stamp,
                    xmin=float(low[0]), ymin=float(low[1]),
                    xmax=float(high[0]), ymax=float(high[1]),
                    vx=0., vy=0., has_moved=False, fixed_world=True,
                    visibility_hazard=False, memory_state='observed',
                )
                requests.append(dict(source_scan_stamp=stamp, scan_pose=[0, 0, 0],
                                     occupancies=[obstacle], obstacle_points=points.tolist()))
            completed = subprocess.run(
                [str(binary), str(assets), str(RELEASE / 'compact_config.json')],
                input=''.join(json.dumps(item) + '\n' for item in requests),
                text=True, capture_output=True, check=True, timeout=30,
            )
            responses = [json.loads(line) for line in completed.stdout.splitlines()]
            assert len(responses) == len(requests), view
            scores = []
            for request, response in zip(requests, responses):
                assert len(response['occupancies']) == 1
                for key in ['id', 'stamp', 'xmin', 'ymin', 'xmax', 'ymax']:
                    assert request['occupancies'][0][key] == response['occupancies'][0][key]
                identity = response['lead_identity']
                assert not identity['identity_loss_requests_stop']
                if identity['reason'] == 'associated':
                    scores.append(identity['shape_error_m'])
            assert responses[-1]['lead_identity']['reason'] == 'associated', view
            assert scores and max(scores) <= .06, (view, scores)
            checks.append(dict(view=view, associated=True, maximum_shape_error_m=max(scores)))
    print(json.dumps(dict(passed=True, physical_motion=False,
                          scope='Same-capture replay; not independent recognition accuracy',
                          checks=checks), indent=2))


if __name__ == '__main__':
    run()
