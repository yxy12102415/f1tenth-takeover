"""Non-actuating concurrent benchmark; no ROS nodes or motor publishers.

The box is reconstructed from the 22:09 tracked centroid/extent, not a raw
scan replay. Both versions use the identical 155-candidate workload and CPU
affinity. Run via ../offline_env.sh with single-thread numeric libraries.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'open_route_frenet_v1_20260923'))
from return_lattice_planner import ClosedLoopLatticePlanner


def load_tracker(path):
    spec = importlib.util.spec_from_file_location('benchmark_tracker', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LocalPathTracker


def summary(values):
    return dict(count=len(values), median_ms=float(np.median(values))*1000.,
                p99_ms=float(np.quantile(values, .99))*1000.,
                max_ms=float(max(values))*1000.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracker', type=Path, default=ROOT/'local_path_tracker.py')
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    planner = ClosedLoopLatticePlanner(
        ROOT.parents[1]/'routes/manual_route_20260923_172359_controller_ready.csv',
        ROOT.parents[1]/'maps/381_2_0.yaml', static_echo_tolerance_m=.25,
        offsets=(-.4, -.3, -.2, -.15, -.1, 0., .1, .15, .2, .3, .4),
        minimum_initial_avoidance_offset_m=.15)
    pose = planner.route[15]
    c = (2.7497, -.2355); half = .0786; s = np.linspace(-half, half, 17)
    box = np.vstack((np.c_[c[0]+s, np.full_like(s, c[1]-half)],
                     np.c_[c[0]+s, np.full_like(s, c[1]+half)],
                     np.c_[np.full_like(s, c[0]-half), c[1]+s],
                     np.c_[np.full_like(s, c[0]+half), c[1]+s]))
    selected, candidates = planner.plan(*pose[:2], box, heading=pose[2], horizon_m=5.)
    if not selected.valid:
        raise RuntimeError(selected.reason)
    tracker = load_tracker(args.tracker)()
    tracker.compute(selected.path, *pose, .05, 1.)
    planning = []; controls = []; periods = []; errors = []
    barrier = threading.Barrier(2)
    def control():
        try:
            barrier.wait()
            end = time.perf_counter()+args.seconds
            deadline = previous = time.perf_counter()
            while time.perf_counter() < end:
                start = time.perf_counter()
                periods.append(start-previous); previous = start
                tracker.compute(selected.path, *pose, .05, 1.)
                controls.append(time.perf_counter()-start)
                deadline += .05
                time.sleep(max(0., deadline-time.perf_counter()))
        except Exception as error:
            errors.append(repr(error))
    thread = threading.Thread(target=control)
    thread.start(); barrier.wait()
    end = time.perf_counter()+args.seconds
    while time.perf_counter() < end:
        start = time.perf_counter()
        result, choices = planner.plan(*pose[:2], box, heading=pose[2], horizon_m=5.)
        planning.append(time.perf_counter()-start)
        if not result.valid or len(choices) != len(candidates):
            errors.append('planner workload changed')
        # Match the runtime interval measured from planning completion.
        time.sleep(.2)
    thread.join()
    report = dict(tracker=str(args.tracker), numpy=np.__version__,
                  affinity=sorted(os.sched_getaffinity(0)),
                  fixture='recorded box centroid with synthetic square edges',
                  candidates=len(candidates), path_points=len(selected.path),
                  planner=summary(planning), tracker_compute=summary(controls),
                  control_period=summary(periods[1:]), errors=errors,
                  planning_over_500ms=sum(v>.5 for v in planning),
                  control_compute_over_50ms=sum(v>.05 for v in controls))
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
