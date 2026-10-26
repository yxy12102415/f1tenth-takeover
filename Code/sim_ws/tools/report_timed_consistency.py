#!/usr/bin/env python3
"""Audit a completed batch without altering trial evidence.

Unlike the original behavioral passed flag, explicitly report control failures
and invalid-planner samples after initialization. No acceptance is relaxed.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results_root', type=Path)
    parser.add_argument('run_prefix')
    parser.add_argument('--expected-runs', type=int)
    parser.add_argument('--require-clean', action='store_true',
                        help='Fail on incomplete/failed trials, controller failures, or invalid plans after 2 s')
    parser.add_argument('--output', type=Path, help='Write a new report; refuses overwrite')
    args = parser.parse_args()
    reports = []
    for p in sorted(args.results_root.glob('*/*/summary.json')):
        s = json.loads(p.read_text())
        if not s.get('run_id', '').startswith(args.run_prefix):
            continue
        trajectory = p.with_name('trajectory.csv')
        if not trajectory.exists():
            raise SystemExit(f'Incomplete run: {p.parent}')
        with trajectory.open(newline='') as handle:
            rows = list(csv.DictReader(handle))
        moving = [r for r in rows if float(r['time_s']) > 2.0]
        invalid = sum(r.get('planner_valid') == 'False' for r in moving)
        runtime = s['runtime']
        def peak(key):
            values=[float(r[key]) for r in rows if r.get(key) not in ('',None)]
            return max((v for v in values if math.isfinite(v)),default=None)
        config = json.loads(p.with_name('config.json').read_text())
        complete = bool(rows) and s['duration_s'] + .75 >= config['duration_s']
        clean = (complete and s['passed'] and runtime['controller_failures'] == 0
                 and runtime['solver_timeouts'] == 0 and runtime.get('sqp_timeouts', 0) == 0 and invalid == 0)
        reports.append({
            'scenario': s['scenario'], 'run_id': s['run_id'], 'seed': s['seed'],
            'duration_s': s['duration_s'], 'samples': s['samples'],
            'ego_max_speed_mps': config.get('ego_max_speed_mps'),
            'lead_speed_mps': config.get('lead_speed_mps'),
            'controller_model': config.get('controller_model','v8'),
            'ego_max_observed_mps': peak('ego_speed_mps'),
            'lead_max_observed_mps': peak('lead_speed_mps'),
            'working_diff_sha256': config.get('working_diff_sha256'),
            'runtime_source_sha256': config.get('runtime_source_sha256'),
            'planning_backend': config.get('planning_backend', 'frenet_candidates'),
            'adapter_source_sha256': config.get('adapter_source_sha256'),
            'sqp_calls': runtime.get('sqp_calls', 0),
            'sqp_failures': runtime.get('sqp_failures', 0),
            'sqp_timeouts': runtime.get('sqp_timeouts', 0),
            'sqp_preview_checks': runtime.get('sqp_preview_checks', 0),
            'sqp_preview_rejections': runtime.get('sqp_preview_rejections', {}),
            'track_assisted_prediction': config.get('track_assisted_prediction'),
            'timed_control': config.get('timed_control'),
            'configured_behavior_and_safety_passed': s['passed'],
            'complete_duration': complete,
            'clean_motion_passed': clean,
            'zero_controller_failures': runtime['controller_failures'] == 0,
            'planner_invalid_samples_after_2s': invalid,
            'controller_failures': runtime['controller_failures'],
            'solver_recoveries': runtime.get('solver_recoveries', 0),
            'solver_timeouts': runtime['solver_timeouts'],
            'freshness_triggered_replans': runtime.get('freshness_triggered_replans', 0),
            'planner_failures_total': runtime['planner_failures'],
            'controller_rejection_reasons': runtime.get('controller_rejection_reasons', {}),
            'return_preview_checks': runtime.get('return_preview_checks', 0),
            'return_preview_rejections': runtime.get('return_preview_rejections', {}),
            'overtaking': s['overtaking'], 'safety': s['safety'], 'control': s['control'],
            'planning_p95_ms': runtime['planning_p95_ms'],
            'mpc_p95_ms': runtime['mpc_p95_ms'],
            'planning_max_ms': peak('planning_ms'),
            'mpc_max_ms': peak('mpc_solve_ms'),
        })
    if not reports:
        raise SystemExit('No matching completed runs')
    count_matches = args.expected_runs is None or len(reports) == args.expected_runs
    result = {
        'run_prefix': args.run_prefix, 'runs': reports,
        'expected_runs': args.expected_runs, 'run_count_matches': count_matches,
        'all_clean': count_matches and all(r['clean_motion_passed'] for r in reports),
        'note': 'Planner-invalid samples are 20 Hz evaluation samples, not independent planner failures. '
                'Three repeats are repeatability checks, not broad generalization or vehicle approval. '
                'Clean-motion is stricter than configured behavior/safety acceptance; it is not '
                'appropriate for deliberately injected fail-stop tests.',
    }
    encoded = json.dumps(result, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as handle:
            handle.write(encoded)
        print(f"Wrote {args.output}: {len(reports)} runs, all_clean={result['all_clean']}")
    else:
        print(encoded)
    if not count_matches or (args.require_clean and not result['all_clean']):
        raise SystemExit('Strict audit failed; all original trial evidence was retained')


if __name__ == '__main__':
    main()
