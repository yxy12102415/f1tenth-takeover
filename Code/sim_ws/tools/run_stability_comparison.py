#!/usr/bin/env python3
"""Serial matched screen of legacy versus progressive passing-speed proposals.

Both arms use the same recovery fixes. Not a comparison with frozen 76d3398.
Each process is a replicate; keep all failed trials and do not change scoring.
"""
import argparse
import json
import os
from pathlib import Path
import random
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--schedule-seed', type=int, default=20261104)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--profile-comparison', action='store_true',
                        help='Compare legacy timing with per-stage passing and reachable braking')
    args = parser.parse_args()
    if args.repeats < 1 or not re.fullmatch(r'[A-Za-z0-9_-]+', args.prefix):
        parser.error('Require positive repeats and a safe, unique prefix')
    rng = random.Random(args.schedule_seed)
    schedule = []
    for repeat in range(1, args.repeats+1):
        positions = ['00', '25', '50', '75']
        rng.shuffle(positions)
        for position in positions:
            policies = [False, True]
            rng.shuffle(policies)
            for progressive in policies:
                arm = ('predictive_profile' if args.profile_comparison else 'progressive') if progressive else 'legacy_speed'
                schedule.append(dict(scenario=f'fitzpatrick_position_{position}',
                    repeat=repeat, seed=repeat,
                    progressive_pass_speed=progressive and not args.profile_comparison,
                    predictive_speed_profile=progressive and args.profile_comparison,
                    run_id=f'{args.prefix}_{arm}_r{repeat}'))
    design = dict(schedule_seed=args.schedule_seed, schedule=schedule,
        early_verified_return=False, profile_comparison=args.profile_comparison,
        unit='One fresh ROS/Gym process; not individual CSV samples',
        blocking='Matched position and seed; randomized arm order inside each block',
        scope='Small engineering screen, not a statistical success-rate estimate',
        acceptance='Existing single-pass/rejoin, body collision and boundary checks unchanged')
    if args.plan_only:
        print(json.dumps(design, indent=2))
        return 0
    output = ROOT/'results/takeover_stability_batches'/args.prefix
    output.mkdir(parents=True, exist_ok=False)
    (output/'schedule.json').write_text(json.dumps(design, indent=2)+'\n')
    reports = []
    for trial in schedule:
        print(f"Running {trial['scenario']} {trial['run_id']}", flush=True)
        env = dict(os.environ, TAKEOVER_PLANNING_BACKEND='upstream_sqp',
                   TAKEOVER_EGO_START_DELAY_S='0',
                   TAKEOVER_EARLY_VERIFIED_RETURN='false',
                   TAKEOVER_PREDICTIVE_SPEED_PROFILE=str(trial['predictive_speed_profile']).lower(),
                   TAKEOVER_PROGRESSIVE_PASS_SPEED=str(trial['progressive_pass_speed']).lower())
        # The existing runner supervises simulated duration, tears down its
        # process group, verifies graph isolation and preserves failed output.
        result = subprocess.run(['bash', 'run_takeover_trial.sh', trial['scenario'], '',
            str(trial['seed']), trial['run_id'], 'false'], cwd=ROOT, env=env)
        directory = ROOT/'results/takeover'/trial['scenario']/trial['run_id']
        report = dict(trial, exit_code=result.returncode, results_dir=str(directory))
        if (directory/'summary.json').exists():
            report['summary'] = json.loads((directory/'summary.json').read_text())
        else:
            report['error'] = 'No summary; inspect retained launch/build logs'
        reports.append(report)
        (output/'results.json').write_text(json.dumps(dict(design=design, runs=reports), indent=2)+'\n')
        # A launch/build error without completed CSV is infrastructure trouble:
        # stop rather than repeatedly launching onto a potentially stale graph.
        if not (directory/'trajectory.csv').exists():
            return 2
    print(f'Results: {output}', flush=True)
    return 0 if all(r['exit_code']==0 and r['summary']['passed'] for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
