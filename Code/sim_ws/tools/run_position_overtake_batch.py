#!/usr/bin/env python3
"""Serial, seeded location screen; retain failures and report actual entry locations."""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import random
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = [f"fitzpatrick_position_{s}" for s in ("00", "25", "50", "75")]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--schedule-seed', type=int, default=20261024)
    p.add_argument('--prefix', default='positions_' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    args = p.parse_args()
    if args.repeats < 1 or not re.fullmatch(r'[A-Za-z0-9_-]+', args.prefix):
        p.error('Positive repeats and an alphanumeric/underscore/hyphen prefix required')
    output = ROOT / 'results/takeover_position_batches' / args.prefix
    output.mkdir(parents=True, exist_ok=False)
    rng = random.Random(args.schedule_seed)
    schedule = []
    for repeat in range(1, args.repeats+1):
        order = SCENARIOS.copy()
        rng.shuffle(order)
        schedule.extend(dict(scenario=s, repeat=repeat, run_id=f'{args.prefix}_repeat{repeat}') for s in order)
    design = dict(schedule_seed=args.schedule_seed, schedule=schedule,
                  expected='single_pass_and_rejoin', ego_start_delay_s=0.,
                  note='Exploratory location screen. Each fresh process is one run; CSV samples are not replicates. '
                       'Same route gap, speed caps and duration. Entry is decided online, not forced at spawn.')
    (output/'schedule.json').write_text(json.dumps(design, indent=2)+'\n')
    env = dict(os.environ, TAKEOVER_PLANNING_BACKEND='upstream_sqp', TAKEOVER_EGO_START_DELAY_S='0')
    reports = []
    for trial in schedule:
        print(f"Running {trial['scenario']} repeat {trial['repeat']}", flush=True)
        result = subprocess.run(['bash', 'run_takeover_trial.sh', trial['scenario'], '',
                                 str(trial['repeat']), trial['run_id'], 'false'], cwd=ROOT, env=env)
        directory = ROOT/'results/takeover'/trial['scenario']/trial['run_id']
        report = dict(trial, exit_code=result.returncode, results_dir=str(directory))
        if (directory/'summary.json').exists():
            summary = json.loads((directory/'summary.json').read_text())
            config = json.loads((directory/'config.json').read_text())
            report.update(passed=summary['passed'], safety=summary['safety'],
                          overtaking=summary['overtaking'], runtime=summary['runtime'],
                          start_route_fraction=config['start_route_fraction'])
            with (directory/'trajectory.csv').open() as f:
                rows = list(csv.DictReader(f))
            entries = [r for i, r in enumerate(rows) if r['behavior']=='OVERTAKE' and
                       (i==0 or rows[i-1]['behavior']!='OVERTAKE')]
            report['entries'] = [{k:float(r[k]) for k in ('time_s', 'ego_x', 'ego_y', 'ego_route_index_unwrapped')}
                                 for r in entries]
        else:
            report.update(passed=False, error='No completed summary; inspect preserved build/launch logs')
        reports.append(report)
        (output/'results.json').write_text(json.dumps(dict(design=design, runs=reports), indent=2)+'\n')
    print(f'Results: {output}', flush=True)
    return 0 if all(r['passed'] and r['exit_code']==0 for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
