#!/usr/bin/env python3
"""Read archived speed-screen trials; do not change acceptance or raw results."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def finite(rows, key):
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_results', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    batch = json.loads(args.batch_results.read_text())
    output = {'design': batch['design'], 'runs': []}
    for run in batch['runs']:
        directory = Path(run['results_dir'])
        summary = run.get('summary')
        if summary is None and (directory/'summary.json').exists():
            summary=json.loads((directory/'summary.json').read_text())
        if summary is None:
            output['runs'].append(dict(run, clean=False))
            continue
        with (directory/'trajectory.csv').open() as handle:
            rows = list(csv.DictReader(handle))
        invalid = sum(float(r['time_s']) > 2 and r['planner_valid'] == 'False'
                      for r in rows)
        cancellations = sum(r['behavior'] == 'CANCEL' and
                            (i == 0 or rows[i-1]['behavior'] != 'CANCEL')
                            for i, r in enumerate(rows))
        passing = [r for r in rows if r['behavior'] == 'OVERTAKE']
        config = json.loads((directory/'config.json').read_text())
        graph = json.loads((directory/'graph_audit.json').read_text())
        runtime = summary['runtime']
        entry = {k: run.get(k,config.get(k)) for k in ('scenario', 'run_id', 'seed',
                 'predictive_speed_profile', 'results_dir')}
        entry.update(passed=summary['passed'], clean=bool(summary['passed'] and
                     not invalid and not runtime['controller_failures'] and
                     not runtime['solver_timeouts'] and not runtime['sqp_timeouts']),
            truth_completion_time_s=(summary['overtaking']['truth_events'][0]['completion_time_s']
                if summary['overtaking']['truth_events'] else None),
            safety=summary['safety'], runtime=runtime,
            invalid_samples_after_2s=invalid, cancellations=cancellations,
            maximum_ego_speed_mps=max(finite(rows,'ego_speed_mps'), default=None),
            mean_overtake_speed_mps=(statistics.mean(finite(passing,'ego_speed_mps'))
                                    if passing else None),
            maximum_abs_mpc_acceleration_command_mps2=max(
                (abs(v) for v in finite(rows,'mpc_acceleration_command_mps2')), default=None),
            graph_audit=graph, runtime_source_sha256=config['runtime_source_sha256'])
        output['runs'].append(entry)
    rendered=json.dumps(output, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
        print(args.output)
    else:
        print(rendered)


if __name__ == '__main__':
    main()
