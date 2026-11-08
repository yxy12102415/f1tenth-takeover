#!/usr/bin/env python3
"""Replay recorded launch inputs through the original moving solver rejection.

No ROS/devices. Recorded states stay fixed; this is not a closed-loop rollout.
"""
import argparse
import json
import subprocess
from pathlib import Path

D = Path(__file__).resolve().parent
ROOT = Path('/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909')
SOURCE = ROOT/'ground_381_2_0_compact_overtake_20261108_v3'
LOG = SOURCE/'run_logs/20261108_111120-closed-frenet-ground-attempt'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--release', type=Path, required=True)
    ap.add_argument('--name', required=True)
    ap.add_argument('--probe', type=Path)
    args = ap.parse_args()
    data = json.loads((LOG/'tracking_trace.json').read_text())['controller']
    path = json.loads((SOURCE/'compact_config.json').read_text())['path']
    p = subprocess.Popen([str(args.probe or args.release/'bin/core_probe'), str(SOURCE/'runtime_assets.json')],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    rows = []; previous_failed = True
    try:
        for item in data:
            at = item['received_mono']
            if not 2437.30 <= at <= 2439.483: continue
            v = item.get('control_input_snapshot')
            if not v: continue
            # Distinct control ticks may reuse the same still-fresh sensor
            # sample while internal applied-command state advances. Keep them.
            # Actual output window began at 2438.07574325; its 0.10 s hold
            # was still preview-only. A preview solve is reset on every tick.
            preview = at < 2438.17574325
            q = {k: v[k] for k in ('pose', 'speed', 'lateral', 'yaw_rate', 'remaining', 'cap', 'dt')}
            q.update(op='joint', type='dynamic_mpc', path=path,
                     reset=preview or previous_failed, obstacles=item.get('obstacle_points', []))
            p.stdin.write(json.dumps(q)+'\n'); p.stdin.flush()
            result = json.loads(p.stdout.readline()); previous_failed = not result['ok']
            diag = result.get('diag', result.get('rejected_diagnostics', {}))
            rows.append(dict(at=at, preview=preview, original_reason=item['reason'],
                             ok=result['ok'], error=result.get('error'),
                             equality=diag.get('equality_residual'), inequality=diag.get('inequality_residual'),
                             iterations=diag.get('mpc_iterations'), compute_ms=diag.get('controller_compute_ms'),
                             warm_start=diag.get('warm_start_policy')))
    finally:
        p.stdin.close(); p.wait(timeout=5)
    summary = dict(release=str(args.release), source=str(LOG), scope=__doc__,
                   samples=len(rows), moving_samples=sum(not r['preview'] for r in rows),
                   moving_failures=[r for r in rows if not r['preview'] and not r['ok']],
                   last=rows[-1], passed=all(r['ok'] for r in rows if not r['preview']))
    (D/'reports/compact_v4_design'/(args.name+'.json')).write_text(json.dumps(dict(summary=summary, rows=rows), indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__': main()
