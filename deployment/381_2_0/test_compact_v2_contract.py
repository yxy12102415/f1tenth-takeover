#!/usr/bin/env python3
"""Non-actuating tests of v2 admission, body guards and rear visibility."""
import json
import math
import os
import subprocess
from pathlib import Path

D = Path(__file__).resolve().parent
RELEASE = Path(os.environ.get('COMPACT_RELEASE', '/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261107_v2'))
config = json.loads((RELEASE/'compact_config.json').read_text())
origin = config['path'][0]
c, s = math.cos(origin[2]), math.sin(origin[2])


def pose(x, y=0., yaw=0.):
    return [origin[0]+x*c-y*s, origin[1]+x*s+y*c, origin[2]+yaw]


class Probe:
    def __init__(self):
        self.p = subprocess.Popen([os.environ.get('COMPACT_PROBE', str(D/'build/checks/compact_probe_v2')),
            str(RELEASE/'runtime_assets.json'), str(RELEASE/'compact_config.json')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def ask(self, q):
        self.p.stdin.write(json.dumps(q)+'\n')
        self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())

    def update(self, stamp, x=0., yaw=0., enable=False, present=True):
        tracks = [{'id': 1, 'points': [pose(1.45, y)[:2] for y in [-.09, 0., .09]]}] if present else []
        return self.ask(dict(stamp=stamp, pose=[pose(x, 0., yaw)], enable=enable, tracks=tracks))

    def prime(self):
        for k in range(7):
            r = self.update(100+k*.2)
        return r

    def close(self):
        self.p.stdin.close()
        assert self.p.wait(timeout=5) == 0


def main():
    results = []
    def record(name, ok, detail):
        results.append(dict(name=name, passed=bool(ok), detail=detail))
        print(json.dumps(results[-1]), flush=True)

    p = Probe()
    r = p.prime()
    record('launch_admitted', r.get('ok') and r.get('lead_bounds', {}).get('vmax') == .55, r)
    if not r.get('ok'):
        p.close()
        raise RuntimeError('Initial admission failed')
    lead = r['lead_bounds']
    for name, q, expected in [
        ('body_collision', dict(prediction=[pose(1.6)], now=101.2), 'compact_dynamic_body_conflict'),
        ('stale_measurement', dict(prediction=[pose(0)], now=102.), 'compact_target_stale'),
        ('clear_offset', dict(prediction=[pose(1.6, -.65)], now=101.2), None)]:
        r = p.ask(dict(op='check', lead=lead, **q))
        record(name, r.get('ok') if expected is None else r.get('error') == expected, r)
    p.update(101.4, enable=True)
    r = p.update(101.8, x=3., yaw=.60, enable=True, present=False)
    record('turning_body_fully_ahead_enters_rear_blind_mode', r.get('ok') and
           r.get('lead_bounds', {}).get('passed') and r['lead_bounds']['stamp'] == 101.4, r)
    if r.get('ok'):
        r = p.ask(dict(op='check', lead=r['lead_bounds'], prediction=[pose(2.6)], now=104.))
        record('rear_reachable_region_still_checked', r.get('error') == 'compact_dynamic_body_conflict', r)
    p.close()

    p = Probe()
    p.prime()
    p.update(101.4, enable=True)
    r = p.update(101.8, x=2., yaw=.60, enable=True, present=False)
    record('not_fully_ahead_still_stops_on_loss', r.get('error') == 'compact_target_lost', r)
    r = p.update(102., enable=True)
    record('loss_remains_latched', r.get('error') == 'compact_latched_failure', r)
    p.close()
    p = Probe()
    rear = dict(xmin=3.35, xmax=3.75, ymin=-.05, ymax=.05, stamp=100.,
                half_length=.18, half_width=.13, vmax=.55, passed=True)
    pred = [pose(5.65+.20*k/24) for k in range(25)]
    r = p.ask(dict(op='check', lead=rear, prediction=pred, now=102.2))
    record('completed_merge_checks_current_rear_body', r.get('ok'), r)
    r = p.ask(dict(op='check', lead=rear, prediction=[pose(5.55)]*25, now=102.2))
    record('incomplete_merge_keeps_future_body_guard', r.get('error') == 'compact_dynamic_body_conflict', r)
    r = p.ask(dict(op='check', lead=rear, prediction=[pose(6.8)]*25, now=102.2))
    record('completed_merge_still_checks_map', r.get('error') == 'compact_full_body_map', r)
    p.close()
    for stationary in [False, True]:
        p = Probe()
        p.prime()
        p.update(101.4, enable=True)
        p.update(101.8, x=3., yaw=.60, enable=True, present=False)
        r = p.ask(dict(stamp=108.5, pose=[pose(5.85)], enable=True,
                       tracks=[], stationary=stationary))
        record('stopped_arrival_recognized' if stationary else 'moving_endpoint_keeps_body_guard',
               r.get('ok') and r.get('observed_endpoint_stationary') if stationary
               else r.get('error') == 'compact_dynamic_body_conflict', r)
        p.close()
    Path(os.environ.get('COMPACT_TEST_REPORT', str(D/'reports/compact_v2_design/latest_native_contract.json'))).write_text(json.dumps(results, indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)


if __name__ == '__main__':
    main()
