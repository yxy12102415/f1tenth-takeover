#!/usr/bin/env python3
"""Non-actuating regression: scan/PF jitter must not collapse target identity.

Synthetic measurements reproduce the recorded interval collapse; these are
not an exact raw-scan replay (failed scans were not retained in the old logs).
"""
import json
import math
import os
import subprocess
from pathlib import Path

D = Path(__file__).resolve().parent
ROOT = Path('/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909')
R = ROOT/'ground_381_2_0_compact_overtake_20261108_v3'
OLD = ROOT/'ground_381_2_0_compact_overtake_20261107_v2'
origin = json.loads((R/'compact_config.json').read_text())['path'][0]
c, s = math.cos(origin[2]), math.sin(origin[2])


def pose(x, y=0., yaw=0.):
    return [origin[0]+c*x-s*y, origin[1]+s*x+c*y, origin[2]+yaw]


class Probe:
    def __init__(self, old=False):
        self.p = subprocess.Popen([os.environ.get('COMPACT_PROBE', str(D/'build/checks/compact_probe_v3')),
            str(R/'runtime_assets.json'), str((OLD if old else R)/'compact_config.json')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def ask(self, q):
        self.p.stdin.write(json.dumps(q)+'\n'); self.p.stdin.flush()
        return json.loads(self.p.stdout.readline())

    def update(self, t, x=1.60, y=-.1099, enable=False, present=True):
        tracks = [dict(id=5, points=[pose(x-.15, y+d)[:2] for d in [-.09, 0., .09]])] if present else []
        return self.ask(dict(stamp=t, pose=[pose(0)], enable=enable, tracks=tracks))

    def prime(self):
        for i in range(7):
            r = self.update(100+i*.2)
        assert r['ok'], r
        return r

    def close(self):
        self.p.stdin.close(); assert self.p.wait(timeout=5) == 0


def main():
    cases = []
    def record(name, passed, detail):
        cases.append(dict(name=name, passed=bool(passed), detail=detail))
        print(json.dumps(cases[-1]), flush=True)

    for old in [True, False]:
        p = Probe(old); r = p.prime()
        width = r['lead_bounds']['ymax']-r['lead_bounds']['ymin']
        p.update(101.4, enable=True)
        r = p.update(101.6, x=1.7, y=-.115, enable=True)
        r = p.update(101.8, x=1.8, y=-.115, enable=True)
        record('v2_reproduces_false_loss' if old else 'v3_accepts_visible_jittered_lead',
               r.get('error') == 'compact_target_lost' if old else r.get('ok'),
               dict(prior_lateral_width_m=width, result=r))
        p.close()

    for name, kwargs in [('true_loss', dict(present=False)),
                         ('wrong_lateral_object', dict(y=.4)),
                         ('implausible_forward_jump', dict(x=3.))]:
        p = Probe(); p.prime(); p.update(101.4, enable=True)
        r = p.update(101.8, enable=True, **kwargs)
        record(name, r.get('error') == 'compact_target_lost', r)
        r = p.update(102., enable=True)
        record(name+'_latched', r.get('error') == 'compact_latched_failure', r)
        p.close()

    p = Probe(); p.prime(); p.update(101.4, enable=True)
    ok = True; last = None
    for i in range(1, 21):
        last = p.update(101.4+.1*i, x=1.6+.05*i,
                        y=-.1099+(.008 if i % 2 else -.008), enable=True)
        ok = ok and last.get('ok', False)
        if not ok: break
    record('moving_visible_target_with_alternating_jitter', ok, last)
    p.close()

    p = Probe(); lead = p.prime()['lead_bounds']
    record('stationary_wait_does_not_invent_backward_motion', lead['xmin'] >= 1.55-1e-9, lead)
    for name, q, expected in [
        ('current_body_collision', dict(prediction=[pose(1.6)], now=101.2), 'compact_dynamic_body_conflict'),
        ('uncertain_lateral_prediction', dict(prediction=[pose(1.6, -.48)]*25, now=101.2), 'compact_dynamic_body_conflict'),
        ('stale_source_still_rejected', dict(prediction=[pose(0)], now=102.), 'compact_target_stale'),
        ('map_guard_retained', dict(prediction=[pose(6.8)], now=101.2), 'compact_full_body_map')]:
        r = p.ask(dict(op='check', lead=lead, **q))
        record(name, r.get('error') == expected, r)
    p.close()
    p = Probe(); p.prime(); p.update(101.4, enable=True)
    for i in range(1, 4):
        r = p.update(101.4+.2*i, x=1.6+.1*i, y=0., enable=True)
        assert r['ok'], r
    r = p.ask(dict(stamp=102.4, pose=[pose(2.7, -.6, .30)], enable=True, tracks=[]))
    record('brief_rear_visibility_gap_keeps_original_stamp', r.get('ok') and
           r.get('compact_association', {}).get('rear_visibility_hold') and
           r['lead_bounds']['stamp'] == 102., r)
    r = p.ask(dict(stamp=102.7, pose=[pose(2.7, -.6, .30)], enable=True, tracks=[]))
    record('rear_visibility_gap_does_not_extend_indefinitely', r.get('error') == 'compact_target_lost', r)
    p.close()
    p = Probe()
    rear = dict(xmin=3.25, xmax=3.45, ymin=-.05, ymax=.05, stamp=100.,
                half_length=.18, half_width=.13, vmax=.55, passed=False, bounded_association=True)
    for name, q, expected in [
        ('rear_visibility_prediction_bridge', dict(prediction=[pose(4.2, -.6, .30)]*25, now=100.6), None),
        ('rear_visibility_prediction_expires', dict(prediction=[pose(4.2, -.6, .30)]*25, now=100.8), 'compact_target_stale'),
        ('front_target_gets_no_rear_visibility_exception', dict(prediction=[pose(2.0, -.6, .30)]*25, now=100.6), 'compact_target_stale')]:
        r = p.ask(dict(op='check', lead=rear, **q))
        record(name, r.get('ok') if expected is None else r.get('error') == expected, r)
    p.close()
    (D/'reports/compact_v3_association_tests.json').write_text(json.dumps(cases, indent=2)+'\n')
    assert all(c['passed'] for c in cases)


if __name__ == '__main__':
    main()
