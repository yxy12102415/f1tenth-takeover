#!/usr/bin/env python3
"""Synthetic visible returns against stored native sweeps, not raw-scan replay."""
import json
import math
from test_compact_v3_association import D, Probe, pose, origin, c, s

trace = json.loads((D/'reports/compact_v2_design/final_length585.json').read_text())['trace']
reports = []
for ycenter in [0., -.04]:
    p = Probe()
    for k in range(7):
        r = p.update(100+.2*k, y=ycenter)
    prev = pose(0); lead = r['lead_bounds']; checked = 0; failure = None
    for i, row in enumerate(trace):
        t = row['t']-.05; now = 101.4+t
        if i % 4 == 0:
            coast = min(max(t-3, 0), .5/.4)
            cx = 1.6+.5*min(t, 3)+.5*coast-.2*coast*coast
            cy = ycenter+.008*math.sin(t*8)
            ex = (prev[0]-origin[0])*c+(prev[1]-origin[1])*s
            rx = cx+(.15 if ex > cx else -.15)
            tracks = [dict(id=5, points=[pose(rx, cy+dy)[:2] for dy in [-.09, 0., .09]])]
            r = p.ask(dict(stamp=now, pose=[prev], enable=True, tracks=tracks))
            if not r['ok']:
                failure = dict(phase='association', time=t, result=r); break
            lead = r['lead_bounds']
        r = p.ask(dict(op='check', lead=lead, prediction=row['diag']['predicted_path'], now=now))
        if not r['ok']:
            failure = dict(phase='prediction', time=t, result=r, lead=lead); break
        checked += 1; prev = [row['x'], row['y'], row['yaw']]
    p.close()
    reports.append(dict(ycenter=ycenter, passed=failure is None, checked=checked, failure=failure))
    print(json.dumps(reports[-1]), flush=True)
(D/'reports/compact_v3_prediction_replay.json').write_text(json.dumps(dict(scope=__doc__, cases=reports), indent=2)+'\n')
raise SystemExit(0 if all(x['passed'] for x in reports) else 1)
