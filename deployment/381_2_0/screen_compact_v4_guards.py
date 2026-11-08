#!/usr/bin/env python3
"""Check candidate native sweeps with the actual compact occupied-body guard.

Synthetic opponent returns, not physical raw-scan replay. No ROS or devices.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

D = Path(__file__).resolve().parent
R = Path('/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v4')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('candidates', nargs='+')
    ap.add_argument('--assets', type=Path, default=R/'runtime_assets.json')
    ap.add_argument('--output', default='final_geometry_guard_screen.json'); args = ap.parse_args()
    root = D/'reports/compact_v4_design'; base = json.loads((R/'compact_config.json').read_text())
    o = base['path'][0]; c, s = math.cos(o[2]), math.sin(o[2])
    def pose(x, y=0., yaw=0.): return [o[0]+c*x-s*y, o[1]+s*x+c*y, o[2]+yaw]
    reports = []
    for name in args.candidates:
        r = json.loads((root/(name+'.json')).read_text()); spec = r['spec']
        if not r['summary']['passed']: continue
        asset = json.loads(args.assets.read_text())
        length = math.hypot(asset['route'][-1][0]-o[0], asset['route'][-1][1]-o[1])
        cfg = dict(base, path=r['path'], compact_geometry_v4=True, lead_initial_center_distance_m=1.8,
                   route_length_m=length,
                   nominal_trace=[[q['t'], q['x'], q['y'], q['yaw']] for q in r['trace']],
                   geometry=dict(entry_m=spec['entry_m'], hold_m=spec['hold_m'], exit_m=spec['exit_m'], offset_m=-spec['offset_m']))
        cf = root/(name+'.checked.config.json'); cf.write_text(json.dumps(cfg)+'\n'); cases = []
        for center_y, speed, stop in [(0., .5, 3.), (-.04, .5, 3.), (0., 0., 3.), (0., .5, 100.)]:
            p = subprocess.Popen([str(D/'build/checks/compact_probe_v4'), str(args.assets), str(cf)],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
            def ask(q):
                p.stdin.write(json.dumps(q)+'\n'); p.stdin.flush(); return json.loads(p.stdout.readline())
            def tracks(cx, cy, front=False):
                return [dict(id=5, points=[pose(cx+(.15 if front else -.15), cy+dy)[:2] for dy in [-.09, 0., .09]])]
            for k in range(7):
                ans = ask(dict(stamp=100+.2*k, pose=[pose(0)], enable=False, tracks=tracks(1.8, center_y)))
            prev = pose(0); lead = ans.get('lead_bounds'); failure = None; checked = 0
            if not ans['ok']: failure = dict(phase='admission', answer=ans)
            else:
                for i, row in enumerate(r['trace']):
                    t = row['t']-.05; now = 101.4+t
                    if i % 4 == 0:
                        coast = min(max(t-stop, 0.), speed/.4)
                        cx = 1.8+speed*min(t, stop)+speed*coast-.2*coast*coast
                        cy = center_y+.008*math.sin(t*8)
                        ex = (prev[0]-o[0])*c+(prev[1]-o[1])*s
                        ans = ask(dict(stamp=now, pose=[prev], enable=True, tracks=tracks(cx, cy, ex > cx)))
                        if not ans['ok']: failure = dict(phase='association', time=t, answer=ans); break
                        lead = ans['lead_bounds']
                    ans = ask(dict(op='check', lead=lead, prediction=row['diag']['predicted_path'], now=now))
                    if not ans['ok']: failure = dict(phase='prediction', time=t, answer=ans); break
                    checked += 1; prev = [row['x'], row['y'], row['yaw']]
            p.stdin.close(); p.wait(timeout=5)
            cases.append(dict(center_y=center_y, lead_speed=speed, lead_stop_after=stop,
                              passed=failure is None, checked=checked, failure=failure))
        report = dict(candidate=name, initial_center_gap_m=1.8, passed=all(q['passed'] for q in cases), cases=cases)
        reports.append(report); print(json.dumps(report), flush=True)
    (root/args.output).write_text(json.dumps(dict(scope=__doc__, candidates=reports), indent=2)+'\n')


if __name__ == '__main__': main()
