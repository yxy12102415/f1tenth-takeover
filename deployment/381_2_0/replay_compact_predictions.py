#!/usr/bin/env python3
"""Replay actual native MPC sweeps against synthetic moving lead observations.

No ROS or hardware. The range clusters are synthetic, so passing does not
validate detection/association of the physical ESP32 vehicle.
"""
import argparse,json
from test_compact_contract import Probe,D,pose,track,origin,c,s

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--rollout',type=__import__('pathlib').Path,default=D/'reports/compact_mpc_rollout/wide_125.json');ap.add_argument('--name',default='compact_prediction_replay');args=ap.parse_args()
    rollout=json.loads(args.rollout.read_text())
    cases=[]
    for speed in [0.,.1,.15,.3,.35]:
        p=Probe();r=p.prime();lead=r.get('lead_bounds');failure=None;checked=0;next_plan=0.
        previous=pose(0)
        try:
            if not r['ok']:raise RuntimeError('admission:'+str(r))
            for row in rollout['trace']:
                t=row['t']-.05;now=101.4+t
                if t+1e-8>=next_plan:
                    ego_x=(previous[0]-origin[0])*c+(previous[1]-origin[1])*s
                    target=1.6+speed*t
                    r=p.ask(dict(stamp=now,pose=[previous],tracks=[track(target,'front' if ego_x>target else 'rear')],enable=True))
                    if not r['ok']:raise RuntimeError('plan:'+str(r))
                    lead=r['lead_bounds'];next_plan=t+.2
                r=p.ask(dict(op='check',lead=lead,prediction=row['diag']['predicted_path'],now=now))
                if not r['ok']:raise RuntimeError('sweep:'+str(r))
                checked+=1;previous=[row['x'],row['y'],row['yaw']]
        except RuntimeError as e:failure=str(e)
        finally:p.close()
        result=dict(lead_speed_mps=speed,passed=failure is None,checked_sweeps=checked,failure=failure)
        cases.append(result);print(json.dumps(result),flush=True)
    (D/'reports'/(args.name+'.json')).write_text(json.dumps(dict(cases=cases,physical_motion=False,scope=__doc__),indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in cases) else 1)
if __name__=='__main__':main()
