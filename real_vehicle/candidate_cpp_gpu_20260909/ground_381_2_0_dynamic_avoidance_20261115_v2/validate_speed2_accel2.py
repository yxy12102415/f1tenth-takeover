"""No hardware: test speed/acceleration increase against plant-response variation."""
import json
from validate_joint_rollout import ROOT, rollout
reports=[]
for lag,servo,accel in ((.15,.10,2.),(.25,.15,2.),(.25,.15,1.)):
    report,trace=rollout('dynamic_mpc',False,lag,servo,True,accel)
    aa=[r['diag']['acceleration_command_mps2'] for r in trace]
    report.update(max_command_speed_mps=max(r['target'] for r in trace),
                  max_command_acceleration_mps2=max(aa),
                  min_command_acceleration_mps2=min(aa))
    assert all(0<=r['target']<=2.00001 for r in trace)
    assert min(aa)>=-.80001 and 1.1<max(aa)<=2.00001
    assert all(abs(b-a)<=.20001 for a,b in zip(aa,aa[1:]))
    report['command_constraints_passed']=True
    name=f'speed2_accel2_lag{lag}_plantaccel{accel}'
    (ROOT/'run_logs'/f'{name}.json').write_text(json.dumps(dict(summary=report,trace=trace),indent=2)+'\n')
    print(json.dumps(report),flush=True);reports.append(report)
(ROOT/'run_logs/speed2_accel2_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
assert all(r['passed'] for r in reports)
