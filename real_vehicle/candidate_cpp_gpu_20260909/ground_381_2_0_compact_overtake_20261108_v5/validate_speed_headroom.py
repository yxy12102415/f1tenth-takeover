"""Conservative gain-error scenarios, not identified hardware dynamics."""
import json
from validate_joint_rollout import ROOT,rollout
reports=[]
for gain in (1.06,1.10):
 r,trace=rollout('dynamic_mpc',False,.15,.1,True,2.,gain)
 r['max_command_mps']=max(x['target'] for x in trace)
 r['overspeed_margin_verified']=r['max_speed']<2.1 and r['max_command_mps']<=1.85001
 reports.append(r);print(json.dumps(r),flush=True)
 (ROOT/'run_logs'/f'headroom_gain{gain}.json').write_text(json.dumps(dict(summary=r,trace=trace),indent=2)+'\n')
(ROOT/'run_logs/headroom_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
assert all(r['passed'] and r['overspeed_margin_verified'] for r in reports)
