"""Coast sensitivity and unchanged-planner checks; no ROS or hardware."""
import json, numpy as np
from validate_pid_tuning import ROOT,BASE,Native,rollout
reports=[]
for decel in (.4,.8):
 r,t=rollout(ROOT,True,coast=decel);reports.append(r)
 (ROOT/'run_logs'/f'coast_{decel}.json').write_text(json.dumps(dict(summary=r,trace=t)))
 print(json.dumps(r),flush=True)
(ROOT/'run_logs/coast_sensitivity.json').write_text(json.dumps(reports,indent=2))
assert all(r['passed'] for r in reports)
# Prefer additional tracking room on identical recorded poses; this is a
# deliberate planner policy change, not a path-equivalence claim.
data=json.loads((BASE/'run_logs/20260927_000416-closed-frenet-ground-attempt/tracking_trace.json').read_text())
frames=[r for r in data['controller'] if r.get('valid') and r.get('speed_command',0)>0][::5]
new=Native(ROOT);count=0;buffered=0
try:
 for r in frames:
  pose=min(data['poses'],key=lambda p:abs(p['received_mono']-r['received_mono']))
  arg=dict(op='plan',reset=True,pose=[pose['x'],pose['y'],pose['yaw']],obstacles=r['obstacle_points'],horizon=3.2,candidates=True)
  b=new.ask(arg)
  if not b['valid']:continue
  distance=b.get('minimum_footprint_point_distance_m')
  assert distance is None or distance>=.15
  if distance is None or distance>=.18:buffered+=1
  count+=1
finally:new.close()
assert count>0
(ROOT/'run_logs/planner_margin.json').write_text(json.dumps(dict(passed=True,valid_cases=count,buffered_cases=buffered,source='Recorded poses/obstacle points, reset per sample; full closed-loop checks remain required.'),indent=2))
print('Planner margin samples:',count,buffered)
