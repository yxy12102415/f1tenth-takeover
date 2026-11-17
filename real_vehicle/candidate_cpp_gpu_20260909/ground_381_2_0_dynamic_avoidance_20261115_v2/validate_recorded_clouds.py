"""Closed-loop models with frozen visible scan points from the physical drive.
These points are an incomplete obstacle surface, not a ground-truth body map.
"""
import argparse,json,math
from validate_pid_tuning import ROOT,BASE,rollout
SOURCE=BASE/'run_logs/20260927_000416-closed-frenet-ground-attempt'
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');a=p.parse_args();root=BASE if a.baseline else ROOT;data=json.loads((SOURCE/'tracking_trace.json').read_text());reports=[]
 frames=[r for r in data['controller'] if r.get('valid') and r.get('speed_command',0)>0 and len([p for p in r.get('obstacle_points',[]) if math.hypot(p[0]-2.8,p[1]+.24)<.4])>=4]
 for progress in (1.7,2.3,2.8):
  frame=min(frames,key=lambda r:abs(r.get('lap_progress_m',0)-progress));pid=frame['plan_id']
  points=[p for p in frame['obstacle_points'] if math.hypot(p[0]-2.8,p[1]+.24)<.4]
  for delayed in (False,True):
   r,trace=rollout(root,obstacle=True,box_points=points,steering_delay=.1 if delayed else 0,pose_delay=.05 if delayed else 0)
   r.update(plan_id=pid,delayed=delayed,surface_only=True);reports.append(r)
   (ROOT/'run_logs'/f'cloud_{"baseline" if a.baseline else "candidate"}_{pid}_{int(delayed)}.json').write_text(json.dumps(dict(summary=r,trace=trace),indent=2)+'\n');print(json.dumps(r),flush=True)
 (ROOT/'run_logs'/f'cloud_{"baseline" if a.baseline else "candidate"}_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
 if not a.baseline and not all(r['passed'] for r in reports):raise SystemExit(1)
