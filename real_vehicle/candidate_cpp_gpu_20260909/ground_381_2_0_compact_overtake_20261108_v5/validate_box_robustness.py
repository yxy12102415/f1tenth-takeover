"""Extra box positions and scan perturbations around the observed footprint."""
import json,argparse
from validate_pid_tuning import ROOT,BASE,rollout
cases=[dict(box_center=(x,y),box_half=half,scan_noise=noise) for x,y,half,noise in [(2.67,-.21,.07,0),(2.84,-.24,.065,0),(2.84,-.24,.08,0),(2.75,-.21,.08,.01),(2.75,-.26,.08,.01),(2.84,-.24,.08,.01)]]
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');a=p.parse_args();root=BASE if a.baseline else ROOT;reports=[]
 for i,c in enumerate(cases):
  for delayed in (False,True):
   r,trace=rollout(root,obstacle=True,steering_delay=.1 if delayed else 0,pose_delay=.05 if delayed else 0,**c);r.update(case=i,delayed=delayed);reports.append(r)
   (ROOT/'run_logs'/f'box_{"baseline" if a.baseline else "candidate"}_{i}_{int(delayed)}.json').write_text(json.dumps(dict(summary=r,trace=trace),indent=2)+'\n');print(json.dumps(r),flush=True)
 (ROOT/'run_logs'/f'box_{"baseline" if a.baseline else "candidate"}_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
 if not a.baseline and not all(r['passed'] for r in reports):raise SystemExit(1)
