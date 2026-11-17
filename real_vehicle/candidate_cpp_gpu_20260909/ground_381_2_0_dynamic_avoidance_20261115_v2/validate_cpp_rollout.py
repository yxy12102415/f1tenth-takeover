"""Native planner/tracker model rollout; Python integrates the test plant.

This reuses the v3 longitudinal oracle, which is checked separately through
ROS integration. It is not real vehicle or tire identification evidence.
"""
import json,subprocess
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import validate_dynamic_rollout as original
ROOT=Path(__file__).resolve().parent
class Native:
 def __init__(self,kind):
  self.kind=kind
  self.process=subprocess.Popen([str(ROOT/'bin/core_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 def ask(self,j):
  self.process.stdin.write(json.dumps(j)+'\n');self.process.stdin.flush()
  n=json.loads(self.process.stdout.readline())
  if not n['ok']:raise ValueError(n['error'])
  return n
 def close(self):self.process.stdin.close();self.process.wait(timeout=5)
class Planner:
 def __init__(self,*args,**kwargs):
  oracle=original.ClosedLoopLatticePlanner(*args,**kwargs)
  self.route=oracle.route;self.segment_m=oracle.segment_m;self.first=True;self.progress_index=0;self.candidate_switch_count=0
 def plan(self,x,y,box,heading,horizon_m):
  n=native.ask(dict(op='plan',reset=self.first,pose=[x,y,heading],obstacles=box.tolist(),horizon=horizon_m));self.first=False
  self.progress_index=n['progress'];self.mode=n['mode'];self.candidate_switch_count=n['switches']
  return SimpleNamespace(valid=n['valid'],reason=n['reason'],path=np.asarray(n['path'])),[]
class Tracker:
 def __init__(self):self.first=True;self.last={}
 def reset(self):self.first=True;self.last={}
 def compute(self,path,x,y,heading,dt,speed_mps,lateral_speed_mps,yaw_rate_rad_s):
  n=native.ask(dict(op='track',type=native.kind,reset=self.first,path=path.tolist(),pose=[x,y,heading],dt=dt,speed=speed_mps,lateral=lateral_speed_mps,yaw_rate=yaw_rate_rad_s));self.first=False
  self.last=n['diag'];self.last.setdefault('mpc_solve_ms',self.last['controller_compute_ms'])
  if 'cross_track_error_m' not in self.last:
   r=path[np.argmin(np.linalg.norm(path[:,:2]-[x,y],axis=1))];self.last['cross_track_error_m']=float(-np.sin(r[2])*(x-r[0])+np.cos(r[2])*(y-r[1]))
  return n['angle']
if __name__=='__main__':
 original.LocalPathTracker=Tracker
 reports=[]
 for kind in ('pp','pid','dynamic_mpc'):
  for obstacle in (False,True):
   native=Native(kind)
   try:report,rows=original.rollout('vesc_estimate',obstacle,planner_class=Planner)
   finally:native.close()
   report['controller']=kind;report['implementation']='native_cpp_planner_and_tracker_python_test_plant'
   (ROOT/'run_logs'/('cpp_rollout_'+kind+('_box' if obstacle else '_clear')+'.json')).write_text(json.dumps(dict(summary=report,trace=rows),indent=2)+'\n')
   reports.append(report);print(json.dumps(report),flush=True)
 (ROOT/'run_logs/cpp_rollout_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
 if not all(r['passed'] for r in reports):raise SystemExit(1)
