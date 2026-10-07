"""Two Fitzpatrick obstacles: native planner/controller plus angular scan sampling.
No hardware; lagged independent plant. Circle returns include map occlusion,
3-point clustering and 2-frame confirmation via native perception at 5 Hz.
"""
import json,math
from pathlib import Path
import numpy as np
import validate_pid_tuning as model
ROOT=Path(__file__).resolve().parent
assets=json.loads((ROOT/'runtime_assets.json').read_text());route=np.array(assets['route']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
centers=[route[np.argmin(abs(arc-s)),:2] for s in (6.,36.)]
clearance=np.array(assets['clearance']).reshape(assets['height'],assets['width']);origin=np.array(assets['origin'][:2]);res=assets['resolution']
truth=[]
for c in centers:
 for a in np.linspace(0,2*math.pi,64,endpoint=False):truth.append((c+.05*np.array([math.cos(a),math.sin(a)])).tolist())
Original=model.Native
class Sensed(Original):
 def __init__(self,root):super().__init__(root);self.frame=0;self.points=[];self.detections=[]
 def ask(self,j):
  if j['op']=='plan':
   x,y,h=j['pose'];laser=np.array([x,y])+.27*np.array([math.cos(h),math.sin(h)]);angles=np.linspace(-2.35,2.35,1081)+h;dirs=np.c_[np.cos(angles),np.sin(angles)];ranges=np.full(len(angles),np.inf)
   for c in centers:
    v=c-laser;projection=dirs@v;disc=.05**2-(v@v-projection**2);ok=(disc>=0)&(projection>0);t=np.full(len(angles),np.inf);t[ok]=projection[ok]-np.sqrt(disc[ok]);ranges=np.minimum(ranges,t)
   points=[];beams=[]
   for i in np.flatnonzero((ranges>=.05)&(ranges<=7)):
    p=laser+ranges[i]*dirs[i];samples=laser+np.arange(.05,ranges[i],.05)[:,None]*dirs[i];cols=np.floor((samples[:,0]-origin[0])/res).astype(int);rows=assets['height']-1-np.floor((samples[:,1]-origin[1])/res).astype(int)
    valid=(cols>=0)&(cols<assets['width'])&(rows>=0)&(rows<assets['height'])
    if not valid.all() or np.any(clearance[rows,cols]<=.05):continue
    points.append(p.tolist());beams.append(int(i))
   seen=super().ask(dict(op='perception',reset=self.frame==0,points=points,beams=beams,stamp=1.+self.frame*.2));self.frame+=1;self.points=seen['points']
   self.detections.append(dict(frame=self.frame,pose=j['pose'],raw_points=len(points),confirmed_points=len(self.points)))
  if j['op'] in ('plan','joint'):j=dict(j,obstacles=self.points)
  return super().ask(j)
 def close(self):
  super().close();(ROOT/'run_logs/two_obstacle_detection.json').write_text(json.dumps(self.detections)+'\n')
model.Native=Sensed
reports=[]
planned=[c.copy() for c in centers]
# Surface-cluster medians are only approximate fixture centers, not surveyed truth.
recorded=[np.array([6.11684,1.60322]),np.array([-1.24711,7.58771])]
for name,lag,servo,gain,delay,fixture in [('nominal',.2,.2,1.,0.,planned),('slow',.25,.3,1.,0.,planned),('recorded_approx',.2,.2,1.,0.,recorded),('recorded_response',.25,.3,1.1,.05,recorded)]:
 centers=fixture
 truth=[(c+.05*np.array([math.cos(a),math.sin(a)])).tolist() for c in centers for a in np.linspace(0,2*math.pi,64,endpoint=False)]
 result,trace=model.rollout(ROOT,True,lag=lag,servo_lag=servo,box_points=truth,steering_gain=gain,steering_delay=delay,kind='dynamic_mpc');result.update(case=name,box_centers=[x.tolist() for x in centers],diameters_m=[.1,.1],perception='1081 angular samples, native clustering and temporal confirmation, map occlusion',physical_tested=False)
 (ROOT/'run_logs'/('two_obstacle_'+name+'.json')).write_text(json.dumps(dict(summary=result,trace=trace))+'\n');reports.append(result);print(json.dumps(result),flush=True)
(ROOT/'run_logs/two_obstacle_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
assert all(x['passed'] for x in reports),reports
