"""Native lateral feedback invariants for the new PD+feedforward structure."""
import json,math
from validate_pid_tuning import ROOT,Native
n=Native(ROOT)
# In-map artificial straight segment used only by the tracker probe.
p=[[i*.05,0.,0.] for i in range(50)]
def call(pose,reset=False,speed=1.0,path=p,dt=.05):
 return n.ask(dict(op='track',type='pid',path=path,pose=pose,reset=reset,speed=speed,dt=dt))
try:
 center=call([.5,0.,0.],True);assert abs(center['angle'])<1e-10
 left=call([.5,.05,0.],True);assert left['angle']<0
 right=call([.5,-.05,0.],True);assert right['angle']>0
 turning=call([.5,0.,.10],True);assert turning['angle']<0
 # A path shift cannot produce a numerical derivative spike: D uses heading,
 # not a time difference of the changed cross-track target or lookahead point.
 call([.5,0.,0.],True)
 shifted=call([.5,0.,0.],path=[[x,.04,yaw] for x,y,yaw in p]);assert abs(shifted['diag']['pid_d_rad'])<1e-10
 assert abs(shifted['angle'])<=.0550001
 angles=[]
 for i in range(60):
  r=call([.5,.025*math.sin(i),.04*math.sin(i*.7)],reset=i==0,dt=.041 if i%3==0 else .05);angles.append(r['angle'])
  assert abs(r['angle'])<=.269001
  if i:assert abs(angles[-1]-angles[-2])<=1.1*(.041 if i%3==0 else .05)+1e-8
 clean=call([.5,0.,0.],True);assert abs(clean['angle'])<1e-10 and clean['diag']['servo_estimate_rad']==0
 result=dict(passed=True,correct_error_sign=True,reference_step_without_derivative_kick=True,reset_clears_prediction=True,jitter_slew_bounds=True)
finally:n.close()
(ROOT/'run_logs/pid_response_contract.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
