"""Native speed policy regression with zero wheel feedback and a final STOP.
Also exercises the exact independent window (actual outgoing command ramp).
"""
import json,math
import numpy as np
from validate_pid_tuning import ROOT,Native
from stand_window import StandWindow
assets=json.loads((ROOT/'runtime_assets.json').read_text());route=np.array(assets['route']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
def pose_at(s):
 return [float(np.interp(s,arc,route[:,0])),float(np.interp(s,arc,route[:,1])),float(np.interp(s,arc,np.unwrap(route[:,2])))]
def call(n,s,speed,reset=False):
 return n.ask(dict(op='joint',type='dynamic_mpc',reset=reset,path=route.tolist(),pose=pose_at(s),speed=speed,lateral=0.,yaw_rate=0.,remaining=float(arc[-1]-s),dt=.05,obstacles=[]))
n=Native(ROOT);commands=[];diagnostics=[]
try:
 # Stationary/zero wheel feedback must still permit a bounded soft launch.
 for i in range(120):
  r=call(n,.05,0.,i==0);commands.append(r['speed']);diagnostics.append(r['diag'])
 assert commands[0]<=.01+1e-8 and max(commands)>.5
 assert all(commands[i]-commands[i-1]<=.050001 for i in range(1,len(commands)))
 # Persistent zero wheel feedback during approach must not prevent a latched
 # stop or produce positive sub-floor commands. Motion/endpoint accuracy is
 # checked separately with independent closed-loop plants and actual ROS.
 cut=None
 # Preserve the original short-route fixture's <=2 cm per 50 ms pose step.
 # A fixed 300 points would teleport along Fitzpatrick at nearly 4 m/s.
 for arc_pos in np.linspace(.05,arc[-1]-.15,max(300,int(math.ceil((arc[-1]-.20)/.02))+1)):
  r=call(n,float(arc_pos),0.);commands.append(r['speed']);diagnostics.append(r['diag'])
  if r['diag'].get('terminal_coasting') and cut is None:
   cut=dict(remaining_m=float(arc[-1]-arc_pos),previous_command_mps=commands[-2])
 assert cut is not None,('zero_feedback_prevented_stop',diagnostics[-1])
 assert cut['remaining_m']>.10,('late_zero_feedback_stop',cut)
 assert r['speed']==0
 # A pose correction placing the car farther away cannot re-arm the coast.
 for i in range(20):
  r=call(n,float(arc[-1]-.35),0.);commands.append(r['speed']);assert r['speed']==0
 reached=False
 for v in commands:
  if v>=.5:reached=True
  assert not (reached and 0<v<.5-1e-5)
finally:n.close()
# Independent actuator-facing window retains its original soft launch, then
# sends .5 directly to zero; final latch rejects any later positive command.
w=StandWindow();w.joy(False,0.);floor_seen=False;actual=[]
for i in range(150):
 now=i*.02;w.health(True,now);w.permit(True,now);w.joy(True,now);w.command(1. if i<100 else .5,0.,now);v,_,why=w.evaluate(now);actual.append(v)
 if v>=.5:floor_seen=True
 assert not (floor_seen and 0<v<.5-1e-5)
assert actual[0]<=.02 and actual[-1]==.5
w.command(0,0,3.);v,_,why=w.evaluate(3.);assert v==0 and why=='upstream_stop'
w.health(True,3.02);w.permit(True,3.02);w.joy(True,3.02);w.command(.5,0,3.02);assert w.evaluate(3.02)[0]==0
report=dict(passed=True,nominal_start_step_mps=commands[0],actuator_start_step_mps=actual[0],soft_start_preserved=True,zero_feedback_coast_cut=cut,minimum_positive_after_start_mps=min(v for v in commands[commands.index(next(v for v in commands if v>=.5)):] if v>0),terminal_zero_latched=True,actuator_zero_latched=True)
(ROOT/'run_logs/dmpc_speed_floor_contract.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
