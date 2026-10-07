"""Recorded failure replay + independent model-state continuity oracle; no hardware."""
import json, hashlib, math
from pathlib import Path
import numpy as np
from validate_pid_tuning import Native
ROOT=Path(__file__).resolve().parent
SOURCES=[('v4','20260927_215927'),('v7','20260927_230151')]
def check(source):
 data=json.loads(source.read_text());paths={};latest=None
 for r in data['controller']:
  if r.get('local_path_snapshot'):latest=r['local_path_snapshot']
  if latest is not None:paths[r.get('plan_id')]=latest
 start=next(r['received_mono'] for r in data['controller'] if r.get('speed_command',0)>0)
 bad=next(r['received_mono'] for r in data['controller'] if r['received_mono']>=start and not r['valid'])
 rows=[r for r in data['controller'] if start<=r['received_mono']<=bad and 'control_input_snapshot' in r]
 native=Native(ROOT);prev=np.zeros(2);err=0.;results=[];first_call=None
 try:
  for i,r in enumerate(rows):
   q=r['control_input_snapshot'];call=dict(op='joint',type='dynamic_mpc',reset=i==0,path=paths[r['plan_id']],pose=q['pose'],speed=q['speed'],lateral=q['lateral'],yaw_rate=q['yaw_rate'],remaining=q['remaining'],cap=q['cap'],dt=q['dt'],obstacles=r['obstacle_points'])
   if first_call is None:first_call=call.copy()
   out=native.ask(call);d=out['diag'];state=np.array([d['model_lateral_speed_mps'],d['model_yaw_rate_rad_s']])
   assert np.max(abs(state-prev))<1e-12
   assert d['command_derived_yaw_rate_rad_s']==q['yaw_rate']
   assert d['input_lateral_speed_mps']==q['lateral']
   dt=min(.05,q['dt']);v=max(q['speed'],.35);m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
   A=np.array([[-(cf+cr)/(m*v),-v-(cf*lf-cr*lr)/(m*v)],[-(cf*lf-cr*lr)/(iz*v),-(cf*lf*lf+cr*lr*lr)/(iz*v)]])
   b=np.array([cf/m,lf*cf/iz]);servo=d['initial_servo_estimate_rad']+dt/(.2+dt)*(out['angle']-d['initial_servo_estimate_rad'])
   z=np.linalg.solve(np.eye(2)-dt*A,state+dt*b*servo)
   blend=np.clip((q['speed']-.25)/.30,0,1);blend=blend*blend*(3-2*blend)
   next_v=q['speed']+dt/(.15+dt)*(out['speed']-q['speed'])
   expected=np.array([blend*z[0],blend*z[1]+(1-blend)*next_v/.25*math.tan(servo)])
   prev=np.array([d['next_model_lateral_speed_mps'],d['next_model_yaw_rate_rad_s']]);err=max(err,float(np.max(abs(prev-expected))))
   assert err<1e-10
   assert 0<=out['speed']<=2.00001 and abs(out['angle'])<=.26901
   assert d['predicted_lateral_acceleration_limit_mps2']==1.65
   results.append(out)
  restart=native.ask(first_call)
  for key in ('model_lateral_speed_mps','model_yaw_rate_rad_s','initial_servo_estimate_rad'):assert restart['diag'][key]==0
  assert abs(restart['angle']-results[0]['angle'])<1e-8
 finally:native.close()
 return dict(passed=True,source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),frames=len(rows),model_oracle_max_error=err,reset_verified=True,unmodified_recorded_inputs=True,physical_counterfactual=False,last_diag=results[-1]['diag'])
if __name__=='__main__':
 reports=[]
 for version,stamp in SOURCES:
  report=check(ROOT.with_name('ground_fitzpatrick_joint_20260927_'+version)/'run_logs'/(stamp+'-closed-frenet-ground-attempt')/'tracking_trace.json');reports.append(report);print(version,report['frames'],'passed',flush=True)
 (ROOT/'run_logs/observer_replay.json').write_text(json.dumps(dict(passed=True,cases=reports),indent=2)+'\n')
