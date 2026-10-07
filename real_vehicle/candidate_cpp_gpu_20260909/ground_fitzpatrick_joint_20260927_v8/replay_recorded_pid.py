"""Replay recorded paths/poses, without ROS or actuator access.
Old logs lack exact controller input snapshots: recover pose by matching the
reported endpoint distance/yaw, and bound receipt-time dt and speed uncertainty.
This is recorded-input replay, never a counterfactual closed-loop vehicle test.
"""
import json, math, subprocess, argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.with_name('ground_381_2_0_joint_pid_20260926_v3')/'run_logs/20260926_225754-closed-frenet-ground-attempt'
def frames():
 t=json.loads((SOURCE/'tracking_trace.json').read_text());assets=json.loads((ROOT/'runtime_assets.json').read_text());goal=assets['route'][-1];paths={}
 for r in t['controller']:
  if 'local_path_snapshot' in r:paths[r['plan_id']]=r['local_path_snapshot']
 first=next(r for r in t['controller'] if r.get('speed_command',0)>0)
 bad=next(r for r in t['controller'] if r['received_mono']>=first['received_mono'] and not r['valid'])
 rows=[r for r in t['controller'] if first['received_mono']<=r['received_mono']<=bad['received_mono']]
 result=[];last_speed=0
 for i,r in enumerate(rows):
  candidates=[p for p in t['poses'] if abs(p['received_mono']-r['received_mono'])<.15]
  def err(p):
   return abs(math.hypot(p['x']-goal[0],p['y']-goal[1])-r['endpoint_distance_m'])+abs(math.atan2(math.sin(p['yaw']-goal[2]-r['endpoint_heading_error_rad']),math.cos(p['yaw']-goal[2]-r['endpoint_heading_error_rad'])))
  pose=min(candidates,key=err);diag=r.get('tracker_diagnostics') or {};last_speed=diag.get('measured_speed_mps',last_speed)
  dt=.05 if i==0 else min(.05,max(.04,r['runtime_timing']['tick_start_mono']-rows[i-1]['runtime_timing']['tick_start_mono']))
  call=dict(op='joint',type='pid',reset=i==0,path=paths[r['plan_id']],pose=[pose['x'],pose['y'],pose['yaw']],speed=last_speed,remaining=r['lap_length_m']-r['lap_progress_m'],dt=dt,obstacles=r['obstacle_points'])
  result.append(dict(input=call,recorded=r,pose_match_error=err(pose),speed_inferred=not bool(diag)))
 return result

def run(root=ROOT,tuning=None,prediction=None,constant_dt=False):
 rows=frames();proc=subprocess.Popen([str(root/'bin/core_probe'),str(root/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);trace=[]
 try:
  for i,row in enumerate(rows):
   call=row['input'].copy()
   if tuning is not None:call['pid_tuning']=tuning
   if prediction is not None:call['pid_prediction_s']=prediction
   if constant_dt:call['dt']=.05
   proc.stdin.write(json.dumps(call)+'\n');proc.stdin.flush();out=json.loads(proc.stdout.readline())
   trace.append(dict(index=i,t=row['recorded']['received_mono'],plan_id=row['recorded']['plan_id'],pose_match_error=row['pose_match_error'],speed_inferred=row['speed_inferred'],recorded_angle=row['recorded']['steering_command'],recorded_valid=row['recorded']['valid'],input=call,output=out))
   if not out['ok']:break
 finally:proc.stdin.close();proc.wait(timeout=5)
 errors=[abs(r['recorded_angle']-r['output']['angle']) for r in trace if r['output']['ok'] and r['recorded_valid']]
 report=dict(source=str(SOURCE),release=root.name,count=len(trace),input_count=len(rows),first_failure=trace[-1]['output'].get('error'),failure_index=trace[-1]['index'] if not trace[-1]['output']['ok'] else None,max_pose_match_error=max(r['pose_match_error'] for r in trace),mean_steering_difference_rad=sum(errors)/len(errors),max_steering_difference_rad=max(errors),tuning=tuning,prediction=prediction,constant_dt=constant_dt)
 return report,trace
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--constant-dt',action='store_true');a=p.parse_args();report,trace=run(constant_dt=a.constant_dt)
 (ROOT/'run_logs/recorded_replay.json').write_text(json.dumps(dict(summary=report,trace=trace),indent=2)+'\n');print(json.dumps(report,indent=2))
