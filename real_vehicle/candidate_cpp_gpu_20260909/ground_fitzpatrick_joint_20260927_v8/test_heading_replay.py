"""Replay real wrapped-yaw failure; no ROS or actuator access."""
import json,math,subprocess,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.with_name('ground_fitzpatrick_joint_20260927_v2')/'run_logs/20260927_213237-closed-frenet-ground-attempt/tracking_trace.json'
def frames():
 data=json.loads(SOURCE.read_text());paths={};latest=None
 for r in data['controller']:
  if r.get('local_path_snapshot'):latest=r['local_path_snapshot']
  if latest is not None:paths[r.get('plan_id')]=latest
 start=next(r['received_mono'] for r in data['controller'] if r.get('speed_command',0)>0)
 bad=next(r['received_mono'] for r in data['controller'] if r['received_mono']>=start and r.get('reason')=='joint_mpc_solver_4')
 return [(r,paths[r['plan_id']]) for r in data['controller'] if start<=r['received_mono']<=bad and 'control_input_snapshot' in r]
def run(mode):
 proc=subprocess.Popen([str(ROOT/'bin/core_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);trace=[];prev=None
 try:
  for i,(r,path) in enumerate(frames()):
   q=r['control_input_snapshot'];pose=q['pose'].copy()
   if mode=='continuous' and prev is not None:pose[2]=prev+math.atan2(math.sin(pose[2]-prev),math.cos(pose[2]-prev))
   if mode=='alternating_equivalent':pose[2]+=2*math.pi*(1 if i%2 else -1)
   prev=pose[2]
   call=dict(op='joint',type='dynamic_mpc',reset=i==0,path=path,pose=pose,speed=q['speed'],lateral=q['lateral'],yaw_rate=q['yaw_rate'],remaining=q['remaining'],cap=q['cap'],dt=q['dt'],obstacles=r['obstacle_points'])
   proc.stdin.write(json.dumps(call)+'\n');proc.stdin.flush()
   while True:
    line=proc.stdout.readline()
    if not line:raise RuntimeError('probe_exited')
    try:out=json.loads(line);break
    except ValueError:pass
   trace.append(out)
   assert out['ok'],(mode,i,out)
 finally:proc.stdin.close();proc.wait(timeout=5)
 return trace
if __name__=='__main__':
 results={m:run(m) for m in ('wrapped','continuous','alternating_equivalent')};base=results['continuous'];diff={}
 for mode,trace in results.items():
  diff[mode]=max(abs(a[k]-b[k]) for a,b in zip(trace,base) for k in ('speed','angle'))
  assert diff[mode]<1e-4,(mode,diff[mode])
 report=dict(passed=True,input_count=len(base),source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),modes=list(results),maximum_command_difference=diff,known_old_failure='joint_mpc_solver_4 at +pi to -pi crossing',physical_retest_pending=True)
 (ROOT/'run_logs/heading_replay.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
