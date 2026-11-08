"""Recorded pose/obstacle inputs with a replanned path; not a simulated drive."""
import json,subprocess,math
from replay_recorded_pid import ROOT,SOURCE,frames

def run(root):
 data=json.loads((SOURCE/'tracking_trace.json').read_text());rows=frames();limit=rows[-1]['recorded']['plan_id'];plans=[];seen=set()
 for row in data['controller']:
  pid=row.get('plan_id')
  if pid is None or pid in seen or pid>limit or 'local_path_snapshot' not in row or row.get('input_pose_stamp') is None:continue
  seen.add(pid);p=min(data['poses'],key=lambda p:abs(p['stamp']-row['input_pose_stamp']))
  assert abs(p['stamp']-row['input_pose_stamp'])<1e-5
  plans.append(dict(id=pid,recorded=row,pose=[p['x'],p['y'],p['yaw']]))
 proc=subprocess.Popen([str(root/'bin/core_probe'),str(root/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 def ask(call):
  proc.stdin.write(json.dumps(call)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
 output=[];pi=0;selected=None;plan_trace=[];previous_progress=0
 try:
  for frame in rows:
   r=frame['recorded']
   while pi<len(plans) and plans[pi]['id']<=r['plan_id']:
    rec=plans[pi];rr=rec['recorded']
    selected=ask(dict(op='plan',reset=pi==0,pose=rec['pose'],obstacles=rr['obstacle_points'],horizon=rr['lap_length_m']-previous_progress,candidates=True))
    previous_progress=rr['lap_progress_m'];pi+=1
    plan_trace.append(dict(id=rec['id'],recorded_offset=rr['offset_m'],recorded_clearance=rr['minimum_obstacle_clearance_m'],selected=selected))
   if selected is None or not selected.get('valid'):
    output.append(dict(plan_id=r['plan_id'],output=dict(ok=False,error='plan_invalid')));break
   call=frame['input'].copy();call['path']=selected['path'];out=ask(call)
   output.append(dict(plan_id=r['plan_id'],input=call,output=out))
   if not out['ok']:break
 finally:proc.stdin.close();proc.wait(timeout=5)
 summary=dict(release=root.name,frames=len(output),total_frames=len(rows),failure=output[-1]['output'].get('error'),last_plan_id=output[-1]['plan_id'],selected_offset=selected['offset_m'],selected_clearance=selected['minimum_obstacle_clearance_m'],selected_footprint=selected.get('minimum_footprint_point_distance_m'),limitation='Uses recorded vehicle poses; cannot represent the changed physical trajectory after replanning.')
 return dict(summary=summary,trace=output,plans=plan_trace)
if __name__=='__main__':
 out=run(ROOT);(ROOT/'run_logs/replanned_recorded_inputs.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out['summary'],indent=2))
