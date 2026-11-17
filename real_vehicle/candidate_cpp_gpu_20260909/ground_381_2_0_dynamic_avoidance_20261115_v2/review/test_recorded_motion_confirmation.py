"""Temporal regression on recorded bounding supports, not a raw LiDAR replay."""
import json,subprocess,pathlib,copy,hashlib,os
R=pathlib.Path(__file__).resolve().parents[1]
f=json.loads((R/'review/dmpc_20261117_000736_fixture.json').read_text());rows=json.loads((R/'run_logs/20261117_000736-closed-frenet-ground-attempt/tracking_trace.json').read_text())['controller'];cut=next(o['stamp'] for o in f['occupancies'] if o['id']==8);timeline={}
for row in rows:
 for scene in [row,(row.get('control_input_snapshot') or {}).get('collision_guard_observation') or {}]:
  for o in scene.get('occupancies') or []:
   if o['id']==8 and o['stamp']<=cut:timeline[o['stamp']]=o
assert len(timeline)>15
proc=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
snapshots=[]
try:
 for stamp,o in sorted(timeline.items()):
  pts=[[o['xmin'],o['ymin']],[o['xmin'],o['ymax']],[o['xmax'],o['ymin']],[o['xmax'],o['ymax']]]
  q=dict(stamp=stamp,poses=[[0,0,0]],clusters=[dict(id=8,points=pts)],free_range=.3,allow_search=False)
  proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();v=json.loads(proc.stdout.readline());candidates=v.get('occupancies',[]);assert len(candidates)==1,(stamp,candidates);snapshots.append(candidates[0])
finally:proc.stdin.close();proc.wait(timeout=5)
last=snapshots[-1];assert last['fixed_world'] and not last['has_moved'] and last['vx']==0 and last['vy']==0,last
# Preserve the exact recorded geometry and all other objects; replace only the
# temporal classification learned from reconstructed support boxes.
g=copy.deepcopy(f)
for o in g['occupancies']:
 if o['id']==8:
  for k in ['fixed_world','has_moved','vx','vy','stationary_history']:o[k]=last[k]
g['scope']='Exact recorded state/path/geometry plus classification from recorded support-box sequence; cold MPC seed. Raw clusters and warm solver state were not captured.'
q=R/'review/dmpc_20261117_000736_filtered_fixture.json';q.write_text(json.dumps(g,indent=2))
p=subprocess.run([str(R/'bin/replay_corridor_failure'),str(R/'runtime_assets.json'),str(q),str(R/'compact_config.json')],capture_output=True,text=True,env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'));out=json.loads(p.stdout);assert out['valid'] and out['predicted_body_guard_passed'],out
unit=json.loads(subprocess.check_output([str(R/'bin/test_motion_confirmation'),str(R/'runtime_assets.json')],text=True))
report=dict(passed=True,physical_motion=False,scope=g['scope'],unit=unit,recorded_support_observations=len(snapshots),last_filtered_support=last,recorded_state_replay=out,source_sha256={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in ['src/cpp_frenet/tracked_occupancy.hpp','review/test_motion_confirmation.cpp','review/test_recorded_motion_confirmation.py']})
(R/'review/motion_confirmation_regression.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(passed=True,support_observations=len(snapshots),static_latched=True,recorded_state_replay_valid=True,equality_residual=out['diagnostics']['equality_residual'],motion_detection_delays_s=unit['start_detection_delays_s'])))
