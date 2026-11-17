"""Regression for stale motion during silhouette revelation, plus field counterfactual.
The captured-scene check is not a raw-scan replay or a completed physical test.
"""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1]
def sequence(binary,boxes):
 p=subprocess.Popen([str(R/binary),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);out=[]
 try:
  for n,(lo,hi) in enumerate(boxes):
   points=[[lo[0],lo[1]],[lo[0],hi[1]],[hi[0],lo[1]],[hi[0],hi[1]]]
   req=dict(stamp=100+n*.1,poses=[[0,0,0]],speed=0,clusters=[dict(id=1,points=points)],cap=2.15,max_offset=.65,allow_search=False)
   p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();v=json.loads(p.stdout.readline());out.append(v['occupancies'][0])
 finally:p.stdin.close();p.wait(timeout=5)
 return out
boxes=[]
for n in range(45):
 y=min(n,6)*.04;extra=.009*min(max(n-6,0),22)
 boxes.append(([2.6,-.15+y+extra],[2.9,.15+y]))
old=sequence('review/pre_exact_map_preview/generic_probe',boxes);new=sequence('bin/generic_probe',boxes)
assert old[25]['has_moved'] and abs(old[25]['vy'])>.1
assert not new[25]['has_moved'] and abs(new[25]['vy'])<1e-7,new[25]
moving=sequence('bin/generic_probe',[([1.5+n*.048,-.15],[1.8+n*.048,.15]) for n in range(35)])
assert all(v['has_moved'] and v['vx']>.35 for v in moving[10:]),moving[-1]
restart_boxes=boxes+[( [2.6+(n+1)*.048,-.15+.24+.198], [2.9+(n+1)*.048,.15+.24]) for n in range(12)]
restart=sequence('bin/generic_probe',restart_boxes)
assert all(v['has_moved'] and v['vx']>.35 for v in restart[-5:]),restart[-1]
f=json.loads((R/'review/field_search_budget_fixture.json').read_text());v=f['failed'];req=dict(stamp=v['source_scan_stamp'],poses=[v['scan_pose']],speed=f['last_valid']['control_input_snapshot']['speed'],plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,preferred_gap=.10,seed_reserve=.02,search_budget_ms=1000,initial_incumbent=json.loads((R/'review/field_search_held_path.json').read_text()),initial_offset=f['last_valid']['offset_m'])
for o in req['plan_occupancies']:
 if o['id']==8:o['has_moved']=False;o['vx']=o['vy']=0.
p=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True);counterfactual=json.loads(p.stdout);assert counterfactual['valid'],counterfactual
out=dict(passed=True,physical_motion=False,old_silhouette_frame25=old[25],new_silhouette_frame25=new[25],moving_frame=moving[-1],restart_frame=restart[-1],captured_scene_stationary_counterfactual=counterfactual,limitation='Synthetic memory sequence and captured-scene stationary counterfactual; no full raw-scan replay and no physical success claimed.')
(R/'review/stationary_settle_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(passed=True,old_false_speed=old[25]['vy'],new_speed=new[25]['vy'],moving_speed=moving[-1]['vx'],restart_speed=restart[-1]['vx'],field_stationary_counterfactual_valid=True)))
