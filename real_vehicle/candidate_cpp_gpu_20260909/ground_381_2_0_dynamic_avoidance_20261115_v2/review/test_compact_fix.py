from pathlib import Path
import json, subprocess, copy
R=Path(__file__).resolve().parents[1]
T=json.loads((R/'run_logs/20261115_225052-closed-frenet-ground-attempt/tracking_trace.json').read_text())
bad=next(x for x in T['controller'] if x.get('reason')=='generic_predicted_occupancy_conflict')
row=next(x for x in T['controller'] if x.get('plan_id')==418 and x.get('occupancies'))
o=next(x for x in row['occupancies'] if x['id']==150)
now=bad['control_input_snapshot']['odom_stamp']+bad['input_age']['odom']['source_age_s']
pred=bad['rejected_control']['diagnostics']['predicted_path']
def probe(binary,request):
 p=subprocess.run([str(R/binary),str(R/'runtime_assets.json')],input=json.dumps(request)+'\n',capture_output=True,text=True,check=True)
 return json.loads(p.stdout)
req=dict(stamp=now,poses=[pred[0]],check_prediction=pred,occupancies=[o])
old=probe('review/pre_compact_fix/generic_probe',req);new=probe('bin/generic_probe',req)
assert old.get('reason')=='generic_predicted_occupancy_conflict',old
assert new['valid'],new
cases={'recorded_wall_residual_before':old,'recorded_wall_residual_after':new}
# Keep real overlap blocking, even when an object is tiny and map-supported.
direct=copy.deepcopy(o);direct.update(xmin=pred[0][0]-.01,xmax=pred[0][0]+.01,ymin=pred[0][1]-.01,ymax=pred[0][1]+.01)
v=probe('bin/generic_probe',dict(req,occupancies=[direct]));assert v.get('reason')=='generic_predicted_occupancy_conflict',v;cases['direct_overlap_still_rejected']=v
for label,changed in [('unmapped',dict(map_supported=False)),('moving',dict(has_moved=True,vx=.2)),('occluded_hazard',dict(visibility_hazard=True)),('larger_surface',dict(xmax=o['xmin']+.15))]:
 q=dict(o,**changed);rr=dict(req,occupancies=[q]);before=probe('review/pre_compact_fix/generic_probe',rr);after=probe('bin/generic_probe',rr)
 if label in ('unmapped','larger_surface'):
  # Operator-authorized zero static process growth changes these quiet,
  # nonhazard cases. Motion/unknown visibility still use the original fallback.
  assert after['valid'],(label,before,after);cases[label+'_zero_static_growth']=after
 else:
  assert before==after,(label,before,after);cases[label+'_unchanged']=after
# Replan a captured initial scene; compare departure, endpoint and early deviation.
initial=next(x for x in reversed(T['controller']) if x.get('received_mono',1e20)<640.062131878 and x.get('valid') and x.get('occupancies') and x.get('control_input_snapshot'))
pose=initial['control_input_snapshot']['pose']
request=dict(stamp=initial['source_scan_stamp'],poses=[pose],speed=0,plan_occupancies=initial['occupancies'],cap=2.15,max_offset=.65,search_budget_ms=1000,include_candidates=True)
for key,binary in [('old','review/pre_compact_fix/generic_probe'),('new','bin/generic_probe')]:
 v=probe(binary,request);cases[key+'_captured_initial_plan']=v
current=R/'review/current_placement_failed_snapshot.json'
if current.exists():
 snap=json.loads(current.read_text());v=snap['controller'];pose=[snap['pose'][k] for k in ['x','y','yaw']]
 request=dict(stamp=v['source_scan_stamp'],poses=[pose],speed=0,plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,search_budget_ms=200,include_candidates=True)
 fixed=probe('bin/generic_probe',request)
 assert fixed['valid'],fixed.get('candidate_reasons')
 assert fixed['maximum_curvature_1pm']<=1.14,fixed['maximum_curvature_1pm']
 cases['current_placement_replanned']=fixed
 wall=next(o for o in v['occupancies'] if o['id']==19)
 hold=[.7199307353040626,-.1897322080379354,-.23538543215278213]
 rr=dict(stamp=v['source_scan_stamp']+.3,poses=[hold],check_prediction=[hold]*25,occupancies=[wall])
 old=probe('review/pre_compact_fix/generic_probe',rr);new=probe('bin/generic_probe',rr)
 assert old.get('reason')=='generic_predicted_occupancy_conflict',old
 assert new['valid'],new
 cases['mapped_wall_occlusion_before']=old;cases['mapped_wall_occlusion_after']=new
 for label,override in [('moving_wall_support',dict(has_moved=True,vx=.2)),('unmapped_occlusion',dict(map_supported=False))]:
  q=dict(rr,occupancies=[dict(wall,**override)]);out=probe('bin/generic_probe',q)
  assert out.get('reason')=='generic_predicted_occupancy_conflict',out
  cases[label+'_still_rejected']=out
live=R/'review/ready_revalidation_failed_snapshot.json'
if live.exists():
 snap=json.loads(live.read_text());v=snap['observation'];pose=v['scan_pose']
 req=dict(stamp=v['source_scan_stamp'],poses=[pose],speed=0,plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,search_budget_ms=200,include_candidates=True)
 # This fixture is standstill PREARM, not a moving-control callback. Match
 # the production persistent process/cache and repeated zero-output prearm
 # planning. Keep the first cold failure as evidence, never call it a pass.
 child=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 attempts=[];out=None
 try:
  for attempt in range(4):
   child.stdin.write(json.dumps(req)+'\n');child.stdin.flush();out=json.loads(child.stdout.readline());attempts.append(out)
 finally:
  child.stdin.close();child.wait(timeout=5)
 assert all(v['valid'] for v in attempts[-2:]),[v.get('reason') for v in attempts]
 cases['ready_scene_cold_start']={'valid':attempts[0]['valid'],'reason':attempts[0].get('reason'),'scope':'standstill prearm only; no motion admitted until continuously valid'}
 cases['ready_scene_persistent_prearm']={'passed':True,'attempts':[{'valid':v['valid'],'reason':v.get('reason')} for v in attempts],'search_budget_ms_each':200,'cold_failure_not_counted_as_success':True}
 cases['ready_revalidation_scene_replanned']=out
(R/'review/compact_fix_regression.json').write_text(json.dumps(cases,indent=2)+'\n')
print(json.dumps({k:{z:v[z] for z in ['valid','reason','departure_s','first_blocking_s','offset'] if z in v} for k,v in cases.items()},indent=2))
