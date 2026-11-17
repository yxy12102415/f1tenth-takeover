from pathlib import Path
import json,subprocess,copy,hashlib
R=Path(__file__).resolve().parents[1];f=json.loads((R/'review/static_occlusion_priority_field_fixture.json').read_text());d=f['candidate_collision_diagnostics'][0];o=d['occupancy'];assert o['fixed_world'] and o['visibility_hazard'] and not o['map_supported']
def ask(q,old=False):
 binary=R/('review/pre_static_occlusion_priority/generic_probe' if old else 'bin/generic_probe');p=subprocess.run([str(binary),str(R/'runtime_assets.json')],input=json.dumps(q)+'\n',text=True,capture_output=True,check=True);return json.loads(p.stdout)
q=dict(stamp=d['now'],poses=[d['ego_pose']],forecast_occupancy=o,future_s=d['future_s'],route_prior=True)
old=ask(q,True);new=ask(q);assert old['box'][0]<o['xmin']-.4,old;assert new['box']==[o[k] for k in ['xmin','ymin','xmax','ymax']],new
p=dict(stamp=d['now'],poses=[d['ego_pose']],check_prediction=[d['ego_pose']]*14,occupancies=[o]);before=ask(p,True);after=ask(p);assert before['reason']=='generic_predicted_occupancy_conflict' and after['valid'],(before,after)
unknown=copy.deepcopy(q);unknown['forecast_occupancy']['fixed_world']=False;u=ask(unknown);assert u['box'][0]<o['xmin']-.4,u
moving=copy.deepcopy(q);moving['forecast_occupancy'].update(fixed_world=False,has_moved=True,vx=.3);m=ask(moving);assert m['box'][0]<o['xmin'],m
req=dict(stamp=f['source_scan_stamp'],poses=[f['planning_pose']],speed=0.,plan_occupancies=f['occupancies'],cap=2.15,max_offset=.65,search_budget_ms=200,include_candidates=True)
child=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);runs=[]
try:
 for n in range(4):child.stdin.write(json.dumps(req)+'\n');child.stdin.flush();runs.append(json.loads(child.stdout.readline()))
finally:child.stdin.close();child.wait(timeout=5)
assert all(x['valid'] for x in runs[-2:]),[x.get('reason') for x in runs]
out=dict(passed=True,physical_motion=False,old_inflated_box=old['box'],new_static_box=new['box'],recorded_false_conflict_cleared=True,nonfixed_unknown_still_grows=True,moving_evidence_still_preserved=True,standstill_prearm=[dict(valid=x['valid'],reason=x['reason']) for x in runs],search_budget_ms_each=200,source_sha256=hashlib.sha256((R/'src/cpp_frenet/generic_avoidance.hpp').read_bytes()).hexdigest())
(R/'review/static_occlusion_priority_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
