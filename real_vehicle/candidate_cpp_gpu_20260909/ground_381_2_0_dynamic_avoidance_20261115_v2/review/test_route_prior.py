"""No actuation: constrained forecast guards, measured support and captured plans."""
from pathlib import Path
import json,copy,subprocess,math
R=Path(__file__).resolve().parents[1]
def ask(req):
 p=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True)
 return json.loads(p.stdout)
f=json.loads((R/'review/field_search_budget_fixture.json').read_text());v=f['failed'];o=next(x for x in v['occupancies'] if x['id']==8)
moving=dict(o,has_moved=True,vx=.5,vy=0.)
req=dict(stamp=o['stamp']+.3,poses=[v['scan_pose']],forecast_occupancy=moving,future_s=1.55)
old=ask(dict(req,route_prior=False));new=ask(dict(req,route_prior=True));assert new['valid'] and new['route_prior_applied'],new
assert new['box'][3]-new['box'][1]<old['box'][3]-old['box'][1]-.10,(old,new)
zero=ask(dict(req,stamp=o['stamp'],future_s=0,route_prior=True))
assert zero['box'][0]<=o['xmin'] and zero['box'][1]<=o['ymin'] and zero['box'][2]>=o['xmax'] and zero['box'][3]>=o['ymax'],zero
fallback={}
for name,change in [('lateral',dict(vx=0.,vy=.3)),('occluded',dict(memory_state='predicted_hold')),('unknown',dict(visibility_hazard=True)),('wall',dict(map_supported=True)),('tentative',dict(hits=1)),('off_route',dict(ymin=2.,ymax=2.3))]:
 q=dict(req,forecast_occupancy=dict(moving,**change));a=ask(dict(q,route_prior=False));b=ask(dict(q,route_prior=True));assert b['valid'] and not b['route_prior_applied'] and a['box']==b['box'],(name,a,b);fallback[name]=True
stale=ask(dict(req,stamp=o['stamp']+.51,route_prior=True));assert not stale['route_prior_applied']
# A body overlapping the live observation is still rejected under the new prior.
centre=[(o['xmin']+o['xmax'])/2,(o['ymin']+o['ymax'])/2,0]
blocked=ask(dict(stamp=o['stamp'],poses=[centre],check_prediction=[centre],occupancies=[o],route_prior=True));assert blocked.get('reason')=='generic_predicted_occupancy_conflict',blocked
# Same captured scene and held path, no manual replacement of motion flags.
plan=dict(stamp=v['source_scan_stamp'],poses=[v['scan_pose']],speed=f['last_valid']['control_input_snapshot']['speed'],plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,seed_reserve=.02,preferred_gap=.10,search_budget_ms=1000,initial_incumbent=json.loads((R/'review/field_search_held_path.json').read_text()),initial_offset=f['last_valid']['offset_m'])
plans={}
for flag in [False,True]:plans[str(flag)]=ask(dict(plan,route_prior=flag))
assert plans['True']['valid'],plans['True']
out=dict(passed=True,physical_motion=False,unconstrained_forecast=old,route_constrained_forecast=new,fallbacks=fallback,stale_falls_back=True,measured_support_preserved=True,direct_overlap_still_rejected=blocked,field_plans=plans,wall_margin_m=json.loads((R/'runtime_assets.json').read_text())['wall_margin_m'],limitation='Captured occupancy snapshot replay, not a physical success or a raw-scan replay.')
(R/'review/route_prior_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(passed=True,old_width=old['box'][3]-old['box'][1],new_width=new['box'][3]-new['box'][1],field_valid=plans['True']['valid'],old_field_valid=plans['False']['valid'])))
