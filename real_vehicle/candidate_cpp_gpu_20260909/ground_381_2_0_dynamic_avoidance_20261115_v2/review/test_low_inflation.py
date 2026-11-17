"""Recorded false-block and measured-shape tests; no hardware/actuation."""
from pathlib import Path
import json,subprocess,math,copy,hashlib
R=Path(__file__).resolve().parents[1]
def ask(req,old=False):
 binary=R/('review/pre_zero_static_growth/generic_probe' if old else 'bin/generic_probe')
 p=subprocess.run([str(binary),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True);return json.loads(p.stdout)
def forecast(o,now,future=0,route=True):return ask(dict(stamp=now,poses=[[0,0,0]],forecast_occupancy=o,future_s=future,route_prior=route))
base=dict(id=1,stamp=100.,hits=8,has_moved=False,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed',xmin=1.4,xmax=1.8,ymin=-.2,ymax=.2,vx=0.,vy=0.,sensor_x=0.,sensor_y=0.)
static=[]
for hits in [1,8]:
 o=dict(base,hits=hits)
 for age,future in [(0,0),(.3,1.2),(.5,1.6),(.8,2.)]:
  q=forecast(o,100+age,future);assert q['valid'] and q['box']==[1.4,-.2,1.8,.2],q;static.append(q)
# The exact recorded box formerly gained 28.8 cm/side at this query.
f=json.loads((R/'review/dmpc_20261116_155527_moving_failure.json').read_text());diag=f['candidate_collision_diagnostics'][0];o=diag['occupancy'];now=diag['now'];req=dict(stamp=now,poses=[diag['ego_pose']],forecast_occupancy=o,future_s=0,route_prior=True);old=ask(req,True);new=ask(req);assert new['box']==[o[k] for k in ['xmin','ymin','xmax','ymax']]
q=dict(stamp=now,poses=[diag['ego_pose']],check_prediction=[diag['ego_pose']],occupancies=[o],route_prior=True);rejection=ask(q,True);cleared=ask(q);assert rejection['reason']=='generic_predicted_occupancy_conflict' and cleared['valid'],(rejection,cleared)
# Retain nonlinear geometry inside a convex support, not only separate points.
hull=dict(base,xmin=2.,xmax=3.05,ymin=-.8,ymax=.2,support_hull=[[2.,-.8],[3.,.2],[3.05,.2]])
empty_pose=[2.1,.08,0.];test=dict(stamp=100.,poses=[empty_pose],check_prediction=[empty_pose],occupancies=[hull]);empty=ask(test);box_only=copy.deepcopy(test);box_only['occupancies'][0].pop('support_hull');boxed=ask(box_only);assert empty['valid'] and boxed['reason']=='generic_predicted_occupancy_conflict',(empty,boxed)
inside=[2.6,-.2,0];filled=ask(dict(stamp=100.,poses=[inside],check_prediction=[inside],occupancies=[hull]));assert filled.get('reason')=='generic_predicted_occupancy_conflict',filled
line=dict(hull,support_hull=[[2.,-.8],[3.05,.2]])
line_check=ask(dict(test,occupancies=[line]));assert line_check['valid'],line_check
# Compact bodies retain existing measured/hidden physical extent; process
# expansion is zero. This is separate from physical dimensions and 8cm gap.
compact=dict(base,xmin=1.5,xmax=1.7,ymin=-.15,ymax=.15)
c0=forecast(compact,100,0,False);c1=forecast(compact,100.5,1.6,False);assert c0['box']==c1['box']
# Route-aligned established motion is still predicted forward; uncertainty is
# bounded rather than growing to a large isotropic reachable set.
moving=dict(base,xmin=1.4,xmax=1.8,ymin=-.35,ymax=.05,has_moved=True,vx=.5,hits=20)
m=forecast(moving,100.2,1.2);assert m['valid'] and m['route_prior_applied'],m
uncon=forecast(moving,100.2,1.2,False);assert m['box'][3]-m['box'][1]<uncon['box'][3]-uncon['box'][1],(m,uncon)
# Lateral motion / loss of visibility does not silently become track motion.
other=dict(moving,vy=.35);fallback=forecast(other,100.2,1.2);prior_off=forecast(other,100.2,1.2,False);assert not fallback['route_prior_applied'] and fallback['box']==prior_off['box']
hazard=dict(base,visibility_hazard=True,memory_state='occluded_unknown');unknown=forecast(hazard,100.2,1.2);assert unknown['box'][2]-unknown['box'][0]>.4
# A long-confirmed map-backed static remains static during occlusion.
wall=dict(base,map_supported=True,visibility_hazard=True,memory_state='occluded_unknown',hits=1000)
w0=forecast(wall,100.8,1.6,False);assert w0['box']==[1.4,-.2,1.8,.2],w0
unmapped_fixed=dict(wall,map_supported=False,fixed_world=True)
uf=forecast(unmapped_fixed,100.8,1.6,False);assert uf['box']==[1.4,-.2,1.8,.2],uf
# A formerly moving map-adjacent object does not inherit that static prior.
wall_mover=dict(wall,has_moved=True);wm=forecast(wall_mover,100.8,1.6,False);assert wm['box'][2]-wm['box'][0]>.4,wm
# Production memory must actually publish the measured contour.
trace=json.loads((R/'run_logs/20261116_155527-closed-frenet-ground-attempt/tracking_trace.json').read_text());row=next(x for x in trace['controller'] if x.get('valid') and x.get('input_scan_stamp')==o['stamp'] and x.get('obstacle_points'));pts=[p for p in row['obstacle_points'] if o['xmin']-1e-6<=p[0]<=o['xmax']+1e-6 and o['ymin']-1e-6<=p[1]<=o['ymax']+1e-6]
mem=ask(dict(stamp=100.,poses=[[0,0,0]],clusters=[dict(id=7,points=pts)],allow_search=False,free_range=.3));contours=[x for x in mem.get('occupancies',[]) if x.get('support_hull')];assert contours and len(contours[0]['support_hull'])>=3,mem
# Revalidate the exact incumbent from the failed run, without replacing scene.
path=next(x['local_path_snapshot'] for x in trace['controller'] if x.get('plan_id')==466 and x.get('local_path_snapshot'))
request=dict(stamp=f['source_scan_stamp'],poses=[f['planning_pose']],speed=f['planning_speed_mps'],plan_occupancies=f['occupancies'],initial_incumbent=path,initial_offset=row['offset_m'],cap=2.15,max_offset=.65,route_prior=True,map_speed_feasibility=True,running_floor=.6,seed_reserve=.02,preferred_gap=.10,search_budget_ms=200,include_candidates=True)
replayed=ask(request);assert replayed['valid'],replayed
out=dict(passed=True,physical_motion=False,static_zero_growth_cases=len(static),recorded_old_expanded_box=old['box'],recorded_new_box=new['box'],recorded_conflict_cleared=cleared['valid'],empty_aabb_corner_cleared=empty['valid'],convex_interior_still_rejected=filled['reason'],compact_hidden_extent_kept=True,route_moving_forecast=m,lateral_fallback_kept=True,unknown_occlusion_kept=True,published_support_hull_vertices=len(contours[0]['support_hull']),recorded_incumbent_revalidated=replayed,physical_retest_pending=True)
out['source_sha256']={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in ['src/cpp_frenet/tracked_occupancy.hpp','src/cpp_frenet/generic_avoidance.hpp']}
out['static_occlusion_scope']='all_confirmed_static'
(R/'review/low_inflation_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='recorded_incumbent_revalidated'},indent=2))
