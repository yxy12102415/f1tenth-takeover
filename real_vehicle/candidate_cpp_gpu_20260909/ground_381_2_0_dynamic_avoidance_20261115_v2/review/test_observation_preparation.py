"""Exact prediction-decision parity and paired captured-input timings."""
from pathlib import Path
import json,subprocess,time,statistics,copy,hashlib
R=Path(__file__).resolve().parents[1];fixture=json.loads((R/'review/dmpc_20261116_203413_live_prearm_failure.json').read_text());budget=json.loads((R/'review/dmpc_20261116_203413_budget_replay.json').read_text());held=next(x['result'] for x in budget if x['result'].get('valid'))
class Probe:
 def __init__(self,old=False):self.p=subprocess.Popen([str(R/('review/pre_observation_preparation/generic_probe' if old else 'bin/generic_probe')),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 def ask(self,q):
  t=time.perf_counter();self.p.stdin.write(json.dumps(q)+'\n');self.p.stdin.flush();out=json.loads(self.p.stdout.readline());return out,(time.perf_counter()-t)*1000
 def close(self):self.p.stdin.close();self.p.wait(timeout=5)
old=Probe(True);new=Probe();a=json.loads((R/'runtime_assets.json').read_text());route=a['route'];base=dict(stamp=fixture['source_scan_stamp']+.3,poses=[fixture['planning_pose']],occupancies=fixture['occupancies']);cases=[]
for start in [0,8,16,24,35,50,70,90]:
 q=dict(base,poses=[route[start]],check_prediction=route[start:min(len(route),start+25)]);cases.append(q)
for x,y in [(-1.,-2.),(0.,0.),(1.,0.),(2.5,-.3),(3.,-.5),(5.,1.),(-2.54,-5.60)]:cases.append(dict(base,poses=[[x,y,0]],check_prediction=[[x,y,0]]*25))
# Explicit moving, laterally moving, unknown-occluded and confirmed-static
# scenes share the exact same paths and source times in both implementations.
obj=next(o for o in fixture['occupancies'] if o['id']==9)
for change in [dict(has_moved=True,vx=.4),dict(has_moved=True,vy=-.3),dict(visibility_hazard=True,fixed_world=False),dict(visibility_hazard=True,fixed_world=True)]:
 q=copy.deepcopy(base);q['occupancies']=[dict(obj,**change)];q['check_prediction']=route[:65];cases.append(q)
try:
 for q in cases:
  before,_=old.ask(q);after,_=new.ask(q);assert before==after,(before,after)
 request=dict(stamp=fixture['source_scan_stamp'],poses=[fixture['planning_pose']],speed=0,plan_occupancies=fixture['occupancies'],initial_incumbent=held['path'],initial_offset=held['offset_m'],cap=2.15,max_offset=.65,search_budget_ms=200)
 timing=[]
 for repeat in range(6):
  if repeat%2:before,bt=old.ask(request);after,at=new.ask(request)
  else:after,at=new.ask(request);before,bt=old.ask(request)
  assert before['valid'] and after['valid'] and before['path']==after['path'] and before['offset_m']==after['offset_m'],(before.get('reason'),after.get('reason'))
  timing.append(dict(before_ms=bt,after_ms=at))
 guard_timing=[]
 guard=dict(base,check_prediction=held['path'][:25])
 for repeat in range(6):
  if repeat%2:a0,b0=old.ask(guard);a1,b1=new.ask(guard)
  else:a1,b1=new.ask(guard);a0,b0=old.ask(guard)
  assert a0==a1 and a1['valid'],(a0,a1)
  guard_timing.append(dict(before_ms=b0,after_ms=b1))
 # A fresh obstacle on the retained path must revoke that path, regardless of
 # caching. A new plan may be found; retaining the old unsafe one is forbidden.
 q=copy.deepcopy(request);point=held['path'][min(20,len(held['path'])-1)];obs=dict(obj,id=999,hits=10,xmin=point[0]-.12,xmax=point[0]+.12,ymin=point[1]-.12,ymax=point[1]+.12,stamp=q['stamp'],fixed_world=True,has_moved=False,vx=0.,vy=0.)
 q['plan_occupancies'].append(obs);changed,_=new.ask(q);assert not changed['valid'] or not changed.get('score_terms',{}).get('generic_held_path',False),changed
finally:old.close();new.close()
# Cold search is measured separately: equal inputs/budgets, but paths may
# differ when a faster implementation can finish the preferred-gap stage.
cold=[]
for repeat in range(3):
 p=Probe();q=dict(request);q.pop('initial_incumbent');q.pop('initial_offset');q['search_budget_ms']=200
 try:out,ms=p.ask(q);cold.append(dict(elapsed_ms=ms,valid=out['valid'],reason=out.get('reason'),offset_m=out.get('offset_m')))
 finally:p.close()
assert all(x['valid'] for x in cold),cold
b=statistics.median(x['before_ms'] for x in timing[1:]);n=statistics.median(x['after_ms'] for x in timing[1:]);assert n<b
out=dict(passed=True,physical_motion=False,paired_prediction_guard_timings=guard_timing,before_warm_prediction_guard_median_ms=statistics.median(x['before_ms'] for x in guard_timing[1:]),after_warm_prediction_guard_median_ms=statistics.median(x['after_ms'] for x in guard_timing[1:]),exact_prediction_decision_cases=len(cases),identical_held_paths=True,new_obstacle_revokes_retained_path=True,paired_held_timings=timing,before_warm_held_median_ms=b,after_warm_held_median_ms=n,cold_search_200ms=cold,budget_unchanged_ms=200,scope='Paired fixed-input prediction and identical held-path replay. Cold search may choose a different valid preferred-gap path. No physical qualification.',source_sha256=hashlib.sha256((R/'src/cpp_frenet/generic_avoidance.hpp').read_bytes()).hexdigest())
(R/'review/observation_preparation_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
