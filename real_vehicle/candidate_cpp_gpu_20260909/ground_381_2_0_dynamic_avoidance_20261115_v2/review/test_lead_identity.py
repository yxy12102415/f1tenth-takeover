"""Native C++ identity regression, real same-capture holdouts and scenario tests."""
from pathlib import Path
import json,subprocess,sys,copy,hashlib,time
import numpy as np
R=Path(__file__).resolve().parents[1];W=R.parent.parent;S=W/'analysis/lead_shell_multiview_20261117'
sys.path.insert(0,str(W/'lead_route_identity_20261117'))
from lead_route_tracker import principal_support
assets=json.loads((R/'runtime_assets.json').read_text());assets['route']=[[i*.1,0,0] for i in range(101)];assets['speed']=[0]+[.7]*99+[0]
asset=R/'review/lead_identity_straight_assets.json';asset.write_text(json.dumps(assets,separators=(',',':')))
class Probe:
 def __init__(self,real=False):self.p=subprocess.Popen([str(R/'bin/lead_identity_probe'),str(R/'runtime_assets.json' if real else asset),str(R/'compact_config.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 def ask(self,v):
  assert self.p.poll() is None,self.p.stderr.read();self.p.stdin.write(json.dumps(v)+'\n');self.p.stdin.flush();line=self.p.stdout.readline();assert line,self.p.stderr.read();out=json.loads(line)
  assert len(out['occupancies'])==len(v['occupancies'])
  for before,after in zip(v['occupancies'],out['occupancies']):
   assert all(before[key]==after[key] for key in ['id','stamp','xmin','ymin','xmax','ymax'])
  return out
 def close(self):self.p.stdin.close();self.p.wait(timeout=5);assert self.p.returncode==0,self.p.stderr.read()
def obj(points,ts,id=1):
 lo=points.min(axis=0);hi=points.max(axis=0);return dict(id=id,hits=20,stamp=ts,xmin=float(lo[0]),ymin=float(lo[1]),xmax=float(hi[0]),ymax=float(hi[1]),vx=.5,vy=0.,has_moved=True,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed',observation_age_s=0.)
def scene(points,ts,id=1,others=()):return dict(valid=True,source_scan_stamp=ts,scan_pose=[0,0,0],occupancies=[obj(points,ts,id)]+list(others),obstacle_points=points.tolist())
bg=[json.loads(l) for l in (S/'background/scans.jsonl').open()];bm=np.nanmedian([[np.nan if x is None else x for x in r['ranges']] for r in bg],axis=0);checks=[];shape=None
for name in ['rear','rear_left','left_side','front_left','front','front_right','right_side','rear_right']:
 p=Probe();rows=[json.loads(l) for l in (S/name/'scans.jsonl').open()];scores=[]
 try:
  for k,row in enumerate(rows[10:191:20]):
   rr=np.array([np.nan if x is None else x for x in row['ranges']]);a=row['angle_min']+np.arange(len(rr))*row['angle_increment'];points=np.column_stack((rr*np.cos(a),rr*np.sin(a)));mask=np.isfinite(rr)&(rr>=row['range_min'])&(rr<10)&(points[:,0]>=.7)&(points[:,0]<=2.3)&(abs(points[:,1])<=.65)&(bm-rr>.08);points=principal_support(points[mask]);v=p.ask(scene(points,100+k*.1,1+k));q=v['lead_identity'];
   if q['reason']=='associated':scores.append(q['shape_error_m'])
  assert q['locked'] and q['reason']=='associated',(name,q)
  checks.append(dict(view=name,associated=True,maximum_shape_error_m=max(scores)))
  if name=='rear':shape=points-(points.min(axis=0)+points.max(axis=0))/2
 finally:p.close()
bg203=dict(id=203,hits=16,stamp=0.,xmin=5.1069,ymin=2.1001,xmax=5.2677,ymax=2.8934,vx=-.0033,vy=-.4132,has_moved=True,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed',observation_age_s=0.)
p=Probe();component_ids=[]
try:
 for k in range(35):
  ts=200+k*.05;v=p.ask(scene(shape+[1.5+.5*k*.05,.1],ts,k+1,[dict(bg203,stamp=ts)]));q=v['lead_identity'];component_ids+=q.get('component_ids',[]);other=v['occupancies'][1];assert other['semantic_role']=='declared_static_background' and other['vx']==other['vy']==0
 assert q['reason']=='associated' and abs(q['speed_mps']-.5)<.03 and abs(q['d_m']-.1)<1e-6,q
 assert len(set(component_ids))>20 and len(q['predicted_route_positions'])==17
 assert abs(q['predicted_route_positions'][-1][2]-.1)<1e-6
 old_s=q['s_m']
 for k in range(1,7):
  ts=201.7+k*.05;z=p.ask(dict(valid=True,source_scan_stamp=ts,scan_pose=[0,0,0],occupancies=[dict(bg203,stamp=ts)],obstacle_points=[]));q=z['lead_identity'];assert q['s_m']==old_s and not q['identity_loss_requests_stop']
 assert q['reason']=='temporarily_unobserved'
finally:p.close()
p=Probe()
try:
 for k in range(10):
  ts=300+k*.1;a=shape+[1.5,-.3];b=shape+[1.5,.3];z=scene(a,ts);z['occupancies'].append(obj(b,ts,99));z['obstacle_points']=np.vstack([a,b]).tolist();q=p.ask(z)['lead_identity']
 assert not q['locked'] and q['reason']=='ambiguous_candidates',q
finally:p.close()
p=Probe()
try:
 for k in range(25):q=p.ask(scene(shape+[1.5+.2*k*.05,.4*k*.05],400+k*.05,k+1))['lead_identity']
 assert q['locked'] and not q['route_aligned'] and not q.get('predicted_route_positions'),q
finally:p.close()
# Reclassify the exact saved terminal failure without inventing an identity.
f=json.loads((R/'review/dmpc_20261117_002617_result_summary.json').read_text())['full_first_failure'];p=Probe(True)
try:
 # Feed the preceding recorded observations in source-time order. The capture
 # predates this template collection, so acquired identity is reported rather
 # than assumed to be independently verified.
 trace=json.loads((R/'run_logs/20261117_002617-closed-frenet-ground-attempt/tracking_trace.json').read_text());obs={}
 for row in trace['controller']:
  for o in [row,(row.get('control_input_snapshot') or {}).get('collision_guard_observation') or {}]:
   ts=o.get('source_scan_stamp')
   if ts and o.get('occupancies') and o.get('obstacle_points') and o.get('scan_pose') and ts<=f['source_scan_stamp']:obs[ts]=dict(o,valid=True)
 states=[];last=None
 for ts,o in sorted(obs.items()):last=p.ask(o);states.append(last['lead_identity'])
 classified=next(o for o in last['occupancies'] if o['id']==203)
 assert classified['semantic_role']=='declared_static_background' and classified['fixed_world'] and not classified['has_moved'],classified
 assert classified['xmin']==next(o for o in f['occupancies'] if o['id']==203)['xmin']
 field=dict(observations=len(states),ever_associated=any(s['reason']=='associated' for s in states),final_identity=last['lead_identity'],classified_terminal_object=classified)
 (R/'review/lead_identity_field_reclassified.json').write_text(json.dumps(dict(original_failure=f,reclassified_occupancies=last['occupancies'],field=field),indent=2))
finally:p.close()
# Actor semantics are part of the projection cache key. The same source
# geometry outside the legacy 35 cm band only gains the 65 cm route prior
# when it is explicitly associated as the route-aligned lead.
from lead_route_tracker import Route
rr=Route(json.loads((R/'runtime_assets.json').read_text())['route']);_,_,tangent=rr.project([2.7,.1])
o=dict(id=901,hits=20,stamp=100.,xmin=2.585,ymin=-.05,xmax=2.815,ymax=.25,vx=float(.5*tangent[0]),vy=float(.5*tangent[1]),sensor_x=0.,sensor_y=0.,has_moved=True,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed')
requests=[]
for role in [False,True,False,True]:
 item=dict(o,semantic_role='lead_vehicle' if role else '',lead_route_aligned=role)
 requests.append(dict(stamp=100.1,poses=[[0,0,0]],forecast_occupancy=item,future_s=1.,route_prior=True))
child=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=''.join(json.dumps(q)+'\n' for q in requests),capture_output=True,text=True,check=True)
forecasts=[json.loads(line) for line in child.stdout.splitlines()]
assert [v['route_prior_applied'] for v in forecasts]==[False,True,False,True],forecasts
assert forecasts[0]==forecasts[2] and forecasts[1]==forecasts[3]
report=dict(passed=True,physical_motion=False,scope='C++ association + same-capture raw-scan holdouts, synthetic motion/ambiguity/ID changes, recorded observation reclassification. Not independent recognition accuracy or full moving-overtake qualification.',holdouts=checks,changing_ids_preserved=True,route_0_5mps_preserves_measured_d=True,ambiguous_acquisition_rejected=True,temporary_identity_loss_no_direct_stop=True,crossing_revokes_route_prior=True,all_geometry_retained=True,terminal_return_203_static=True,lead_role_projection_cache_verified=True,field=field,source_sha256={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in ['src/cpp_frenet/lead_identity.hpp','src/cpp_frenet/node.cpp','src/cpp_frenet/generic_avoidance.hpp','src/cpp_frenet/tracked_occupancy.hpp','compact_config.json','review/test_lead_identity.py']})
(R/'review/lead_identity_regression.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(passed=True,views=len(checks),recorded_observations=field['observations'],recorded_lead_associated=field['ever_associated'],terminal_203_static=True,physical_motion=False)))
