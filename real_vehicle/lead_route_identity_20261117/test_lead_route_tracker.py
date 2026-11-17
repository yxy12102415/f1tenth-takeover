"""Recorded scan holdouts + association scenarios; no actuator interfaces."""
import json,math,hashlib
from pathlib import Path
import numpy as np
from lead_route_tracker import Route,ShapeBank,LeadTracker,principal_support
R=Path(__file__).resolve().parent;W=R.parent;S=W/'analysis/lead_shell_multiview_20261117';bank=ShapeBank(S/'inspection');route=Route([[i*.1,0] for i in range(101)])
config=json.loads((R/'config.json').read_text())
def tracker(r=route):return LeadTracker(r,bank,body_length=config['lead_length_m'],body_width=config['lead_width_m'])
def obstacle(points,stamp,id):
 lo=points.min(axis=0);hi=points.max(axis=0)
 return dict(id=id,hits=20,stamp=stamp,xmin=float(lo[0]),xmax=float(hi[0]),ymin=float(lo[1]),ymax=float(hi[1]),vx=.5,vy=0.,has_moved=True,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed',observation_age_s=0.)
def scene(points,stamp,id=1,others=()):
 return dict(valid=True,source_scan_stamp=stamp,scan_pose=[0,0,0],occupancies=[obstacle(points,stamp,id)]+list(others),obstacle_points=points.tolist())
holdouts=[];shape=None
bg=[json.loads(s) for s in (S/'background/scans.jsonl').open()];bm=np.median([[np.nan if x is None else x for x in r['ranges']] for r in bg],axis=0)
for v in sorted((S/'inspection').glob('*_visible_template.json')):
 view=v.name.replace('_visible_template.json','');rows=[json.loads(s) for s in (S/view/'scans.jsonl').open()];tr=tracker();scores=[]
 for k,row in enumerate(rows[10:191:20]):
  rr=np.array([np.nan if x is None else x for x in row['ranges']]);a=row['angle_min']+np.arange(len(rr))*row['angle_increment'];p=np.column_stack((rr*np.cos(a),rr*np.sin(a)));mask=np.isfinite(rr)&(rr>=row['range_min'])&(rr<10)&(p[:,0]>=.7)&(p[:,0]<=2.3)&(abs(p[:,1])<=.65)&(bm-rr>.08);p=principal_support(p[mask]);out=tr.update(scene(p,100.+.1*k,k+1),[0,0,0]);scores.append(bank.score(p)[0])
 assert out['lead'] and out['reason']=='associated',(view,out)
 holdouts.append(dict(view=view,scan_holdouts=10,shape_error_max_m=max(scores),identity_acquired=True))
 if view=='rear':shape=p-(p.min(axis=0)+p.max(axis=0))/2
assert shape is not None
tr=tracker();ids=[];speeds=[];background=dict(id=203,hits=16,stamp=0.,xmin=5.1069,ymin=2.1001,xmax=5.2677,ymax=2.8934,vx=-.0033,vy=-.4132,has_moved=True,fixed_world=False,map_supported=False,visibility_hazard=False,memory_state='observed',observation_age_s=0.)
for k in range(35):
 ts=200+k*.05;p=shape+[1.5+.5*k*.05,.1];out=tr.update(scene(p,ts,10+k,others=[dict(background,stamp=ts)]),[0,0,0]);
 if out['lead']:ids.extend(out['lead']['ids']);speeds.append(out['lead']['speed'])
 assert out['occupancies'][1]['semantic_role']=='declared_static_background'
 assert out['occupancies'][1]['vx']==out['occupancies'][1]['vy']==0
assert len(set(ids))>20 and abs(speeds[-1]-.5)<.03 and abs(out['lead']['d']-.1)<1e-6,(len(set(ids)),speeds[-5:],out['lead'])
assert len(out['predicted_route_positions'])==17 and abs(out['predicted_route_positions'][-1][2]-.1)<1e-6
# Disappearance cannot acquire a distant new object or directly request a stop.
last_s=out['lead']['s']
for k in range(1,7):
 ss=dict(valid=True,source_scan_stamp=201.7+k*.05,occupancies=[dict(background,stamp=201.7+k*.05)],obstacle_points=[],scan_pose=[0,0,0]);lost=tr.update(ss,[0,0,0]);assert lost['lead']['s']==last_s and not lost['identity_loss_requests_stop']
assert lost['reason']=='temporarily_unobserved'
old=tr.update(ss,[0,0,0]);assert not old['valid'] and old['reason']=='duplicate_or_old_scan'
# Two physically distinct, equally plausible objects must remain ambiguous.
amb=tracker()
for k in range(10):
 ts=300+k*.1;a=shape+[1.5,-.3];b=shape+[1.5,.3];v=scene(a,ts);v['occupancies'].append(obstacle(b,ts,99));v['obstacle_points']=np.vstack([a,b]).tolist();answer=amb.update(v,[0,0,0])
assert answer['lead'] is None and answer['reason']=='ambiguous_candidates'
assert all(o['semantic_role']=='pending_lead_candidate' for o in answer['occupancies'])
# A crossing target must not remain locked to the reference line.
cross=tracker()
for k in range(25):
 ts=400+k*.05;answer=cross.update(scene(shape+[1.5+.2*k*.05,.4*k*.05],ts,k+1),[0,0,0])
assert answer['lead'] and not answer['lead']['route_aligned'] and not answer['predicted_route_positions']
report=dict(passed=True,mode='shadow_only',physical_motion=False,production_runtime_changed=False,dimensions_source=config['dimensions_source'],same_capture_holdouts=holdouts,independent_recognition_accuracy_measured=False,changing_ids_preserved=True,route_speed_last_mps=speeds[-1],lateral_offset_preserved=True,recorded_terminal_background_203_static=True,temporary_loss_does_not_change_identity_or_request_stop=True,ambiguous_acquisition_rejected=True,crossing_revokes_route_prior=True,sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [R/'lead_route_tracker.py',R/'config.json',Path(__file__)]})
(R/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
