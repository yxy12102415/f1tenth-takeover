from pathlib import Path
import json,subprocess,hashlib
R=Path(__file__).resolve().parents[1]
config=json.loads((R/'compact_config.json').read_text())
assert config['adaptive_corridor_v2']['desired_body_gap_m']==.10
assert config['adaptive_corridor_v2']['hard_body_gap_m']==.08
assert config['generic_corridor_seed_reserve_m']==.02
T=json.loads((R/'run_logs/20261115_233514-closed-frenet-ground-attempt/tracking_trace.json').read_text())
row=next(v for v in reversed(T['controller']) if v.get('valid') and v.get('occupancies') and v.get('scan_pose'))
def ask(binary,req):
 p=subprocess.run([str(R/binary),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',capture_output=True,text=True,check=True);return json.loads(p.stdout)
req=dict(stamp=row['source_scan_stamp'],poses=[row['scan_pose']],speed=0,plan_occupancies=row['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,include_candidates=True,search_budget_ms=1000)
old=ask('review/pre_tight_profile_20261115/generic_probe',dict(req,seed_reserve=.07,preferred_gap=.14));new=ask('bin/generic_probe',dict(req,search_budget_ms=200))
assert old['valid'],old.get('candidate_reasons')
assert new['valid'],new.get('candidate_reasons')
assert abs(new['score_terms']['generic_required_body_gap_m']-.10)<1e-9,new['score_terms']
assert new['maximum_curvature_1pm']<=1.14
out=dict(passed=True,scope='Captured stationary ground scene, actual runtime profile settings',old=old,new=new,config_sha256=hashlib.sha256((R/'compact_config.json').read_bytes()).hexdigest(),old_preferred_gap_m=.14,new_preferred_gap_m=.10,old_seed_reserve_m=.07,new_seed_reserve_m=.02,hard_body_gap_m=.08,physical_motion=False)
(R/'review/tight_profile_replay.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items() if k not in ['old','new']},indent=2));print('old_offset',old['offset_m'],'new_offset',new['offset_m'])
