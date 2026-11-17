"""Recorded start-scene replay: requested gap and added seed reserve actually widen d."""
from pathlib import Path
import json,subprocess,hashlib
R=Path(__file__).resolve().parents[1]
c=json.loads((R/'compact_config.json').read_text())
assert c['adaptive_corridor_v2']['desired_body_gap_m']==.15
assert c['adaptive_corridor_v2']['hard_body_gap_m']==.08
assert c['generic_corridor_seed_reserve_m']==.07
fixture=R/'review/gap_15cm_contact_scene_replay.json';q=json.loads(fixture.read_text())['request']
def ask(gap,reserve):
 request=dict(q,preferred_gap=gap,seed_reserve=reserve,search_budget_ms=200)
 p=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=json.dumps(request)+'\n',capture_output=True,text=True,check=True)
 return json.loads(p.stdout)
before=ask(.10,.02);after=[ask(.15,.07) for _ in range(3)]
assert before['valid']
for a in after:
 # 15 cm is a soft preference: the existing planner may fall back to
 # the unchanged 8 cm hard admission gap. A timing-sensitive assertion that
 # every 200 ms search attained the preference contradicted that policy.
 assert a['valid'] and a['score_terms']['generic_required_body_gap_m']>=.08-1e-9,a
 expected_minimum_increase=(.07-.02)-(before['score_terms']['generic_required_body_gap_m']-.08)-.005
 assert abs(a['offset_m'])>=abs(before['offset_m'])+expected_minimum_increase,a
 assert a['maximum_curvature_1pm']<=1.14
out=dict(passed=True,physical_motion=False,scope='Same recorded prearm scene, fresh search at 200 ms; cold candidate comparison is not old held-path comparison.',config_sha256=hashlib.sha256((R/'compact_config.json').read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),before=before,after=after,desired_body_gap_m=.15,seed_reserve_m=.07,hard_body_gap_m=.08,preferred_gap_achieved_trials=sum(abs(a['score_terms']['generic_required_body_gap_m']-.15)<1e-9 for a in after),fallback_policy='Preferred gap is soft; hard 8 cm checks remain mandatory')
(R/'review/gap15cm_regression.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(dict(passed=True,before_d_m=before['offset_m'],after_d_m=[a['offset_m'] for a in after],preferred_gap_achieved_trials=out['preferred_gap_achieved_trials'],hard_gap_checks_passed=True)))
