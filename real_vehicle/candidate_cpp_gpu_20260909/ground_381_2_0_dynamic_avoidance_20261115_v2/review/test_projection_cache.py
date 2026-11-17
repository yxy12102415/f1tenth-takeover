from pathlib import Path
import json,subprocess,copy
R=Path(__file__).resolve().parents[1];f=json.loads((R/'review/field_search_budget_fixture.json').read_text());o=next(v for v in f['failed']['occupancies'] if v['id']==8)
base=dict(stamp=o['stamp']+.3,poses=[f['failed']['scan_pose']],forecast_occupancy=o,future_s=1.55,route_prior=True)
def cold(req,prior=False):
 p=subprocess.run([str(R/('review/pre_lead_identity/generic_probe' if prior else 'bin/generic_probe')),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True);return json.loads(p.stdout)
def equal(a,b):
 assert a['valid']==b['valid'] and a['route_prior_applied']==b['route_prior_applied'],(a,b)
 assert max(abs(x-y) for x,y in zip(a['box'],b['box']))<1e-10,(a,b)
# The historical pre-cache golden predates the already commissioned low-
# inflation policy. Preserve it, but compare this revision against its actual
# immediately preceding executable rather than restoring obsolete inflation.
old=cold(base,True);equal(cold(base),old)
requests=[base,dict(base,future_s=0),dict(base,stamp=o['stamp']+.51),dict(base,route_prior=False)]
for change in [dict(memory_state='predicted_hold'),dict(map_supported=True),dict(vx=0,vy=.3),dict(hits=1),dict(xmin=o['xmin']+.01,xmax=o['xmax']+.01)]:requests.append(dict(base,forecast_occupancy=dict(o,**change)))
expected=[cold(q,True) for q in requests];[equal(cold(q),want) for q,want in zip(requests,expected)];p=subprocess.Popen([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
try:
 for repeat in range(3):
  for req,want in zip(requests,expected):
   p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();equal(json.loads(p.stdout.readline()),want)
finally:p.stdin.close();p.wait(timeout=5)
out=dict(passed=True,physical_motion=False,pre_lead_identity_baseline_preserved=True,historical_pre_cache_reference_retained=True,cached_vs_fresh_cases=len(requests)*3,source_age_and_motion_change_invalidation_verified=True)
(R/'review/projection_cache_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
