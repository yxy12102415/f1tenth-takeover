from pathlib import Path
import subprocess,json,statistics,hashlib
R=Path(__file__).resolve().parents[1]
def run(binary):return json.loads(subprocess.check_output([str(R/binary),str(R/'runtime_assets.json'),str(R/'review/reference_speed_field_fixture.json')],text=True))
checks=run('bin/test_reference_speed_table');pairs=[]
for i in range(5):
 old=run('bin/reference_speed_baseline');new=run('bin/reference_speed_cached');assert old['targets']==new['targets'];pairs.append(dict(before_ms=old['reference_speed_ms'],after_ms=new['reference_speed_ms']))
checks.update(paired_cold_process_reference_queries=pairs,query_inputs='Captured first moving rejection path, pose and speed; 25 fixed stations at measured speed, not full controller replay',before_median_ms=statistics.median(v['before_ms'] for v in pairs),after_median_ms=statistics.median(v['after_ms'] for v in pairs),optimization='Exact path/map/policy-bound precomputed limits produced by planner, admitted atomically with plan; no change to speed function or current-state guards',source_sha256=hashlib.sha256((R/'src/cpp_frenet/speed_geometry.hpp').read_bytes()).hexdigest())
(R/'review/reference_speed_table_regression.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
