from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];fixture=json.loads((R/'review/prearm_retention_fixture.json').read_text())
def req(row,refresh=False):return dict(stamp=row['source_scan_stamp'],poses=[row['scan_pose']],speed=0,plan_occupancies=row['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,seed_reserve=.02,preferred_gap=.10,include_candidates=True,search_budget_ms=1000,refresh_prearm_path=refresh)
def run(binary,refresh):
 p=subprocess.Popen([str(R/binary),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 try:
  values=[]
  for row,f in [(fixture['early'],False),(fixture['steady'],refresh)]:
   p.stdin.write(json.dumps(req(row,f))+'\n');p.stdin.flush();values.append(json.loads(p.stdout.readline()))
  return values
 finally:p.stdin.close();p.wait(timeout=5)
old=run('review/pre_prearm_refresh/generic_probe',False);active=run('bin/generic_probe',False);fresh=run('bin/generic_probe',True)
assert all(v['valid'] for v in old+active+fresh)
assert old[-1]['score_terms']['generic_held_path']
assert active[-1]['score_terms']['generic_held_path']
assert abs(active[-1]['offset_m']-old[-1]['offset_m'])<1e-6
assert not fresh[-1]['score_terms']['generic_held_path']
assert abs(fresh[-1]['offset_m'])<abs(active[-1]['offset_m'])-.04
assert fresh[-1]['maximum_curvature_1pm']<=1.14
# The recorded unsafe state must continue to reject the coast trajectory.
bad=json.loads((R/'review/tight_ground_first_failure.json').read_text());i=bad['control_input_snapshot'];d=bad['rejected_control']['diagnostics']
request=dict(stamp=bad['source_scan_stamp'],poses=[i['pose']],coast_map=True,speed=d['emergency_coast_check_speed_mps'],servo=d['sweep_initial_servo_rad'])
p=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=json.dumps(request)+'\n',text=True,capture_output=True,check=True);coast=json.loads(p.stdout);assert coast.get('reason')=='generic_emergency_coast_map_conflict',coast
out=dict(passed=True,old_retained=old[-1],new_active_retained=active[-1],new_prearm_refreshed=fresh[-1],recorded_unsafe_coast_still_rejected=coast,physical_motion=False)
(R/'review/prearm_refresh_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(passed=True,old_offset_m=old[-1]['offset_m'],refreshed_offset_m=fresh[-1]['offset_m'],active_path_retention_preserved=True,unsafe_coast_guard_preserved=True),indent=2))
