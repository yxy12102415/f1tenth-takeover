from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];f=json.loads((R/'review/reference_frame_failure_fixture.json').read_text());v=f['failed'];i=f['last_valid']['control_input_snapshot'];pose=v['scan_pose'];req=dict(stamp=v['source_scan_stamp'],poses=[pose],speed=i['speed'],plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,seed_reserve=.02,preferred_gap=.10,include_candidates=True,search_budget_ms=200)
def ask(binary,request):
 p=subprocess.run([str(R/binary),str(R/'runtime_assets.json')],input=json.dumps(request)+'\n',capture_output=True,text=True,check=True);return json.loads(p.stdout)
old=ask('review/pre_reference_frame_fix/generic_probe',req);new=ask('bin/generic_probe',req)
assert old.get('reason')=='tracking_cross_track_error',old
trace=json.loads((R/'run_logs/20261116_003103-closed-frenet-ground-attempt/tracking_trace.json').read_text())
local=next(row['local_path_snapshot'] for row in trace['controller'] if row.get('plan_id')==f['last_valid']['plan_id'] and row.get('local_path_snapshot'))
guard_pose=[local[0][0],local[0][1]+1.,local[0][2]]
guard=ask('bin/generic_probe',dict(stamp=v['source_scan_stamp'],poses=[guard_pose],arrival_path=local))
assert guard.get('reason')=='tracking_cross_track_error',guard
out=dict(scope='Captured real planner failure while ego is offset from the reference during avoidance',old=old,new=new,genuine_local_tracking_error_still_rejected=guard,passed=new.get('valid',False),physical_motion=False)
(R/'review/reference_timing_frame_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(old_reason=old['reason'],new_reason=new['reason'],new_valid=new['valid'],new_reasons=new.get('candidate_reasons')),indent=2));assert new['valid'],new.get('candidate_reasons')
