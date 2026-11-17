from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];records=json.loads((R/'reports/compact_fix/pid_delayed_static/records.json').read_text());v=records['planner'][-1]
# Preserve the failed integration snapshot before later runs overwrite its report.
fixture=R/'review/forecast_seed_failure_fixture.json'
if fixture.exists():data=json.loads(fixture.read_text());v=data['planner'];speed=data['speed']
else:
 speed=next(x['control_input_snapshot']['speed'] for x in reversed(records['controller']) if x.get('valid') and x.get('control_input_snapshot'))
 fixture.write_text(json.dumps(dict(planner=v,speed=speed),indent=2)+'\n')
req=dict(stamp=v['source_scan_stamp'],poses=[v['scan_pose']],speed=speed,plan_occupancies=v['occupancies'],cap=2.15,max_offset=.65,map_speed_feasibility=True,seed_reserve=.02,preferred_gap=.10,include_candidates=True,search_budget_ms=200)
def ask(binary):
 p=subprocess.run([str(R/binary),str(R/'runtime_assets.json')],input=json.dumps(req)+'\n',capture_output=True,text=True,check=True);return json.loads(p.stdout)
old=ask('review/pre_forecast_seed/generic_probe');new=ask('bin/generic_probe')
report=dict(passed=new['valid'],scope='Captured software integration failure, not physical sensor data',old=old,new=new,physical_motion=False)
(R/'review/forecast_seed_regression.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(old_valid=old['valid'],old_reason=old['reason'],new_valid=new['valid'],new_reason=new['reason'],new_reasons=new.get('candidate_reasons')),indent=2));assert new['valid'],new.get('candidate_reasons')
