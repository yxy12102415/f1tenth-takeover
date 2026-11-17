from pathlib import Path
import json,subprocess,tempfile
R=Path(__file__).resolve().parents[1]
a=json.loads((R/'runtime_assets.json').read_text());assert a['wall_margin_m']==.03
q=dict(stamp=100,poses=[a['route'][0]],coast_map=True,speed=0,servo=0)
def probe(binary,assets,req):
 p=subprocess.run([str(R/binary),str(assets)],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True);return json.loads(p.stdout)
with tempfile.TemporaryDirectory() as d:
 d=Path(d);(d/'compact_config.json').write_text('{}');p=d/'runtime_assets.json'
 # Algebraic map-distance fixture: sample clearance 7.5 cm, cover 3.1 cm.
 x=dict(a,clearance=[.075]*len(a['clearance']));p.write_text(json.dumps(x))
 old=probe('review/pre_route_prior/bin/generic_probe',p,q);new=probe('bin/generic_probe',p,q)
 assert old.get('reason')=='generic_emergency_coast_map_conflict',old
 assert new.get('valid') and new['minimum_map_reserve_m']>0,new
 x['clearance']=[.02]*len(a['clearance']);p.write_text(json.dumps(x));overlap=probe('bin/generic_probe',p,q);assert overlap.get('reason')=='generic_emergency_coast_map_conflict',overlap
f=json.loads((R/'review/settle_field_failure.json').read_text());i=f['control_input_snapshot'];dg=f['rejected_control']['diagnostics'];real=probe('bin/generic_probe',R/'runtime_assets.json',dict(stamp=f['source_scan_stamp'],poses=[i['pose']],coast_map=True,speed=dg['emergency_coast_check_speed_mps'],servo=dg['sweep_initial_servo_rad']))
out=dict(passed=True,physical_motion=False,margin_m=.03,previous_margin_m=.05,algebra_fixture_old=old,algebra_fixture_new=new,inside_wall_still_rejected=overlap,latest_recorded_coast=real,limitation='Shrinking margin does not alone guarantee acceptance of the last recorded high-speed coast state.')
(R/'review/wall_margin_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(passed=True,new_margin_m=.03,last_recorded_coast_reason=real.get('reason'))))
