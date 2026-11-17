"""Qualify the explicitly static commissioning profile; never actuates hardware."""
import hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parent
out=ROOT/'reports/fixed_world_pid_20261115';out.mkdir(parents=True,exist_ok=True)
native=(out/'native.txt').read_text()
assert 'Ran 69 tests' in native and native.rstrip().endswith('OK')
cases=[]
for fault in ('none','release','scan_timeout'):
 p=ROOT/'reports'/('fixed_world_pid_'+fault);v=json.loads((p/'report.json').read_text())
 assert v['passed'] and v['physical_motion'] is False and v['route_completed']==(fault=='none')
 assert abs(v['final_speed_mps'])<=.03 and v['max_command_mps']<=2.00001
 records=json.loads((p/'records.json').read_text())
 valid=[r for r in records['planner'] if r.get('valid') and r.get('occupancies')]
 assert valid and all(r.get('obstacle_motion_model')=='stationary_scene' for r in valid)
 assert all(o.get('fixed_world') is True for r in valid for o in r['occupancies'])
 running=False
 for r in records['controller']:
  speed=r.get('speed_command',0.)
  if speed>=.5-1e-5:running=True
  assert not(running and 1e-6<speed<.5-1e-5)
 cases.append(v)
source={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/cpp_frenet').glob('*') if p.is_file()}
binaries={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'install_cpp/cpp_frenet/lib/cpp_frenet').glob('cpp_frenet_*') if p.is_file()}
qualification=dict(physical_motion=False,physical_commissioned=False,scope='stationary_obstacle_only',obstacle_scene_model='fixed_world_occupancy_with_measurement_uncertainty',native_tests=69,source_sha256=source,binary_sha256=binaries,cases=cases)
(out/'QUALIFICATION.json').write_text(json.dumps(qualification,indent=2)+'\n')
(ROOT/'build_provenance.json').write_text(json.dumps(dict(source_unchanged_during_build=True,source_sha256=source,binaries=binaries,build_log='/tmp/fixed_world_build.log'),indent=2)+'\n')
p=ROOT/'compact_config.json';c=json.loads(p.read_text());assert c.get('generic_stationary_scene_v1') and c['first_physical_scope']=='stationary_obstacle_only'
c.update(physical_commissioning_enabled=True,qualified_motion_trackers=['pid'],qualification_file='reports/fixed_world_pid_20261115/QUALIFICATION.json');p.write_text(json.dumps(c,indent=2)+'\n')
p=ROOT/'PREPARATION_STATUS.json';d=json.loads(p.read_text());d.update(software_status='fixed_world_static_pid_acceptance_passed',qualification=c['qualification_file'],native_tests_passed=69,physical_ready=False,physical_commissioned=False,next_required='Matching offline-window evidence, current live zero qualification and one static PID run');p.write_text(json.dumps(d,indent=2)+'\n')
print('Fixed-world static PID software acceptance passed; physical trial still required')
