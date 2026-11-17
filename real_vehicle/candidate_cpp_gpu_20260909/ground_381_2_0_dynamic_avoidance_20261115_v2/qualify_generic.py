"""Summarize actual non-actuating acceptance evidence. Never fabricates results."""
import hashlib,json,math
from pathlib import Path
from datetime import datetime,timezone
r=Path(__file__).resolve().parent
reports=json.loads((r/'reports/suite.json').read_text())
expected={'pid_static','mpc_static','pid_wheels','mpc_wheels','pid_moving015','mpc_moving015','pid_release','mpc_scan_timeout','mpc_no_obstacle'}
assert {v['name'] for v in reports}==expected,'Incomplete ROS suite'
for v in reports:
    assert v['passed'] and v['exit_code']==0,(v['name'],v.get('failure'))
    assert v['physical_motion'] is False
    assert v['max_command_mps']<=1.25001
    if v['fault']=='none':
        assert v['route_completed'],v['name']
        gap=v.get('minimum_independent_sat_separation_m')
        assert gap is None or gap>0
        assert abs(v['final_speed_mps'])<=.03
    else:assert not v['route_completed'] and abs(v['final_speed_mps'])<=.03
native=(r/'reports_native.txt').read_text()
assert 'Ran 47 tests' in native and native.rstrip().endswith('OK')
controls=json.loads((r/'reports/control_contract.json').read_text())
assert len(controls)==2 and all(v['passed'] and v['terminal_zero_latched'] for v in controls)
build=json.loads((r/'build_provenance.json').read_text())
assert build['source_unchanged_during_build']
for p,sha in build['source_sha256'].items():assert hashlib.sha256((r/p).read_bytes()).hexdigest()==sha,p
for p,sha in build['binaries'].items():assert hashlib.sha256((r/'install_cpp/cpp_frenet/lib/cpp_frenet'/p).read_bytes()).hexdigest()==sha,p
live=json.loads((r/'reports/live_scan_capture/replay.json').read_text())
assert live['passed'] and live['physical_motion'] is False
result=dict(live_scan_replay=live,qualified_utc=datetime.now(timezone.utc).isoformat(),physical_motion=False,native_tests=47,controller_contracts=controls,ros_cases=[dict(name=v['name'],passed=v['passed'],route_completed=v['route_completed'],minimum_independent_sat_separation_m=v.get('minimum_independent_sat_separation_m')) for v in reports],binary_sha256=build['binaries'],scope='Synthetic qualification only; first physical trial stationary obstacle, cap1.25m/s')
(r/'reports/QUALIFICATION.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
