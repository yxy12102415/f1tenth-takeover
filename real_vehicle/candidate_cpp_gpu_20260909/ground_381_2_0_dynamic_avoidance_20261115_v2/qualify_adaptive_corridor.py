"""Admit only trackers with inspectable current-build static/fault evidence."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib
r=Path(__file__).resolve().parent;d=r/'reports/adaptive_corridor_20261113'
profile_config=json.loads((r/'compact_config.json').read_text());lateral_limit=profile_config['generic_max_lateral_offset_m']
build=json.loads((r/'build_provenance.json').read_text());assert build['source_unchanged_during_build']
for p,sha in build['source_sha256'].items():assert hashlib.sha256((r/p).read_bytes()).hexdigest()==sha,p
for p,sha in build['binaries'].items():assert hashlib.sha256((r/'install_cpp/cpp_frenet/lib/cpp_frenet'/p).read_bytes()).hexdigest()==sha,p
native=(d/'native.txt').read_text();assert 'Ran 53 tests' in native and native.rstrip().endswith('OK')
controls=json.loads((r/'reports/control_contract.json').read_text());assert len(controls)==2 and all(c['passed'] and c['terminal_zero_latched'] for c in controls)
suite=json.loads((d/'closed_loop.json').read_text());assert suite['binary_sha256']==build['binaries']
rows={c['name']:c for c in suite['cases']}
expected={'adaptive_pid_static','adaptive_mpc_static','adaptive_pid_release','adaptive_pid_scan_timeout','adaptive_pid_wheels','adaptive_pid_no_obstacle','adaptive_pid_moving015','adaptive_mpc_release','adaptive_mpc_scan_timeout'}
assert set(rows)==expected
required={'pid':['adaptive_pid_static','adaptive_pid_release','adaptive_pid_scan_timeout','adaptive_pid_wheels','adaptive_pid_no_obstacle'],
          'dynamic_mpc':['adaptive_mpc_static','adaptive_mpc_release','adaptive_mpc_scan_timeout']}
def valid(c):
    if not(c.get('passed') and c.get('exit_code')==0 and c.get('physical_motion') is False):return False
    cap=profile_config.get('generic_tracker_caps_mps',{}).get(c['tracker'],profile_config['generic_first_trial_effective_cap_mps'])
    if c['max_command_mps']>cap+.00001 or abs(c['final_speed_mps'])>.03:return False
    if c['fault']=='none' and not c['route_completed']:return False
    if c['fault']!='none' and c['route_completed']:return False
    gap=c.get('minimum_independent_sat_separation_m')
    return gap is None or gap>0
timing={}
for name,c in rows.items():
    if not c.get('passed'):continue
    rec=json.loads((r/'reports'/name/'records.json').read_text())
    plans=[v for v in rec['planner'] if v.get('valid')]
    ages=[age for v in plans for age in v.get('source_age_at_publication_s',{}).values()]
    assert ages and max(ages)<=.5,(name,'source-age qualification')
    assert all(abs(v.get('offset_m',0))<=lateral_limit+1e-6 for v in plans),(name,'offset limit')
    timing[name]=dict(maximum_plan_compute_s=max((v.get('last_plan_duration_s',0) for v in plans),default=0),maximum_valid_source_age_s=max(ages))
map_audit=json.loads((d/'independent_map_clearance.json').read_text())
map_rows={c['name']:c for c in map_audit['cases']}
for name,c in rows.items():
    if c.get('passed'):assert map_rows[name]['dense_body_map_lower_estimate_m']>=.05,(name,'independent body/map clearance')
qualified=[name for name,names in required.items() if all(valid(rows[n]) for n in names)]
assert 'pid' in qualified,'PID stationary/fault acceptance incomplete'
replay=json.loads((d/'replay_comparison.json').read_text());assert replay['new']['maximum_reference_offset_m']<=lateral_limit+1e-6
v=dict(qualified_utc=datetime.now(timezone.utc).isoformat(),physical_motion=False,physical_commissioned=False,
       scope='Stationary-obstacle software acceptance: PID cap0.8m/s, dynamic MPC cap0.7m/s, reference offset<=0.58m. Moving software case is diagnostic, not physical overtaking acceptance.',
       native_tests=53,qualified_motion_trackers=qualified,planning_timing=timing,independent_map_audit=map_audit,
       profile=dict(command_cap_mps=.8,tracker_caps_mps={'pid':.8,'dynamic_mpc':.7},maximum_reference_offset_m=lateral_limit,seed_reserve_m=profile_config['generic_corridor_seed_reserve_m'],acceleration_limit_mps2=2.,stationary_first_physical_scope=True),binary_sha256=build['binaries'],source_sha256=build['source_sha256'],
       required_cases=required,closed_loop_cases=suite['cases'],recorded_scene=replay,controller_contracts=controls)
(d/'QUALIFICATION.json').write_text(json.dumps(v,indent=2)+'\n')
config=json.loads((r/'compact_config.json').read_text());config.update(version=2,qualified_motion_trackers=qualified,physical_commissioning_enabled=True,physical_commissioned=False)
(r/'compact_config.json').write_text(json.dumps(config,indent=2)+'\n')
state=json.loads((r/'PREPARATION_STATUS.json').read_text())
for stale in ['last_zero_reason','last_zero_failure','last_zero_attempt','last_offline_window','last_offline_passed','last_motion_permission','latest_stop_reason']:
    state.pop(stale,None)
state['compact_profile'].update(speed_cap_mps=.8,max_lateral_offset_m=.58,tracker_caps_mps={'pid':.8,'dynamic_mpc':.7},seed_reserve_m=.07)
state.update(software_status='adaptive_corridor_static_and_fault_acceptance_passed',native_tests_passed=53,ros_cases_passed=sum(valid(c) for c in rows.values()),
             controller_contracts_passed=[c['tracker'] for c in controls],qualification='reports/adaptive_corridor_20261113/QUALIFICATION.json',
             software_qualification='reports/adaptive_corridor_20261113/QUALIFICATION.json',compact_qualified_trackers=qualified,
             physical_ready=False,physical_commissioned=False,physical_zero_current_fingerprint=False,command_cap_mps=.8,tracker_caps_mps={'pid':.8,'dynamic_mpc':.7},operator_reset_confirmed_for_current_build=False,
             compact_mpc_status=('current adaptive-path static/release/scan-timeout software acceptance passed' if 'dynamic_mpc' in qualified else 'not qualified; inspect current closed-loop evidence'),
             next_required='Fresh reset after last physical motion; matching5s zero/localization admission. No new physical motion performed by this qualification.')
(r/'PREPARATION_STATUS.json').write_text(json.dumps(state,indent=2)+'\n')
print(json.dumps(dict(qualified_motion_trackers=qualified,native_tests=53,cases={name:c['passed'] for name,c in rows.items()},physical_motion=False),indent=2))
