from pathlib import Path
import json,subprocess,hashlib,os
R=Path(__file__).resolve().parents[1]
unit=json.loads(subprocess.check_output([str(R/'bin/test_speed_transition')],text=True))
proc=subprocess.run([str(R/'bin/replay_speed_transition'),str(R/'runtime_assets.json'),str(R/'review/speed_transition_field_fixture.json'),str(R/'compact_config.json')],text=True,capture_output=True,env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
replay=json.loads(proc.stdout);assert proc.returncode==0 and replay['valid'],replay
fixture=json.loads((R/'review/speed_transition_field_fixture.json').read_text());d=replay['diagnostics'];state=fixture['state'];a=d['acceleration_command_mps2'];assert -.8<=a<=1. and abs(a-state[6])<=.2001
assert fixture['braking_cap']<replay['speed']<=state[8] and replay['speed']<=2.
assert d['speed_bound_recovery_active'] and d['mpc_solver_status']==0 and d['equality_residual']<.001 and d['inequality_residual']<.001
assert replay['predicted_body_guard_passed'] and replay['nominal_coast_reserve_m']>=0
# The exact independent stopping-space guard still rejects a genuinely unsafe
# sweep; the transition is not a bypass of the map/body rejection.
i=fixture['input'];request=dict(stamp=fixture['prediction_now'],poses=[i['pose']],coast_map=True,speed=2.,servo=state[9])
p=subprocess.run([str(R/'bin/generic_probe'),str(R/'runtime_assets.json')],input=json.dumps(request)+'\n',text=True,capture_output=True,check=True);unsafe=json.loads(p.stdout);assert unsafe.get('reason')=='generic_emergency_coast_map_conflict',unsafe
out=dict(passed=True,physical_motion=False,unit=unit,recorded_state_replay=replay,unsafe_coast_still_rejected=unsafe,scope=fixture['scope'],absolute_max_command_mps=2.,running_floor_mps=.6,residual_tolerance=.001,hard_deadline_ms=200.,source_sha256={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in ['src/cpp_frenet/speed_transition.hpp','src/cpp_frenet/joint_control.hpp','src/cpp_frenet/node.cpp','src/cpp_frenet/generic_avoidance.hpp']})
(R/'review/speed_transition_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(passed=True,next_command_mps=replay['speed'],next_acceleration_mps2=a,solver_status=d['mpc_solver_status'],equality_residual=d['equality_residual'],body_guard_passed=True,unsafe_coast_rejected=True,scope=fixture['scope']),indent=2))
