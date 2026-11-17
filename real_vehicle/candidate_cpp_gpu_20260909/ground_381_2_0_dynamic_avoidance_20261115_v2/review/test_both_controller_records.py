"""Validate production-node closed-loop records; no physical actuation."""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1];out={}
for tracker in ['pid','dynamic_mpc']:
 p=R/'reports/compact_fix'/(tracker+'_delayed_static');report=json.loads((p/'report.json').read_text());assert report['passed'] and report['route_completed'],report
 rows=json.loads((p/'records.json').read_text())['controller'];active=[v for v in rows if v.get('valid') and v.get('reason')=='ok'];assert active
 published=[v for v in active if 'route_aligned_opponent_prior' in v];assert published and all(v['route_aligned_opponent_prior'] and abs(v['wall_margin_m']-.03)<1e-9 for v in published)
 assert all(v.get('lead_identity',{}).get('enabled') for v in published)
 assert any(v.get('lead_identity',{}).get('reason')=='associated' for v in published)
 assert all(not v['lead_identity']['identity_loss_requests_stop'] for v in published)
 assert any(o.get('semantic_role')=='lead_vehicle' for v in published for o in v.get('occupancies',[]))
 assert all('lead_identity_ms' in v and v['lead_identity_ms']>=0 for v in published)
 driving=[v for v in active if (v.get('tracker_diagnostics') or {}).get('speed_phase')=='driving'];assert driving and min(v['speed_command'] for v in driving)>=.6-1e-5
 assert all(abs((v.get('tracker_diagnostics') or {})['compute_hard_deadline_ms']-200.)<1e-8 for v in driving)
 assert all(abs((v.get('tracker_diagnostics') or {})['full_control_hard_deadline_ms']-200.)<1e-8 for v in driving)
 assert all(0.<v['tracker_diagnostics']['control_to_emit_ms']<=200. for v in active)
 assert all('newest_prediction_guard_ms' in v['tracker_diagnostics']['node_stage_ms'] for v in active)
 assert all(v['runtime_timing']['control_to_drive_publish_ms']>=v['tracker_diagnostics']['control_to_emit_ms']-1e-3 for v in active)
 assert report['minimum_independent_sat_separation_m']>=.08,report
 summary=dict(passed=True,route_completed=True,driving_command_min_mps=min(v['speed_command'] for v in driving),minimum_independent_separation_m=report['minimum_independent_sat_separation_m'],lead_identity_associated=True,maximum_lead_identity_ms=max(v['lead_identity_ms'] for v in published))
 if tracker=='dynamic_mpc':
  d=[v['tracker_diagnostics'] for v in active if (v.get('tracker_diagnostics') or {}).get('dynamic_corridor_enabled')];assert d
  assert all(v['equality_residual']<.001 and v['inequality_residual']<.001 for v in d)
  assert all(v['absolute_command_cap_mps']<=2.+1e-8 and len(v['stage_command_upper_mps'])==24 for v in d)
  assert all(max(v['stage_command_upper_mps'])<=v['absolute_command_cap_mps']+1e-8 for v in d)
  assert max(v['speed_command'] for v in active)<=2.+1e-8
  assert all(v['mpc_pure_solver_ms']>=0 and v['mpc_pure_solver_ms']<=v['controller_compute_ms'] for v in d)
  assert all(v['controller_compute_ms']<=100 for v in d)
  assert all('observation_preparation_ms' in v['node_stage_ms'] for v in d)
  assert all('initial_or_pose_change' in v.get('prearm_refresh_policy','') for v in published)
  radius=min(v['minimum_dynamic_corridor_radius_m'] for v in d);assert radius<.20
  shift=max(v['maximum_dynamic_reference_shift_m'] for v in d);assert shift<=.080001
  policies=sorted(set(v['warm_start_policy'] for v in d));assert any('exact_generated_dynamics' in p for p in policies)
  summary.update(minimum_predicted_tube_radius_m=radius,maximum_reference_shift_m=shift,maximum_control_compute_ms=max(v['controller_compute_ms'] for v in d),maximum_equality_residual=max(v['equality_residual'] for v in d),maximum_total_control_guard_ms=max(v.get('controller_and_dynamic_guard_ms',0) for v in d),warm_start_policies=policies)
 out[tracker]=summary
out.update(passed=True,physical_motion=False,scope='Current C++ nodes, stationary opponent and startup deadzone stress fixture; not physical qualification or full moving overtake',binary_sha256=hashlib.sha256((R/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller').read_bytes()).hexdigest())
(R/'review/both_controller_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
