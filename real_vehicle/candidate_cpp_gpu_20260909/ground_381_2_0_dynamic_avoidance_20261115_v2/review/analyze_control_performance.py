from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1]
def stats(values):
 a=sorted(values)
 return dict(n=len(a),median=a[len(a)//2],p95=a[int(.95*(len(a)-1))],maximum=a[-1]) if a else None
def active(path):return [v for v in json.loads(path.read_text())['controller'] if v.get('valid') and v.get('reason')=='ok']
base=active(R/'review/pre_control_perf_opt/reports/compact_fix/dynamic_mpc_delayed_static/records.json')
out=dict(physical_motion=False,scope='Same stationary-opponent synthetic plant; before/after runtime samples are not paired identical trajectories. The coast-preview microbenchmark uses paired fixed inputs.',preview=json.loads((R/'review/coast_preview_optimization_regression.json').read_text()),baseline_measured_primary_guard=stats([v['tracker_diagnostics']['controller_and_dynamic_guard_ms'] for v in base]),controllers={})
for name in ['pid','dynamic_mpc']:
 rows=active(R/'reports/compact_fix'/f'{name}_delayed_static'/'records.json');assert rows
 metrics={}
 keys=['controller_compute_ms','controller_and_dynamic_guard_ms','control_to_emit_ms','mpc_pure_solver_ms','mpc_reference_setup_ms','mpc_tube_setup_ms','mpc_reference_speed_ms','mpc_static_corridor_ms','mpc_parameter_update_ms','mpc_seed_rollout_ms','mpc_solver_residual_ms','mpc_independent_residual_ms']
 for key in keys:
  values=[v['tracker_diagnostics'][key] for v in rows if key in v.get('tracker_diagnostics',{})]
  if values:metrics[key]=stats(values)
 for key in ['observation_preparation_ms','path_speed_preview_ms','state_coast_cap_ms','speed_policy_ms','primary_prediction_guard_ms','emergency_coast_guard_ms','newest_prediction_guard_ms']:
  values=[v['tracker_diagnostics']['node_stage_ms'][key] for v in rows if key in v['tracker_diagnostics'].get('node_stage_ms',{})]
  if values:metrics[key]=stats(values)
 metrics['control_to_drive_publish_ms']=stats([v['runtime_timing']['control_to_drive_publish_ms'] for v in rows])
 assert metrics['control_to_emit_ms']['maximum']<=200.
 if name=='dynamic_mpc':assert metrics['mpc_pure_solver_ms']['n']>0
 out['controllers'][name]=metrics
out.update(passed=True,binary_sha256=hashlib.sha256((R/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller').read_bytes()).hexdigest(),measurement_notes=['mpc_tube_setup_ms is a subset of mpc_reference_setup_ms','path_speed_preview_ms and state_coast_cap_ms are subsets of speed_policy_ms','controller_and_dynamic_guard_ms keeps its legacy scope for before/after comparison','control_to_emit_ms includes newest-observation revalidation; actual DDS publish timing is separate'])
(R/'review/control_performance_optimization.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(dict(preview=out['preview']['speedup'],before=out['baseline_measured_primary_guard'],after=out['controllers']),indent=2))
