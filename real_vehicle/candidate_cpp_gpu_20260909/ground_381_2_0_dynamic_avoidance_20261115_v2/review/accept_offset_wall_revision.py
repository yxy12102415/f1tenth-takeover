"""Targeted qualification; retain prior integration evidence without restamping it."""
from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parents[1]
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=R/'reports/dynamic_preparation/acceptance.json';old=read(p)
backup=R/'review/preceding_full_suite_acceptance.json'
if not backup.exists():backup.write_text(json.dumps(old,indent=2)+'\n')
old=read(backup)
required=[v for v in old['tests'] if v.get('required_for_381_avoidance')]
assert len(required)==8 and all(v['passed'] for v in required),'Preceding full integration suite not qualified'
build=read(R/'build_provenance.json');assert build['source_unchanged_during_build']
assert all(sha(R/k)==h for k,h in build['source_sha256'].items())
new=[]
for tracker in ['pid','dynamic_mpc']:
 report=R/'reports/compact_fix'/(tracker+'_delayed_static')/'report.json';q=read(report)
 assert report.stat().st_mtime>=(R/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller').stat().st_mtime
 assert q['passed'] and q['route_completed'] and q['ego_start_lateral_m']==-.12 and q['ego_start_heading_offset_rad']==-.05,q
 assert abs(q['plant_startup_command_deadzone_mps']-.70)<1e-9
 records=read(report.parent/'records.json')
 driving=[v.get('speed_command',0) for v in records['controller'] if v.get('valid') and (v.get('tracker_diagnostics') or {}).get('speed_phase')=='driving']
 assert driving and min(driving)>=.6-1e-5
 assert all(abs((v.get('tracker_diagnostics') or {}).get('minimum_drivable_speed_mps',.6)-.6)<1e-7 for v in records['controller'] if v.get('valid'))
 new.append(dict(name=tracker+'_offset_static',passed=True,report=str(report.relative_to(R)),route_completed=True,source='current_binary'))
replay=read(R/'review/compact_fix_regression.json')
assert replay['current_placement_replanned']['valid'] and replay['mapped_wall_occlusion_after']['valid']
assert replay['ready_revalidation_scene_replanned']['valid']
prearm=read(R/'review/prearm_refresh_regression.json');assert prearm['passed']
forecast=read(R/'review/forecast_seed_regression.json');assert forecast['passed']
frame=read(R/'review/reference_timing_frame_regression.json');assert frame['passed']
settle=read(R/'review/stationary_settle_regression.json');assert settle['passed']
route_prior=read(R/'review/route_prior_regression.json');assert route_prior['passed']
wall=read(R/'review/wall_margin_regression.json');assert wall['passed']
assert read(R/'review/projection_cache_regression.json')['passed']
assert read(R/'review/prediction_tube_regression.json')['passed']
assert read(R/'review/deadline_200ms_regression.json')['passed']
assert read(R/'review/coast_preview_optimization_regression.json')['passed']
assert read(R/'review/reference_speed_table_regression.json')['passed']
assert read(R/'review/low_inflation_regression.json')['passed']
transition=read(R/'review/speed_transition_regression.json');assert transition['passed']
for path,value in transition['source_sha256'].items():assert sha(R/path)==value
assert read(R/'review/static_occlusion_memory_regression.json')['passed']
prepared=read(R/'review/observation_preparation_regression.json');assert prepared['passed']
assert prepared['source_sha256']==sha(R/'src/cpp_frenet/generic_avoidance.hpp')
refresh=read(R/'review/prearm_refresh_policy_regression.json');assert refresh['passed']
for path,value in refresh['source_sha256'].items():assert sha(R/path)==value
motion=read(R/'review/motion_confirmation_regression.json');assert motion['passed']
for path,value in motion['source_sha256'].items():assert sha(R/path)==value
assert motion['unit']['moving_stop_restart_passed']
assert motion['recorded_state_replay']['valid'] and motion['recorded_state_replay']['predicted_body_guard_passed']
identity=read(R/'review/lead_identity_regression.json');assert identity['passed']
for path,value in identity['source_sha256'].items():assert sha(R/path)==value
assert identity['terminal_return_203_static'] and identity['lead_role_projection_cache_verified']
terminal=read(R/'review/lead_identity_terminal_path_replay.json')
assert not terminal['results']['original']['valid'] and terminal['results']['classified']['valid']
assert terminal['results']['classified']['score_terms']['generic_held_path']
priority=read(R/'review/static_occlusion_priority_regression.json');assert priority['passed']
assert priority['source_sha256']==sha(R/'src/cpp_frenet/generic_avoidance.hpp')
for evidence in ['low_inflation_regression.json','static_occlusion_memory_regression.json']:
 for path,value in read(R/'review'/evidence)['source_sha256'].items():assert sha(R/path)==value
assert read(R/'review/control_performance_optimization.json')['passed']
assert read(R/'review/deadline_200ms_regression.json')['hard_deadline_ms']==200.
assert read(R/'runtime_assets.json')['wall_margin_m']==read(R/'compact_config.json')['wall_margin_m']==.03
for name in ['direct_overlap_still_rejected','moving_wall_support_still_rejected','unmapped_occlusion_still_rejected']:
 assert replay[name]['reason']=='generic_predicted_occupancy_conflict'
native=read(R/'reports/dynamic_preparation/native_memory.json');assert native['passed']
assert native['binary_hashes']['generic_probe']==sha(R/'bin/generic_probe')
for v in required:
 v['evidence_reused']=True;v['report_is_current']=False;v['execution_scope']='preceding_binary_full_suite_not_rerun_for_this_targeted_fix'
a=dict(old,passed=True,scope='Targeted offset-continuity and strict mapped-wall prediction revision; prior full-suite results retained explicitly as preceding-binary evidence.',tests=required,current_revision_tests=new,prior_full_suite_evidence=str(backup.relative_to(R)),sources=build['source_sha256'],binaries=build['binaries'],full_dynamic_overtake_passed=False,complete_overtaking_is_experimental=True,physical_commissioned=False,physical_retest_pending=True,revision_qualification_script='review/accept_offset_wall_revision.py',revision_qualification_script_sha256=sha(Path(__file__)),recorded_scene_regression='review/compact_fix_regression.json',physical_motion=False)
config=read(R/'compact_config.json')
if config.get('driving_profile')=='tight_clearance_10cm_plus_2cm_reserve':
 tight=read(R/'review/tight_profile_replay.json');assert tight['passed'] and tight['config_sha256']==sha(R/'compact_config.json')
 for tracker in ['pid','dynamic_mpc']:
  records=read(R/'reports/compact_fix'/(tracker+'_delayed_static')/'records.json')
  published=[v for v in records['controller'] if 'desired_body_gap_m' in v]
  assert published and all(abs(v['desired_body_gap_m']-.10)<1e-8 and abs(v['corridor_seed_reserve_m']-.02)<1e-8 for v in published)
  assert any(v.get('prearm_path_refreshed') is True for v in published) and any(v.get('prearm_path_refreshed') is False for v in published)

 a['tight_profile']=dict(desired_body_gap_m=.10,hard_body_gap_m=.08,seed_reserve_m=.02,report='review/tight_profile_replay.json',config_sha256=sha(R/'compact_config.json'),physical_tested=False)
if config.get('driving_profile')=='clearance_15cm_plus_7cm_seed_reserve':
 gap=read(R/'review/gap15cm_regression.json')
 assert gap['passed'] and gap['config_sha256']==sha(R/'compact_config.json')
 assert config['adaptive_corridor_v2']['desired_body_gap_m']==.15
 assert config['generic_corridor_seed_reserve_m']==.07
 for tracker in ['pid','dynamic_mpc']:
  records=read(R/'reports/compact_fix'/(tracker+'_delayed_static')/'records.json')
  published=[v for v in records['controller'] if 'desired_body_gap_m' in v]
  assert published and all(abs(v['desired_body_gap_m']-.15)<1e-8 and abs(v['corridor_seed_reserve_m']-.07)<1e-8 for v in published)
 a.pop('tight_profile',None)
 a['clearance_profile']=dict(desired_body_gap_m=.15,hard_body_gap_m=.08,seed_reserve_m=.07,report='review/gap15cm_regression.json',config_sha256=sha(R/'compact_config.json'),physical_tested=False)
a['observation_preparation_regression']='review/observation_preparation_regression.json'
a['motion_confirmation_regression']='review/motion_confirmation_regression.json'
a['motion_confirmation_policy']='coherent_support_translation_300ms_three_scans_static_latch'
a['lead_identity_regression']='review/lead_identity_regression.json'
a['lead_identity_terminal_replay']='review/lead_identity_terminal_path_replay.json'
a['controlled_scene_prior']='only_continuously_associated_lead_may_move;unknown_near_lead_retains_conservative_prediction'
a['lead_reported_outer_dimensions_m']={'length':read(R/'compact_config.json')['lead_identity_v1']['operator_body_length_m'],'width':read(R/'compact_config.json')['lead_identity_v1']['operator_body_width_m'],'source':'operator_reported'}
a['prearm_refresh_policy']='initialization_or_pose_change;all_new_scans_revalidate_existing_path'
a['prearm_refresh_policy_regression']='review/prearm_refresh_policy_regression.json'
a['prearm_refresh_regression']='review/prearm_refresh_regression.json'
a['forecast_seed_regression']='review/forecast_seed_regression.json'
a['reference_timing_frame_regression']='review/reference_timing_frame_regression.json'
a['stationary_settle_regression']='review/stationary_settle_regression.json'
a['route_prior_regression']='review/route_prior_regression.json'
a['wall_margin_regression']='review/wall_margin_regression.json'
a['moving_dynamic_mpc_qualified']=False
a['minimum_running_command_mps']=.6
a['control_hard_deadline_ms']=200.
a['mpc_soft_budget_ms']=45.
a['mpc_max_consecutive_soft_misses']=3
a['deadline_regression']='review/deadline_200ms_regression.json'
a['reference_speed_table_regression']='review/reference_speed_table_regression.json'
a['static_motion_inflation_m']=0.
a['route_moving_uncertainty_max_m']={'longitudinal':.02,'lateral':.01}
a['static_occlusion_scope']='all_confirmed_static_until_reobserved_motion_or_free_ray_clearance'
a['speed_transition_regression']='review/speed_transition_regression.json'
a['speed_preview_policy']='jerk_feasible_stage_braking_envelope_with_absolute_speed_and_independent_geometry_guards'
a['low_inflation_regression']='review/low_inflation_regression.json'
a['static_occlusion_priority_regression']='review/static_occlusion_priority_regression.json'
a['static_occlusion_regression']='review/static_occlusion_memory_regression.json'
a['performance_regression']='review/control_performance_optimization.json'
a['coast_preview_parity']='review/coast_preview_optimization_regression.json'
a['qualified_motion_trackers']=['pid','dynamic_mpc']
a['preceding_dynamic_mpc_failure']=read(R/'review/pre_dmpc_dynamic_corridor/preceding_dynamic_mpc_report.json')
a.pop('dynamic_mpc_current_delayed_start_result',None)
a['scope']='Current stationary-opponent PID and dynamic MPC with route-aligned prior, 3 cm wall margin and obstacle-aware MPC tube; preceding integration evidence reused explicitly; full moving overtaking not qualified.'
a['both_controller_regression']='review/both_controller_regression.json'
assert read(R/'review/both_controller_regression.json')['passed']
a['revision_status']='targeted_software_verified_physical_pending';p.write_text(json.dumps(a,indent=2)+'\n')
s=read(R/'DYNAMIC_PREPARATION_STATUS.json');s.update(software_acceptance_passed=True,current_revision_status='targeted_software_verified_physical_pending',physical_retest_pending=True,software_qualified_trackers=['pid','dynamic_mpc']);(R/'DYNAMIC_PREPARATION_STATUS.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=True,current_closed_loop_tests=new,prior_full_suite_cases_reused=8,physical_motion=False),indent=2))
