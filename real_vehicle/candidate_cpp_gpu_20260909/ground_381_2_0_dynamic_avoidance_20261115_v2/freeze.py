"""Create a new manifest; never replace existing release evidence."""
import hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FILES=['build_mpc_native.sh','src/mpc_iteration.cpp','src/dynamic_mpc.cpp','lib/libmpc_iteration.so','native_mpc.py','native_dynamic_mpc.py','test_native_mpc.py','test_dynamic_mpc.py','benchmark_runtime.py','benchmark_controller_node.py','validate_dynamic_rollout.py',
       'build_health_only.sh','main_lease.py','test_main_lease.py','stand_follow.py','stand_window.py','stand_readiness.py','stand_test_inputs.py',
       'virtual_sink.py','motion_policy.py','health_guard_core.py','health_interlock.py','stand_map_io_optimized.py','optimize_route_clearance.py','../../routes/manual_route_20260923_172359_controller_ready.csv','../../routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv',
       '../../maps/381_2_0.yaml','../../maps/381_2_0.pgm',
       'avoidance_controller.py','local_path_tracker.py','test_local_path_tracker.py','test_continuous_tracking.py','obstacle_tracker.py','test_obstacle_tracker.py','test_closed_lattice_controller.py','return_lattice_planner.py','test_return_path.py',
       'test_route_rollout.cpp','test_route_rollout','telemetry.py','start.sh','freeze.py','test_route_profile.py','test_health_window.py','run_offline_suite.py','build.sh',
       'src/ground_pp/core.hpp','src/ground_pp/node.cpp',
       'src/ground_pp/health_startup.hpp','src/ground_pp/test_health_startup.cpp','install_v1/ground_pp/lib/ground_pp/test_health_startup','src/ground_pp/health_core.hpp','src/ground_pp/health_node.cpp','src/ground_pp/health_probe.cpp','test_health_equivalence.py',
       'install_v1/ground_pp/lib/ground_pp/health_interlock_node','install_v1/ground_pp/lib/ground_pp/health_core_probe',
       'src/ground_pp/CMakeLists.txt','src/ground_pp/package.xml',
       'install_v1/ground_pp/lib/ground_pp/ground_pp_node',
       '../../stand_speed_15_monitor_v1/install_clean_v2/vesc_driver/lib/libvesc_driver.so',
       '../../stand_speed_15_monitor_v1/install_clean_v2/vesc_driver/lib/vesc_driver/vesc_driver_node']
FILES += ['launch.sh','bin/core_probe','controller_config.py','runtime_assets.json','export_assets.py','build_cpp.sh','validate_cpp_core.py','validate_cpp_ros.py', 'validate_cpp_rollout.py','validate_cpp_integration.py']
FILES += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'src/cpp_frenet').glob('*')) if p.is_file()]
FILES += ['install_cpp/cpp_frenet/lib/cpp_frenet/'+n for n in ('cpp_frenet_planner','cpp_frenet_controller')]
FILES += ['build_joint.sh','generate_joint_solver.py','joint_solver.json','test_joint_contract.py','validate_joint_rollout.py','validate_pid_tuning.py','compare_pid_parameters.py','test_pid_terminal.py','analyze_pid_drive.py','analyze_steering_response.py','compare_recovery_control.py','compare_prediction.py','validate_recovery.py','test_pid_response.py']
FILES += [str(p.relative_to(ROOT)) for p in sorted((ROOT/'generated_joint').rglob('*')) if p.is_file() and p.suffix in ('.c','.h')]
FILES += ['lib/'+n for n in ('libacados.so','libhpipm.so','libblasfeo.so','libacados_ocp_solver_joint_vehicle.so')]
FILES += ['replay_recorded_pid.py','replay_replanned_pid.py','validate_box_robustness.py','validate_recorded_clouds.py','test_stop_observation.py','test_pid_obstacle_regression.py']
FILES += ['test_pid_speed_floor.py','validate_terminal_ros.py']
FILES += ['test_dmpc_speed_floor.py','validate_dmpc_extra.py']
FILES += ['test_wait_scan_ros.py']
FILES += ['test_compute_timing_ros.py','replay_compute_timeout.py','bin/test_control_timing','bin/libtest_compute_delay.so']
FILES += ['validate_speed2_accel2.py','test_launch_sync_ros.py','validate_speed_headroom.py']
FILES += ['../../plan_381_2_0_straight_route.py','../../routes/381_2_0_compact_overtake_595m_20261108_v4.geometry.json']
FILES += ['compact_config.json','review/test_gap15cm.py','review/gap_15cm_contact_scene_replay.json']
FILES += ['review/test_motion_confirmation.cpp','review/test_recorded_motion_confirmation.py','bin/test_motion_confirmation']
FILES += ['src/cpp_frenet/lead_identity.hpp','review/lead_identity_probe.cpp','review/test_lead_identity.py','bin/lead_identity_probe']
FILES += ['operator_gate_v5.py','perception_evidence_v7.json','hypothesis_reduction_evidence_v8.json','joint_feasibility_evidence_v9.json']
FILES += ['exit_tuning_evidence_v10.json']
FILES += ['ego_returns_evidence_v11.json']
FILES += ['compact_pid_evidence_v2.json','stand_readiness.py','cpp_reuse_provenance.json']
FILES += ['test_generic_avoidance.py','test_generic_controls.py','test_generic_ros.py','run_generic_ros_suite.py','replay_generic_qp.py','build_generic_probes.sh','build_provenance.json','qualify_generic.py','capture_generic_scan.py','replay_live_scan.py','bin/generic_probe','GENERIC_AVOIDANCE.md','../../cpp_vehicle_session.py','../../generic_avoidance_vehicle_session.py','../../381_2_0_generic_avoidance.sh']
FILES += ['test_independent_fixes_20261115.py']
FILES += ['test_adaptive_corridor.py','qualify_adaptive_corridor.py','controller_assets.py','controller_profiles/dynamic_mpc/compact_config.json','controller_profiles/dynamic_mpc/runtime_assets.json']
FILES += ['regression_20261111_early_stop.json','compact_profile_fixture_20261112.json']
FILES += ['test_planner_refresh_20261115.py','test_zero_qualification_20261115.py','planner_refresh_fixture_20261115.json']
FILES += ['test_stationary_memory_20261115.py','stationary_memory_fixture_20261115.json']
FILES += ['qualify_fixed_world_pid.py','review/test_both_controller_records.py','review/test_projection_cache.py','review/test_prediction_tube.cpp','review/test_coast_preview_optimization.py','review/analyze_control_performance.py']
FILES += ['review/test_observation_preparation.py','review/test_prearm_refresh_policy.cpp','bin/test_prearm_refresh_policy']
FILES += ['review/test_static_occlusion_priority.py','review/static_occlusion_priority_field_fixture.json']
FILES += ['review/test_speed_transition.cpp','review/replay_speed_transition.cpp','review/test_speed_transition.py','review/speed_transition_field_fixture.json','bin/test_speed_transition','bin/replay_speed_transition']
FILES += ['review/test_low_inflation.py','review/test_static_occlusion_memory.cpp','bin/test_static_occlusion_memory']
FILES += ['review/test_reference_speed_table.cpp','review/test_reference_speed_table.py','review/reference_speed_benchmark.cpp','review/reference_speed_field_fixture.json','bin/test_reference_speed_table']
FILES += ['review/test_route_prior.py','review/test_wall_margin.py','review/test_stationary_settle.py','review/accept_offset_wall_revision.py']
# Explicit retired compact evidence; every other missing runtime file fails.
RETIRED=['perception_evidence_v7.json', 'hypothesis_reduction_evidence_v8.json', 'joint_feasibility_evidence_v9.json', 'exit_tuning_evidence_v10.json', 'ego_returns_evidence_v11.json', 'compact_pid_evidence_v2.json', 'cpp_reuse_provenance.json']
FILES=[n for n in FILES if n not in RETIRED]
FILES += ['test_dynamic_memory.py','bin/test_dynamic_memory','OPEN_SOURCE_REFERENCES.json','OPEN_SOURCE_REVIEW.md','DYNAMIC_PREPARATION_STATUS.json']
FILES += ['finalize_dynamic_preparation.py','prepare_dynamic_long_fixture.py','replay_captured_memory.py','DYNAMIC_OPERATIONS.md','../../381_2_0_dynamic_avoidance.sh','../../381_2_0_dynamic_avoidance_prepare.sh','../../dynamic_avoidance_vehicle_session.py','../../cpp_vehicle_session.py']
# Human notes and mutable progress are not runtime identity.
FILES=[name for name in FILES if not name.endswith('.md') and name not in ('DYNAMIC_PREPARATION_STATUS.json','OPEN_SOURCE_REFERENCES.json')]
if __name__=='__main__':
    from controller_assets import generate_profiles
    generate_profiles(ROOT)
    with (ROOT/'runtime_manifest.sha256').open('w') as out:
        for name in sorted(set(FILES)):
            out.write(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()+'  '+name+'\n')
