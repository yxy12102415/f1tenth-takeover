"""Create a new manifest; never replace existing release evidence."""
import hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FILES=['build_mpc_native.sh','src/mpc_iteration.cpp','src/dynamic_mpc.cpp','lib/libmpc_iteration.so','native_mpc.py','native_dynamic_mpc.py','test_native_mpc.py','test_dynamic_mpc.py','benchmark_runtime.py','benchmark_controller_node.py','validate_dynamic_rollout.py',
       'build_health_only.sh','main_lease.py','test_main_lease.py','stand_follow.py','stand_window.py','stand_readiness.py','stand_test_inputs.py',
       'virtual_sink.py','motion_policy.py','health_guard_core.py','health_interlock.py','stand_map_io_optimized.py','optimize_route_clearance.py','../../routes/381_2_0_compact_overtake_595m_20261108_v4.csv','../../routes/381_2_0_compact_overtake_595m_20261108_v4_speed_profile.csv',
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
FILES += ['compact_config.json']
FILES += ['operator_gate_v5.py']
# Copied C++ binaries intentionally retain the immutable v4 library RPATH.
FILES += ['../ground_381_2_0_compact_overtake_20261108_v4/lib/'+n for n in ('libacados.so','libhpipm.so','libblasfeo.so','libacados_ocp_solver_joint_vehicle.so')]
if __name__=='__main__':
    with (ROOT/'runtime_manifest.sha256').open('w') as out:
        for name in FILES:
            out.write(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()+'  '+name+'\n')
