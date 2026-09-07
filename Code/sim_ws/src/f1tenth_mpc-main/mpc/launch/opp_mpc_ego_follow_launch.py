import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    run_id = LaunchConfiguration('run_id')
    results_root = LaunchConfiguration('results_root')
    map_name = LaunchConfiguration('map_name')
    method = LaunchConfiguration('method')
    ego_odom_topic = LaunchConfiguration('ego_odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    drive_output_topic = LaunchConfiguration('drive_output_topic')
    max_speed = LaunchConfiguration('max_speed')
    max_accel = LaunchConfiguration('max_accel')
    max_steer = LaunchConfiguration('max_steer')
    target_timeout = LaunchConfiguration('target_timeout')
    start_lead_controller = LaunchConfiguration('start_lead_controller')
    start_evaluator = LaunchConfiguration('start_evaluator')
    waypoints_file = LaunchConfiguration('waypoints_file')
    waypoint_path = PathJoinSubstitution([
        get_package_share_directory('mpc'), 'waypoints', waypoints_file
    ])

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='constant_speed'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('results_root', default_value=os.path.join(os.getcwd(), 'results')),
        DeclareLaunchArgument('map_name', default_value='Melbourne_map'),
        DeclareLaunchArgument('method', default_value='nominal'),
        DeclareLaunchArgument('ego_odom_topic', default_value='/ego_racecar/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('drive_output_topic', default_value='/drive'),
        DeclareLaunchArgument('max_speed', default_value='3.0'),
        DeclareLaunchArgument('max_accel', default_value='1.8'),
        DeclareLaunchArgument('max_steer', default_value='0.5'),
        DeclareLaunchArgument('target_timeout', default_value='1.2'),
        DeclareLaunchArgument('start_lead_controller', default_value='true'),
        DeclareLaunchArgument('start_evaluator', default_value='true'),
        DeclareLaunchArgument('waypoints_file', default_value='Melbourne_map_mpc.csv'),
        Node(
            package='mpc',
            executable='opp_pid_node.py',
            name='opp_pid_node',
            condition=IfCondition(start_lead_controller),
            parameters=[{
                'waypoints_path': ParameterValue(waypoint_path, value_type=str),
                'pose_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
                'lookahead_distance': 1.8,
                'max_speed': 2.4,
                'min_speed': 0.2,
                'max_accel': 1.0,
                'constant_speed': 1.2,
                # In the turning scenario this is only an upper bound; the
                # generated waypoint curvature profile slows the lead in bends.
                'turning_speed': 2.4,
                'kp_steer': 0.9,
                'ki_steer': 0.0,
                'kd_steer': 0.25,
                'kp_speed': 1.0,
                'ki_speed': 0.0,
                'kd_speed': 0.05,
                'scenario': scenario,
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='opponent_ekf_tracker.py',
            name='opp_ekf_tracker',
            parameters=[{
                'measurement_source': 'lidar',
                'scan_topic': scan_topic,
                'ego_odom_topic': ego_odom_topic,
                'lidar_fov_deg': 190.0,
                'cluster_gap': 0.18,
                'min_cluster_width': 0.12,
                'max_cluster_width': 0.85,
                'min_cluster_points': 5,
                'max_cluster_points': 140,
                'association_gate': 0.75,
                'reacquisition_gate': 1.8,
                'reacquisition_after_missed_scans': 3,
                'association_sigma': 3.0,
                'prediction_timeout': ParameterValue(
                    target_timeout, value_type=float
                ),
                'global_reacquisition_after': 1.5,
                'global_reacquisition_confirmation_scans': 4,
                'global_reacquisition_track_radius': 0.35,
                'global_reacquisition_min_speed': 0.18,
                'global_reacquisition_min_observation_time': 0.35,
                'global_reacquisition_max_motion_rmse': 0.08,
                'global_reacquisition_existence_threshold': 0.60,
                'target_center_offset': 0.29,
                'max_target_speed': 3.0,
                'min_forward_x': -0.5,
                'output_topic': '/ego_racecar/opp_odom_ekf',
                'output_pose_topic': '/ego_racecar/opp_odom_ekf_pose',
                'status_topic': '/ego_tracker/status',
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='ego_mpc_follower.py',
            name='ego_mpc_follower',
            parameters=[{
                'ego_odom_topic': ego_odom_topic,
                'target_odom_topic': '/ego_racecar/opp_odom_ekf',
                'target_status_topic': '/ego_tracker/status',
                'drive_topic': '/drive_nominal',
                'debug_drive_topic': '/ego_mpc_debug_cmd',
                'scan_topic': scan_topic,
                'method': method,
                'follow_distance': 1.0,
                'target_timeout': ParameterValue(
                    target_timeout, value_type=float
                ),
                'target_state_timeout': 0.4,
                'tracker_status_timeout': 0.4,
                'scan_timeout': 0.4,
                'odom_timeout': 0.4,
                'recovery_valid_cycles': 3,
                'control_tolerance': 0.001,
                'max_transient_planner_failures': 2,
                'hybrid_astar_time_budget_ms': 120.0,
                'target_occupancy_exclusion_radius': 0.45,
                'target_center_offset': 0.29,
                'target_heading_alpha': 0.15,
                'target_heading_min_speed': 0.4,
                'target_heading_full_speed': 1.0,
                'target_yaw_rate_alpha': 0.2,
                'max_target_yaw_rate': 1.5,
                'target_prediction_horizon': 1.0,
                'corridor_lateral_offset': 0.58,
                'corridor_min_lateral_offset': 0.58,
                'corridor_max_lateral_offset': 0.88,
                'corridor_lateral_offset_step': 0.05,
                'corridor_boundary_min_points': 6,
                'corridor_boundary_min_span': 0.55,
                'corridor_boundary_fit_max_rmse': 0.18,
                'corridor_curve_blend_distance': 0.60,
                'corridor_blockage_max_range': 4.6,
                'corridor_blockage_confirmation_scans': 3,
                'corridor_blockage_match_radius': 0.25,
                'corridor_geometry_refresh_period': 0.20,
                'candidate_planning_budget_ms': 90.0,
                'planning_start_clearance_radius': 0.24,
                'visibility_fov_deg': 190.0,
                'visibility_max_range': 6.0,
                'visibility_samples': 5,
                # Candidate corridors span the complete obstacle pass.  A
                # one-second view ended before the predicted lead reached the
                # occluder and could not distinguish the two sides.
                'visibility_prediction_horizon': 3.0,
                'visibility_distance_gain': 0.8,
                'risk_release_alpha': 0.15,
                'candidate_visibility_weight': 2.5,
                'candidate_visibility_margin': 0.075,
                'time_headway': 0.15,
                'maximum_follow_distance': 2.5,
                'gap_speed_gain': 0.6,
                'max_catchup_speed_bonus': 0.6,
                'minimum_motion_gap': 0.75,
                'safe_gap_gamma': 1.0,
                'minimum_safe_distance': 0.9,
                'goal_yaw_weight': 0.25,
                'coasting_min_speed_scale': 0.55,
                'min_speed_command': 0.3,
                'max_speed': ParameterValue(max_speed, value_type=float),
                'max_accel': ParameterValue(max_accel, value_type=float),
                'max_steer': ParameterValue(max_steer, value_type=float),
                'startup_delay': 1.0,
                'wall_stop_distance': 0.2,
                'wall_slow_distance': 1.0,
                'cbf_front_gamma': 2.0,
                'cbf_q_accel': 1.0,
                'cbf_q_steer': 30.0,
                'mpc_steer_input_cost': 5.0,
                'mpc_steer_rate_cost': 20.0,
                'mpc_heading_cost': 4.0,
                'mpc_speed_cost': 0.8,
                'turn_slowdown_gain': 0.25,
                'turn_min_speed_scale': 0.75,
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='safety_arbiter.py',
            name='safety_arbiter',
            parameters=[{
                'nominal_topic': '/drive_nominal',
                'drive_topic': drive_output_topic,
                'nominal_timeout': 0.2,
                'publish_rate': 20.0,
                'max_speed': ParameterValue(max_speed, value_type=float),
                'max_steer': ParameterValue(max_steer, value_type=float),
                'recovery_valid_commands': 3,
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='baseline_evaluator.py',
            name='baseline_evaluator',
            condition=IfCondition(start_evaluator),
            parameters=[{
                'scenario': scenario,
                'map_name': map_name,
                'run_id': run_id,
                'results_root': results_root,
                'follow_distance': 1.0,
                'estimate_timeout': 0.4,
                'sample_rate': 20.0,
                'control_topic': drive_output_topic,
                'method': method,
            }],
            output='screen',
        ),
    ])
