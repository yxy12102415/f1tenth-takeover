import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gym_launch = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"),
        "launch",
        "gym_bridge_launch.py",
    )
    arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "map_name",
            "gui",
            "ego_start_x",
            "ego_start_y",
            "ego_start_yaw",
            "opp_start_x",
            "opp_start_y",
            "opp_start_yaw",
        )
    }
    scenario = LaunchConfiguration("scenario")
    lead_speed = LaunchConfiguration("lead_speed")
    ego_max_speed = LaunchConfiguration("ego_max_speed")
    target_timeout = LaunchConfiguration("target_timeout")

    declarations = [
        DeclareLaunchArgument("scenario", default_value="constant_speed"),
        DeclareLaunchArgument("trial_scenario", default_value="constant_speed"),
        DeclareLaunchArgument("expected_outcome", default_value="safe"),
        DeclareLaunchArgument("map_name", default_value="benchmark_straight"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("mode", default_value="takeover"),
        DeclareLaunchArgument("assets_path"),
        DeclareLaunchArgument("waypoints_path"),
        DeclareLaunchArgument("results_dir"),
        DeclareLaunchArgument("run_id", default_value="trial"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("lead_speed", default_value="0.8"),
        DeclareLaunchArgument("ego_max_speed", default_value="1.5"),
        DeclareLaunchArgument("target_timeout", default_value="1.2"),
        DeclareLaunchArgument("measurement_noise_std", default_value="0.0"),
        DeclareLaunchArgument("dropout_start_s", default_value="-1.0"),
        DeclareLaunchArgument("dropout_duration_s", default_value="0.0"),
        DeclareLaunchArgument("ego_start_x", default_value="6.25"),
        DeclareLaunchArgument("ego_start_y", default_value="3.0"),
        DeclareLaunchArgument("ego_start_yaw", default_value="0.0"),
        DeclareLaunchArgument("opp_start_x", default_value="9.25"),
        DeclareLaunchArgument("opp_start_y", default_value="3.0"),
        DeclareLaunchArgument("opp_start_yaw", default_value="0.0"),
    ]

    return LaunchDescription(declarations + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gym_launch),
            launch_arguments=arguments.items(),
        ),
        Node(
            package="mpc",
            executable="opp_pid_node.py",
            name="takeover_lead_controller",
            parameters=[{
                "waypoints_path": LaunchConfiguration("waypoints_path"),
                "pose_topic": "/opp_racecar/odom",
                "drive_topic": "/opp_drive",
                "lookahead_distance": 1.2,
                "max_speed": 1.0,
                "min_speed": 0.2,
                "max_accel": 0.8,
                "constant_speed": ParameterValue(lead_speed, value_type=float),
                "turning_speed": ParameterValue(lead_speed, value_type=float),
                "scenario": scenario,
            }],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="opponent_ekf_tracker.py",
            name="takeover_lidar_tracker",
            parameters=[{
                "measurement_source": "lidar",
                "scan_topic": "/scan",
                "ego_odom_topic": "/ego_racecar/odom",
                "output_topic": "/ego_racecar/opp_odom_ekf",
                "output_pose_topic": "/ego_racecar/opp_odom_ekf_pose",
                "status_topic": "/ego_tracker/status",
                "lidar_fov_deg": 190.0,
                "cluster_gap": 0.18,
                "min_cluster_width": 0.12,
                "max_cluster_width": 0.85,
                "min_cluster_points": 5,
                "max_cluster_points": 140,
                # Keep segmented corridor walls outside the identity gate when
                # the lead leaves the LiDAR field during a side-by-side pass.
                "association_gate": 0.55,
                "reacquisition_gate": 0.9,
                "prediction_timeout": ParameterValue(target_timeout, value_type=float),
                "target_center_offset": 0.29,
                "max_target_speed": 3.0,
                "min_forward_x": -0.5,
                "simulation_seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                "simulation_measurement_noise_std": ParameterValue(
                    LaunchConfiguration("measurement_noise_std"), value_type=float
                ),
                "simulation_dropout_start_s": ParameterValue(
                    LaunchConfiguration("dropout_start_s"), value_type=float
                ),
                "simulation_dropout_duration_s": ParameterValue(
                    LaunchConfiguration("dropout_duration_s"), value_type=float
                ),
            }],
            output="screen",
        ),
        Node(
            package="takeover_sim",
            executable="takeover_controller_node",
            name="takeover_controller",
            parameters=[{
                "assets_path": LaunchConfiguration("assets_path"),
                "mode": LaunchConfiguration("mode"),
                "cruise_speed": ParameterValue(ego_max_speed, value_type=float),
                "target_timeout": ParameterValue(target_timeout, value_type=float),
                "drive_topic": "/drive_nominal",
            }],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="safety_arbiter.py",
            name="takeover_safety_arbiter",
            parameters=[{
                "nominal_topic": "/drive_nominal",
                "drive_topic": "/drive",
                "nominal_timeout": 0.2,
                "publish_rate": 20.0,
                "max_speed": ParameterValue(ego_max_speed, value_type=float),
                "max_steer": 0.269,
                "recovery_valid_commands": 3,
            }],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="takeover_evaluator.py",
            name="takeover_evaluator",
            parameters=[{
                "scenario": LaunchConfiguration("trial_scenario"),
                "expected_outcome": LaunchConfiguration("expected_outcome"),
                "run_id": LaunchConfiguration("run_id"),
                "results_dir": LaunchConfiguration("results_dir"),
                "map_name": LaunchConfiguration("map_name"),
                "waypoints_path": LaunchConfiguration("waypoints_path"),
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
            }],
            output="screen",
        ),
    ])
