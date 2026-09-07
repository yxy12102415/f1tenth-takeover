from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start only the ego pure-LiDAR stack; default output is non-actuating."""
    method = LaunchConfiguration('method')
    ego_odom_topic = LaunchConfiguration('ego_odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    drive_output_topic = LaunchConfiguration('drive_output_topic')
    max_speed = LaunchConfiguration('max_speed')
    max_accel = LaunchConfiguration('max_accel')
    max_steer = LaunchConfiguration('max_steer')
    target_timeout = LaunchConfiguration('target_timeout')
    shared_launch = (
        get_package_share_directory('mpc')
        + '/launch/opp_mpc_ego_follow_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='nominal'),
        DeclareLaunchArgument('ego_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        # /drive_shadow is not listed in the physical vehicle's mux config.
        # Live actuation therefore requires an explicit operator override.
        DeclareLaunchArgument(
            'drive_output_topic', default_value='/drive_shadow'
        ),
        DeclareLaunchArgument('max_speed', default_value='0.5'),
        DeclareLaunchArgument('max_accel', default_value='0.5'),
        DeclareLaunchArgument('max_steer', default_value='0.30'),
        DeclareLaunchArgument('target_timeout', default_value='0.5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shared_launch),
            launch_arguments={
                'method': method,
                'ego_odom_topic': ego_odom_topic,
                'scan_topic': scan_topic,
                'drive_output_topic': drive_output_topic,
                'max_speed': max_speed,
                'max_accel': max_accel,
                'max_steer': max_steer,
                'target_timeout': target_timeout,
                'start_lead_controller': 'false',
                'start_evaluator': 'false',
                'scenario': 'constant_speed',
                'map_name': 'physical_vehicle',
            }.items(),
        ),
    ])
