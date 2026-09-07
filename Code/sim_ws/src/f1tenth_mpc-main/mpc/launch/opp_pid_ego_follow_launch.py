import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = os.path.join(get_package_share_directory('mpc'), 'waypoints', '')
    waypoint_path = share_directory + 'Melbourne_map_mpc.csv'

    return LaunchDescription([
        Node(
            package='mpc',
            executable='opp_pid_node.py',
            name='opp_pid_node',
            parameters=[{
                'waypoints_path': waypoint_path,
                'pose_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
                'lookahead_distance': 1.8,
                'max_speed': 3.0,
                'min_speed': 0.2,
                'max_accel': 0.8,
                'kp_steer': 0.9,
                'ki_steer': 0.0,
                'kd_steer': 0.25,
                'kp_speed': 1.0,
                'ki_speed': 0.0,
                'kd_speed': 0.05,
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='opponent_ekf_tracker.py',
            name='opp_ekf_tracker',
            parameters=[{
                'measurement_source': 'lidar',
                'scan_topic': '/scan',
                'ego_odom_topic': '/ego_racecar/odom',
                'lidar_fov_deg': 180.0,
                'min_cluster_width': 0.18,
                'max_cluster_width': 0.66,
                'min_cluster_points': 9,
                'max_cluster_points': 100,
                'association_gate': 0.7,
                'output_topic': '/ego_racecar/opp_odom_ekf',
                'output_pose_topic': '/ego_racecar/opp_odom_ekf_pose',
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='ego_ekf_follower.py',
            name='ego_pid_follower',
            parameters=[{
                'ego_odom_topic': '/ego_racecar/odom',
                'target_odom_topic': '/ego_racecar/opp_odom_ekf',
                'drive_topic': '/drive',
                'debug_drive_topic': '/ego_pid_debug_cmd',
                'follow_distance': 1.0,
                'lookahead_distance': 2.4,
                'max_speed': 3.0,
                'min_accel': -2.0,
                'min_speed': 0.2,
                'max_steer': 0.4189,
                'kp_steer': 0.25,
                'ki_steer': 0.0,
                'kd_steer': 0.0,
                'max_steer_rate': 1.8,
                'steer_smoothing': 0.84,
                'kp_gap': 2.2,
                'ki_gap': 0.0,
                'kd_gap': 0.02,
                'max_accel': 2.5,
                'dt': 0.05,
                'target_timeout': 1.2,
            }],
            output='screen',
        ),
    ])
