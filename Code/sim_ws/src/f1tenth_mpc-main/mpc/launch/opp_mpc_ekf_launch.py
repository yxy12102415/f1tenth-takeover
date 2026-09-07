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
            executable='mpc_node.py',
            name='opp_mpc_node',
            parameters=[{
                'waypoints_path': waypoint_path,
                'pose_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
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
    ])
