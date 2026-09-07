# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']
    map_name = LaunchConfiguration('map_name')
    map_path = PathJoinSubstitution([
        get_package_share_directory('f1tenth_gym_ros'), 'maps', map_name
    ])
    ld.add_action(DeclareLaunchArgument('map_name', default_value='Melbourne_map'))
    start_defaults = config_dict['bridge']['ros__parameters']
    for argument, parameter in (
        ('ego_start_x', 'sx'),
        ('ego_start_y', 'sy'),
        ('ego_start_yaw', 'stheta'),
        ('opp_start_x', 'sx1'),
        ('opp_start_y', 'sy1'),
        ('opp_start_yaw', 'stheta1'),
    ):
        ld.add_action(
            DeclareLaunchArgument(argument, default_value=str(start_defaults[parameter]))
        )

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {
            'map_path': ParameterValue(map_path, value_type=str),
            'sx': ParameterValue(LaunchConfiguration('ego_start_x'), value_type=float),
            'sy': ParameterValue(LaunchConfiguration('ego_start_y'), value_type=float),
            'stheta': ParameterValue(LaunchConfiguration('ego_start_yaw'), value_type=float),
            'sx1': ParameterValue(LaunchConfiguration('opp_start_x'), value_type=float),
            'sy1': ParameterValue(LaunchConfiguration('opp_start_y'), value_type=float),
            'stheta1': ParameterValue(LaunchConfiguration('opp_start_yaw'), value_type=float),
        }]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')],
        prefix=(
            'env '
            '-u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT '
            '-u SNAP_COOKIE -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME '
            '-u SNAP_LAUNCHER_ARCH_TRIPLET -u SNAP_LIBRARY_PATH -u SNAP_NAME '
            '-u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID -u SNAP_USER_COMMON '
            '-u SNAP_USER_DATA -u SNAP_VERSION '
            '-u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH '
            '-u GIO_MODULE_DIR '
            'XDG_DATA_HOME=$HOME/.local/share '
            'XDG_DATA_DIRS=/usr/local/share:/usr/share '
            'LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'
        )
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': ParameterValue([map_path, '.yaml'], value_type=str)},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
