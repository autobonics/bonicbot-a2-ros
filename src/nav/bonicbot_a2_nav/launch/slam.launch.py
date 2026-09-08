"""slam_toolbox in MAPPING mode — owns the map->odom transform.

Must never run at the same time as nav2_amcl: two nodes publishing map->odom
fight over the transform tree and the robot's pose jumps between them.
navigation.launch.py's `slam` argument exists to enforce that — set it true and
AMCL/map_server are skipped because this file owns the transform instead.

Start mapping:
    ros2 launch bonicbot_a2_nav bringup.launch.py slam:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_nav')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )
    params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
        description='slam_toolbox parameter file (mode: mapping)',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox,
    ])
