"""BonicBot A2 navigation bringup — EKF, optional SLAM, and Nav2.

Runs on top of either hardware.launch.py (real robot) or sim.launch.py
(Gazebo). Owns no hardware.

    slam:=false  (default)  navigate on a saved map; AMCL owns map->odom
    slam:=true              build a map live; slam_toolbox owns map->odom

The `slam` argument is forwarded to navigation.launch.py so the two can never
disagree about which node publishes map->odom — running slam_toolbox and AMCL
at once puts two publishers on that transform and the pose flips between them.

    ros2 launch bonicbot_a2_nav bringup.launch.py                    # navigate
    ros2 launch bonicbot_a2_nav bringup.launch.py slam:=true         # map

Note for simulation: sim.launch.py already starts rsp and the EKF, so pass
use_ekf:=false there (or launch slam/navigation directly) to avoid duplicates.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: map with slam_toolbox. false: localize with AMCL.'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Start robot_localization. Set false under sim.launch.py, '
                        'which starts its own.'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2. Set false to map without navigating.'),
        DeclareLaunchArgument(
            'use_vision', default_value='false',
            description='Start vision_pipeline.py (YOLO / pose / face / gesture / aruco)'),
        # Forwarded to navigation.launch.py — without these, a saved map could
        # only be selected by launching navigation.launch.py directly.
        DeclareLaunchArgument(
            'maps_dir',
            default_value=os.environ.get('BONICBOT_MAPS_DIR', '/maps'),
            description='Directory holding saved maps'),
        DeclareLaunchArgument(
            'map_name', default_value='bonicbot_a2_map.yaml',
            description='Map file name inside maps_dir (including .yaml)'),
    ]

    # EKF fuses wheel odometry with the ESP IMU's yaw rate and owns the
    # odom->base_link transform (diff_cont has enable_odom_tf: false so they
    # do not both publish it).
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'slam.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, 'launch', 'navigation.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,   # forwarded so AMCL is skipped when slam_toolbox runs
            'maps_dir': LaunchConfiguration('maps_dir'),
            'map_name': LaunchConfiguration('map_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    vision = Node(
        package='bonicbot_a2_nav',
        executable='vision_pipeline.py',
        name='vision_pipeline',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_vision')),
    )

    return LaunchDescription(declare_args + [ekf, slam_toolbox, navigation, vision])
