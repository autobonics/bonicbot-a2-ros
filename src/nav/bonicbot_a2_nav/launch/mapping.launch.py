"""BonicBot A2 mapping session — slam_toolbox (mapping) + Nav2 core, together.

One composite launch = one process group, so robot_app's NavModeManager
(bonicOS-robot-app) can bring a whole mapping session up and tear it down as a
unit when the operator switches between mapping and navigation at runtime.

This is the `slam:=true` half of the boot-time choice, packaged so it can be
started on demand:
  - slam.launch.py runs slam_toolbox in mapping mode — it owns map->odom live
    and publishes /map as the map is built.
  - navigation.launch.py slam:=true runs the Nav2 core (planner / controller /
    costmaps / bt_navigator) WITHOUT AMCL or map_server, so nothing fights
    slam_toolbox for map->odom. The costmaps consume slam's live /map.

The base stack (drive, sensors, TF, controllers — hardware.launch.py on the
robot or sim.launch.py in Gazebo) is launched separately and must already be up.

The navigation-mode counterpart is navigation.launch.py with slam:=false, run
directly by NavModeManager — no composite is needed there because that launch
already bundles map_server + AMCL + the Nav2 core.

Mirrors bonicbot_m1_nav/launch/mapping.launch.py; robot_app drives both series
through the same interface.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    nav_share = get_package_share_directory('bonicbot_a2_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        # slam:=true is what suppresses AMCL/map_server here.
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'true',
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam,
        nav_core,
    ])
