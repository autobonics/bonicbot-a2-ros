"""The whole base stack in ONE launch file — hardware + rsp + EKF.

This is the file `robot_app` launches when an operator starts the robot from
the dashboard. It exists because robot_app's BaseSessionManager owns exactly
one child process per session (one `ros2 launch` = one process group it can
signal), while A2's base stack is really two launches:

    bonicbot_a2_hardware hardware.launch.py     # rsp, controller_manager, /dev/*
    bonicbot_a2_nav      bringup.launch.py      # EKF (+ SLAM/Nav2, off here)

`start_session_robot.sh` composes those two by hand, which is why starting the
robot used to require SSH. Pointing robot_app straight at `bringup.launch.py`
does not work either: that file owns no hardware, so it can never satisfy
robot_app's health gate (`robot_state_publisher` + `controller_manager`) and
every dashboard start rolled straight back. Composing them here gives
robot_app the single launch file its model expects.

`use_nav2:=false` is deliberate and load-bearing. The nav stack (SLAM or
AMCL+Nav2) is NOT part of the base stack — robot_app starts and stops it at
runtime through NavModeManager, and exactly one of slam_toolbox/AMCL may own
map->odom at a time. Bringing Nav2 up here would put a second owner on that
transform the moment an operator picked a map.

Lives in the nav package rather than the hardware package only because
robot_app resolves the base launch out of `nav_pkg` (ROBOT_CONFIG's
`base_file`). The hardware/nav source split is unaffected: this composes two
launch files, it does not import across the boundary — and hardware.launch.py
already includes the nav package's joystick.launch.py in the same way.

    ros2 launch bonicbot_a2_nav base.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    hardware_share = get_package_share_directory('bonicbot_a2_hardware')
    nav_share = get_package_share_directory('bonicbot_a2_nav')

    declare_args = [
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start the CSI camera (v4l2_camera on /dev/video0)'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the RPLIDAR C1M1 (/dev/lidar)'),
        DeclareLaunchArgument(
            'use_joystick', default_value='true',
            description='Start joystick teleop on the high-priority twist_mux lane'),
        DeclareLaunchArgument(
            'angle_compensate', default_value='true',
            description='LiDAR scan angular resampling — see rplidar.launch.py'),
        DeclareLaunchArgument(
            'use_vision', default_value='false',
            description='Start vision_pipeline.py (YOLO / pose / face / gesture)'),
        # Mirrors the `sleep 12` in start_session_robot.sh. The ESP link has to
        # come up and the controllers have to spawn before the EKF starts
        # fusing /diff_cont/odom — starting it against a silent topic just
        # produces a filter with nothing to filter, and on a busy Pi the
        # controller spawners genuinely take this long.
        DeclareLaunchArgument(
            'ekf_delay_s', default_value='12.0',
            description='Seconds to wait after hardware before starting EKF'),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(hardware_share, 'launch', 'hardware.launch.py')]),
        launch_arguments={
            'use_camera': LaunchConfiguration('use_camera'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_joystick': LaunchConfiguration('use_joystick'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
        }.items(),
    )

    # use_nav2:=false — see the module docstring. NavModeManager owns the nav
    # stack; this brings up the EKF (and optionally vision) only.
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav_share, 'launch', 'bringup.launch.py')]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_nav2': 'false',
            'slam': 'false',
            'use_vision': LaunchConfiguration('use_vision'),
        }.items(),
    )

    return LaunchDescription(declare_args + [
        hardware,
        TimerAction(period=LaunchConfiguration('ekf_delay_s'),
                    actions=[nav_bringup]),
    ])
