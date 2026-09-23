"""BonicBot A2 docking pipeline — AprilTag detection + Nav2 Docking Server.

ADDON ONLY. Never started on a robot without the docking addon: on real
hardware there is no /dev/dock_cam to feed it, and in the URDF there is no
docking_camera_link_optical frame for the tag pose to hang under. See
docs/bonicbot_a2_docking.md §1.

This package has no hardware access (CLAUDE.md's hardware/nav split), so it
never launches the camera itself. hardware.launch.py must already be running
with use_docking_camera:=true — or, in simulation, sim.launch.py with
use_docking:=true providing the Gazebo camera bridges.

Pipeline:
  docking_camera/image_raw --[rectify_node]--> docking_camera/image_rect
  image_rect + camera_info --[apriltag_node]--> docking_camera/detections (+ /tf)
  detections + /tf --[dock_pose_publisher.py]--> detected_dock_pose
  detected_dock_pose --[docking_server]--> DockRobot / UndockRobot

The dock's pose arrives with the action call rather than from a static
`docks:` database (§4), keeping this launch file itself pipeline-only.

── Lifecycle: this is designed to be started and stopped, not left running ──

The detector chain is camera + rectify + CPU AprilTag detection, and A2 runs on
an RPi4 already at ~22% idle during a live Nav2 session (docs/CLAUDE.md,
profiled 2026-08-29). Leaving it up for a session to serve a dock attempt that
happens twice a day is not free.

So this file is written to work BOTH ways, with no second code path:
  mode A  navigation.launch.py use_docking:=true  — up for the whole session.
          Simplest, and the right thing for bring-up.
  mode B  robot_app spawns this standalone per dock attempt and tears it down
          on the result. The recommended target.

Mode B pays ~8 s of startup per attempt and that latency is free: DockRobot
with navigate_to_staging_pose:=true drives to the staging pose FIRST, which
takes far longer than the launch settle, so the detector is warm long before
initial_perception_timeout starts mattering at the end of the approach.
See §5.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_nav')
    docking_params = os.path.join(pkg_share, 'config', 'docking.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── There is no battery in Gazebo ────────────────────────────────────
    #
    # SimpleChargingDock's final step is "am I actually charging" — on A2 that
    # reads /battery_state, published by the ESP hardware interface from
    # RESP_BATTERY. Nothing publishes it in simulation, so the robot completes
    # the approach correctly and then sits in wait_charge_timeout before
    # reporting failure: a correct docking run that reports as a failed one.
    #
    # Exposed as an argument rather than left to a file edit, because editing
    # docking.yaml for a sim run means a config difference that can be forgotten
    # and shipped. ROS parameters flatten with dots, so the nested
    # dock_charger.use_battery_status is addressable directly and overrides the
    # value docking.yaml sets — passed after the file, so it wins.
    use_battery_status_arg = DeclareLaunchArgument(
        'use_battery_status', default_value='true',
        description='Require /battery_state to confirm charging. Set false in '
                    'simulation, which has no battery',
    )

    # image_proc's rectify_node, undistorting with the intrinsics from
    # camera_info. apriltag_node needs a rectified image: it solves the tag's
    # pose from the geometry of its corners, and lens distortion bends exactly
    # those corners.
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        namespace='docking_camera',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('image', 'image_raw')],
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='docking_camera',
        parameters=[docking_params, {'use_sim_time': use_sim_time}],
    )

    # Namespaced nowhere: detected_dock_pose is what docking_server subscribes
    # to at the root. Only the input is remapped, into the camera's namespace.
    dock_pose_publisher = Node(
        package='bonicbot_a2_nav',
        executable='dock_pose_publisher.py',
        name='dock_pose_publisher',
        output='screen',
        parameters=[docking_params, {'use_sim_time': use_sim_time}],
        remappings=[('detections', '/docking_camera/detections')],
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[
            docking_params,
            {'use_sim_time': use_sim_time},
            # value_type=bool: a substitution resolves to the STRING "false",
            # which is truthy to a bool parameter's YAML inference and would
            # silently do the opposite of what was asked.
            {'dock_charger.use_battery_status': ParameterValue(
                LaunchConfiguration('use_battery_status'), value_type=bool)},
        ],
    )

    # Own lifecycle manager, separate from lifecycle_manager_navigation. Mode B
    # starts and stops this group independently of the nav session, and a shared
    # manager would take the whole Nav2 stack down with it.
    #
    # bond_timeout follows navigation.launch.py's reasoning — Nav2's 4 s default
    # declares a merely-unscheduled node dead on a loaded RPi4 and tears the
    # group down. 20 s tolerates a scheduling stall while still catching a real
    # crash. Not 0.0, which disables the check entirely.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 20.0,
            'node_names': ['docking_server'],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_battery_status_arg,
        rectify_node,
        apriltag_node,
        dock_pose_publisher,
        docking_server,
        lifecycle_manager,
    ])
