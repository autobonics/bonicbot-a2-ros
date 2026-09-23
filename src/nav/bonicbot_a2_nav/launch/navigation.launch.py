"""Nav2 for BonicBot A2, with the map->odom owner selected by `slam`.

    slam:=false  (default)  production — nav2_amcl + map_server localize against
                            a previously saved map, and AMCL owns map->odom.
    slam:=true              mapping — slam_toolbox (started by bringup.launch.py)
                            owns map->odom, so AMCL and map_server are SKIPPED.

Running slam_toolbox and AMCL together is the failure this argument prevents:
both publish map->odom, the transform tree gets two parents for the same frame,
and the robot's pose flips between their estimates.

Map storage follows BONICBOT_MAPS_DIR (default /maps, the Docker volume mount).

`use_docking` additionally starts the docking pipeline, on robots carrying the
docking addon only. It defaults from $DOCKING_ADDON so that a fitted robot needs
no extra argument and an unfitted one can never start it.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: slam_toolbox owns map->odom, skip AMCL. '
                        'false: AMCL + map_server localize on a saved map. '
                        'MUST match bringup.launch.py'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Auto-transition the Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'maps_dir',
            # Not the process CWD: on the robot this is the /maps Docker volume,
            # and bare-metal dev exports BONICBOT_MAPS_DIR at the workspace.
            default_value=os.environ.get('BONICBOT_MAPS_DIR', '/maps'),
            description='Directory holding saved maps'),
        # ── docking addon, mode A of docs/bonicbot_a2_docking.md §5 ──
        #
        # Bundles the docking pipeline into the nav session: detector chain and
        # docking_server up for as long as navigation is. Simple, M1-shaped,
        # and the right thing for bring-up and for simulation.
        #
        # Mode B — robot_app spawning docking.launch.py per dock attempt and
        # tearing it down on the result — is the target on real hardware,
        # because the detector chain is real CPU on an RPi4 that has none
        # spare. Same launch file either way; this argument is the only
        # difference.
        #
        # Defaults from the env so an addon-fitted robot gets it without anyone
        # passing an argument, and a plain A2 never starts nodes for hardware
        # it does not have. Same flag the URDF and hardware.launch.py read.
        DeclareLaunchArgument(
            'use_docking', default_value=os.environ.get('DOCKING_ADDON', 'false'),
            description='Start the docking pipeline (AprilTag detector + '
                        'docking_server). Defaults from $DOCKING_ADDON'),
        DeclareLaunchArgument(
            'use_battery_status', default_value='true',
            description='Docking: require /battery_state to confirm charging. '
                        'Set false in simulation, which has no battery'),
        DeclareLaunchArgument(
            'map_name', default_value='bonicbot_a2_map.yaml',
            # Extension INCLUDED, matching bonicbot_m1_nav. robot_app's
            # NavModeManager passes `map_name:=<name>.yaml`, so appending
            # ".yaml" here would look for "<name>.yaml.yaml".
            description='Map file name inside maps_dir (including .yaml)'),
    ]

    map_yaml = PathJoinSubstitution([
        LaunchConfiguration('maps_dir'),
        LaunchConfiguration('map_name'),
    ])

    # Nav2's bond heartbeat defaults to 4.0 s: if a managed server misses it,
    # lifecycle_manager declares that server DOWN and tears the whole group
    # down with it. On an RPi4 running SLAM/Nav2 + camera + robot_app that is
    # far too tight — the box sits near 22% idle at load ~9, and a node simply
    # not scheduled for four seconds is not a crashed node. Observed on
    # hardware 2026-08-29:
    #
    #   CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a
    #   heartbeat for 4000 ms. Shutting down related nodes.
    #
    # map_server was healthy; it was starved. AMCL was deactivated with it, so
    # map->odom stopped, the `map` frame vanished, and everything above blamed
    # localization: "Waiting for map...", no pose, and load_map service calls
    # timing out because map_server was no longer active.
    #
    # 20 s is tolerant of scheduling stalls while still catching a genuinely
    # dead server. NOT 0.0, which disables the check altogether and would hide
    # a real crash.
    bond_timeout = 20.0

    # ── localization: ONLY when slam:=false ──────────────────────
    localization = GroupAction(
        condition=UnlessCondition(slam),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'bond_timeout': bond_timeout,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
        ],
    )

    # ── navigation: always ───────────────────────────────────────
    # Velocity chain, matching upstream nav2_bringup's remappings:
    #     controller_server --cmd_vel_nav--> velocity_smoother --cmd_vel--> twist_mux
    # twist_mux then arbitrates against /cmd_vel_joy (joystick wins) and forwards
    # the winner to /diff_cont/cmd_vel_unstamped.
    nav2_nodes = [
        ('nav2_controller', 'controller_server', 'controller_server',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    navigation = GroupAction(actions=[
        Node(
            package=pkg,
            executable=exe,
            name=name,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remaps,
        )
        for pkg, exe, name, remaps in nav2_nodes
    ] + [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': bond_timeout,
                'node_names': [name for _, _, name, _ in nav2_nodes],
            }],
        ),
    ])

    # ── docking: only with the addon ─────────────────────────────
    # use_sim_time is forwarded, not re-derived: docking_server and the detector
    # chain must share the nav stack's clock or the tag pose and the robot pose
    # are stamped on different timelines and every tf2 lookup fails.
    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, 'launch', 'docking.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Forwarded so a sim session can switch it off in one place —
            # Gazebo has no battery, so the charge confirmation can never
            # succeed. See docking.launch.py.
            'use_battery_status': LaunchConfiguration('use_battery_status'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_docking')),
    )

    return LaunchDescription(declare_args + [localization, navigation, docking])
