"""Nav2 for BonicBot A2, with the map->odom owner selected by `slam`.

    slam:=false  (default)  production — nav2_amcl + map_server localize against
                            a previously saved map, and AMCL owns map->odom.
    slam:=true              mapping — slam_toolbox (started by bringup.launch.py)
                            owns map->odom, so AMCL and map_server are SKIPPED.

Running slam_toolbox and AMCL together is the failure this argument prevents:
both publish map->odom, the transform tree gets two parents for the same frame,
and the robot's pose flips between their estimates.

Map storage follows BONICBOT_MAPS_DIR (default /maps, the Docker volume mount).

Composed by default (use_composition:=true), as nav2_bringup does on Jazzy:
every Nav2 server is a component in ONE process, nav2_container. On the Pi
that is one process and one DDS participant instead of ten, so far less memory,
discovery traffic and context switching. use_composition:=false runs each
server as its own process again, which is easier to debug or profile one node.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')

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
            'use_composition', default_value='true',
            description='Run the Nav2 servers as components of one container '
                        'process (false: one process per server)'),
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
        DeclareLaunchArgument(
            'map_name', default_value='bonicbot_a2_map.yaml',
            # Extension INCLUDED: robot_app's NavModeManager passes
            # `map_name:=<name>.yaml`, so appending ".yaml" here would look for
            # "<name>.yaml.yaml".
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

    # ── the servers ─────────────────────────────────────────────
    # (package, component plugin, executable, node name, extra params, remaps)
    #
    # Velocity chain, matching nav2_bringup's remappings minus the collision
    # monitor this robot does not run:
    #     controller_server ┐
    #     behavior_server   ┴─cmd_vel_nav─> velocity_smoother ─cmd_vel─> twist_mux
    # Recoveries (spin/backup/drive_on_heading) go through the smoother too, as
    # upstream does, so they are acceleration-limited like path following.
    # twist_mux then arbitrates against /cmd_vel_joy (joystick wins) and
    # forwards the winner to diff_cont. All TwistStamped
    # (enable_stamped_cmd_vel in nav2_params.yaml).
    localization_nodes = [
        ('nav2_map_server', 'nav2_map_server::MapServer', 'map_server', 'map_server',
         {'yaml_filename': map_yaml}, []),
        ('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl', 'amcl', {}, []),
    ]
    navigation_nodes = [
        ('nav2_controller', 'nav2_controller::ControllerServer',
         'controller_server', 'controller_server', {}, [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'nav2_smoother::SmootherServer',
         'smoother_server', 'smoother_server', {}, []),
        ('nav2_planner', 'nav2_planner::PlannerServer',
         'planner_server', 'planner_server', {}, []),
        ('nav2_behaviors', 'behavior_server::BehaviorServer',
         'behavior_server', 'behavior_server', {}, [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
         'bt_navigator', 'bt_navigator', {}, []),
        ('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
         'waypoint_follower', 'waypoint_follower', {}, []),
        ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
         'velocity_smoother', 'velocity_smoother', {},
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    def server_group(nodes, manager_name, condition=None):
        """The servers plus their lifecycle_manager, composed or standalone."""
        manager_params = {
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': bond_timeout,
            'node_names': [name for _, _, _, name, _, _ in nodes],
        }

        def params(extra):
            return [params_file, {'use_sim_time': use_sim_time, **extra}]

        composed = LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(package=pkg, plugin=plugin, name=name,
                               parameters=params(extra), remappings=remaps)
                for pkg, plugin, _, name, extra, remaps in nodes
            ] + [
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name=manager_name, parameters=[manager_params]),
            ],
        )
        standalone = GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                Node(package=pkg, executable=exe, name=name, output='screen',
                     parameters=params(extra), remappings=remaps)
                for pkg, _, exe, name, extra, remaps in nodes
            ] + [
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name=manager_name, output='screen', parameters=[manager_params]),
            ],
        )
        return GroupAction(condition=condition, actions=[composed, standalone])

    # component_container_isolated: one single-threaded executor per component,
    # so a busy server cannot starve another's callbacks inside the process.
    container = Node(
        condition=IfCondition(use_composition),
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Localization ONLY when slam:=false — slam_toolbox owns map->odom otherwise.
    localization = server_group(
        localization_nodes, 'lifecycle_manager_localization',
        condition=UnlessCondition(slam))
    navigation = server_group(navigation_nodes, 'lifecycle_manager_navigation')

    return LaunchDescription(declare_args + [container, localization, navigation])
