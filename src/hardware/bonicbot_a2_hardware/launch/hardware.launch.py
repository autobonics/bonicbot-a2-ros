"""BonicBot A2 hardware bringup — everything that touches /dev/*.

Starts robot_state_publisher, the ros2_control controller_manager bound to the
ESP32-S3 USB CDC interface, all seven controllers, twist_mux, the RPLIDAR and
(optionally) the CSI camera.

Navigation runs separately:
    ros2 launch bonicbot_a2_nav bringup.launch.py

The simulation equivalent of this file is bonicbot_a2_sim/sim.launch.py, which
swaps the ESP interface for gz_ros2_control and is otherwise the same set of
controllers.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('bonicbot_a2_hardware')
    description_share = get_package_share_directory('bonicbot_a2_description')
    nav_share = get_package_share_directory('bonicbot_a2_nav')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Start the CSI camera (v4l2_camera on /dev/video0)',
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Start the RPLIDAR C1M1 (/dev/lidar)',
    )
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick', default_value='true',
        description='Start joystick teleop on the high-priority twist_mux lane',
    )
    # Forwarded to rplidar.launch.py so the LiDAR's dominant CPU cost can be
    # toggled from the top-level launch without editing files. See the note on
    # the parameter in rplidar.launch.py before setting this false.
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate', default_value='true',
        description='LiDAR scan angular resampling. False reclaims most of '
                    'rplidar_composition\'s CPU at some cost to scan geometry',
    )
    joint_states_throttle_hz_arg = DeclareLaunchArgument(
        'joint_states_throttle_hz', default_value='10.0',
        description='Rate for /joint_states_throttled, which robot_app '
                    'subscribes to instead of the raw 50 Hz /joint_states. '
                    '0 disables the throttle node entirely',
    )

    # ── robot description ────────────────────────────────────────
    # sim_mode:=false selects the ESP hardware interface in ros2_control.xacro.
    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    # See rsp.launch.py for why ParameterValue(value_type=str) is required here:
    # without it launch_ros YAML-parses the URDF string, and any ": " in the
    # generated XML (comments included) aborts the launch with a message that
    # blames robot_description instead of the comment.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=false']),
        value_type=str)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(description_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items(),
    )

    # ── ros2_control ─────────────────────────────────────────────
    # controllers.yaml hardcodes wheel_radius: 0.06 for diff_cont's own
    # cmd_vel -> wheel-velocity conversion — a SEPARATE number from the
    # wheel_radius ros2_control.xacro passes the hardware interface (which
    # IS per-robot, via $(optenv WHEEL_RADIUS)). If only the xacro side moved,
    # diff_cont would compute wheel speed against 0.06 while the hardware
    # interface converts back using the robot's real radius, silently scaling
    # every drive command by the ratio of the two. Same fix, same source
    # (bonicOS-robot-app's robot_config.yaml, hardware.wheel_radius) — applied
    # here as a parameter override loaded AFTER controllers.yaml, so it wins
    # for this one key and controllers.yaml's default still applies to
    # everything else. No env var set (no calibration provisioned for this
    # robot) means no override dict is added at all, so behaviour is
    # byte-identical to before this existed.
    controller_manager_params = [
        {'robot_description': robot_description},
        os.path.join(pkg_share, 'config', 'controllers.yaml'),
    ]
    if 'WHEEL_RADIUS' in os.environ:
        controller_manager_params.append(
            {'diff_cont': {'ros__parameters':
                {'wheel_radius': float(os.environ['WHEEL_RADIUS'])}}})

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=controller_manager_params,
        output='screen',
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager-timeout', '120',
                '--switch-timeout', '50',
                # All seven spawners fire at once against a controller_manager
                # that is still cold — and on A2 it is colder than most, because
                # on_configure() opens the CDC port and sleeps 500 ms settling it
                # before the manager can serve anything.
                #
                # --service-call-timeout is per-CALL and separate from
                # --controller-manager-timeout (which only covers waiting for the
                # services to appear). Its 10s default can expire on the first
                # load_controller call, and that spawner then dies with exit
                # code 1 leaving its controller LOADED BUT NEVER CONFIGURED:
                # the node exists and the graph looks healthy, but it has no
                # command subscription. Bit M1 on diff_cont, where it silently
                # made the base undrivable.
                '--service-call-timeout', '60',
            ],
            parameters=[{'use_sim_time': False}],
        )

    # ── twist_mux ────────────────────────────────────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(pkg_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )

    # joint_state_broadcaster publishes at the controller_manager update rate
    # (50 Hz) and that rate is not negotiable — the control loop needs it. But
    # the only consumer above ROS is robot_app, which forwards joint angles to
    # the dashboard's 3D robot model; the frontend's own WebRTC provider
    # documents that it expects 10-15 Hz there. Subscribing to the raw topic
    # meant 50 rclpy callbacks/second in Python for a 10 Hz picture, and rclpy
    # rebuilds its waitset in Python on every one (py-spy on the real A2,
    # 2026-08-29: the executor was the single largest CPU consumer in
    # robot_app). This relay does the 50 Hz half in C++ and hands robot_app a
    # tenth of the wakeups.
    #
    # robot_app subscribes to /joint_states_throttled — see ROBOT_CONFIG["A"]
    # in bonicOS-robot-app/app/config.py. The two MUST agree: point robot_app
    # at a topic nothing publishes and its joint telemetry goes silently dead,
    # exactly the way /odom did before commit 6d6fa38.
    joint_states_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='joint_states_throttle',
        arguments=['messages', '/joint_states',
                   LaunchConfiguration('joint_states_throttle_hz'),
                   '/joint_states_throttled'],
        output='screen',
    )

    # ── the other /dev/* owners ──────────────────────────────────
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rplidar.launch.py')]),
        launch_arguments={
            'angle_compensate': LaunchConfiguration('angle_compensate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'camera.launch.py')]),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav_share, 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    return LaunchDescription([
        use_camera_arg,
        use_lidar_arg,
        use_joystick_arg,
        angle_compensate_arg,
        joint_states_throttle_hz_arg,
        rsp,
        controller_manager,
        spawner('diff_cont'),
        spawner('joint_broad'),
        spawner('left_arm_controller'),
        spawner('right_arm_controller'),
        spawner('head_controller'),
        spawner('left_gripper_controller'),
        spawner('right_gripper_controller'),
        joint_states_throttle,
        twist_mux,
        rplidar,
        camera,
        joystick,
    ])
