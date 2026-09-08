"""BonicBot A2 Gazebo simulation — dev only, never runs on the robot.

Sim-side replacement for bonicbot_a2_hardware: same controllers.yaml the real
robot uses (via gz_ros2_control instead of the ESP32 CDC link), same twist_mux,
same EKF, same teleop. The ESP32, USB CDC protocol and /dev/* devices do not
exist here — ros2_control.xacro's sim_mode branch binds GazeboSimSystem instead.

SLAM / navigation launch separately from bonicbot_a2_nav with use_sim_time:=true,
exactly as on the real robot.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_share = get_package_share_directory('bonicbot_a2_description')
    hardware_share = get_package_share_directory('bonicbot_a2_hardware')
    nav_share = get_package_share_directory('bonicbot_a2_nav')

    # Gazebo resolves package://bonicbot_a2_description/meshes/... by searching
    # this path, so it must point at the DIRECTORY CONTAINING the description
    # package's share dir, not the share dir itself.
    description_share_parent = os.path.dirname(description_share)

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=description_share_parent,
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=description_share_parent,
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_bot_world.sdf',
        description='World file name (must exist in bonicbot_a2_sim/worlds/)',
    )
    use_real_camera_arg = DeclareLaunchArgument(
        'use_real_camera',
        default_value='False',
        description='Use a real webcam via v4l2_camera (True) or the Gazebo camera bridge (False)',
    )
    use_real_camera = LaunchConfiguration('use_real_camera')

    world_path = PathJoinSubstitution([
        FindPackageShare('bonicbot_a2_sim'), 'worlds', LaunchConfiguration('world'),
    ])

    # ── description ──────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(description_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
    )

    # ── teleop (same lanes as the real robot) ────────────────────
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav_share, 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(hardware_share, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )

    # ── Gazebo ───────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'bonicbot_a2',
                   '-z', '0.1'],   # spawn slightly above ground to avoid clipping
        output='screen',
    )

    # Gazebo → ROS. /imu/data matches what the ESP publishes on the real robot,
    # so ekf.yaml's imu0 needs no remap between sim and hardware.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Explicit name: every parameter_bridge defaults to "ros_gz_bridge", so
        # two of them collide and ros2 warns about duplicate node names on every
        # command. A standing warning hides a real one.
        name='gz_bridge_sensors',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_bridge_face_camera',
        arguments=['/face_camera/image_raw'],
        output='screen',
        condition=UnlessCondition(use_real_camera),
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_face_camera_info',
        arguments=['/face_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='screen',
        condition=UnlessCondition(use_real_camera),
    )

    # Real webcam instead of the simulated one — publishes the same
    # /face_camera/image_raw, so vision_pipeline.py cannot tell the difference.
    real_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(hardware_share, 'launch', 'camera.launch.py')]),
        condition=IfCondition(use_real_camera),
    )

    # ── controllers (identical set to hardware.launch.py) ────────
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager-timeout', '120',
                '--switch-timeout', '50',
                # See hardware.launch.py for why --service-call-timeout is raised:
                # all seven spawners race a cold controller_manager, and the 10s
                # default leaves a controller loaded-but-unconfigured on timeout,
                # which looks healthy in the node graph but accepts no commands.
                '--service-call-timeout', '60',
            ],
            parameters=[{'use_sim_time': True}],
        )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'ekf.yaml'),
                    {'use_sim_time': True}],
    )

    return LaunchDescription([
        set_ign_resource_path,
        set_gz_resource_path,
        world_arg,
        use_real_camera_arg,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        bridge,
        image_bridge,
        camera_info_bridge,
        real_camera,
        spawner('diff_cont'),
        spawner('joint_broad'),
        spawner('left_arm_controller'),
        spawner('right_arm_controller'),
        spawner('head_controller'),
        spawner('left_gripper_controller'),
        spawner('right_gripper_controller'),
        ekf,
    ])
