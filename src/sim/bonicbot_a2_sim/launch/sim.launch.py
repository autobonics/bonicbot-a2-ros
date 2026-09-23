"""BonicBot A2 Gazebo simulation — dev only, never runs on the robot.

Sim-side replacement for bonicbot_a2_hardware: same controllers.yaml the real
robot uses (via gz_ros2_control instead of the ESP32 CDC link), same twist_mux,
same EKF, same teleop. The ESP32, USB CDC protocol and /dev/* devices do not
exist here — ros2_control.xacro's sim_mode branch binds GazeboSimSystem instead.

SLAM / navigation launch separately from bonicbot_a2_nav with use_sim_time:=true,
exactly as on the real robot.

Docking:
    ros2 launch bonicbot_a2_sim sim.launch.py \
        world:=obstacle_world.sdf use_docking:=true

adds the rear docking camera to the model and bridges it out of Gazebo. The
detector chain and docking_server come from bonicbot_a2_nav's docking.launch.py,
not from here — this package owns the simulated robot, not the pipeline.
See docs/bonicbot_a2_docking.md.
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

    sim_share = get_package_share_directory('bonicbot_a2_sim')

    # Gazebo resolves package://<pkg>/... by stripping the scheme and searching
    # these paths, so each entry must be the DIRECTORY CONTAINING a package's
    # share dir, not the share dir itself.
    #
    # BOTH packages are listed, not just the description one. Without an
    # isolated (non-merged) colcon install each package has its own prefix, so
    # dirname(description_share) does not contain bonicbot_a2_sim — and
    # obstacle_world.sdf's package://bonicbot_a2_sim/meshes/apriltag_plate.obj
    # then silently fails to load. Gazebo does not error on an unresolvable
    # visual mesh; it draws nothing, so the dock's tag is simply absent and the
    # detector looks broken.
    #
    # Any pre-existing value is preserved: a dev with their own models on the
    # path should not lose them by launching this.
    resource_paths = [os.path.dirname(description_share),
                      os.path.dirname(sim_share)]

    def _resource_path(existing_var):
        existing = os.environ.get(existing_var, '')
        parts = resource_paths + ([existing] if existing else [])
        return os.pathsep.join(parts)

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=_resource_path('IGN_GAZEBO_RESOURCE_PATH'),
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=_resource_path('GZ_SIM_RESOURCE_PATH'),
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

    # ── docking addon in simulation ──────────────────────────────────────
    #
    # The whole docking pipeline is testable here BEFORE the addon hardware
    # exists, and that is worth more than it sounds: every bug found in sim is
    # one not found while a robot is reversing into furniture.
    #
    # Default from the env so `export DOCKING_ADDON=1` behaves the same as it
    # does on a real robot, and so a plain sim run is unchanged.
    use_docking_arg = DeclareLaunchArgument(
        'use_docking',
        default_value=os.environ.get('DOCKING_ADDON', 'false'),
        description='Add the rear docking camera to the model and bridge it out '
                    'of Gazebo. The dock itself lives in obstacle_world.sdf',
    )
    use_docking = LaunchConfiguration('use_docking')

    # Sets the SAME env var the real robot uses, so rsp.launch.py's xacro call
    # includes docking_camera.xacro and the model actually carries the camera.
    # Passing use_docking:=true without this would bridge topics that the
    # simulated robot never publishes.
    #
    # ORDER MATTERS: this must be visited before `rsp`, because the xacro
    # Command that reads $(optenv DOCKING_ADDON false) is evaluated when the
    # robot_state_publisher node's parameters are resolved. It is first in the
    # returned LaunchDescription for that reason — do not reorder it below rsp.
    set_docking_env = SetEnvironmentVariable(
        name='DOCKING_ADDON', value='1',
        condition=IfCondition(use_docking),
    )

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

    # ── docking camera bridges ───────────────────────────────────────────
    #
    # Two separate bridges for one camera, matching the face camera above:
    # ros_gz_image for the image (it handles the image payload efficiently) and
    # parameter_bridge for camera_info (ros_gz_image does not carry it).
    #
    # BOTH are required, and camera_info is the one that looks optional and is
    # not: rectify_node needs the intrinsics to undistort, and apriltag_node
    # derives the tag's DISTANCE from them. Bridge only the image and the
    # detector sits waiting on a camera_info that never arrives — no error, just
    # no detections.
    #
    # Unlike the real robot, sim needs no calibration file: Gazebo's camera
    # publishes camera_info that exactly describes its own pinhole model, which
    # is the one case where the intrinsics are perfect for free.
    docking_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_bridge_docking_camera',
        arguments=['/docking_camera/image_raw'],
        output='screen',
        condition=IfCondition(use_docking),
    )

    docking_camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_docking_camera_info',
        arguments=['/docking_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='screen',
        condition=IfCondition(use_docking),
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
        use_docking_arg,
        # Before rsp — see the comment on set_docking_env.
        set_docking_env,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        bridge,
        image_bridge,
        camera_info_bridge,
        docking_image_bridge,
        docking_camera_info_bridge,
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
