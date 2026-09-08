import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_path = os.path.join(get_package_share_directory('bonicbot_a2_description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    # ParameterValue(..., value_type=str) is NOT optional. Without it launch_ros
    # YAML-parses the generated URDF to infer the parameter's type, and a plain
    # ": " anywhere in that 47 KB string — including inside an XML comment, which
    # xacro copies through verbatim — makes YAML read a mapping key and abort.
    # That surfaces as "Unable to parse the value of parameter robot_description
    # as yaml", which points at the parameter rather than at the comment that
    # actually broke it, while xacro itself still exits 0 with valid XML.
    # Bit us on 2026-08-25 with a dated note in body.xacro's elbow joint.
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_ros2_control:=', use_ros2_control,
                 ' sim_mode:=', use_sim_time]),
        value_type=str)

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_ros2_control)  # Only run when ros2_control is false
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher,
        joint_state_publisher_gui,
    ])
