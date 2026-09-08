"""RPLIDAR C1M1 — Pi-direct USB, /dev/lidar (udev symlink)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Was a raw /dev/serial/by-path/... string, which breaks whenever the LiDAR
    # is plugged into a different USB port. config/udev/99-bonicbot.rules pins
    # it to /dev/lidar by vendor/product instead.
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lidar',
        description='RPLIDAR serial device (udev symlink)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate', default_value='true',
        description='Resample scans into evenly-spaced angular bins. '
                    'True is better for SLAM; false reclaims most of this '
                    'node\'s CPU. See the note on the parameter below.',
    )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            # angle_compensate resamples each scan into evenly-spaced angular
            # bins, interpolating every point. It is the dominant CPU cost in
            # rplidar_ros and it shows: on 2026-08-25 bench testing this node
            # was the single largest consumer on the RPi4 during navigation,
            # at 44% of a core, ahead of every Nav2 node.
            #
            # It stays True because evenly-spaced scans are what slam_toolbox's
            # scan matcher and the costmap ray-tracing assume; turning it off
            # degrades map quality and localisation, which is a bad trade while
            # the Pi still has headroom (53% idle with the full stack, no IDE
            # session and no robot_app).
            #
            # MEASURED 2026-08-29: this is not a CPU lever at all. A valid A/B
            # (parameter verified via `ros2 param get /rplidar
            # angle_compensate`, camera off in both runs) gave 35.3% / 23.5%
            # with it on and 27.8% / 33.3% with it off — ranges that overlap
            # completely, so run-to-run variance exceeds any difference.
            #
            # The cost of turning it off, by contrast, is immediate:
            # slam_toolbox logs "LaserRangeScan contains 508 range readings,
            # expected 512" on nearly every scan, counts varying 502-511,
            # because the raw revolution is no longer resampled into fixed
            # evenly-spaced bins.
            #
            # LEAVE IT TRUE. The launch argument exists so the trade can be
            # re-measured, not because it is worth taking.
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': 'Standard',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        angle_compensate_arg,
        rplidar,
    ])
