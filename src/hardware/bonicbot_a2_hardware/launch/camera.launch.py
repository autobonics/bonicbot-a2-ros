"""BonicBot A2 head camera — RPi CSI module on /dev/video0, via v4l2_camera.

Named `face_camera`, matching bonicbot_m1_hardware_bringup/usb_cameras.launch.py.
A2 fits exactly one camera and it sits in the head (camera_joint parents to
`head`), so it is the same thing M1 calls its face camera. Sharing the topic
name means robot_app addresses both series identically — its per-series
`cameras` map has a "face" entry either way, and a WebRTC client sees the same
track name on both robots.

M1 additionally has `docking_camera` and `depth_camera`; A2 has neither.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='v4l2 capture device for the CSI camera',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='6',
        description='Capture frame rate, applied to the V4L2 device directly',
    )

    # ── Frame rate is set on the DEVICE, not on the node ─────────────────
    #
    # This node has NO frame-rate parameter. `ros2 param list` on it returns
    # image_size, pixel_format, output_encoding and the V4L2 control set —
    # and nothing for timing. The `time_per_frame: [1, 6]` that used to sit in
    # the parameter block below was silently ignored: `ros2 param get ...
    # time_per_frame` answered "Parameter not set".
    #
    # So the camera free-ran at its 30 fps default. Measured 2026-08-28 on real
    # hardware: v4l2_camera_node at 105.6% of a core — more than every Nav2
    # node combined — which saturated the RPi4 to 23% idle and made even the
    # SSH session sluggish. At the intended 6 fps the same node costs 17.6%.
    # Five sixths of that load was frames nobody asked for.
    #
    # v4l2-ctl sets the rate on the device, and the node does NOT override it
    # when it opens the handle — verified by reading --get-parm back after
    # startup. Chained via OnProcessExit so it is guaranteed to run first;
    # launch actions are otherwise unordered.
    #
    # Requires v4l-utils (`sudo apt install v4l-utils`). If it is missing this
    # exits non-zero and the camera never starts, which is deliberate — better
    # a loud failure than silently running at 30 fps again.
    set_frame_rate = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', LaunchConfiguration('video_device'),
             ['--set-parm=', LaunchConfiguration('fps')]],
        output='screen',
    )

    # Vertical flip — a real V4L2 control on this sensor (confirmed via
    # `v4l2-ctl -d /dev/video0 --list-ctrls`: `vertical_flip 0x00980915
    # (bool)`, separate from `horizontal_flip`), so this costs nothing extra —
    # the ISP applies it at readout, same as image_size. NOT a software flip
    # in the image pipeline, which would cost real CPU per frame.
    #
    # Per-robot: some units have the camera mounted upside down, some don't.
    # Source is bonicOS-robot-app's robot_config.yaml (hardware.
    # camera_vertical_flip -> config.py -> os.environ). Absent/unset (the
    # common case) means no 'vertical_flip' key is added at all, and the
    # control keeps its device default (0/false) — same as before this
    # existed.
    camera_params = {
        'video_device': LaunchConfiguration('video_device'),
        'image_size': [640, 480],
    }
    if os.environ.get('CAMERA_VERTICAL_FLIP', '').strip().lower() in \
            ('1', 'true', 'yes', 'on'):
        camera_params['vertical_flip'] = True

    face_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='face_camera',
        namespace='face_camera',
        output='screen',
        parameters=[{
            **camera_params,
            # Frame rate is NOT set here — see set_frame_rate above. This node
            # has no parameter for it and silently ignored the one that used
            # to be in this block.
            # DO NOT set output_encoding: 'yuv422_yuy2' on this build.
            #
            # It looks attractive — the node logs "Image encoding not the same
            # as requested output, performing possibly slow conversion:
            # yuv422_yuy2 => rgb8" on every frame, and publishing natively
            # would skip that and shrink messages by a third (2 bytes/pixel
            # rather than 3). Tried 2026-08-29 and it produces MALFORMED
            # messages: the encoding field reads yuv422_yuy2 correctly, but
            # `step` is published as 640 where 640-wide YUYV requires 1280.
            #
            #   rqt_image_view: "Image is wrongly formed: step < width *
            #   byte_depth * num_channels or 640 != 640 * 1 * 2"
            #
            # rqt shows grey and refuses the frame; RViz reads past the end of
            # the buffer and SEGFAULTS — adding an Image display closes it
            # instantly. /face_camera/image_raw/compressed still publishes at
            # 6 Hz, so the topic looks healthy and the payload is not.
            #
            # The rgb8 conversion stays. It is not free, but the camera is
            # already down to 17.6% of a core after the frame-rate fix, and
            # that conversion is a small part of what remains.
            'camera_name': 'face_camera',
            'camera_frame_id': 'face_camera_link_optical',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        video_device_arg,
        use_sim_time_arg,
        fps_arg,
        set_frame_rate,
        # Start the node only once the device rate is applied.
        RegisterEventHandler(
            OnProcessExit(target_action=set_frame_rate, on_exit=[face_camera])),
    ])
