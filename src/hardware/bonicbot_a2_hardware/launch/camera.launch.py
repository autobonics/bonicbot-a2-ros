"""BonicBot A2 head camera — RPi CSI module (ov5647) via libcamera.

Named `face_camera`, matching bonicbot_m1_hardware_bringup/usb_cameras.launch.py.
A2 fits exactly one camera and it sits in the head (camera_joint parents to
`head`), so it is the same thing M1 calls its face camera. Sharing the topic
name means robot_app addresses both series identically — its per-series
`cameras` map has a "face" entry either way, and a WebRTC client sees the same
track name on both robots.

M1 additionally has `docking_camera` and `depth_camera`; A2 has neither.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── libcamera, NOT v4l2_camera ───────────────────────────────────────────
#
# unicam hands out the sensor's native format and NOTHING else. On this ov5647
# that is SGBRG10 (10-bit Bayer, fourcc GB10) — the only format for which
# VIDIOC_STREAMON succeeds. Every other fourcc the device advertises, YUYV
# included, fails with EINVAL and leaves `unicam fe801000.csi: Failed to start
# media pipeline: -22` in dmesg. Verified on hardware 2026-09-10 by streaming
# each advertised format in turn: GB10 captured, pGCC/GBRG/BA81/GRBG/RGGB/
# YUYV/RGB3 all failed.
#
# v4l2_camera cannot consume 10-bit Bayer, so it could never work here at any
# parameter setting — it asked for YUYV, got EINVAL, and logged "Failed stream
# start: Invalid argument (22)" forever. The advertised format list is a lie:
# the unicam driver enumerates fourccs it will accept in S_FMT but cannot
# actually deliver, so a format only proves itself at STREAMON.
#
# libcamera is the supported path on this hardware: it drives unicam for raw
# Bayer and routes it through the bcm2835-isp HARDWARE ISP, which does the
# debayer and colour conversion at zero CPU cost, then hands out ordinary
# RGB/YUV streams. `RPI vc4.cpp: Registered camera ov5647@36 to Unicam device
# /dev/media1 and ISP device /dev/media0`.
#
# Requires ros-humble-camera-ros (pulls ros-humble-libcamera 0.1.0). Ubuntu's
# own libcamera0 is a 2020 snapshot and is NOT a substitute.


def _camera_node(context, *args, **kwargs):
    fps = float(LaunchConfiguration('fps').perform(context))

    # libcamera sets rate as a frame-DURATION limit in microseconds, pinned
    # min==max to hold a fixed rate. This is the working equivalent of the old
    # `v4l2-ctl --set-parm` step, which this device rejected outright
    # ("VIDIOC_S_PARM: failed: Inappropriate ioctl for device") — unicam has no
    # S_PARM support, so that step could only ever have been a silent no-op.
    #
    # The rate still matters as much as it did at 30 fps: measured 2026-08-28,
    # v4l2_camera_node free-running cost 105.6% of a core, more than every Nav2
    # node combined, against 17.6% at 6 fps.
    frame_us = int(round(1_000_000.0 / fps))

    params = {
        'width': 640,
        'height': 480,
        # libcamera's format names are BYTE-ORDER INVERTED relative to ROS:
        # libcamera BGR888 publishes ROS rgb8, and libcamera RGB888 publishes
        # ROS bgr8 (both verified on hardware 2026-09-10). BGR888 is the one
        # that reproduces v4l2_camera's rgb8 exactly, so nothing downstream
        # changes. Either is labelled correctly and cv_bridge/image_transport
        # convert as needed — this just avoids surprising a consumer that
        # assumes the historical encoding.
        'format': 'BGR888',
        'frame_id': 'face_camera_link_optical',
        'FrameDurationLimits': [frame_us, frame_us],
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    # ── An upside-down camera is fixed in config.txt, NOT here ───────────
    #
    # camera_ros exposes an `orientation` parameter and it does NOT work on
    # this stack: it is implemented against libcamera's Orientation API, added
    # in libcamera 0.2, and ros-humble-libcamera is 0.1.0 (the only version
    # the ROS repo ships). Setting it logs, once, and is then ignored:
    #
    #   [face_camera]: parameter 'orientation' not supported on libcamera 0.1
    #
    # Nor can the sensor's own `vertical_flip`/`horizontal_flip` V4L2 controls
    # be poked directly — both carry `flags=modify-layout`, i.e. flipping
    # shifts the Bayer pattern, and libcamera would then debayer with the wrong
    # phase and produce wrong colours. That is exactly why libcamera insists on
    # owning the flips itself.
    #
    # The supported mechanism is the device tree: `dtoverlay=<sensor>,
    # rotation=180` sets V4L2_CID_CAMERA_SENSOR_ROTATION (visible as the
    # read-only `camera_sensor_rotation` control), which libcamera reads and
    # compensates for automatically, Bayer order included. It costs no CPU —
    # the sensor reads out flipped.
    #
    # So `hardware.camera_vertical_flip` in robot_config.yaml is NOT consumed
    # here; a robot with an inverted camera needs rotation declared in
    # /boot/firmware/config.txt instead.

    # No namespace: camera_ros publishes PRIVATE topics (`~/image_raw`), where
    # v4l2_camera published relative ones. Node name alone therefore yields
    # /face_camera/image_raw — adding a namespace would nest it twice.
    return [Node(
        package='camera_ros',
        executable='camera_node',
        name='face_camera',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='6',
        description='Capture frame rate, applied as a libcamera frame-duration limit',
    )

    return LaunchDescription([
        use_sim_time_arg,
        fps_arg,
        OpaqueFunction(function=_camera_node),
    ])
