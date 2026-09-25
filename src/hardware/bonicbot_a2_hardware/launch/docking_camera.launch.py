"""BonicBot A2 docking camera — rear-facing USB webcam, ADDON-ONLY.

Started only on robots fitted with the docking addon: hardware.launch.py's
`use_docking_camera` defaults from $DOCKING_ADDON, which robot_app sets from
robot_config.yaml's `addons: {docking: true}`. On a plain A2 this file is never
included and nothing here exists. See docs/bonicbot_a2_docking.md §1.

Named `docking_camera`, publishing on `/docking_camera/image_raw` (and
`/compressed`, which is why the transports below are trimmed rather than left
at their defaults). robot_app streams it over WebRTC automatically on an
addon-fitted robot — its `Settings.cameras` adds a `docking` entry whenever
the same `DOCKING_ADDON` flag that starts this node is set, so no env var has
to be kept in agreement with it. `CAMERAS` still overrides the whole set for a
robot or sim whose topics differ.


── Why usb_cam and not camera_ros ───────────────────────────────────────

The head camera goes through camera_ros/libcamera because A2's CSI sensor
hands unicam raw 10-bit Bayer and nothing else (see camera.launch.py — that
whole path exists to get it debayered on the hardware ISP). None of that
applies here. This is a UVC webcam: it delivers MJPEG over USB, the kernel's
uvcvideo driver owns it, and usb_cam is the driver that speaks to it. Routing
it through libcamera would add a Bayer pipeline to a device that produces no
Bayer.

It has to be USB at all because A2's single CSI port is already taken by the
head camera.

── Framerate: 10, not 30 ─────────────────────────────────────────────────

A2 runs on an RPi4 that already sits at ~22% idle during a live Nav2 session
(docs/CLAUDE.md, profiled 2026-08-29) and feeds a CPU AprilTag detector. 30
fps buys nothing on an approach measured in tens of seconds, and the head
camera has already demonstrated on this exact board what an unconstrained
camera node costs: 105.6% of a core at 30 fps against 17.6% at 6 fps.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/dock_cam',
        description='udev symlink for the rear docking camera '
                    '(config/udev/99-bonicbot.rules)',
    )
    framerate_arg = DeclareLaunchArgument(
        'framerate', default_value='10.0',
        description='Capture frame rate. 10, not 30 — this runs on an RPi4 '
                    'feeding a CPU AprilTag detector, see the note in this file',
    )
    # ── Intrinsics are NOT optional for docking ──────────────────────────
    #
    # AprilTag pose estimation solves for distance from the tag's apparent
    # size, so it reads focal length and the distortion coefficients straight
    # out of camera_info. Uncalibrated, usb_cam publishes a placeholder
    # camera_info and the detector still reports a pose — a confidently wrong
    # one, off by a scale factor that grows as the robot closes in. That is the
    # worst failure shape available here: it looks like it works and then
    # misses the dock.
    #
    # Defaults into the MAPS VOLUME, not the container filesystem. /maps is a
    # bind mount; everything else in the container is an image layer that a
    # `docker compose up` recreate throws away — and a calibration lost that
    # way does not announce itself, it comes back as a dock attempt that misses
    # by a few centimetres. usb_cam's own fallback
    # (~/.ros/camera_info/<camera_name>.yaml) is exactly that kind of
    # throwaway path, which is why this does not rely on it.
    #
    # If the file is absent, usb_cam publishes placeholder intrinsics and warns
    # with the path it wanted — treat that warning as blocking, because
    # image_proc's rectify_node then refuses to publish image_rect at all and
    # apriltag receives nothing (observed on hardware 2026-09-24). A missing
    # calibration fails loudly, not subtly; do not "fix" it by removing rectify.
    _maps = os.environ.get('BONICBOT_MAPS_DIR', '/maps')
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=os.environ.get(
            'DOCK_CAM_INFO_URL', f'file://{_maps}/calib/docking_camera.yaml'),
        description='file:// URL of this robot\'s docking camera intrinsics. '
                    'Empty = uncalibrated; AprilTag range will be wrong',
    )

    # Plain RGB, so trim the transports image_transport would otherwise
    # advertise by default: compressedDepth is meaningless for RGB (its codec
    # only accepts depth encodings) and theora is unused. Fewer advertised
    # encodings matters once this streams over WebRTC.
    rgb_only_transports = ['image_transport/raw', 'image_transport/compressed']

    docking_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='docking_camera',
        namespace='docking_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 640,
            'image_height': 480,
            # value_type=float, not the bare LaunchConfiguration: a
            # substitution resolves to a STRING, and usb_cam declares
            # framerate as a double — handing it "10.0" aborts the node with
            # InvalidParameterTypeException rather than falling back.
            'framerate': ParameterValue(LaunchConfiguration('framerate'),
                                        value_type=float),
            'pixel_format': 'mjpeg2rgb',
            'camera_name': 'docking_camera',
            # MUST match docking_camera.xacro's optical link. apriltag_ros
            # publishes the tag's TF as a child of whatever frame_id arrives on
            # camera_info, so a mismatch here does not error — it plants the
            # tag under a frame that is not in the robot's TF tree, and
            # dock_pose_publisher's lookup then fails blaming TF.
            'frame_id': 'docking_camera_link_optical',
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'image_raw.enable_pub_plugins': rgb_only_transports,
        }],
    )

    return LaunchDescription([
        video_device_arg,
        framerate_arg,
        camera_info_url_arg,
        docking_camera,
    ])
