#!/usr/bin/env python3
"""TF -> PoseStamped bridge for Nav2's opennav_docking SimpleChargingDock plugin.

Same parameters (use_first_detection, dock_tag_family, dock_tag_id) and same
output topic (detected_dock_pose) as the reference dock_pose_publisher
(NVIDIA-ISAAC-ROS/nova_carter, nova_carter_docking/src/dock_pose_publisher.cpp
— the implementation Nav2's own docking tutorial points to).

That reference reads pose directly out of each detection
(isaac_ros_apriltag_interfaces/AprilTagDetectionArray embeds a pose per
tag). apriltag_ros (github.com/christianrauch/apriltag_ros, used here — see
docs/bonicbot_a2_docking.md) does not: apriltag_msgs/AprilTagDetection has
no pose field, only id/family/corners/homography — pose is broadcast on
/tf instead, as child frame `tag<family>:<id>`. So this node uses the
detections array only to know which tag was seen and when, then looks up
that frame's pose via tf2 rather than reading it out of the message.

COPIED VERBATIM from bonicbot_m1_nav, deliberately. This node is
series-agnostic — it touches no hardware, no A2- or M1-specific topic, and no
parameter that differs between them. Only the CMakeLists install line and the
launch file's `package=` differ. If it needs fixing, fix it in both.
See docs/bonicbot_a2_docking.md §3.

ROS parameters:
  use_first_detection  bool    default: True       — dock on whichever tag is seen first
  dock_tag_family       string  default: 'tag36h11' — only used when use_first_detection is False
  dock_tag_id           int     default: 0          — only used when use_first_detection is False
  fixed_frame           string  default: 'odom'     — frame detected_dock_pose is published in
                                                       (matches docking_server's fixed_frame)

Topics:
  Sub: detections           apriltag_msgs/AprilTagDetectionArray
  Pub: detected_dock_pose   geometry_msgs/PoseStamped
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class DockPosePublisher(Node):

    def __init__(self):
        super().__init__('dock_pose_publisher')

        self.declare_parameter('use_first_detection', True)
        self.declare_parameter('dock_tag_family', 'tag36h11')
        self.declare_parameter('dock_tag_id', 0)
        self.declare_parameter('fixed_frame', 'odom')

        self.use_first_detection = self.get_parameter('use_first_detection').value
        self.dock_tag_family = self.get_parameter('dock_tag_family').value
        self.dock_tag_id = self.get_parameter('dock_tag_id').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.subscription_ = self.create_subscription(
            AprilTagDetectionArray, 'detections', self.detection_callback, 10)

    def detection_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        if self.use_first_detection:
            detection = msg.detections[0]
        else:
            # detection.family is un-prefixed ('36h11'), dock_tag_family is
            # the tf-frame form ('tag36h11') — strip the prefix to compare.
            wanted_family = self.dock_tag_family[3:] if self.dock_tag_family.startswith('tag') else self.dock_tag_family
            detection = next(
                (d for d in msg.detections
                 if d.family == wanted_family and d.id == self.dock_tag_id),
                None)
            if detection is None:
                return

        tag_frame = f'tag{detection.family}:{detection.id}'
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, tag_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'No transform {self.fixed_frame} -> {tag_frame}: {e}',
                throttle_duration_sec=2.0)
            return

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.publisher_.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
