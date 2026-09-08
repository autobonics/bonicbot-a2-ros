#!/usr/bin/env python3
"""
Object follower node — follows any YOLO-detected object class.

ROS parameters (set via --ros-args -p name:=value):
  target_class    string  default: 'person'  — YOLO class name to follow
  target_width_m  float   default: 0.45      — real-world width of target (metres)
                                               used for distance estimation via bbox width

Topics:
  Sub: /vision/yolo_detections  std_msgs/String     JSON list of detections
  Sub: /face_camera/camera_info      sensor_msgs/CameraInfo
  Pub: /cmd_vel                 geometry_msgs/Twist
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import json
import math
import time


# ── Tunable parameters ────────────────────────────────────────────────────────
TARGET_DIST      = 1.0    # metres — desired follow distance
STOP_DIST        = 0.4    # metres — stop if closer than this
LINEAR_DEADBAND  = 0.05   # metres — tight control for smoothness
ANGULAR_DEADBAND = math.radians(2.0)   # 2 degree — tight control
MAX_LINEAR       = 0.5    # m/s (increased for more "alive" feel)
MAX_ANGULAR      = 0.8    # rad/s (increased for sharper turns)
KP_LINEAR        = 0.6    # Proportional gain for distance
KD_LINEAR        = 0.2    # Derivative gain (Damping) to slow down smoothly
KP_ANGULAR       = 1.5    # Proportional gain for steering
LOST_TIMEOUT     = 1.5    # seconds before stopping when target disappears
EMA_ALPHA        = 0.2    # Lower = smoother/slower response, Higher = faster/noisier


class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')

        self.declare_parameter('target_class',   'person')
        self.declare_parameter('target_width_m', 0.45)

        self.target_class   = self.get_parameter('target_class').get_parameter_value().string_value
        self.target_width_m = self.get_parameter('target_width_m').get_parameter_value().double_value

        self._smooth_distance = None
        self._smooth_angle    = None
        self._last_error      = 0.0
        self._last_seen     = None
        self._fx            = None
        self._img_w         = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String,     '/vision/yolo_detections', self._detection_cb, 10)
        self.create_subscription(CameraInfo, '/face_camera/camera_info',     self._camera_info_cb, 10)

        # 20 Hz control loop — smoother and more responsive
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Object follower: Smooth Pursuit Engine Initialized.')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._img_w = msg.width
            self._fx = msg.k[0] if msg.k[0] > 10.0 else msg.width * 1.1

    def _detection_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not detections or self._fx is None:
            return

        img_w = self._img_w or 640
        fx    = self._fx

        targets = [d for d in detections if d['class'] == self.target_class]
        nearest = None
        for d in targets:
            bw_px = d['bbox'][2] * img_w
            if bw_px < 1: continue
            dist      = self.target_width_m * fx / bw_px
            cx_offset = (d['bbox'][0] - 0.5) * img_w
            angle     = math.atan2(cx_offset, fx)
            if nearest is None or dist < nearest[0]:
                nearest = (dist, angle)

        if nearest:
            # Apply Exponential Moving Average (EMA) for butter-smooth data
            if self._smooth_distance is None:
                self._smooth_distance = nearest[0]
                self._smooth_angle    = nearest[1]
            else:
                self._smooth_distance = (EMA_ALPHA * nearest[0]) + ((1 - EMA_ALPHA) * self._smooth_distance)
                self._smooth_angle    = (EMA_ALPHA * nearest[1]) + ((1 - EMA_ALPHA) * self._smooth_angle)
            
            self._last_seen = self.get_clock().now()

    def _control_loop(self):
        """20 Hz control loop using Smooth Pursuit PD logic."""
        twist = Twist()

        if self._last_seen is None:
            return

        dt = (self.get_clock().now() - self._last_seen).nanoseconds / 1e9
        if dt > LOST_TIMEOUT:
            self.cmd_pub.publish(twist) # STOP
            self._smooth_distance = self._smooth_angle = None
            self._last_seen = None
            return

        # PD Controller for distance
        dist_error = self._smooth_distance - TARGET_DIST
        
        # Calculate Derivative (D-term) for damping oscillations
        d_error = (dist_error - self._last_error) / 0.05
        self._last_error = dist_error

        # 1. Angular Control (Steer toward target)
        if abs(self._smooth_angle) > ANGULAR_DEADBAND:
            twist.angular.z = max(-MAX_ANGULAR, min(-KP_ANGULAR * self._smooth_angle, MAX_ANGULAR))

        # 2. Linear Control (Chase/Flee)
        if self._smooth_distance < STOP_DIST:
            # Safety stop
            twist.linear.x = 0.0
        elif abs(dist_error) > LINEAR_DEADBAND:
            # P + D control
            linear_v = (KP_LINEAR * dist_error) + (KD_LINEAR * d_error)
            
            # Dynamic Speed Limit: Scale speed based on steering severity
            # (Slow down slightly while making sharp turns for stability)
            speed_factor = max(0.4, 1.0 - abs(twist.angular.z) / MAX_ANGULAR)
            twist.linear.x = max(-MAX_LINEAR * 0.5, min(linear_v * speed_factor, MAX_LINEAR))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
