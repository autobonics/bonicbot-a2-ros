#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints, DriveOnHeading, Spin
from nav_msgs.msg import Odometry, OccupancyGrid
from action_msgs.msg import GoalStatus
import subprocess
import signal
import os
import math
import json
import threading
import time
import tf2_ros

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # Get use_sim_time parameter (automatically declared by ROS2)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        # State variables
        self.mapping_active = False
        self.navigation_active = False
        self.camera_active = False
        self.explore_active = False
        self.vision_active = False
        self.precise_move_active = False
        self.follow_object_active = False
        self.agent_active = False
        self.slam_process = None
        self.nav2_process = None
        self.camera_process = None
        self.explore_process = None
        self.vision_process = None
        self.precise_move_process = None
        self.follow_object_process = None
        self.agent_process = None

        # Precise movement queue
        self.precise_move_queue = []
        self.precise_move_lock = threading.Lock()

        # Per-detector enabled flags (only meaningful when vision_active)
        self.yolo_enabled      = False
        self.pose_enabled      = False
        self.face_enabled      = False
        self.gesture_enabled   = False
        self.aruco_enabled     = False
        
        # Navigation state
        self.nav_status = 'idle'  # idle, navigating, goal_reached, goal_failed, cancelled
        self.current_goal = None
        self.current_pose = None
        self.distance_to_goal = 0.0
        self._nav_preempting = False  # True when new goal sent before old one finished

        # Patrol state
        self.patrol_active       = False
        self.patrol_status       = 'idle'
        self.patrol_goal_handle  = None
        self.waypoint_client     = None
        self.waypoint_goal_handle = None
        self.spin_client         = None
        self.drive_on_heading_client = None
        self.nav_goal_handle     = None # Shared for all Nav2 actions

        # Precise movement (Internal Engine)
        self.precise_move_queue = []
        self.precise_move_active = False
        self.precise_move_lock = threading.RLock()
        self.precise_move_start_pose = None
        self.precise_move_current_pose = None
        self.precise_move_cfg = None

        self._prec_odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self._prec_odom_callback, 
            10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Named locations
        self.locations_file = os.path.expanduser('~/maps/my_map_locations.json')
        self.session_locations = {}   # in-memory; reset on each new mapping session

        # Cache the latest /map message so we can write it directly (avoids map_saver QoS issues)
        self.latest_map: OccupancyGrid = None
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)

        # TF listener for map-frame pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Action client for navigation
        self.nav_to_pose_client = None
        self.nav_goal_handle = None
        
        # Vision control topic — transient-local so vision node gets current state on startup
        vision_ctrl_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.vision_control_pub = self.create_publisher(String, '/vision/control', vision_ctrl_qos)

        # Publishers for Flutter app
        self.state_pub = self.create_publisher(String, '/robot/state', 10)
        self.mapping_status_pub = self.create_publisher(Bool, '/robot/mapping_active', 10)
        self.nav_status_pub = self.create_publisher(Bool, '/robot/navigation_active', 10)
        self.camera_status_pub = self.create_publisher(Bool, '/robot/camera_active', 10)
        self.explore_status_pub = self.create_publisher(Bool, '/robot/explore_active', 10)
        self.vision_status_pub     = self.create_publisher(Bool, '/robot/vision_active',      10)
        self.yolo_enabled_pub      = self.create_publisher(Bool, '/robot/yolo_enabled',       10)
        self.pose_enabled_pub      = self.create_publisher(Bool, '/robot/pose_enabled',       10)
        self.face_enabled_pub      = self.create_publisher(Bool, '/robot/face_enabled',       10)
        self.gesture_enabled_pub   = self.create_publisher(Bool, '/robot/gesture_enabled',    10)
        self.aruco_enabled_pub     = self.create_publisher(Bool, '/robot/aruco_enabled',      10)
        self.precise_move_active_pub    = self.create_publisher(Bool, '/robot/precise_move_active',    10)
        self.follow_object_active_pub   = self.create_publisher(Bool, '/robot/follow_object_active',   10)
        self.patrol_active_pub          = self.create_publisher(Bool,   '/robot/patrol_active',           10)
        self.patrol_status_pub          = self.create_publisher(String, '/robot/patrol_status',           10)
        self.agent_active_pub           = self.create_publisher(Bool,   '/robot/agent_active',            10)
        
        # Navigation status publishers
        self.nav_state_pub = self.create_publisher(String, '/robot/nav_status', 10)
        self.distance_to_goal_pub = self.create_publisher(Float32, '/robot/distance_to_goal', 10)
        self.current_goal_pub = self.create_publisher(PoseStamped, '/robot/current_goal', 10)
        
        # Subscribe to odometry for current position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10
        )
        
        # Services for Flutter app
        self.start_mapping_srv = self.create_service(
            Trigger, '/robot/start_mapping', self.start_mapping_callback)
        self.stop_mapping_srv = self.create_service(
            Trigger, '/robot/stop_mapping', self.stop_mapping_callback)
        self.start_navigation_srv = self.create_service(
            Trigger, '/robot/start_navigation', self.start_navigation_callback)
        self.stop_navigation_srv = self.create_service(
            Trigger, '/robot/stop_navigation', self.stop_navigation_callback)
        self.save_map_srv = self.create_service(
            Trigger, '/robot/save_map', self.save_map_callback)
        self.cancel_nav_srv = self.create_service(
            Trigger, '/robot/cancel_navigation', self.cancel_navigation_callback)
        self.start_camera_srv = self.create_service(
            Trigger, '/robot/start_camera', self.start_camera_callback)
        self.stop_camera_srv = self.create_service(
            Trigger, '/robot/stop_camera', self.stop_camera_callback)
        self.start_explore_srv = self.create_service(
            Trigger, '/robot/start_explore', self.start_explore_callback)
        self.stop_explore_srv = self.create_service(
            Trigger, '/robot/stop_explore', self.stop_explore_callback)

        # Vision pipeline services
        self.create_service(Trigger, '/robot/start_vision',         self.start_vision_callback)
        self.create_service(Trigger, '/robot/stop_vision',          self.stop_vision_callback)
        self.create_service(Trigger, '/robot/enable_yolo',          self.enable_yolo_callback)
        self.create_service(Trigger, '/robot/disable_yolo',         self.disable_yolo_callback)
        self.create_service(Trigger, '/robot/enable_pose',          self.enable_pose_callback)
        self.create_service(Trigger, '/robot/disable_pose',         self.disable_pose_callback)
        self.create_service(Trigger, '/robot/enable_face',          self.enable_face_callback)
        self.create_service(Trigger, '/robot/disable_face',         self.disable_face_callback)
        self.create_service(Trigger, '/robot/enable_gesture',       self.enable_gesture_callback)
        self.create_service(Trigger, '/robot/disable_gesture',      self.disable_gesture_callback)
        self.create_service(Trigger, '/robot/enable_aruco',         self.enable_aruco_callback)
        self.create_service(Trigger, '/robot/disable_aruco',        self.disable_aruco_callback)

        # Precise motion — publish JSON to trigger: {"mode":"move","distance":0.5,"speed":0.15}
        self.create_subscription(String, '/robot/precise_move', self.precise_move_callback, 10)
        self.create_service(Trigger, '/robot/stop_precise_move',   self.stop_precise_move_callback)
        self.create_service(Trigger, '/robot/stop_follow_object',  self.stop_follow_object_callback)
        # publish class name (e.g. "person", "chair") to /robot/follow_object to start following
        self.create_subscription(String, '/robot/follow_object', self.follow_object_callback, 10)

        # Location subscriptions (publish name to topic to trigger)
        self.create_subscription(String, '/robot/save_location',      self.save_location_callback,   10)
        self.create_subscription(String, '/robot/goto_location',      self.goto_location_callback,   10)
        self.create_subscription(String, '/robot/delete_location',    self.delete_location_callback, 10)
        # Patrol: publish JSON array of location names e.g. '["kitchen","hallway","bedroom"]'
        self.create_subscription(String, '/robot/patrol_locations',   self.patrol_locations_callback, 10)
        self.create_service(Trigger,     '/robot/stop_patrol',        self.stop_patrol_callback)

        # AI Agent
        self.create_service(Trigger, '/robot/start_agent', self.start_agent_callback)
        self.create_service(Trigger, '/robot/stop_agent',  self.stop_agent_callback)

        # Publish list of saved locations (JSON string)
        self.locations_list_pub = self.create_publisher(String, '/robot/locations_list', 10)

        # Publish whether a saved map file exists on disk
        self.map_available_pub = self.create_publisher(Bool, '/robot/map_available', 10)

        # Subscribe to navigation goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Timer to publish status
        self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info(f'Robot Manager Node Started (use_sim_time: {self.use_sim_time})')
        
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
        # distance_to_goal is updated via nav_feedback_callback (Nav2 path distance)
    
    def goal_callback(self, msg):
        """Handle new navigation goal"""
        if not self.navigation_active:
            self.get_logger().warn('Received goal but navigation is not active')
            return
        
        self.current_goal = msg
        self.nav_status = 'navigating'
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Cancel any active goal before sending the new one
        if self.nav_goal_handle is not None:
            self._nav_preempting = True
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

        # Send goal to Nav2
        if self.nav_to_pose_client is None:
            self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.nav_status = 'goal_failed'
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        self.nav_goal_handle = future.result()
        
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.nav_status = 'goal_failed'
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get result
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.distance_to_goal = feedback.distance_remaining
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        # If we intentionally sent a new goal, this result is from the old aborted goal — ignore it
        if self._nav_preempting:
            self._nav_preempting = False
            return

        # future.result() is a GetResult.Response object
        result_response = future.result()
        status = result_response.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_status = 'goal_reached'
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.nav_status = 'cancelled'
            self.get_logger().info('Navigation cancelled')
        else:
            # In simulation, we sometimes get 'Aborted' even if we are at the goal due to jitter
            if self.use_sim_time and self.distance_to_goal < 0.15:
                self.nav_status = 'goal_reached'
                self.get_logger().info('Goal reached (resolved from aborted state at low distance)')
            else:
                self.nav_status = 'goal_failed'
                self.get_logger().error(f'Navigation finished with status: {status}')
        
        self.current_goal = None
        self.distance_to_goal = 0.0
        self.nav_goal_handle = None

        # IF this was part of a precise move queue, trigger the next one
        if self.precise_move_active:
            self._trigger_next_precise_move()
        
    def cancel_navigation_callback(self, request, response):
        """Cancel current navigation goal"""
        if self.nav_goal_handle is None:
            response.success = False
            response.message = 'No active navigation goal to cancel'
            return response
        
        try:
            # Cancel the goal
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            self.nav_status = 'cancelled'
            self.current_goal = None
            self.distance_to_goal = 0.0
            
            response.success = True
            response.message = 'Navigation cancelled successfully'
            self.get_logger().info('Navigation cancelled by user')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to cancel navigation: {str(e)}'
            self.get_logger().error(f'Failed to cancel: {str(e)}')
        
        return response
        
    def publish_status(self):
        """Publish current robot state"""
        # System state
        state_msg = String()
        if self.explore_active:
            state_msg.data = 'exploring'
        elif self.mapping_active and self.navigation_active:
            state_msg.data = 'mapping_and_navigating'
        elif self.mapping_active:
            state_msg.data = 'mapping'
        elif self.navigation_active:
            if self.nav_status == 'goal_reached':
                state_msg.data = 'goal_reached'
            elif self.nav_status == 'navigating':
                state_msg.data = 'navigating'
            else:
                state_msg.data = 'navigation_ready'
        else:
            state_msg.data = 'idle'
        self.state_pub.publish(state_msg)
        
        # Individual statuses
        mapping_msg = Bool()
        mapping_msg.data = self.mapping_active
        self.mapping_status_pub.publish(mapping_msg)
        
        nav_msg = Bool()
        nav_msg.data = self.navigation_active
        self.nav_status_pub.publish(nav_msg)
        
        camera_msg = Bool()
        camera_msg.data = self.camera_active
        self.camera_status_pub.publish(camera_msg)

        explore_msg = Bool()
        explore_msg.data = self.explore_active
        self.explore_status_pub.publish(explore_msg)

        vision_msg = Bool()
        vision_msg.data = self.vision_active
        self.vision_status_pub.publish(vision_msg)

        # Auto-detect precise_move process exit
        if self.precise_move_active and self.precise_move_process is not None:
            if self.precise_move_process.poll() is not None:
                self.precise_move_process = None
                self.precise_move_active = False
                self.get_logger().info('Precise move completed')

        precise_move_msg = Bool()
        precise_move_msg.data = self.precise_move_active
        self.precise_move_active_pub.publish(precise_move_msg)

        # Auto-detect follow_object process exit
        if self.follow_object_active and self.follow_object_process is not None:
            if self.follow_object_process.poll() is not None:
                self.follow_object_process = None
                self.follow_object_active = False
                self.get_logger().info('Person follower stopped')

        follow_msg = Bool()
        follow_msg.data = self.follow_object_active
        self.follow_object_active_pub.publish(follow_msg)

        patrol_msg = Bool()
        patrol_msg.data = self.patrol_active
        self.patrol_active_pub.publish(patrol_msg)

        patrol_status_msg = String()
        patrol_status_msg.data = self.patrol_status
        self.patrol_status_pub.publish(patrol_status_msg)

        # Auto-detect agent process exit
        if self.agent_active and self.agent_process is not None:
            if self.agent_process.poll() is not None:
                self.agent_process = None
                self.agent_active = False
                self.get_logger().info('AI agent stopped')

        agent_msg = Bool()
        agent_msg.data = self.agent_active
        self.agent_active_pub.publish(agent_msg)

        for pub, val in [
            (self.yolo_enabled_pub,     self.yolo_enabled),
            (self.pose_enabled_pub,     self.pose_enabled),
            (self.face_enabled_pub,     self.face_enabled),
            (self.gesture_enabled_pub,  self.gesture_enabled),
            (self.aruco_enabled_pub,    self.aruco_enabled),
        ]:
            m = Bool()
            m.data = val
            pub.publish(m)

        # Navigation status
        nav_state_msg = String()
        nav_state_msg.data = self.nav_status
        self.nav_state_pub.publish(nav_state_msg)
        
        # Distance to goal
        distance_msg = Float32()
        distance_msg.data = self.distance_to_goal
        self.distance_to_goal_pub.publish(distance_msg)
        
        # Current goal
        if self.current_goal:
            self.current_goal_pub.publish(self.current_goal)

        # Saved locations list
        locations_msg = String()
        locations_msg.data = json.dumps(self._load_locations())
        self.locations_list_pub.publish(locations_msg)

        # Map available on disk
        map_avail_msg = Bool()
        map_avail_msg.data = os.path.exists(os.path.expanduser('~/maps/my_map.yaml'))
        self.map_available_pub.publish(map_avail_msg)
    
    def start_mapping_callback(self, request, response):
        """Start SLAM Toolbox for mapping"""
        if self.mapping_active:
            response.success = False
            response.message = 'Mapping already active'
            return response
        
        try:
            from ament_index_python.packages import get_package_share_directory
            params_file = os.path.join(
                get_package_share_directory('my_bot'),
                'config', 'mapper_params_online_async.yaml'
            )
            
            self.slam_process = subprocess.Popen([
                'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                f'params_file:={params_file}',
                f'use_sim_time:={str(self.use_sim_time).lower()}'
            ])
            
            self.mapping_active = True
            self.session_locations = {}   # clear in-memory locations for new map session
            response.success = True
            response.message = 'Mapping started successfully'
            self.get_logger().info('SLAM Toolbox started')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start mapping: {str(e)}'
            self.get_logger().error(f'Failed to start SLAM: {str(e)}')
        
        return response
    
    def stop_mapping_callback(self, request, response):
        """Stop SLAM Toolbox"""
        if not self.mapping_active:
            response.success = False
            response.message = 'Mapping not active'
            return response
        
        try:
            if self.slam_process:
                self.slam_process.send_signal(signal.SIGINT)
                self.slam_process.wait(timeout=5)
                self.slam_process = None
            
            self.mapping_active = False
            response.success = True
            response.message = 'Mapping stopped successfully'
            self.get_logger().info('SLAM Toolbox stopped')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop mapping: {str(e)}'
            self.get_logger().error(f'Failed to stop SLAM: {str(e)}')
        
        return response
    
    def start_navigation_callback(self, request, response):
        """Start Nav2"""
        if self.navigation_active:
            response.success = False
            response.message = 'Navigation already active'
            return response
        
        try:
            from ament_index_python.packages import get_package_share_directory
            
            params_filename = 'nav2_params_sim.yaml' if self.use_sim_time else 'nav2_params.yaml'
            params_file = os.path.join(
                get_package_share_directory('my_bot'),
                'config', params_filename
            )
            
            # If mapping is active, use online SLAM navigation
            if self.mapping_active:
                self.nav2_process = subprocess.Popen([
                    'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    f'params_file:={params_file}',
                    f'use_sim_time:={str(self.use_sim_time).lower()}'
                ])
            else:
                # Nav2 with saved map
                map_file = os.path.expanduser('~/maps/my_map.yaml')
                if not os.path.exists(map_file):
                    response.success = False
                    response.message = 'No saved map found. Please map first or start mapping.'
                    return response
                
                self.nav2_process = subprocess.Popen([
                    'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                    f'map:={map_file}',
                    f'params_file:={params_file}',
                    f'use_sim_time:={str(self.use_sim_time).lower()}'
                ])
            
            # Wait for Nav2 action server to be ready (costmaps loaded)
            self.get_logger().info('Waiting for Nav2 to become ready...')
            if self.nav_to_pose_client is None:
                self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            if self.nav_to_pose_client.wait_for_server(timeout_sec=60.0):
                self.navigation_active = True
                self.nav_status = 'idle'
                response.success = True
                response.message = 'Navigation started successfully'
                self.get_logger().info('Nav2 ready')
            else:
                self.nav2_process.send_signal(signal.SIGINT)
                self.nav2_process = None
                response.success = False
                response.message = 'Nav2 failed to become ready within 60s'
                self.get_logger().error('Nav2 did not become ready in time')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start navigation: {str(e)}'
            self.get_logger().error(f'Failed to start Nav2: {str(e)}')
        
        return response
    
    def stop_navigation_callback(self, request, response):
        """Stop Nav2"""
        if not self.navigation_active:
            response.success = False
            response.message = 'Navigation not active'
            return response
        
        try:
            # Cancel any active goal first
            if hasattr(self, 'nav_goal_handle') and self.nav_goal_handle:
                self.get_logger().info('Cancelling active goal for shutdown...')
                self.nav_goal_handle.cancel_goal_async()
            
            if self.nav2_process:
                self.get_logger().info('Stopping Nav2 process...')
                self.nav2_process.send_signal(signal.SIGINT)
                try:
                    self.nav2_process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn('Nav2 did not stop gracefully, forcing termination...')
                    self.nav2_process.terminate()
                    self.nav2_process.wait(timeout=5)
                self.nav2_process = None
            
            self.navigation_active = False
            self.nav_status = 'idle'
            self.current_goal = None
            self.distance_to_goal = 0.0
            self.nav_goal_handle = None
            response.success = True
            response.message = 'Navigation stopped successfully'
            self.get_logger().info('Nav2 stopped')
            
        except Exception as e:
            # If we timed out but still cleaned up, don't crash
            self.nav2_process = None
            self.navigation_active = False
            response.success = False
            response.message = f'Failed to stop navigation cleanly (timed out), system was forced shut.'
            self.get_logger().error(f'Failed to stop Nav2 cleanly: {str(e)}')
        
        return response
    
    def save_map_callback(self, request, response):
        """Save current map via SLAM Toolbox service (reliable when Nav2 also running)"""
        if not self.mapping_active:
            response.success = False
            response.message = 'Mapping not active. Start mapping first.'
            return response

        success, message = self._save_map_internal()
        response.success = success
        response.message = message
        return response
    
    def start_camera_callback(self, request, response):
        """Start camera node"""
        if self.camera_active:
            response.success = False
            response.message = 'Camera already active'
            return response
        
        try:
            # In simulation, camera is always active via camera.xacro in Gazebo
            if self.use_sim_time:
                self.camera_active = True
                response.success = True
                response.message = 'Camera activated (Gazebo camera via camera.xacro)'
                self.get_logger().info('Camera activated in simulation mode (Gazebo camera)')
            else:
                # Real hardware: launch v4l2_camera node
                # Kill any orphaned camera process from a previous session
                # (e.g. Ctrl+C on the launch file without clean shutdown)
                subprocess.run(
                    ['pkill', '-SIGTERM', '-f', 'v4l2_camera_node'],
                    capture_output=True
                )
                time.sleep(0.8)  # wait for device to be released

                from ament_index_python.packages import get_package_share_directory
                camera_launch = os.path.join(
                    get_package_share_directory('my_bot'),
                    'launch', 'camera.launch.py'
                )

                self.camera_process = subprocess.Popen([
                    'ros2', 'launch', camera_launch,
                    'use_sim_time:=false'
                ])
                
                self.camera_active = True
                response.success = True
                response.message = 'Camera started successfully (v4l2_camera hardware)'
                self.get_logger().info('Camera node started (real hardware)')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start camera: {str(e)}'
            self.get_logger().error(f'Failed to start camera: {str(e)}')
        
        return response
    
    def stop_camera_callback(self, request, response):
        """Stop camera node"""
        if not self.camera_active:
            response.success = False
            response.message = 'Camera not active'
            return response
        
        try:
            # Auto-stop vision pipeline first (it depends on camera frames)
            if self.vision_active:
                self._stop_vision_internal()
                self.get_logger().info('Vision pipeline auto-stopped (camera stopping)')

            # In simulation, just deactivate flag (can't stop Gazebo camera)
            if self.use_sim_time:
                self.camera_active = False
                response.success = True
                response.message = 'Camera deactivated (Gazebo camera still running)'
                self.get_logger().info('Camera deactivated in simulation mode')
            else:
                # Real hardware: stop v4l2_camera process
                subprocess.run(['pkill', '-SIGINT', '-f', 'v4l2_camera_node'], capture_output=True)
                if self.camera_process:
                    try:
                        self.camera_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.camera_process.kill()
                    self.camera_process = None

                self.camera_active = False
                response.success = True
                response.message = 'Camera stopped successfully (v4l2_camera hardware)'
                self.get_logger().info('Camera node stopped (real hardware)')

        except Exception as e:
            response.success = False
            response.message = f'Failed to stop camera: {str(e)}'
            self.get_logger().error(f'Failed to stop camera: {str(e)}')

        return response

    def _monitor_explore_output(self):
        """Background thread: watch explore_lite output and auto-stop when done"""
        try:
            for line in self.explore_process.stdout:
                if 'Successfully returned to initial pose' in line:
                    self.get_logger().info('Exploration complete — auto-stopping explore_lite')
                    self.explore_process.send_signal(signal.SIGINT)
                    self.explore_process.wait(timeout=5)
                    self.explore_process = None
                    self.explore_active = False
                    break
        except Exception as e:
            self.get_logger().error(f'explore monitor error: {str(e)}')

    def start_explore_callback(self, request, response):
        """Start explore_lite for autonomous mapping"""
        if self.explore_active:
            response.success = False
            response.message = 'Exploration already active'
            return response

        if not self.mapping_active:
            response.success = False
            response.message = 'Start mapping first before exploring'
            return response

        if not self.navigation_active:
            response.success = False
            response.message = 'Start navigation first before exploring'
            return response

        try:
            self.explore_process = subprocess.Popen(
                [
                    'ros2', 'launch', 'explore_lite', 'explore.launch.py',
                    f'use_sim_time:={str(self.use_sim_time).lower()}',
                    'return_to_init:=true'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            self.explore_active = True
            threading.Thread(target=self._monitor_explore_output, daemon=True).start()
            response.success = True
            response.message = 'Autonomous exploration started'
            self.get_logger().info('explore_lite started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start exploration: {str(e)}'
            self.get_logger().error(f'Failed to start explore_lite: {str(e)}')

        return response

    def stop_explore_callback(self, request, response):
        """Stop explore_lite"""
        if not self.explore_active:
            response.success = False
            response.message = 'Exploration not active'
            return response

        try:
            if self.explore_process:
                self.explore_process.send_signal(signal.SIGINT)
                self.explore_process.wait(timeout=5)
                self.explore_process = None
            self.explore_active = False
            response.success = True
            response.message = 'Exploration stopped'
            self.get_logger().info('explore_lite stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop exploration: {str(e)}'
            self.get_logger().error(f'Failed to stop explore_lite: {str(e)}')

        return response

# ── Vision pipeline callbacks ─────────────────────────────────────────────────

    def _publish_vision_control(self):
        """Publish current detector config to /vision/control (transient-local)."""
        msg = String()
        msg.data = json.dumps({
            'yolo':      self.yolo_enabled,
            'pose':      self.pose_enabled,
            'face':      self.face_enabled,
            'gesture':   self.gesture_enabled,
            'aruco':     self.aruco_enabled,
        })
        self.vision_control_pub.publish(msg)

    def start_vision_callback(self, request, response):
        if self.vision_active:
            response.success = False
            response.message = 'Vision pipeline already active'
            return response
        if not self.camera_active:
            response.success = False
            response.message = 'Start camera first before starting vision'
            return response
        try:
            self.vision_process = subprocess.Popen(
                ['ros2', 'run', 'my_bot', 'vision_pipeline.py'])
            self.vision_active = True
            self._publish_vision_control()   # send current (all-off) config on startup
            response.success = True
            response.message = 'Vision pipeline started'
            self.get_logger().info('Vision pipeline started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start vision: {str(e)}'
            self.get_logger().error(f'Failed to start vision: {str(e)}')
        return response

    def _stop_vision_internal(self):
        """Stop vision process and reset all detector flags. Safe to call even if not active."""
        subprocess.run(['pkill', '-SIGINT', '-f', 'vision_pipeline.py'], capture_output=True)
        if self.vision_process:
            try:
                self.vision_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.vision_process.kill()
            self.vision_process = None
        self.vision_active   = False
        self.yolo_enabled    = False
        self.pose_enabled    = False
        self.face_enabled    = False
        self.gesture_enabled = False
        self.aruco_enabled   = False

    def stop_vision_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Vision not active'
            return response
        try:
            self._stop_vision_internal()
            response.success = True
            response.message = 'Vision pipeline stopped'
            self.get_logger().info('Vision pipeline stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop vision: {str(e)}'
            self.get_logger().error(f'Failed to stop vision: {str(e)}')
        return response

    def enable_yolo_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.yolo_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'YOLO enabled'
        return response

    def disable_yolo_callback(self, request, response):
        self.yolo_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'YOLO disabled'
        return response

    def enable_pose_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.pose_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Body pose enabled'
        return response

    def disable_pose_callback(self, request, response):
        self.pose_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Body pose disabled'
        return response

    def enable_face_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.face_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Face detection enabled'
        return response

    def disable_face_callback(self, request, response):
        self.face_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Face detection disabled'
        return response

    def enable_gesture_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.gesture_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'Gesture recognition enabled'
        return response

    def disable_gesture_callback(self, request, response):
        self.gesture_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'Gesture recognition disabled'
        return response

    def enable_aruco_callback(self, request, response):
        if not self.vision_active:
            response.success = False
            response.message = 'Start vision pipeline first'
            return response
        self.aruco_enabled = True
        self._publish_vision_control()
        response.success = True
        response.message = 'ArUco enabled'
        return response

    def disable_aruco_callback(self, request, response):
        self.aruco_enabled = False
        self._publish_vision_control()
        response.success = True
        response.message = 'ArUco disabled'
        return response

# ── Precise motion callbacks ─────────────────────────────────────────────────

    def _prec_odom_callback(self, msg: Odometry):
        """Internal Precise Motion logic matched 1:1 with standalone script."""
        if not self.precise_move_active or not self.precise_move_cfg:
            return

        # NEW: Only skip processing if use_nav2 is enabled AND Nav2 is actually running
        # This provides a safe fallback to internal engine if Nav2 is off.
        if self.precise_move_cfg.get('use_nav2', False) and self.navigation_active:
            return

        # Get current pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        curr = (p.x, p.y, yaw)

        # Latch start pose
        if self.precise_move_start_pose is None:
            self.precise_move_start_pose = curr
            self.get_logger().info('Internal Precise Move: Start pose latched.')
            # Standalone script returns here, so we do too for exact parity
            return

        cfg = self.precise_move_cfg
        mode = cfg.get('mode', 'move')
        speed = float(cfg.get('speed', 0.4 if mode == 'move' else 45.0))
        twist = Twist()
        done = False

        if mode == 'move':
            target = float(cfg.get('distance', 0.0))
            traveled = math.sqrt((curr[0] - self.precise_move_start_pose[0])**2 + 
                                (curr[1] - self.precise_move_start_pose[1])**2)
            if traveled >= abs(target):
                done = True
            else:
                twist.linear.x = speed if target >= 0 else -speed
        else: # rotate
            target = math.radians(float(cfg.get('angle', 0.0)))
            speed_rad = math.radians(speed)
            diff = curr[2] - self.precise_move_start_pose[2]
            # Normalize angle to (-pi, pi]
            while diff >  math.pi: diff -= 2.0 * math.pi
            while diff < -math.pi: diff += 2.0 * math.pi
            
            reached = (target >= 0 and diff >= target) or (target < 0 and diff <= target)
            if reached:
                done = True
            else:
                twist.angular.z = speed_rad if target >= 0 else -speed_rad

        if done:
            self.get_logger().info(f'Internal Precise {mode} complete.')
            self._cmd_vel_pub.publish(Twist()) # Full stop
            self._trigger_next_precise_move()
        else:
            self._cmd_vel_pub.publish(twist)

    def precise_move_callback(self, msg: String):
        """Internal Queueing for precise move."""
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'precise_move: bad JSON: {e}')
            return

        with self.precise_move_lock:
            self.precise_move_queue.append(cfg)
            self.get_logger().info(f'Internal Precise move queued: {cfg}')
            if not self.precise_move_active:
                self._trigger_next_precise_move()

    def _trigger_next_precise_move(self):
        """Start the next internal command in the queue."""
        with self.precise_move_lock:
            if not self.precise_move_queue:
                self.precise_move_active = False
                self.precise_move_cfg = None
                self.precise_move_start_pose = None
                return

            self.precise_move_cfg = self.precise_move_queue.pop(0)
            self.precise_move_start_pose = None # Reset for latching
            self.precise_move_active = True

        if self.precise_move_cfg.get('use_nav2', False):
            # Using Nav2 dedicated actions for Safe Mode
            if not self.navigation_active:
                self.get_logger().warn('Nav2 not active - falling back to internal high-speed engine.')
                return # Return allows internal engine callback to pick it up immediately

            mode = self.precise_move_cfg.get('mode', 'move')
            if mode == 'move':
                dist = float(self.precise_move_cfg.get('distance', 0.0))
                speed = float(self.precise_move_cfg.get('speed', 0.15))
                self._send_drive_goal(dist, speed)
            else: # rotate
                angle = math.radians(float(self.precise_move_cfg.get('angle', 0.0)))
                # speed in spin is radians/sec
                speed = math.radians(float(self.precise_move_cfg.get('speed', 45.0)))
                self._send_spin_goal(angle, speed)

    def _send_drive_goal(self, distance, speed):
        if self.drive_on_heading_client is None:
            self.drive_on_heading_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        
        if not self.drive_on_heading_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DriveOnHeading action server not available')
            self._trigger_next_precise_move()
            return
            
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance)
        goal.speed = float(abs(speed))
        goal.time_allowance.sec = 15
        
        send_goal_future = self.drive_on_heading_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._nav_action_goal_response_cb)

    def _send_spin_goal(self, angle_rad, speed_rad):
        if self.spin_client is None:
            self.spin_client = ActionClient(self, Spin, 'spin')
            
        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Spin action server not available')
            self._trigger_next_precise_move()
            return
            
        goal = Spin.Goal()
        goal.target_yaw = float(angle_rad)
        goal.time_allowance.sec = 10
        
        send_goal_future = self.spin_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._nav_action_goal_response_cb)

    def _nav_action_goal_response_cb(self, future):
        """Generalized Nav2 action goal response (Spin, DriveOnHeading)."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Precise Nav Goal rejected')
            self._trigger_next_precise_move()
            return

        self.nav_goal_handle = goal_handle # Store for cancelling if stop() called
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_action_result_cb)

    def _nav_action_result_cb(self, future):
        """Triggered when DriveOnHeading or Spin is complete."""
        self.nav_goal_handle = None
        self.get_logger().info('Precise Nav Action finished.')
        if self.precise_move_active:
             self._trigger_next_precise_move()

    def stop_precise_move_callback(self, request, response):
        """Stop an in-progress internal or Nav2 precise move."""
        with self.precise_move_lock:
            self.precise_move_queue = []
            self.precise_move_active = False
            self.precise_move_cfg = None
            self.precise_move_start_pose = None

        # Stop internal engine motors
        if hasattr(self, '_cmd_vel_pub'):
            self._cmd_vel_pub.publish(Twist())
        
        # Stop Nav2 engine if it was performing a precise move
        if self.nav_goal_handle is not None:
             self.get_logger().info('Stopping Nav2 precise move...')
             self.nav_goal_handle.cancel_goal_async()
             self.nav_goal_handle = None

        response.success = True
        response.message = 'All precise movements stopped and queue cleared'
        return response

# ── Object follower callbacks ─────────────────────────────────────────────────

    def follow_object_callback(self, msg: String):
        """Publish a YOLO class name to /robot/follow_object to start following it.
        e.g. ros2 topic pub --once /robot/follow_object std_msgs/String "data: 'chair'"
        Stops any active follower first, then starts following the new class.
        """
        target = msg.data.strip()
        if not target:
            return
        if not self.camera_active:
            self.get_logger().error('follow_object: start camera first')
            return
        if not self.vision_active:
            self.get_logger().error('follow_object: start vision pipeline first')
            return
        if not self.yolo_enabled:
            self.get_logger().error('follow_object: enable YOLO first')
            return
        # Stop existing follower if running (including any orphaned instances)
        subprocess.run(['pkill', '-SIGINT', '-f', 'object_follower.py'], capture_output=True)
        if self.follow_object_active:
            if self.follow_object_process:
                try:
                    self.follow_object_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.follow_object_process.kill()
                self.follow_object_process = None
            self.follow_object_active = False
        time.sleep(0.5)  # wait for killed process to fully exit before launching new one
        try:
            self.follow_object_process = subprocess.Popen([
                'ros2', 'run', 'my_bot', 'object_follower.py',
                '--ros-args', '-p', f'target_class:={target}',
            ])
            self.follow_object_active = True
            self.get_logger().info(f'Object follower started (class="{target}")')
        except Exception as e:
            self.get_logger().error(f'Failed to start object follower: {e}')

    def stop_follow_object_callback(self, request, response):
        if not self.follow_object_active:
            response.success = False
            response.message = 'Object follower not active'
            return response
        try:
            if self.follow_object_process:
                self.follow_object_process.send_signal(signal.SIGINT)
                try:
                    self.follow_object_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.follow_object_process.kill()
                self.follow_object_process = None
            # Kill any orphaned instances not tracked by this process
            subprocess.run(['pkill', '-SIGINT', '-f', 'object_follower.py'], capture_output=True)
            self.follow_object_active = False
            response.success = True
            response.message = 'Object follower stopped'
            self.get_logger().info('Object follower stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop: {str(e)}'
        return response

# ── Named location helpers ────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        """Cache the latest map for direct file writing."""
        self.latest_map = msg

    def _get_map_pose(self):
        """Return (x, y, yaw) in map frame, or None if TF unavailable."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        except Exception as e:
            self.get_logger().error(f'TF map→base_link unavailable: {e}')
            return None

    def _save_map_internal(self):
        """Write the cached OccupancyGrid directly to .pgm + .yaml. Returns (success, message)."""
        if self.latest_map is None:
            return False, 'No map received yet — wait for SLAM to build a map'

        try:
            map_dir = os.path.expanduser('~/maps')
            os.makedirs(map_dir, exist_ok=True)
            map_path = os.path.join(map_dir, 'my_map')
            pgm_path  = map_path + '.pgm'
            yaml_path = map_path + '.yaml'

            info = self.latest_map.info
            width, height = info.width, info.height
            data = self.latest_map.data   # flat row-major, row 0 = bottom

            # Build PGM pixel array (P5 binary, 8-bit grey).
            # ROS convention: free=0→255, occupied=100→0, unknown=-1→205
            pixels = bytearray(width * height)
            for i, v in enumerate(data):
                if v == 0:
                    pixels[i] = 254
                elif v == 100:
                    pixels[i] = 0
                else:
                    pixels[i] = 205   # unknown

            # PGM is stored top-row-first; ROS data is bottom-row-first → flip
            rows = [pixels[r * width:(r + 1) * width] for r in range(height)]
            rows.reverse()

            with open(pgm_path, 'wb') as f:
                header = f'P5\n{width} {height}\n255\n'.encode()
                f.write(header)
                for row in rows:
                    f.write(row)

            origin = info.origin
            ox = origin.position.x
            oy = origin.position.y
            q = origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            yaml_content = (
                f'image: my_map.pgm\n'
                f'mode: trinary\n'
                f'resolution: {info.resolution}\n'
                f'origin: [{ox}, {oy}, {yaw}]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n'
            )
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)

            self.get_logger().info(f'Map saved: {pgm_path} ({width}x{height})')
            return True, f'Map saved to {map_path}.yaml'

        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
            return False, f'Failed to save map: {str(e)}'

    def _load_locations(self):
        """Load locations JSON from disk. Returns dict (empty if missing/corrupt)."""
        try:
            if os.path.exists(self.locations_file):
                with open(self.locations_file, 'r') as f:
                    return json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load locations: {e}')
        return {}

    def _write_locations(self, locations: dict):
        """Write locations dict to disk."""
        try:
            os.makedirs(os.path.dirname(self.locations_file), exist_ok=True)
            with open(self.locations_file, 'w') as f:
                json.dump(locations, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to write locations: {e}')

# ── Named location callbacks ──────────────────────────────────────────────────

    def save_location_callback(self, msg: String):
        """Save current map-frame pose or specific coordinates under a name.
        Input can be just a name (captures current pose) or JSON:
        {"name": "living_room", "x": 1.5, "y": -2.0, "yaw": 0.0}
        """
        data = msg.data.strip()
        if not data:
            self.get_logger().warn('save_location: empty input ignored')
            return

        name = ""
        x, y, yaw = 0.0, 0.0, 0.0
        use_current_pose = True

        # Try parsing as JSON first for custom coordinates
        if data.startswith('{'):
            try:
                cfg = json.loads(data)
                name = cfg.get('name', '').strip()
                x = float(cfg.get('x', 0.0))
                y = float(cfg.get('y', 0.0))
                yaw = float(cfg.get('yaw', 0.0))
                use_current_pose = False
            except (json.JSONDecodeError, ValueError, TypeError) as e:
                self.get_logger().error(f'save_location: invalid JSON/data: {e}')
                return
        else:
            name = data

        if not name:
            self.get_logger().warn('save_location: name missing or empty')
            return

        if use_current_pose:
            pose = self._get_map_pose()
            if pose is None:
                return  # error already logged in _get_map_pose
            x, y, yaw = pose

        # Load existing (necessary if not in mapping session or just ensuring persistence)
        locations = self._load_locations()
        locations[name] = {'x': x, 'y': y, 'yaw': yaw}

        if self.mapping_active:
            # Overwrite in-memory session dict (keeps session and file consistent)
            self.session_locations[name] = locations[name]
            self._write_locations(self.session_locations)
            # Auto-save the map so location and map stay in sync
            self._save_map_internal()
        else:
            # Navigation-only mode → load/update existing locations
            self._write_locations(locations)

        self.get_logger().info(
            f'Location "{name}" saved at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')


    def goto_location_callback(self, msg: String):
        """Navigate to a previously saved named location."""
        name = msg.data.strip()
        locations = self._load_locations()

        if name not in locations:
            self.get_logger().error(
                f'Location "{name}" not found. Saved: {list(locations.keys())}')
            return

        if not self.navigation_active:
            self.get_logger().error('goto_location: navigation is not active')
            return

        loc = locations[name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = loc['x']
        goal_pose.pose.position.y = loc['y']
        yaw = loc['yaw']
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Navigating to "{name}" ({loc["x"]:.2f}, {loc["y"]:.2f})')
        self.goal_callback(goal_pose)

    def delete_location_callback(self, msg: String):
        """Delete a named location."""
        name = msg.data.strip()
        locations = self._load_locations()

        if name not in locations:
            self.get_logger().warn(f'delete_location: "{name}" not found')
            return

        del locations[name]
        self.session_locations.pop(name, None)
        self._write_locations(locations)
        self.get_logger().info(f'Location "{name}" deleted')

# ── Patrol callbacks ──────────────────────────────────────────────────────────

    def patrol_locations_callback(self, msg: String):
        """Publish a JSON array of location names to patrol them in order.
        e.g. ros2 topic pub --once /robot/patrol_locations std_msgs/String "data: '[\"kitchen\",\"hallway\"]'"
        """
        if not self.navigation_active:
            self.get_logger().error('patrol: start navigation first')
            return

        try:
            names = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'patrol: invalid JSON: {e}')
            return

        if not isinstance(names, list) or len(names) == 0:
            self.get_logger().error('patrol: expected a non-empty JSON array of location names')
            return

        locations = self._load_locations()
        waypoints = []
        for name in names:
            if name not in locations:
                self.get_logger().error(f'patrol: location "{name}" not found — aborting')
                return
            loc = locations[name]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = loc['x']
            pose.pose.position.y = loc['y']
            yaw = loc['yaw']
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            waypoints.append(pose)

        # Stop any active patrol before starting a new one
        if self.patrol_active and self.patrol_goal_handle is not None:
            self.patrol_goal_handle.cancel_goal_async()
            self.patrol_goal_handle = None
            self.patrol_active = False

        if self.waypoint_client is None:
            self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('patrol: /follow_waypoints action server not available')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints

        future = self.waypoint_client.send_goal_async(goal)
        future.add_done_callback(self._patrol_goal_response_cb)
        self.patrol_active = True
        self.patrol_status = 'patrolling'
        self.get_logger().info(f'Patrol started: {names}')

    def _patrol_goal_response_cb(self, future):
        self.patrol_goal_handle = future.result()
        if not self.patrol_goal_handle.accepted:
            self.get_logger().error('patrol: goal rejected')
            self.patrol_active = False
            self.patrol_status = 'failed'
            self.patrol_goal_handle = None
            return
        result_future = self.patrol_goal_handle.get_result_async()
        result_future.add_done_callback(self._patrol_result_cb)

    def _patrol_result_cb(self, future):
        self.patrol_active = False
        self.patrol_goal_handle = None
        missed = future.result().result.missed_waypoints
        if missed:
            self.patrol_status = 'failed'
        else:
            self.patrol_status = 'completed'
        if missed:
            self.get_logger().warn(f'Patrol finished — missed {len(missed)} waypoint(s)')
        else:
            self.get_logger().info('Patrol completed successfully')

    def stop_patrol_callback(self, request, response):
        if not self.patrol_active:
            response.success = False
            response.message = 'No active patrol'
            return response
        try:
            if self.patrol_goal_handle:
                self.patrol_goal_handle.cancel_goal_async()
                self.patrol_goal_handle = None
            self.patrol_active = False
            self.patrol_status = 'stopped'
            response.success = True
            response.message = 'Patrol stopped'
            self.get_logger().info('Patrol stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop patrol: {str(e)}'
        return response

# ── AI Agent callbacks ────────────────────────────────────────────────────────

    def start_agent_callback(self, request, response):
        if self.agent_active:
            response.success = False
            response.message = 'AI agent already active'
            return response
        try:
            env = os.environ.copy()
            self.agent_process = subprocess.Popen(
                ['ros2', 'run', 'my_bot', 'robot_agent.py',
                 '--ros-args', '-p', f'use_sim_time:={str(self.use_sim_time).lower()}'],
                env=env,
            )
            self.agent_active = True
            response.success = True
            response.message = 'AI agent started'
            self.get_logger().info('AI agent started')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start AI agent: {str(e)}'
            self.get_logger().error(f'Failed to start AI agent: {e}')
        return response

    def stop_agent_callback(self, request, response):
        if not self.agent_active:
            response.success = False
            response.message = 'AI agent not active'
            return response
        try:
            if self.agent_process:
                self.agent_process.send_signal(signal.SIGINT)
                try:
                    self.agent_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.agent_process.kill()
                self.agent_process = None
            self.agent_active = False
            response.success = True
            response.message = 'AI agent stopped'
            self.get_logger().info('AI agent stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop AI agent: {str(e)}'
            self.get_logger().error(f'Failed to stop AI agent: {e}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node.slam_process:
            node.slam_process.send_signal(signal.SIGINT)
        if node.nav2_process:
            node.nav2_process.send_signal(signal.SIGINT)
        if node.camera_process:
            node.camera_process.send_signal(signal.SIGINT)
        if node.explore_process:
            node.explore_process.send_signal(signal.SIGINT)
        if node.vision_process:
            node.vision_process.send_signal(signal.SIGINT)
        if node.precise_move_process:
            node.precise_move_process.send_signal(signal.SIGINT)
        if node.follow_object_process:
            node.follow_object_process.send_signal(signal.SIGINT)
        if node.agent_process:
            node.agent_process.send_signal(signal.SIGINT)
        node.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()