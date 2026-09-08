#!/usr/bin/env python3
"""
robot_agent_node — AI execution brain for the ROS2 robot.

Receives high-level goals on /agent/goal, runs a Gemini-powered ReAct loop,
and publishes status/speech throughout.  Does NOT handle TTS, voice input,
or the outer conversation — those are separate nodes.
"""

import json
import math
import os
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
from google import genai
from google.genai import types as gtypes

# ─────────────────────────────────────────────────────────────────────────────
# System prompt (passed verbatim per spec)
# ─────────────────────────────────────────────────────────────────────────────
SYSTEM_PROMPT = (
    "You are the execution brain of a semi-humanoid ROS2 robot.\n"
    "You can see through the robot's front-facing camera. An image update is provided in most turns.\n"
    "Call one tool per turn. After each result decide the next action.\n\n"
    "Physical limits:\n"
    "- Differential drive: forward/back and rotation only. You CAN combine these to trace shapes, letters, or complex paths (e.g., drawing an 'A' on the floor).\n"
    "- Two arms: Use `set_arm(side, distance_m, height_m)`. Height 0.0 is shoulder level. Max height ~0.25m.\n"
    "- To 'point' authoritatively at a person, use a height between 0.15m and 0.20m.\n"
    "- Outside arm reach → reposition robot body first.\n"
    "- Gripper: open/close only.\n"
    "- Head: yaw (left/right) only.\n\n"
    "Navigation:\n" 
    "- No map loaded → no Nav2, no named locations. Can still move/rotate/use arms/camera.\n"
    "- Object < 50 cm → use precise motion, not Nav2.\n"
    "- Object > 50 cm and map loaded → use Nav2.\n\n"
    "Always check robot_state before acting. Never send arm to unreachable position.\n"
    "Visual Targeting Rule: To center an object on the LEFT of your camera view, you must rotate LEFT (Positive angle). "
    "To center an object on the RIGHT, rotate RIGHT (Negative angle). DO NOT MIRROR THIS.\n"
    "Efficiency Rule: Do not use tiny increments. Estimate the required angle.\n"
    "Anti-Oscillation Rule: For visual centering (rotating to center an object), be conservative. Use smaller steps (5-10 degrees) or use only 50% of the estimated angle to avoid overshooting.\n"
    "SAFETY: `move_forward` and `rotate` are BLIND moves (no obstacle avoidance). Only use for clear paths.\n"
    "Reachability: If an object is > 0.5m away, you CANNOT reach it with arms. You MUST move the robot body closer first.\n"
    "Verification: If a tool returns 'command_started': False, the robot DID NOT MOVE. Do not assume the pose has changed; you must retry.\n"
    "Call goal_complete only when fully achieved. Call goal_failed only when impossible."
)

# ─────────────────────────────────────────────────────────────────────────────
# Tool declarations (Gemini function-calling schema)
# ─────────────────────────────────────────────────────────────────────────────
_TOOLS = [
    # ── Motion ──────────────────────────────────────────────────────────────
    {"name": "move_forward",
     "description": "Move robot forward (+) or backward (−) by distance in metres. Waits for completion.",
     "parameters": {"type": "object",
                    "properties": {
                        "distance": {"type": "number",
                                     "description": "Metres. Positive = forward, negative = backward."},
                        "speed":    {"type": "number",
                                     "description": "Speed factor 0.1–1.0 (default 0.5)."}},
                    "required": ["distance"]}},

    {"name": "rotate",
     "description": "Rotate robot in place. Positive = left/CCW, negative = right/CW. Waits for completion.",
     "parameters": {"type": "object",
                    "properties": {
                        "angle": {"type": "number",
                                  "description": "Degrees. Positive = LEFT (CCW), negative = RIGHT (CW). "
                                                 "To center an object on the LEFT of the image, rotate LEFT (Positive). "
                                                 "To center an object on the RIGHT, rotate RIGHT (Negative)."},
                        "speed": {"type": "number",
                                  "description": "Speed factor 0.1–1.0 (default 0.5)."}},
                    "required": ["angle"]}},

    {"name": "navigate_to_location",
     "description": "Navigate to a saved named location via Nav2. Requires map to be loaded.",
     "parameters": {"type": "object",
                    "properties": {
                        "name": {"type": "string",
                                 "description": "Name of the saved location."}},
                    "required": ["name"]}},

    {"name": "navigate_to_pose",
     "description": "Navigate to a map-frame (x, y, yaw) pose via Nav2. Requires map.",
     "parameters": {"type": "object",
                    "properties": {
                        "x":   {"type": "number", "description": "Target X in map frame (m)."},
                        "y":   {"type": "number", "description": "Target Y in map frame (m)."},
                        "yaw": {"type": "number", "description": "Target yaw in degrees (0=east, 90=north)."}},
                    "required": ["x", "y", "yaw"]}},

    {"name": "cancel_navigation",
     "description": "Cancel any active Nav2 goal.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "follow_object",
     "description": "Start continuously following a YOLO-detected object class. Non-blocking.",
     "parameters": {"type": "object",
                    "properties": {
                        "class_name": {"type": "string",
                                       "description": "YOLO class name, e.g. 'person', 'ball'."}},
                    "required": ["class_name"]}},

    {"name": "stop_following",
     "description": "Stop following an object.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    # ── Mapping ──────────────────────────────────────────────────────────────
    # ── Navigation stack ─────────────────────────────────────────────────────
    {"name": "start_navigation",
     "description": "Start the Nav2 navigation stack. Must be called before navigate_to_location "
                    "or navigate_to_pose. Requires a saved map to be available.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "stop_navigation",
     "description": "Stop the Nav2 navigation stack.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "start_mapping",
     "description": "Start SLAM mapping. Use when no map is available and the robot needs to build one. "
                    "The robot can drive around to explore the area while mapping.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "stop_mapping",
     "description": "Stop SLAM mapping (without saving).",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "start_explore",
     "description": "Start autonomous exploration — robot drives around automatically to build the map. "
                    "Use after start_mapping when you want the robot to explore on its own.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "stop_explore",
     "description": "Stop autonomous exploration.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "save_map",
     "description": "Save the current map to disk. Call after mapping is complete. "
                    "After saving, navigation becomes available.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "start_vision_pipeline",
     "description": "Start the AI vision pipeline node. Must be called before enabling specific detectors (YOLO, face, etc).",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "stop_vision_pipeline",
     "description": "Stop the AI vision pipeline node.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    # ── Camera ───────────────────────────────────────────────────────────────
    {"name": "start_camera",
     "description": "Start the camera. Must be called before capture_and_analyse or any vision task. "
                    "If capture_and_analyse returns an error about no frame, call this first.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "stop_camera",
     "description": "Stop the camera.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    # ── Vision ───────────────────────────────────────────────────────────────
    {"name": "capture_and_analyse",
     "description": (
         "PRIMARY VISION TOOL — use first for any visual task. "
         "Captures a camera frame and asks the LLM a question about it. "
         "Returns scene description, object positions, spatial estimates. No YOLO config needed."),
     "parameters": {"type": "object",
                    "properties": {
                        "question": {"type": "string",
                                     "description": "Question about the scene, "
                                                    "e.g. 'Is there a ball? Where is it relative to centre?'"}},
                    "required": ["question"]}},

    {"name": "enable_detection",
     "description": "Enable a continuous real-time detector. "
                    "Use only for continuous tracking tasks, not one-shot queries (use capture_and_analyse instead).",
     "parameters": {"type": "object",
                    "properties": {
                        "type": {"type": "string",
                                 "enum": ["yolo", "face", "pose", "aruco", "gesture"],
                                 "description": "Detector type."}},
                    "required": ["type"]}},

    {"name": "get_detected_objects",
     "description": "Read latest YOLO detections. Only useful if YOLO is already enabled.",
     "parameters": {"type": "object",
                    "properties": {
                        "class_filter": {"type": "string",
                                         "description": "Optional: filter by class name."}},
                    "required": []}},

    # ── Arm ──────────────────────────────────────────────────────────────────
    {"name": "check_arm_reachable",
     "description": "Check if a position is within arm reach. Always call before set_arm. "
                    "Returns {reachable: bool, reason: str}.",
     "parameters": {"type": "object",
                    "properties": {
                        "distance_m": {"type": "number",
                                       "description": "Forward distance from shoulder (m)."},
                        "height_m":   {"type": "number",
                                       "description": "Height relative to shoulder (m). "
                                                      "Negative = below shoulder."},
                        "side":       {"type": "string", "enum": ["left", "right"],
                                       "description": "Which arm."}},
                    "required": ["distance_m", "height_m", "side"]}},

    {"name": "set_arm",
     "description": "Move an arm to a target position using 2D IK. "
                    "Reachability is checked internally; call is rejected if out of range.",
     "parameters": {"type": "object",
                    "properties": {
                        "side":       {"type": "string", "enum": ["left", "right"]},
                        "distance_m": {"type": "number",
                                       "description": "Forward distance from shoulder (0.10–0.35 m)."},
                        "height_m":   {"type": "number",
                                       "description": "Height relative to shoulder (-0.05 to +0.25 m)."}},
                    "required": ["side", "distance_m", "height_m"]}},

    {"name": "set_gripper",
     "description": "Open or close a gripper.",
     "parameters": {"type": "object",
                    "properties": {
                        "side":  {"type": "string", "enum": ["left", "right"]},
                        "state": {"type": "string", "enum": ["open", "close"]}},
                    "required": ["side", "state"]}},

    {"name": "reset_arm",
     "description": "Reset arm(s) to home position (shoulder=0°, elbow=0°).",
     "parameters": {"type": "object",
                    "properties": {
                        "side": {"type": "string", "enum": ["left", "right", "both"]}},
                    "required": ["side"]}},

    # ── Head / Map / Utility ─────────────────────────────────────────────────
    {"name": "turn_head",
     "description": "Rotate robot's head. Positive = left, negative = right.",
     "parameters": {"type": "object",
                    "properties": {
                        "yaw_degrees": {"type": "number",
                                        "description": "Yaw in degrees. Positive = left, negative = right."}},
                    "required": ["yaw_degrees"]}},

    {"name": "list_locations",
     "description": "Return all saved named locations.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "save_current_location",
     "description": "Save the robot's current position with a name.",
     "parameters": {"type": "object",
                    "properties": {
                        "name": {"type": "string", "description": "Location name."}},
                    "required": ["name"]}},

    {"name": "speak",
     "description": "Speak text via TTS. Use sparingly — only for meaningful user updates.",
     "parameters": {"type": "object",
                    "properties": {
                        "text": {"type": "string", "description": "Text to speak."}},
                    "required": ["text"]}},

    {"name": "patrol",
     "description": "Start a multi-point patrol through a list of saved named locations. Navigates to each in order.",
     "parameters": {"type": "object",
                    "properties": {
                        "locations": {"type": "array", "items": {"type": "string"},
                                      "description": "List of location names."}},
                    "required": ["locations"]}},

    {"name": "stop_patrol",
     "description": "Stop an active multi-point patrol.",
     "parameters": {"type": "object", "properties": {}, "required": []}},

    {"name": "goal_complete",
     "description": "Mark goal as successfully completed. Call ONLY when fully achieved.",
     "parameters": {"type": "object",
                    "properties": {
                        "summary": {"type": "string",
                                    "description": "Brief summary of what was accomplished."}},
                    "required": ["summary"]}},

    {"name": "goal_failed",
     "description": "Mark goal as failed. Call ONLY when impossible to complete.",
     "parameters": {"type": "object",
                    "properties": {
                        "reason": {"type": "string",
                                   "description": "Why the goal cannot be completed."}},
                    "required": ["reason"]}},

    {"name": "wait",
     "description": "Wait for specified seconds. Use this if a previous movement is 'still_moving'.",
     "parameters": {"type": "object",
                    "properties": {
                        "seconds": {"type": "number",
                                   "description": "Seconds to wait."}},
                    "required": ["seconds"]}},
]


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quaternion(yaw_rad: float):
    """Yaw → (x, y, z, w) quaternion."""
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────────────────────
# Schema helper — converts JSON-schema dicts to gtypes.Schema for google-genai
# ─────────────────────────────────────────────────────────────────────────────

def _schema_from_dict(d: dict) -> gtypes.Schema:
    _type_map = {
        'object':  gtypes.Type.OBJECT,
        'string':  gtypes.Type.STRING,
        'number':  gtypes.Type.NUMBER,
        'integer': gtypes.Type.INTEGER,
        'boolean': gtypes.Type.BOOLEAN,
        'array':   gtypes.Type.ARRAY,
    }
    t     = _type_map.get(str(d.get('type', 'string')).lower(), gtypes.Type.STRING)
    props = {k: _schema_from_dict(v) for k, v in d.get('properties', {}).items()} or None
    items = _schema_from_dict(d.get('items')) if d.get('items') else None
    req   = d.get('required') or None
    enum  = d.get('enum') or None
    desc  = d.get('description', '')
    return gtypes.Schema(type=t, description=desc, properties=props, items=items, required=req, enum=enum)


class RobotAgentNode(Node):

    def __init__(self):
        super().__init__('robot_agent_node')

        # ── Load config ───────────────────────────────────────────────────────
        self.declare_parameter('config_file', '')
        self._load_config(self.get_parameter('config_file').value)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value


        # ── Gemini ────────────────────────────────────────────────────────────
        api_key = (self.cfg.get('agent', {}).get('gemini_api_key', '')
                   or os.environ.get('GEMINI_API_KEY', ''))
        if not api_key:
            self.get_logger().error(
                'No Gemini API key! Set GEMINI_API_KEY env var or agent.gemini_api_key in config.')

        self._gemini = genai.Client(api_key=api_key)
        self._model  = self.cfg.get('agent', {}).get('model', 'gemini-2.5-flash')

        # Build tool declarations for the new SDK
        fn_decls = [gtypes.FunctionDeclaration(
            name=d['name'],
            description=d['description'],
            parameters=_schema_from_dict(d.get('parameters', {})),
        ) for d in _TOOLS]
        self._chat_config = gtypes.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT,
            tools=[gtypes.Tool(function_declarations=fn_decls)],
            temperature=0.1,
        )
        # Vision config — no tools, just multimodal generation
        self._vision_config = gtypes.GenerateContentConfig(temperature=0.1)

        self._max_steps: int = self.cfg.get('agent', {}).get('max_steps', 40)
        self._step_timeout: float = self.cfg.get('agent', {}).get('step_timeout', 30.0)

        # ── Arm / joint config ────────────────────────────────────────────────
        arm = self.cfg.get('arm', {})
        self._L1: float = arm.get('L1', 0.12)
        self._L2: float = arm.get('L2', 0.13)
        self._reach_min_m = arm.get('reach_min_m', 0.10)
        self._reach_max_m = arm.get('reach_max_m', 0.35)
        self._height_min_m = arm.get('height_min_m', -0.05)
        self._height_max_m = arm.get('height_max_m', 0.25)

        jl = self.cfg.get('joint_limits', {})
        self._shoulder_min = math.radians(jl.get('shoulder_pitch_min_deg', -45.0))
        self._shoulder_max = math.radians(jl.get('shoulder_pitch_max_deg',  180.0))
        self._elbow_min    = math.radians(jl.get('elbow_min_deg',    0.0))
        self._elbow_max    = math.radians(jl.get('elbow_max_deg',  50.0))
        self._neck_min     = math.radians(jl.get('neck_yaw_min_deg', -90.0))
        self._neck_max     = math.radians(jl.get('neck_yaw_max_deg',  90.0))
        self._gripper_open  = float(jl.get('gripper_open',  -0.785))
        self._gripper_close = float(jl.get('gripper_close', 1.047))

        # ── CV bridge ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_image: Image = None
        self._image_lock = threading.Lock()

        # ── Robot state (protected by _state_lock) ────────────────────────────
        self._state_lock     = threading.Lock()
        self.nav_status      = 'idle'
        self.map_available   = False
        self.nav_active      = False
        self.camera_active   = False
        self.vision_active   = False
        self.yolo_enabled    = False
        self.precise_active  = False
        self.saved_locations: dict = {}
        self.latest_detections: list = []
        self.current_pose    = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # ── Agent state ───────────────────────────────────────────────────────
        self._goal_active  = False
        self._cancel_flag  = threading.Event()
        self._agent_thread: threading.Thread = None

        # ── QoS ───────────────────────────────────────────────────────────────
        _transient = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_status   = self.create_publisher(String,            '/agent/status',               10)
        self._pub_speak    = self.create_publisher(String,            '/robot/speak',                10)
        self._pub_precise  = self.create_publisher(String,            '/robot/precise_move',         10)
        self._pub_follow   = self.create_publisher(String,            '/robot/follow_object',        10)
        self._pub_save_loc = self.create_publisher(String,            '/robot/save_location',        10)
        self._pub_goto_loc = self.create_publisher(String,            '/robot/goto_location',        10)
        self._pub_goal     = self.create_publisher(PoseStamped,       '/goal_pose',                  10)
        # JointGroupPositionController → Float64MultiArray
        self._pub_arm_l    = self.create_publisher(Float64MultiArray, '/left_arm_controller/commands',          10)
        self._pub_arm_r    = self.create_publisher(Float64MultiArray, '/right_arm_controller/commands',         10)
        self._pub_grip_l   = self.create_publisher(Float64MultiArray, '/left_gripper_controller/commands',      10)
        self._pub_grip_r   = self.create_publisher(Float64MultiArray, '/right_gripper_controller/commands',     10)
        self._pub_head_cmd = self.create_publisher(Float64MultiArray, '/head_controller/commands',              10)
        self._pub_patrol   = self.create_publisher(String,            '/robot/patrol_locations',        10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String,  '/agent/goal',               self._cb_goal,           10)
        self.create_subscription(Empty,   '/agent/cancel',             self._cb_cancel,         10)
        self.create_subscription(String,  '/robot/nav_status',         self._cb_nav_status,     10)
        self.create_subscription(Bool,    '/robot/map_available',      self._cb_map_available,  10)
        self.create_subscription(Bool,    '/robot/navigation_active',  self._cb_nav_active,     10)
        self.create_subscription(Bool,    '/robot/camera_active',      self._cb_camera_active,  10)
        self.create_subscription(Bool,    '/robot/vision_active',      self._cb_vision_active,  10)
        self.create_subscription(Bool,    '/robot/yolo_enabled',       self._cb_yolo_enabled,   10)
        self.create_subscription(Bool,    '/robot/precise_move_active',self._cb_precise_active, 10)
        self.create_subscription(Bool,    '/robot/patrol_active',      self._cb_patrol_active,  10)
        self.create_subscription(String,  '/robot/locations_list',     self._cb_locations,      10)
        self.create_subscription(Image,   '/camera/image_raw',         self._cb_image,          10)
        # Vision pipeline publishes YOLO results as JSON String on /vision/yolo_detections
        self.create_subscription(String,  '/vision/yolo_detections',   self._cb_detections,     10)
        self.create_subscription(Odometry, '/odometry/filtered',        self._cb_odom,           10)

        # ── Service clients ───────────────────────────────────────────────────
        self._svc_cancel_nav   = self.create_client(Trigger, '/robot/cancel_navigation')
        self._svc_stop_follow  = self.create_client(Trigger, '/robot/stop_follow_object')
        self._svc_stop_precise = self.create_client(Trigger, '/robot/stop_precise_move')
        self._svc_enable_yolo    = self.create_client(Trigger, '/robot/enable_yolo')
        self._svc_enable_face    = self.create_client(Trigger, '/robot/enable_face')
        self._svc_enable_pose    = self.create_client(Trigger, '/robot/enable_pose')
        self._svc_enable_aruco   = self.create_client(Trigger, '/robot/enable_aruco')
        self._svc_enable_gesture = self.create_client(Trigger, '/robot/enable_gesture')
        self._svc_start_nav     = self.create_client(Trigger, '/robot/start_navigation')
        self._svc_stop_nav      = self.create_client(Trigger, '/robot/stop_navigation')
        self._svc_start_mapping = self.create_client(Trigger, '/robot/start_mapping')
        self._svc_stop_mapping  = self.create_client(Trigger, '/robot/stop_mapping')
        self._svc_save_map      = self.create_client(Trigger, '/robot/save_map')
        self._svc_start_explore = self.create_client(Trigger, '/robot/start_explore')
        self._svc_stop_explore  = self.create_client(Trigger, '/robot/stop_explore')
        self._svc_start_camera  = self.create_client(Trigger, '/robot/start_camera')
        self._svc_stop_camera   = self.create_client(Trigger, '/robot/stop_camera')
        self._svc_start_vision  = self.create_client(Trigger, '/robot/start_vision')
        self._svc_stop_vision   = self.create_client(Trigger, '/robot/stop_vision')
        self._svc_stop_patrol   = self.create_client(Trigger, '/robot/stop_patrol')

        self.get_logger().info('RobotAgentNode ready.')

    # ─────────────────────────────────────────────────────────────────────────
    # Config
    # ─────────────────────────────────────────────────────────────────────────

    def _load_config(self, config_file: str):
        self.cfg: dict = {}
        if not config_file:
            try:
                pkg_share   = get_package_share_directory('my_bot')
                config_file = os.path.join(pkg_share, 'config', 'agent_config.yaml')
            except Exception:
                config_file = ''
        try:
            import yaml
            with open(config_file) as f:
                self.cfg = yaml.safe_load(f) or {}
            self.get_logger().info(f'Loaded agent config: {config_file}')
        except Exception as e:
            self.get_logger().warn(f'Could not load config ({config_file}): {e}. Using defaults.')

    # ─────────────────────────────────────────────────────────────────────────
    # State callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_nav_status(self, msg: String):
        with self._state_lock:
            self.nav_status = msg.data

    def _cb_map_available(self, msg: Bool):
        with self._state_lock:
            self.map_available = msg.data

    def _cb_nav_active(self, msg: Bool):
        with self._state_lock:
            self.nav_active = msg.data

    def _cb_camera_active(self, msg: Bool):
        with self._state_lock:
            self.camera_active = msg.data

    def _cb_vision_active(self, msg: Bool):
        with self._state_lock:
            self.vision_active = msg.data

    def _cb_yolo_enabled(self, msg: Bool):
        with self._state_lock:
            self.yolo_enabled = msg.data

    def _cb_precise_active(self, msg: Bool):
        with self._state_lock:
            self.precise_active = msg.data

    def _cb_patrol_active(self, msg: Bool):
        with self._state_lock:
            self.patrol_active = msg.data

    def _cb_locations(self, msg: String):
        try:
            locs = json.loads(msg.data)
            with self._state_lock:
                self.saved_locations = locs
        except Exception:
            pass

    def _cb_image(self, msg: Image):
        with self._image_lock:
            self._latest_image = msg

    def _cb_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                with self._state_lock:
                    self.latest_detections = data
        except Exception:
            pass

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._state_lock:
            self.current_pose = {
                'x':   round(p.x, 3),
                'y':   round(p.y, 3),
                'yaw': round(math.degrees(yaw_rad), 2),
            }

    # ─────────────────────────────────────────────────────────────────────────
    # Goal / Cancel entry points
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_goal(self, msg: String):
        goal = msg.data.strip()
        if not goal:
            return
        self.get_logger().info(f'New goal: {goal}')

        # Cancel any in-progress goal
        if self._goal_active:
            self._cancel_flag.set()
            if self._agent_thread and self._agent_thread.is_alive():
                self._agent_thread.join(timeout=3.0)

        self._cancel_flag.clear()
        self._goal_active = True
        self._agent_thread = threading.Thread(
            target=self._react_loop, args=(goal,), daemon=True)
        self._agent_thread.start()

    def _cb_cancel(self, _msg):
        self.get_logger().info('Cancel received.')
        self._cancel_flag.set()
        self._goal_active = False
        self._call_service(self._svc_cancel_nav)
        self._call_service(self._svc_stop_precise)
        self._call_service(self._svc_stop_follow)
        self._call_service(self._svc_stop_patrol)
        self._reset_arms()
        self._speak('Stopping.')

    # ─────────────────────────────────────────────────────────────────────────
    # ReAct loop
    # ─────────────────────────────────────────────────────────────────────────

    def _react_loop(self, goal: str):
        self.get_logger().info(f'ReAct loop started: {goal}')
        self._speak('Understood, working on it.')

        chat = self._gemini.chats.create(model=self._model, config=self._chat_config)

        # ── First turn: goal + initial state ─────────────────────────────────
        initial = (f"Goal: {goal}\n\n"
                   f"Initial robot state:\n{json.dumps(self._get_state(), indent=2)}")
        tool_name, tool_args = self._llm_text_turn(chat, initial)

        for step in range(self._max_steps):
            if self._cancel_flag.is_set():
                self.get_logger().info('Goal cancelled mid-loop.')
                return

            # Handle missing tool call
            if not tool_name:
                self.get_logger().warn(f'Step {step}: no tool call — nudging LLM.')
                tool_name, tool_args = self._llm_text_turn(
                    chat, 'You must call exactly one tool per turn. What is your next action?')
                continue

            self._publish_status(goal, step, tool_name, str(tool_args)[:120])
            hist_len = 0
            try:
                hist_len = len(chat.get_history())
            except:
                if hasattr(chat, '_comprehensive_history'):
                    hist_len = len(chat._comprehensive_history)
                
            self.get_logger().info(f'Step {step} (History: {hist_len}): {tool_name}({json.dumps(tool_args)[:80]})')

            # ── Terminal tools ────────────────────────────────────────────────
            if tool_name == 'goal_complete':
                summary = tool_args.get('summary', 'Done.')
                self._publish_status(goal, step, 'complete', summary, success=True)
                self._speak(summary)
                self._goal_active = False
                return

            if tool_name == 'goal_failed':
                reason = tool_args.get('reason', 'Unknown failure.')
                self._publish_status(goal, step, 'failed', reason,
                                     success=False, error=reason)
                self._speak(f'I cannot complete the task: {reason}')
                self._goal_active = False
                return

            # ── Execute tool ──────────────────────────────────────────────────
            try:
                result = self._execute_tool(tool_name, tool_args)
            except Exception as exc:
                result = {'error': f'Tool execution error: {exc}'}
                self.get_logger().error(f'Tool {tool_name} raised: {exc}')

            # Embed current state so LLM always has fresh context
            result['_current_state'] = self._get_state()

            # ── Send result, get next action ──────────────────────────────────
            tool_name, tool_args = self._llm_tool_result_turn(chat, tool_name, result, include_image=True)

        # Max steps exceeded
        self._publish_status(goal, self._max_steps, 'failed', 'Max steps exceeded',
                             success=False, error='Max steps exceeded')
        self._speak('I was unable to complete the task within the allowed steps.')
        self._goal_active = False

    # ─────────────────────────────────────────────────────────────────────────
    # LLM call helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _get_state(self) -> dict:
        with self._state_lock:
            return {
                'nav_status':        self.nav_status,
                'map_available':     self.map_available,
                'navigation_active': self.nav_active,
                'camera_active':     self.camera_active,
                'vision_active':     self.vision_active,
                'yolo_enabled':      self.yolo_enabled,
                'precise_move_active': self.precise_active,
                'saved_locations':   (list(self.saved_locations.keys())
                                      if isinstance(self.saved_locations, dict)
                                      else self.saved_locations),
                'latest_detections': self.latest_detections[:5],
                'current_pose':      self.current_pose,
                'current_yaw_deg':   round(self.current_pose.get('yaw', 0.0), 1) if self.current_pose else 0.0,
            }

    def _llm_text_turn(self, chat, text: str):
        """Send a plain-text user message. Returns (tool_name, tool_args)."""
        try:
            # Prepare contents: text + current image (only if active)
            contents = [text]
            with self._state_lock:
                cam_ok = self.camera_active
            if cam_ok:
                img_part = self._get_image_part()
                if img_part:
                    contents.append(img_part)

            response = chat.send_message(message=contents)
            return self._extract_tool_call(response)
        except Exception as exc:
            self.get_logger().error(f'LLM text turn failed: {exc}')
            return None, {}

    def _llm_tool_result_turn(self, chat, tool_name: str, result: dict, include_image: bool = False):
        """Send a function_response. Returns (next_tool_name, next_tool_args)."""
        try:
            # Clean up result for JSON serializability
            clean_res = json.loads(json.dumps(result, default=str))
            fn_part   = gtypes.Part.from_function_response(name=tool_name, response=clean_res)
            
            contents = [fn_part]
            if include_image:
                with self._state_lock:
                    cam_ok = self.camera_active
                if cam_ok:
                    img_part = self._get_image_part()
                    if img_part:
                        contents.append(img_part)

            response = chat.send_message(message=contents)
            return self._extract_tool_call(response)
        except Exception as exc:
            self.get_logger().error(f'LLM tool-result turn failed: {exc}')
            return None, {}

    def _extract_tool_call(self, response):
        """Pull (tool_name, tool_args) from a response, logging text/thoughts."""
        if response is None:
            return None, {}
        try:
            # First, log any text/thoughts found in the response
            for part in response.candidates[0].content.parts:
                if hasattr(part, 'text') and part.text:
                    text_out = part.text.strip()
                    if text_out:
                        self.get_logger().info(f'LLM Text: {text_out}')

            # Then, return the first function call found
            for part in response.candidates[0].content.parts:
                fc = getattr(part, 'function_call', None)
                if fc and getattr(fc, 'name', None):
                    return fc.name, dict(fc.args) if fc.args else {}
        except Exception as exc:
            self.get_logger().warn(f'Could not extract parts from response: {exc}')
        return None, {}

    # ─────────────────────────────────────────────────────────────────────────
    # Tool dispatcher
    # ─────────────────────────────────────────────────────────────────────────

    def _execute_tool(self, name: str, args: dict) -> dict:
        _dispatch = {
            'move_forward':          self._tool_move_forward,
            'rotate':                self._tool_rotate,
            'navigate_to_location':  self._tool_navigate_to_location,
            'navigate_to_pose':      self._tool_navigate_to_pose,
            'cancel_navigation':     self._tool_cancel_navigation,
            'follow_object':         self._tool_follow_object,
            'stop_following':        self._tool_stop_following,
            'start_navigation':      self._tool_start_navigation,
            'stop_navigation':       self._tool_stop_navigation,
            'start_mapping':         self._tool_start_mapping,
            'stop_mapping':          self._tool_stop_mapping,
            'save_map':              self._tool_save_map,
            'start_explore':         self._tool_start_explore,
            'stop_explore':          self._tool_stop_explore,
            'start_camera':          self._tool_start_camera,
            'stop_camera':           self._tool_stop_camera,
            'capture_and_analyse':   self._tool_capture_and_analyse,
            'enable_detection':      self._tool_enable_detection,
            'get_detected_objects':  self._tool_get_detected_objects,
            'check_arm_reachable':   self._tool_check_arm_reachable,
            'set_arm':               self._tool_set_arm,
            'set_gripper':           self._tool_set_gripper,
            'reset_arm':             self._tool_reset_arm,
            'turn_head':             self._tool_turn_head,
            'list_locations':        self._tool_list_locations,
            'save_current_location': self._tool_save_location,
            'speak':                 self._tool_speak,
            'start_vision_pipeline': self._tool_start_vision,
            'stop_vision_pipeline':  self._tool_stop_vision,
            'patrol':                self._tool_patrol,
            'stop_patrol':           self._tool_stop_patrol,
            'wait':                  self._tool_wait,
        }
        handler = _dispatch.get(name)
        if handler is None:
            return {'error': f'Unknown tool: {name}'}
        return handler(args)

    # ─────────────────────────────────────────────────────────────────────────
    # Motion tools
    # ─────────────────────────────────────────────────────────────────────────

    def _tool_move_forward(self, args: dict) -> dict:
        dist  = float(args.get('distance', 0.0))
        factor = float(args.get('speed', 0.5))
        speed = max(0.05, min(0.5, factor * 0.5))
        
        with self._state_lock:
            start_pose = dict(self.current_pose) if self.current_pose else None

        cmd   = json.dumps({'mode': 'move', 'distance': dist, 'speed': speed})
        self._pub_precise.publish(String(data=cmd))
        
        # Smart buffer: 20s for sim lag, 10s for real hardware
        buffer = 20.0 if self.use_sim_time else 10.0
        timeout = abs(dist) / speed + buffer
        started, finished = self._wait_for_precise_move(timeout)
        
        # Wait for the Manager's 0.5s status timer to tick
        time.sleep(0.6)
        
        with self._state_lock:
            end_pose = dict(self.current_pose) if self.current_pose else None
            still_active = self.precise_active

        # Calculate actual distance traveled from odom
        actual_dist = 0.0
        if start_pose and end_pose:
            actual_dist = math.sqrt((end_pose['x'] - start_pose['x'])**2 + (end_pose['y'] - start_pose['y'])**2)

        return {
            'command_started': started,
            'command_finished': finished,
            'still_moving': still_active,
            'commanded_dist': dist,
            'actual_dist_m': round(actual_dist, 3),
            'start_pose': start_pose,
            'end_pose': end_pose
        }

    def _tool_rotate(self, args: dict) -> dict:
        angle = float(args.get('angle', 0.0))
        factor = float(args.get('speed', 0.5))
        speed = max(10.0, min(100.0, factor * 100.0))
        
        with self._state_lock:
            start_pose = dict(self.current_pose) if self.current_pose else None

        cmd   = json.dumps({'mode': 'rotate', 'angle': angle, 'speed': speed})
        self._pub_precise.publish(String(data=cmd))
        
        # Smart buffer: 20s for sim lag, 10s for real hardware
        buffer = 20.0 if self.use_sim_time else 10.0
        timeout = abs(angle) / speed + buffer
        started, finished = self._wait_for_precise_move(timeout)
        
        # Wait for the Manager's 0.5s status timer to tick
        time.sleep(0.6)
        
        with self._state_lock:
            end_pose = dict(self.current_pose) if self.current_pose else None
            still_active = self.precise_active

        # Calculate actual rotation achieved from odom
        actual_rot = 0.0
        if start_pose and end_pose:
            actual_rot = end_pose['yaw'] - start_pose['yaw']
            while actual_rot > 180: actual_rot -= 360
            while actual_rot < -180: actual_rot += 360

        return {
            'started': started,
            'finished': finished,
            'still_moving': still_active,
            'commanded_angle': angle,
            'actual_angle_deg': round(actual_rot, 2),
            'start_pose': start_pose,
            'end_pose': end_pose
        }

    def _tool_navigate_to_location(self, args: dict) -> dict:
        name = args.get('name', '')
        with self._state_lock:
            if not self.map_available:
                return {'error': 'No map loaded. Cannot use Nav2.'}
        self._pub_goto_loc.publish(String(data=name))
        status = self._wait_for_nav(timeout=120.0)
        return {'nav_result': status}

    def _tool_navigate_to_pose(self, args: dict) -> dict:
        x   = float(args.get('x', 0.0))
        y   = float(args.get('y', 0.0))
        yaw = math.radians(float(args.get('yaw', 0.0)))
        with self._state_lock:
            if not self.map_available:
                return {'error': 'No map loaded. Cannot use Nav2.'}
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pub_goal.publish(pose)
        status = self._wait_for_nav(timeout=120.0)
        return {'nav_result': status}

    def _tool_cancel_navigation(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_cancel_nav)
        return {'cancelled': ok}

    def _tool_follow_object(self, args: dict) -> dict:
        cls = args.get('class_name', '')
        self._pub_follow.publish(String(data=cls))
        return {'following': cls}

    def _tool_stop_following(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_follow)
        return {'stopped': ok}

    def _tool_start_navigation(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_start_nav, timeout=15.0)
        return {'navigation_started': ok,
                'note': 'Wait a moment for Nav2 to initialise, then use navigate_to_location or navigate_to_pose.'}

    def _tool_stop_navigation(self, _args: dict) -> dict:
        """Safely stop Nav2.
        If navigation is already inactive, skip the service call to avoid timeouts.
        Returns a dict indicating the result.
        """
        # If Nav2 is not active, there's nothing to stop.
        if not getattr(self, 'nav_active', False):
            return {'navigation_stopped': True,
                    'note': 'Nav2 was already inactive; no service call made.'}
        # Otherwise, call the stop navigation service.
        ok = self._call_service(self._svc_stop_nav)
        return {'navigation_stopped': ok,
                'note': 'Nav2 stop service invoked.'}


    def _tool_start_mapping(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_start_mapping, timeout=10.0)
        return {'mapping_started': ok,
                'note': 'Drive around to explore. Call save_map when done.'}

    def _tool_stop_mapping(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_mapping)
        return {'mapping_stopped': ok}

    def _tool_save_map(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_save_map, timeout=15.0)
        return {'map_saved': ok,
                'note': 'Start navigation to load the saved map.' if ok else 'Save failed.'}

    def _tool_start_explore(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_start_explore, timeout=10.0)
        return {'explore_started': ok,
                'note': 'Robot is autonomously exploring. Call stop_explore when coverage is sufficient, then save_map.'}

    def _tool_stop_explore(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_explore)
        return {'explore_stopped': ok}

    # ─────────────────────────────────────────────────────────────────────────
    # Vision tools
    # ─────────────────────────────────────────────────────────────────────────

    def _get_image_part(self) -> gtypes.Part:
        """Utility to grab latest image and wrap in Gemini Part."""
        with self._image_lock:
            img_msg = self._latest_image
        if img_msg is None:
            return None

        try:
            # Use bgr8 correctly for OpenCV, then convert to RGB if needed, or just let CV handles encoding.
            # Most ROS images are BGR, cv_bridge converts to what we ask.
            cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return gtypes.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg')
        except Exception as exc:
            self.get_logger().warn(f'Failed to create image part: {exc}')
            return None

    def _tool_start_camera(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_start_camera, timeout=10.0)
        
        # If success is False, it might be because it's ALREADY active.
        # We check our internal state to verify.
        if not ok:
            with self._state_lock:
                if self.camera_active:
                    return {'camera_started': True, 'note': 'Camera was already active.'}
            return {'camera_started': False, 'note': 'start_camera service failed.'}

        # Wait briefly for first frame to arrive
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._image_lock:
                if self._latest_image is not None:
                    return {'camera_started': True, 'note': 'Camera ready, frame received.'}
            time.sleep(0.1)
        return {'camera_started': True, 'note': 'Camera started but no frame yet — retry capture shortly.'}

    def _tool_stop_camera(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_camera)
        return {'camera_stopped': ok}

    def _tool_capture_and_analyse(self, args: dict) -> dict:
        question = args.get('question', 'Describe what you see.')

        with self._image_lock:
            img_msg = self._latest_image

        if img_msg is None:
            return {'error': 'No camera frame available. Call start_camera first, then retry.'}

        try:
            cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as exc:
            return {'error': f'Image decode failed: {exc}'}

        try:
            _, buf = cv2.imencode('.jpg', cv_img)
        except Exception as exc:
            return {'error': f'Image encode failed: {exc}'}

        try:
            img_part = gtypes.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg')
            resp     = self._gemini.models.generate_content(
                model=self._model,
                contents=[question, img_part],
                config=self._vision_config,
            )
            return {'analysis': resp.text}
        except Exception as exc:
            return {'error': f'Vision LLM call failed: {exc}'}

    def _tool_enable_detection(self, args: dict) -> dict:
        det_type = args.get('type', '')
        svc_map  = {
            'yolo':    self._svc_enable_yolo,
            'face':    self._svc_enable_face,
            'pose':    self._svc_enable_pose,
            'aruco':   self._svc_enable_aruco,
            'gesture': self._svc_enable_gesture,
        }
        svc = svc_map.get(det_type)
        if svc is None:
            return {'error': f'Unknown detection type: {det_type}'}
        ok = self._call_service(svc)
        return {'enabled': det_type, 'success': ok}

    def _tool_start_vision(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_start_vision)
        if not ok:
            with self._state_lock:
                if self.vision_active:
                    return {'vision_pipeline_started': True, 'note': 'Vision pipeline was already active.'}
        return {'vision_pipeline_started': ok}

    def _tool_stop_vision(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_vision)
        return {'vision_pipeline_stopped': ok}

    def _tool_get_detected_objects(self, args: dict) -> dict:
        class_filter = args.get('class_filter', None)
        with self._state_lock:
            dets = list(self.latest_detections)
        if class_filter:
            dets = [d for d in dets
                    if str(d.get('class', '')).lower() == class_filter.lower()]
        return {'detections': dets, 'count': len(dets)}

    # ─────────────────────────────────────────────────────────────────────────
    # Arm tools
    # ─────────────────────────────────────────────────────────────────────────

    def _arm_ik(self, dx: float, dz: float):
        """2D planar IK. Returns (shoulder_rad, elbow_rad) or raises ValueError."""
        dist_sq   = dx * dx + dz * dz
        cos_elbow = (dist_sq - self._L1 ** 2 - self._L2 ** 2) / (2 * self._L1 * self._L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_rad = math.acos(cos_elbow)
        shoulder_rad = (math.atan2(dz, dx)
                        - math.atan2(self._L2 * math.sin(elbow_rad),
                                     self._L1 + self._L2 * math.cos(elbow_rad)))
        # Offset by 90 degrees because robot 0.0 is Straight Down
        shoulder_rad += (math.pi / 2.0)
        return shoulder_rad, elbow_rad

    def _tool_check_arm_reachable(self, args: dict) -> dict:
        dx = float(args.get('distance_m', 0.0))
        dz = float(args.get('height_m',   0.0))

        if not (self._reach_min_m <= dx <= self._reach_max_m):
            return {'reachable': False,
                    'reason': f'Forward distance {dx:.3f} m out of range [{self._reach_min_m}, {self._reach_max_m}] m.'}
        if not (self._height_min_m <= dz <= self._height_max_m):
            return {'reachable': False,
                    'reason': f'Height {dz:.3f} m out of range [{self._height_min_m}, {self._height_max_m}] m.'}

        try:
            s_rad, e_rad = self._arm_ik(dx, dz)
        except ValueError as exc:
            return {'reachable': False, 'reason': str(exc)}

        if not (self._shoulder_min <= s_rad <= self._shoulder_max):
            return {'reachable': False,
                    'reason': (f'Shoulder angle {math.degrees(s_rad):.1f}° outside limits '
                               f'[{math.degrees(self._shoulder_min):.1f}°, '
                               f'{math.degrees(self._shoulder_max):.1f}°].')}
        if not (self._elbow_min <= e_rad <= self._elbow_max):
            return {'reachable': False,
                    'reason': (f'Elbow angle {math.degrees(e_rad):.1f}° outside limits '
                               f'[{math.degrees(self._elbow_min):.1f}°, '
                               f'{math.degrees(self._elbow_max):.1f}°].')}

        return {'reachable': True, 'reason': 'Within reach.',
                'shoulder_deg': round(math.degrees(s_rad), 1),
                'elbow_deg':    round(math.degrees(e_rad), 1)}

    def _tool_set_arm(self, args: dict) -> dict:
        side = args.get('side', 'right')
        dx   = float(args.get('distance_m', 0.25))
        dz   = float(args.get('height_m',   0.0))

        # Safety gate: check reachability before computing IK
        reach = self._tool_check_arm_reachable({'distance_m': dx, 'height_m': dz, 'side': side})
        if not reach.get('reachable', False):
            return {'error': f'Unreachable: {reach["reason"]}'}

        s_rad, e_rad = self._arm_ik(dx, dz)
        # Hard clamp — never trust raw IK output beyond limits
        s_rad = max(self._shoulder_min, min(self._shoulder_max, s_rad))
        e_rad = max(self._elbow_min,    min(self._elbow_max,    e_rad))

        self._pub_arm_positions(side, s_rad, e_rad)
        return {'side': side,
                'shoulder_deg': round(math.degrees(s_rad), 1),
                'elbow_deg':    round(math.degrees(e_rad), 1)}

    def _pub_arm_positions(self, side: str, shoulder_rad: float, elbow_rad: float):
        msg = Float64MultiArray()
        msg.data = [shoulder_rad, elbow_rad]
        if side == 'left':
            self._pub_arm_l.publish(msg)
        else:
            self._pub_arm_r.publish(msg)

    def _tool_set_gripper(self, args: dict) -> dict:
        side  = args.get('side',  'right')
        state = args.get('state', 'open')
        pos   = self._gripper_open if state == 'open' else self._gripper_close
        msg   = Float64MultiArray()
        msg.data = [pos]
        if side == 'left':
            self._pub_grip_l.publish(msg)
        else:
            self._pub_grip_r.publish(msg)
        return {'side': side, 'state': state}

    def _tool_reset_arm(self, args: dict) -> dict:
        side  = args.get('side', 'both')
        sides = ['left', 'right'] if side == 'both' else [side]
        for s in sides:
            self._pub_arm_positions(s, 0.0, 0.0)
        return {'reset': sides}

    def _tool_wait(self, args: dict) -> dict:
        seconds = float(args.get('seconds', 1.0))
        time.sleep(seconds)
        return {'waited_seconds': seconds}

    def _reset_arms(self):
        """Emergency home both arms."""
        self._pub_arm_positions('left',  0.0, 0.0)
        self._pub_arm_positions('right', 0.0, 0.0)

    # ─────────────────────────────────────────────────────────────────────────
    # Head / Map / Utility tools
    # ─────────────────────────────────────────────────────────────────────────

    def _tool_turn_head(self, args: dict) -> dict:
        yaw_deg = float(args.get('yaw_degrees', 0.0))
        yaw_rad = math.radians(
            max(math.degrees(self._neck_min),
                min(math.degrees(self._neck_max), yaw_deg)))
        msg = Float64MultiArray()
        msg.data = [yaw_rad]
        self._pub_head_cmd.publish(msg)
        return {'yaw_deg': yaw_deg}

    def _tool_list_locations(self, _args: dict) -> dict:
        with self._state_lock:
            locs = dict(self.saved_locations)
        return {'locations': locs}

    def _tool_save_location(self, args: dict) -> dict:
        name = args.get('name', '')
        self._pub_save_loc.publish(String(data=name))
        return {'saved': name}

    def _tool_speak(self, args: dict) -> dict:
        text = args.get('text', '')
        self._speak(text)
        return {'spoken': text}

    def _tool_patrol(self, args: dict) -> dict:
        locs = args.get('locations', [])
        self._pub_patrol.publish(String(data=json.dumps(locs)))
        return {'patrol_started': locs}

    def _tool_stop_patrol(self, _args: dict) -> dict:
        ok = self._call_service(self._svc_stop_patrol)
        return {'patrol_stopped': ok}

    # ─────────────────────────────────────────────────────────────────────────
    # Wait helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _wait_for_nav(self, timeout: float = 120.0) -> str:
        """Wait for nav_status to reach a terminal state."""
        # Wait up to 3 s for navigation to start
        start_dl = time.time() + 3.0
        while time.time() < start_dl:
            with self._state_lock:
                if self.nav_status == 'navigating':
                    break
            time.sleep(0.1)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel_flag.is_set():
                return 'cancelled'
            with self._state_lock:
                s = self.nav_status
            if s in ('goal_reached', 'goal_failed', 'cancelled'):
                return s
            time.sleep(0.1)
        return 'timeout'

    def _wait_for_precise_move(self, timeout: float = 30.0):
        """Wait for precise_move_active to go True (started) then False (finished)."""
        started = False
        finished = False

        # Phase 1: wait up to 10s for the subprocess to start (precise_active → True)
        start_dl = time.time() + 10.0
        while time.time() < start_dl:
            if self._cancel_flag.is_set():
                return started, finished
            with self._state_lock:
                if self.precise_active:
                    started = True
                    break
            time.sleep(0.05)

        if not started:
            self.get_logger().warn('Precise move command timed out before starting.')
            return started, finished

        # Phase 2: wait for move to complete (precise_active → False)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel_flag.is_set():
                return started, finished
            with self._state_lock:
                if not self.precise_active:
                    finished = True
                    return started, finished
            time.sleep(0.1)
        
        self.get_logger().warn('Precise move command timed out while moving.')
        return started, finished

    # ─────────────────────────────────────────────────────────────────────────
    # Misc helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _speak(self, text: str):
        self._pub_speak.publish(String(data=text))

    def _publish_status(self, goal: str, step: int, phase: str, detail: str,
                        success=None, error=None):
        payload = {
            'goal': goal, 'phase': phase, 'step': step,
            'detail': detail, 'success': success, 'error': error,
        }
        self._pub_status.publish(String(data=json.dumps(payload)))

    def _call_service(self, client, timeout: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'Service {client.srv_name} not available.')
            return False

        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warn(f'Service {client.srv_name} call timed out.')
                return False
            time.sleep(0.02)

        try:
            return future.result().success
        except Exception as e:
            self.get_logger().error(f'Service {client.srv_name} failed: {e}')
            return False


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotAgentNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
