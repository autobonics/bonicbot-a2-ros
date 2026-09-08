#!/usr/bin/env python3
"""
Single-process vision pipeline for RPi efficiency.
Image decoded ONCE per frame; each detector runs on its own throttled rate.

Control: subscribe to /vision/control (JSON String, transient-local)
  e.g. {"yolo": true, "pose": false, "face": true, "gesture": true, "aruco": false}

Publishes:
  /vision/yolo_detections   std_msgs/String  JSON — YOLO object list
  /vision/pose_landmarks    std_msgs/String  JSON — MediaPipe body pose landmarks (33 pts)
  /vision/face_detections   std_msgs/String  JSON — face bounding boxes + keypoints
  /vision/gestures          std_msgs/String  JSON — hand gestures + 21-point hand landmarks
  /vision/aruco_markers     geometry_msgs/PoseArray — 3D marker poses (camera frame)
  /vision/aruco_ids         std_msgs/String  JSON — marker IDs matching PoseArray order
  /vision/yolo_active       std_msgs/Bool
  /vision/pose_active       std_msgs/Bool
  /vision/face_active       std_msgs/Bool
  /vision/gesture_active    std_msgs/Bool
  /vision/aruco_active      std_msgs/Bool
  /vision/nearest_person    std_msgs/String  JSON — nearest detected person: {"distance": 1.2, "angle": -0.1, "id": 0} or null

Gesture model required at ~/models/gesture_recognizer.task
  Download: https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
  Gestures: Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
import cv2
import numpy as np
import json
import math
import os

# ── Optional dependency guards ────────────────────────────────────────────────

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = True
except ImportError:
    _CV_BRIDGE = False

try:
    import onnxruntime as ort
    _ORT_LIB = True
except ImportError:
    _ORT_LIB = False

# Average adult shoulder width used for distance estimation
_PERSON_WIDTH_M = 0.45

# COCO class names (80 classes — YOLOv8 default)
_COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat',
    'dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack',
    'umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball',
    'kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple',
    'sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair',
    'couch','potted plant','bed','dining table','toilet','tv','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear','hair drier',
    'toothbrush',
]

try:
    import mediapipe as mp
    _MEDIAPIPE_LIB = True
    try:
        from mediapipe.tasks import python as _mp_tasks_python
        from mediapipe.tasks.python import vision as _mp_tasks_vision
        _MP_TASKS = True
    except ImportError:
        _MP_TASKS = False
except ImportError:
    _MEDIAPIPE_LIB = False
    _MP_TASKS = False


class VisionPipeline(Node):
    def __init__(self):
        super().__init__('vision_pipeline')

        if not _CV_BRIDGE:
            self.get_logger().error('cv_bridge not found — install ros-$ROS_DISTRO-cv-bridge')

        # ── Detector enable flags ─────────────────────────────────────────────
        self.yolo_on     = False
        self.mp_pose_on  = False   # body pose (mediapipe key in control JSON)
        self.face_on     = False   # face detection
        self.gesture_on  = False   # hand gesture recognition
        self.aruco_on    = False

        # Frame skip: 1 in N frames processed per detector
        self.yolo_skip    = 5
        self.mp_skip      = 3   # shared by pose, face, gesture
        self.frame_count  = 0

        # ── Detector handles (lazy init) ──────────────────────────────────────
        self.yolo_model     = None
        self.mp_pose        = None
        self.mp_face        = None
        self.mp_gesture     = None
        self.aruco_detector = None

        # ── Camera calibration ────────────────────────────────────────────────
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.marker_size   = 0.05   # metres (5 cm ArUco marker)

        self.bridge = CvBridge() if _CV_BRIDGE else None

        # ── QoS: transient-local so node gets latest control msg on startup ───
        ctrl_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Image,      '/face_camera/image_raw',   self.image_callback,       10)
        self.create_subscription(CameraInfo, '/face_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String,     '/vision/control',     self.control_callback,     ctrl_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self.detections_pub      = self.create_publisher(String,    '/vision/yolo_detections',  10)
        self.pose_pub            = self.create_publisher(String,    '/vision/pose_landmarks',   10)
        self.face_pub            = self.create_publisher(String,    '/vision/face_detections',  10)
        self.gestures_pub        = self.create_publisher(String,    '/vision/gestures',         10)
        self.aruco_pub           = self.create_publisher(PoseArray, '/vision/aruco_markers',    10)
        self.aruco_ids_pub       = self.create_publisher(String,    '/vision/aruco_ids',        10)
        self.yolo_active_pub     = self.create_publisher(Bool,      '/vision/yolo_active',      10)
        self.mp_active_pub       = self.create_publisher(Bool,      '/vision/pose_active',      10)
        self.face_active_pub     = self.create_publisher(Bool,      '/vision/face_active',      10)
        self.gesture_active_pub  = self.create_publisher(Bool,      '/vision/gesture_active',   10)
        self.aruco_active_pub    = self.create_publisher(Bool,      '/vision/aruco_active',     10)
        self.nearest_person_pub  = self.create_publisher(String,    '/vision/nearest_person',   10)

        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Vision Pipeline ready (all detectors off)')

    # ── Control ───────────────────────────────────────────────────────────────

    def control_callback(self, msg: String):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad /vision/control JSON: {e}')
            return

        new_yolo    = bool(cfg.get('yolo',      self.yolo_on))
        new_mp      = bool(cfg.get('pose', self.mp_pose_on))
        new_face    = bool(cfg.get('face',      self.face_on))
        new_gesture = bool(cfg.get('gesture',   self.gesture_on))
        new_aruco   = bool(cfg.get('aruco',     self.aruco_on))

        self._toggle(new_yolo,    self.yolo_on,    self._init_yolo,    self._cleanup_yolo)
        self._toggle(new_mp,      self.mp_pose_on, self._init_pose,    self._cleanup_pose)
        self._toggle(new_face,    self.face_on,    self._init_face,    self._cleanup_face)
        self._toggle(new_gesture, self.gesture_on, self._init_gesture, self._cleanup_gesture)
        self._toggle(new_aruco,   self.aruco_on,   self._init_aruco,   None)

        self.yolo_on    = new_yolo    and self.yolo_model     is not None
        self.mp_pose_on = new_mp      and self.mp_pose        is not None
        self.face_on    = new_face    and self.mp_face        is not None
        self.gesture_on = new_gesture and self.mp_gesture     is not None
        self.aruco_on   = new_aruco   and self.aruco_detector is not None

        self.get_logger().info(
            f'Detectors → yolo:{self.yolo_on} pose:{self.mp_pose_on} '
            f'face:{self.face_on} gesture:{self.gesture_on} aruco:{self.aruco_on}')

    def _toggle(self, want, current, init_fn, cleanup_fn):
        if want and not current:
            init_fn()
        elif not want and current and cleanup_fn:
            cleanup_fn()

    # ── Detector init / cleanup ───────────────────────────────────────────────

    def _init_yolo(self):
        if not _ORT_LIB:
            self.get_logger().error('onnxruntime not installed — pip install onnxruntime')
            return
        onnx_path = os.path.expanduser('~/models/yolov8n.onnx')
        if not os.path.exists(onnx_path):
            self.get_logger().error(
                f'YOLO model not found: {onnx_path}\n'
                'Export on a PC: pip install ultralytics && '
                'yolo export model=yolov8n.pt format=onnx imgsz=320  '
                'then copy yolov8n.onnx to ~/models/ on RPi')
            return
        try:
            sess_opts = ort.SessionOptions()
            sess_opts.inter_op_num_threads = 2
            sess_opts.intra_op_num_threads = 2
            self.yolo_model = ort.InferenceSession(
                onnx_path,
                sess_options=sess_opts,
                providers=['CPUExecutionProvider'],
            )
            self.yolo_input_name  = self.yolo_model.get_inputs()[0].name
            self.yolo_input_size  = 320   # export with imgsz=320 for 2× faster inference
            self.get_logger().info(f'YOLO loaded: {onnx_path} (input {self.yolo_input_size})')
        except Exception as e:
            self.get_logger().error(f'YOLO init failed: {e}')

    def _cleanup_yolo(self):
        self.yolo_model = None

    def _init_pose(self):
        if not _MP_TASKS:
            self.get_logger().error(
                'mediapipe Tasks API not available — pip install -U mediapipe')
            return
        model_path = os.path.expanduser('~/models/pose_landmarker.task')
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Pose landmarker model not found: {model_path}\n'
                'Download: wget -O ~/models/pose_landmarker.task '
                'https://storage.googleapis.com/mediapipe-models/'
                'pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task')
            return
        try:
            base_opts = _mp_tasks_python.BaseOptions(model_asset_path=model_path)
            opts = _mp_tasks_vision.PoseLandmarkerOptions(
                base_options=base_opts,
                num_poses=1,
                min_pose_detection_confidence=0.5,
                min_pose_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.mp_pose = _mp_tasks_vision.PoseLandmarker.create_from_options(opts)
            self.get_logger().info('MediaPipe Pose Landmarker loaded (33 body landmarks)')
        except Exception as e:
            self.get_logger().error(f'Pose init failed: {e}')

    def _cleanup_pose(self):
        if self.mp_pose:
            self.mp_pose.close()
            self.mp_pose = None

    def _init_face(self):
        if not _MP_TASKS:
            self.get_logger().error(
                'mediapipe Tasks API not available — pip install -U mediapipe')
            return
        model_path = os.path.expanduser('~/models/face_detector.tflite')
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Face detector model not found: {model_path}\n'
                'Download: wget -O ~/models/face_detector.tflite '
                'https://storage.googleapis.com/mediapipe-models/'
                'face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite')
            return
        try:
            base_opts = _mp_tasks_python.BaseOptions(model_asset_path=model_path)
            opts = _mp_tasks_vision.FaceDetectorOptions(
                base_options=base_opts,
                min_detection_confidence=0.5,
            )
            self.mp_face = _mp_tasks_vision.FaceDetector.create_from_options(opts)
            self.get_logger().info('MediaPipe Face Detector loaded (Tasks API)')
        except Exception as e:
            self.get_logger().error(f'Face detection init failed: {e}')

    def _cleanup_face(self):
        if self.mp_face:
            self.mp_face.close()
            self.mp_face = None

    def _init_gesture(self):
        if not _MP_TASKS:
            self.get_logger().error(
                'mediapipe Tasks API not available — upgrade: pip install -U mediapipe')
            return
        model_path = os.path.expanduser('~/models/gesture_recognizer.task')
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Gesture model not found: {model_path}\n'
                'Download: https://storage.googleapis.com/mediapipe-models/'
                'gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task')
            return
        try:
            base_opts = _mp_tasks_python.BaseOptions(model_asset_path=model_path)
            opts = _mp_tasks_vision.GestureRecognizerOptions(
                base_options=base_opts,
                num_hands=2,
                min_hand_detection_confidence=0.5,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.mp_gesture = _mp_tasks_vision.GestureRecognizer.create_from_options(opts)
            self.get_logger().info('MediaPipe Gesture Recognizer loaded (up to 2 hands)')
        except Exception as e:
            self.get_logger().error(f'Gesture init failed: {e}')

    def _cleanup_gesture(self):
        if self.mp_gesture:
            self.mp_gesture.close()
            self.mp_gesture = None

    def _init_aruco(self):
        try:
            aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self.get_logger().info('ArUco detector loaded (DICT_4X4_50, 5 cm markers)')
        except Exception as e:
            self.get_logger().error(f'ArUco init failed: {e}')

    # ── Camera info ───────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera calibration received')

    # ── Main image callback ───────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        any_active = self.yolo_on or self.mp_pose_on or self.face_on or self.gesture_on or self.aruco_on
        if not any_active or self.bridge is None:
            return

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        tick = self.frame_count

        if self.yolo_on    and self.yolo_model     and tick % self.yolo_skip == 0:
            self._run_yolo(frame)

        # Pose, face, gesture share the same RGB conversion — do it once if any is needed
        mp_tick = tick % self.mp_skip == 0
        if mp_tick and (self.mp_pose_on or self.face_on or self.gesture_on):
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if self.mp_pose_on  and self.mp_pose:
                self._run_pose(rgb)
            if self.face_on     and self.mp_face:
                self._run_face(rgb)
            if self.gesture_on  and self.mp_gesture:
                self._run_gesture(rgb)

        if self.aruco_on and self.aruco_detector:
            self._run_aruco(frame)

    # ── Detector runners ──────────────────────────────────────────────────────

    def _run_yolo(self, frame):
        try:
            sz = self.yolo_input_size
            # Preprocess: resize → RGB → normalise → CHW → batch
            img = cv2.resize(frame, (sz, sz))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            img = img.transpose(2, 0, 1)[np.newaxis]   # [1, 3, sz, sz]

            output = self.yolo_model.run(None, {self.yolo_input_name: img})[0]
            # YOLOv8 output: [1, 84, 8400]  (4 xywh + 80 class scores)
            preds = output[0].T   # [8400, 84]
            cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
            class_scores  = preds[:, 4:]
            class_ids     = np.argmax(class_scores, axis=1)
            confidences   = class_scores[np.arange(len(class_scores)), class_ids]

            mask = confidences > 0.5
            # Convert to x1y1wh (top-left) for NMS
            boxes_nms = np.stack([
                cx[mask] - w[mask] / 2,
                cy[mask] - h[mask] / 2,
                w[mask],
                h[mask],
            ], axis=1).tolist()
            confs_nms = confidences[mask].tolist()
            # Apply NMS per-class to remove duplicate detections
            indices = cv2.dnn.NMSBoxes(boxes_nms, confs_nms, score_threshold=0.5, nms_threshold=0.45)
            indices = indices.flatten() if len(indices) > 0 else []

            masked_cls  = class_ids[mask]
            masked_conf = confidences[mask]
            masked_cx   = cx[mask]
            masked_cy   = cy[mask]
            masked_w    = w[mask]
            masked_h    = h[mask]

            detections = []
            for i in indices:
                detections.append({
                    'class':      _COCO_CLASSES[int(masked_cls[i])],
                    'confidence': round(float(masked_conf[i]), 3),
                    'bbox':       [round(float(v) / sz, 4) for v in (
                                    masked_cx[i], masked_cy[i], masked_w[i], masked_h[i])],
                })
            msg = String()
            msg.data = json.dumps(detections)
            self.detections_pub.publish(msg)

            # ── Nearest person tracking ───────────────────────────────────────
            img_h, img_w = frame.shape[:2]
            # Use calibrated focal length only if valid (>0); Gazebo sends uncalibrated K=zeros
            if self.camera_matrix is not None and self.camera_matrix[0, 0] > 10.0:
                fx = self.camera_matrix[0, 0]
            else:
                fx = img_w * 1.1   # rough estimate for ~90° FOV
            persons = [d for d in detections if d['class'] == 'person']
            nearest = None
            for i, p in enumerate(persons):
                bw_px = p['bbox'][2] * img_w   # normalised width → pixels
                if bw_px < 1:
                    continue
                dist  = round(_PERSON_WIDTH_M * fx / bw_px, 2)
                cx_offset = (p['bbox'][0] - 0.5) * img_w   # px from image centre
                angle = round(math.atan2(cx_offset, fx), 4)
                if nearest is None or dist < nearest['distance']:
                    nearest = {'distance': dist, 'angle': angle, 'id': i}
            np_msg = String()
            np_msg.data = json.dumps(nearest)
            self.nearest_person_pub.publish(np_msg)
        except Exception as e:
            self.get_logger().error(f'YOLO error: {e}')

    def _run_pose(self, rgb):
        try:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = self.mp_pose.detect(mp_image)
            if not result.pose_landmarks:
                msg = String()
                msg.data = json.dumps({'pose_landmarks': []})
                self.pose_pub.publish(msg)
                return
            landmarks = [
                {'x': lm.x, 'y': lm.y, 'z': lm.z, 'vis': round(lm.visibility, 3)}
                for lm in result.pose_landmarks[0]
            ]
            msg = String()
            msg.data = json.dumps({'pose_landmarks': landmarks})
            self.pose_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Pose error: {e}')

    def _run_face(self, rgb):
        try:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = self.mp_face.detect(mp_image)
            h, w = rgb.shape[:2]
            faces = []
            if not result.detections:
                msg = String()
                msg.data = json.dumps(faces)
                self.face_pub.publish(msg)
                return
            for det in result.detections:
                bb = det.bounding_box
                # keypoint order: right_eye, left_eye, nose_tip, mouth, right_ear, left_ear
                faces.append({
                    'confidence': round(det.categories[0].score, 3),
                    'bbox': {
                        'xmin':   round(bb.origin_x / w, 4),
                        'ymin':   round(bb.origin_y / h, 4),
                        'width':  round(bb.width    / w, 4),
                        'height': round(bb.height   / h, 4),
                    },
                    'keypoints': [
                        {'x': round(kp.x, 4), 'y': round(kp.y, 4)}
                        for kp in (det.keypoints or [])
                    ],
                })
            msg = String()
            msg.data = json.dumps(faces)
            self.face_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Face detection error: {e}')

    def _run_gesture(self, rgb):
        try:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result   = self.mp_gesture.recognize(mp_image)

            if not result.gestures:
                return

            hands = []
            for i, gesture_list in enumerate(result.gestures):
                if not gesture_list:
                    continue
                top = gesture_list[0]
                hand_side = 'Unknown'
                if result.handedness and i < len(result.handedness):
                    hand_side = result.handedness[i][0].display_name  # 'Left' or 'Right'

                hand_data = {
                    'gesture':    top.category_name,   # e.g. Thumb_Up, Victory, Open_Palm
                    'confidence': round(top.score, 3),
                    'hand':       hand_side,
                }

                # Include 21 hand landmarks for arm/gripper control use
                if result.hand_landmarks and i < len(result.hand_landmarks):
                    hand_data['landmarks'] = [
                        {'x': round(lm.x, 4), 'y': round(lm.y, 4), 'z': round(lm.z, 4)}
                        for lm in result.hand_landmarks[i]
                    ]
                hands.append(hand_data)

            if hands:
                msg = String()
                msg.data = json.dumps(hands)
                self.gestures_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Gesture error: {e}')

    def _run_aruco(self, frame):
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                return

            pose_array = PoseArray()
            pose_array.header.stamp    = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'face_camera_link_optical'
            marker_ids = []

            if self.camera_matrix is not None:
                obj_pts = np.array([
                    [-self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2, -self.marker_size / 2, 0],
                    [-self.marker_size / 2, -self.marker_size / 2, 0],
                ], dtype=np.float32)

                for i, corner in enumerate(corners):
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_pts, corner[0].astype(np.float32),
                        self.camera_matrix, self.dist_coeffs)
                    if not ok:
                        continue
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = self._rot_to_quat(rot_mat)
                    pose = Pose()
                    pose.position.x    = float(tvec[0])
                    pose.position.y    = float(tvec[1])
                    pose.position.z    = float(tvec[2])
                    pose.orientation.x = qx
                    pose.orientation.y = qy
                    pose.orientation.z = qz
                    pose.orientation.w = qw
                    pose_array.poses.append(pose)
                    marker_ids.append(int(ids[i][0]))

            if pose_array.poses:
                self.aruco_pub.publish(pose_array)
                ids_msg = String()
                ids_msg.data = json.dumps(marker_ids)
                self.aruco_ids_pub.publish(ids_msg)
        except Exception as e:
            self.get_logger().error(f'ArUco error: {e}')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rot_to_quat(self, r):
        trace = r[0,0] + r[1,1] + r[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            return ((r[2,1]-r[1,2])*s, (r[0,2]-r[2,0])*s, (r[1,0]-r[0,1])*s, 0.25/s)
        elif r[0,0] > r[1,1] and r[0,0] > r[2,2]:
            s = 2.0 * math.sqrt(1.0 + r[0,0] - r[1,1] - r[2,2])
            return (0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s)
        elif r[1,1] > r[2,2]:
            s = 2.0 * math.sqrt(1.0 + r[1,1] - r[0,0] - r[2,2])
            return ((r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s)
        else:
            s = 2.0 * math.sqrt(1.0 + r[2,2] - r[0,0] - r[1,1])
            return ((r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s, (r[1,0]-r[0,1])/s)

    def publish_status(self):
        for pub, val in [
            (self.yolo_active_pub,    self.yolo_on),
            (self.mp_active_pub,      self.mp_pose_on),
            (self.face_active_pub,    self.face_on),
            (self.gesture_active_pub, self.gesture_on),
            (self.aruco_active_pub,   self.aruco_on),
        ]:
            m = Bool()
            m.data = val
            pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_pose()
        node._cleanup_face()
        node._cleanup_gesture()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
