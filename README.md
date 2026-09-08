# bonicbot-a2-ros

ROS2 Humble stack for the **BonicBot A2 Pro** — differential-drive base with two arms,
grippers and a neck, driven by an ESP32-S3 over USB CDC, on a Raspberry Pi 4.

- **Architecture and design rules:** [CLAUDE.md](./CLAUDE.md)
- **Restructure / protocol migration:** [bonicbot_a2_restructure_plan.md](./bonicbot_a2_restructure_plan.md)

> **Verified in simulation** (2026-08-23): builds clean, all seven controllers activate,
> camera/LiDAR/EKF publish, and mapping → save → navigation works end to end driven by
> `robot_app`.
>
> ⚠️ **Nothing has run on real hardware.** The whole ESP32 USB CDC path — servo IDs,
> angle convention, inversion, encoders, battery — is untested. Read the status banner in
> [CLAUDE.md](./CLAUDE.md) before driving the robot.

---

## Packages

| Package | Location | Role |
|---|---|---|
| `bonicbot_a2_hardware` | `src/hardware/` | Owns every `/dev/*`: ESP32 CDC link (wheels, servos, IMU, battery, Wi-Fi relay), RPLIDAR, camera |
| `bonicbot_a2_description` | `src/nav/` | URDF/xacro + meshes |
| `bonicbot_a2_nav` | `src/nav/` | SLAM, Nav2, EKF, teleop, vision pipeline |
| `bonicbot_a2_sim` | `src/sim/` | Gazebo — dev only, never on the robot |

`src/hardware/` and `src/nav/` never import from each other; they talk over ROS2 topics
only. `reference/` holds app-layer scripts that are **not built** — see
[reference/README.md](./reference/README.md).

---

## Build

```bash
cd bonicbot-a2-ros
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Build one group only:

```bash
colcon build --packages-up-to bonicbot_a2_hardware   # hardware
colcon build --packages-up-to bonicbot_a2_nav        # nav
colcon build --packages-select bonicbot_a2_sim       # sim
```

---

## Run — simulation

No hardware needed. Gazebo replaces the ESP32 entirely (`ros2_control.xacro` binds
`gz_ros2_control` when `sim_mode:=true`), so the CDC protocol is never involved.

Backgrounded, with logs and a preflight that refuses to start on top of a
previous session (**recommended**):

```bash
./start_session.sh            # --clean to tear down a previous session first
./stop_session.sh             # when done
```

Or in the foreground, two terminals:

```bash
# Terminal 1 — Gazebo, controllers, EKF, teleop
ros2 launch bonicbot_a2_sim sim.launch.py

# Terminal 2 — navigation (or slam.launch.py to map)
ros2 launch bonicbot_a2_nav navigation.launch.py use_sim_time:=true
```

> **Ctrl+C does not reliably stop a `ros2 launch` tree.** Orphaned nodes break
> the *next* session in ways that look like anything but a stale process — a
> leftover `robot_state_publisher` can feed `gz_ros2_control` the wrong robot's
> URDF, so every controller fails to activate with "Not existing" interfaces.
> Use `./stop_session.sh`; it sweeps and then verifies.

Options: `world:=obstacle_world.sdf`, `use_real_camera:=True` (uses a laptop webcam
instead of the simulated one — both publish `/face_camera/image_raw`).

---

## Run — real robot

One-time per robot:

```bash
sudo ./scripts/setup_udev.sh     # creates /dev/esp and /dev/lidar
```

Then, backgrounded with device preflight and logs (**recommended**):

```bash
./start_session_robot.sh      # --clean to tear down a previous session first
./stop_session.sh             # when done
```

Or in the foreground, two terminals:

```bash
# Terminal 1 — hardware: controller_manager + ESP link + lidar + camera
ros2 launch bonicbot_a2_hardware hardware.launch.py

# Terminal 2 — navigation
ros2 launch bonicbot_a2_nav bringup.launch.py use_sim_time:=false slam:=false
```

`hardware.launch.py` args: `use_camera`, `use_lidar`, `use_joystick` (all default `true`).

### Mapping

```bash
ros2 launch bonicbot_a2_nav bringup.launch.py slam:=true use_nav2:=false
# drive around with the joystick, then:
ros2 run nav2_map_server map_saver_cli -f $BONICBOT_MAPS_DIR/bonicbot_a2_map
```

### Navigating a saved map

```bash
ros2 launch bonicbot_a2_nav bringup.launch.py slam:=false map_name:=bonicbot_a2_map.yaml
```

> **`slam:=true` and `slam:=false` are mutually exclusive by design.** `slam_toolbox` and
> `nav2_amcl` both publish `map→odom`; running them together gives the transform tree two
> parents for the same frame and the robot's pose flips between them. `bringup.launch.py`
> forwards the flag to `navigation.launch.py` so they can never disagree.

Maps live in `$BONICBOT_MAPS_DIR` (default `/maps`, the Docker volume). For bare-metal
dev, export it: `export BONICBOT_MAPS_DIR=$PWD/maps`.

> `map_name` **includes the `.yaml` extension** — same convention as
> `bonicbot_m1_nav`, and what robot_app's NavModeManager passes.

---

## Run — Docker (production)

```bash
echo "ROBOT_ID=A2_001" > .env
docker compose up -d
```

Three services: `bonicbot_ros` (hardware), `bonicbot_nav` (SLAM/Nav2 — same image,
separate process so Nav2 can restart without cycling the hardware link), and `robot_app`
(shared with M1, `ROBOT_SERIES=A`). All `restart: always`.

---

## Vision models (optional)

Only needed for `vision_pipeline.py`. Already present on the robot.

```bash
mkdir -p ~/models
wget -O ~/models/pose_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task
wget -O ~/models/face_detector.tflite \
  https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite
wget -O ~/models/gesture_recognizer.task \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task

# YOLOv8n must be exported at imgsz=320 — no pre-built download
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320
mv yolov8n.onnx ~/models/
```

Start it with `ros2 launch bonicbot_a2_nav bringup.launch.py use_vision:=true`.

> `vision_pipeline.py` expects someone to publish `/vision/control` to enable individual
> detectors. That publisher was `robot_manager.py`, which now lives in `reference/` and
> is not built — until `robot_app` provides it, every detector stays off.

---

## Key topics

| Topic | Direction | Notes |
|---|---|---|
| `/scan` | lidar → nav | RPLIDAR C1M1 |
| `/imu/data` | ESP → nav | Polled over CDC (**not** `/esp/imu` — that's M1) |
| `/diff_cont/odom` | ESP encoders → nav | diff_drive_controller |
| `/odometry/filtered` | EKF | Owns `odom→base_link` |
| `/joint_states` | ESP → nav | Wheels + 7 servos |
| `/battery_state` | ESP → app | From `RESP_BATTERY` |
| `/cmd_vel` | nav → hardware | twist_mux prio 10 |
| `/cmd_vel_joy` | joystick → hardware | twist_mux prio 100 (always wins) |
| `/{left,right}_arm_controller/commands` | app → hardware | `Float64MultiArray` |
| `/face_camera/image_raw` | camera → nav/app | Head camera. Same name M1 uses, so robot_app addresses both series alike |
| `/esp/wifi_credentials` | ESP → robot_app | Phone wrote them over BLE |
| `/esp/wifi_status` | robot_app → ESP | Replies to the phone |

---

## Troubleshooting

**ESP not found**
```bash
ls -l /dev/esp                       # should symlink to ttyACM*
udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"
```
If the IDs differ from `config/udev/99-bonicbot.rules`, update the rules and re-run
`scripts/setup_udev.sh`.

**Robot doesn't move**
```bash
ros2 control list_controllers          # all should be 'active'
ros2 topic echo /joint_states --once   # encoders updating?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
```
A controller stuck in `loaded` (not `active`) usually means its spawner timed out against
a cold `controller_manager` — the launch files pass `--service-call-timeout 60` for
exactly this, so if it still happens the manager is failing to configure, not just slow.

**Pose jumps around**
```bash
ros2 run tf2_tools view_frames         # exactly ONE publisher of map->odom
```
Two means `slam_toolbox` and `amcl` are both running — check the `slam` arg.

**Camera missing** — if `/dev/video0` doesn't exist and the module is a **Camera Module 3
(IMX708)**, it is libcamera-only and will never appear as a v4l2 device. `v4l2_camera`
needs Module 2 (IMX219).

**Camera won't start, or the whole Pi bogs down** — `camera.launch.py` needs `v4l-utils`:

```bash
sudo apt install -y v4l-utils
```

It shells out to `v4l2-ctl --set-parm` to set the capture rate, because
`v4l2_camera_node` has **no frame-rate parameter** — `ros2 param list` on it shows
`image_size`, `pixel_format`, `output_encoding` and the V4L2 controls, and nothing for
timing. Without this the camera free-runs at its 30 fps default and costs **105% of a
core** instead of 17%, which saturates the RPi4 and makes even SSH sluggish. Verify with:

```bash
timeout 10 ros2 topic hz /face_camera/image_raw   # expect ~6, not 30
```

**SSH freezes while `top` shows the Pi idle** — that is the network, not the CPU. Raw
camera frames are ~5.5 MB/s, and if `ROS_LOCALHOST_ONLY` is unset, DDS pushes them over
WiFi to every subscriber on the network. An RViz session on another machine will do it:

```bash
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"   # should be 1 on the robot
```

Set it to `1` for robot-only operation. If you genuinely need RViz from a dev machine,
leave it unset but subscribe to `/face_camera/image_raw/compressed` rather than
`image_raw` — roughly 50x less data over the link.

---

*BonicBot A2 — Autobonics Pvt Ltd. Confidential and proprietary.*
