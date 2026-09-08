# bonicbot-a2-ros
## BonicBot A2 Series ROS2 Stack | Autobonics Pvt Ltd | Confidential

---

> ## Status — hardware verified for manual control, navigation, and robot_app
>
> **Verified on real A2 Pro hardware (2026-08-29).** The ESP32 USB CDC path is
> exercised and working: base drive in all six directions, all 7 servos through their
> full min/mid/max range, battery, IMU, face LED matrix, hot-unplug recovery, and
> Nav2 navigation (SLAM and saved-map/AMCL) including near-wall and narrow-passage
> goals. Direction was verified row-by-row against simulation using
> **[bonicbot_a2_manual_control_reference.md](./bonicbot_a2_manual_control_reference.md)**,
> which carries the full command matrix and the sim-vs-hardware results table.
>
> Everything the previous banner listed as unverified has now been checked on hardware:
>
> | Item | Outcome |
> |---|---|
> | Servo angle convention (signed degrees on the wire) | Correct. Full signed ranges reach the servos. |
> | Servo inversion table | `isServoInverted()` returns false for every ID — the unified firmware handles mirroring itself. The one real direction bug was in the URDF, not here: elbow axes were reversed (physical arm right, RViz mirrored) and are fixed. |
> | Encoder polarity/scaling | Correct. `/diff_cont/odom` signs match commanded motion on all six drive tests. |
> | `RESP_BATTERY` (0x52) over CDC | Publishes. Voltage, current and SOC are live; the rest stays at defaults. |
>
> **`robot_app` now runs on this hardware (2026-08-29).** `start_session_robot.sh`
> brings the base stack up, `robot_app` adopts it, and the frontend navigation tab
> drives and maps end to end: WebRTC peer connects, teleop moves the base (deadman
> zeroes `cmd_vel` on frame loss as designed), mapping mode starts `slam_toolbox`
> from the UI, and `/map` publishes at a steady 0.5 Hz. The
> `/odom` -> `/odometry/filtered` fix is **verified** — `/odometry/filtered`
> publishes at ~10 Hz on the real robot.
>
> Getting there took four `robot_app` fixes, all in `bonicOS-robot-app`, none in
> this repo — the hardware/nav split here was right and robot_app assumed a shape
> it does not have:
>
> | Bug | Fix |
> |---|---|
> | `BaseSessionManager` self-started `<nav_pkg> bringup.launch.py` and then health-checked for `robot_state_publisher` + `controller_manager`. That pair is the **`sim.launch.py` signature** — in Gazebo one launch file is the whole base stack. Here it is two, and `bringup.launch.py` owns no hardware, so the gate could never pass on a real robot. | `base_autostart` now defaults true in sim, **false on real hardware**. The operator (`start_session_robot.sh`) owns the base stack; robot_app adopts. |
> | The failed self-start rolled back through `BASE_SWEEP_CMD` = `stop_session.sh` — a `pkill -f` that cannot tell robot_app's own spawn from a live stack. It killed `controller_manager`, `robot_state_publisher`, the lidar and the camera on every boot. | The health-gate rollback no longer sweeps. It killed its own process group already; anything else matching belongs to someone else. |
> | `install-service.sh` wrote `ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}`, but ROS's own `setup.bash` exports `0` when unset — so the fallback never fired and the installer captured the shell's `0`. robot_app sat on a different discovery scope than the stack (which pins `1`) and a freshly-started robot_app **could not see an already-running base stack at all**, reporting `base_nodes_missing` indefinitely while `ros2 node list` showed both nodes. | Pinned to `1` in the generated env file. Keep `export ROS_LOCALHOST_ONLY=1` in your shell too, or every `ros2` CLI reading is on the wrong scope and will lie to you. |
> | `NavModeManager._teardown()` killed only the process handle robot_app was holding, returning silently when it had none. But a nav session **outlives a robot_app restart by design** (unit is `KillMode=process`, sessions are `setsid`'d), so after a restart the handle is gone while the session runs on. `resume_from_disk` then spawned a second one: **two complete Nav2 stacks, two owners of `map->odom`** — the exact invariant that module exists to protect. Load average went 7.8 → 21.4. | `NavModeManager` got the graph-based detection `BaseSessionManager` already had: `detect_mode()` reads `amcl`/`slam_toolbox` from the graph, `resume_from_disk` **adopts** instead of relaunching, and `_switch`/`stop` refuse rather than stack a second owner. |
>
> **Saved-map navigation has run** (`navigation.launch.py` + AMCL + pose seeding,
> map `classroom_test`), but not cleanly — see the CPU note below. Goals failed two
> ways: `off the global costmap` (goals outside the mapped area — a UI/operator
> issue, planning there can never succeed) and TF-lag failures downstream of a
> starved EKF.
>
> **Under sustained load the Pi runs out of CPU.** After ~4 h with SLAM/Nav2 +
> camera + robot_app, load average sat at 7.8–9.0 on 4 cores (22% idle) with
> `robot_app` itself at 60–90% — against the ~17% idle this file documents. The
> consequence chain is worth recognising: `ekf_node` logs `Failed to meet update
> rate` (0.10–0.28 s against a 0.1 s budget) → `odom->base_link` publishes late →
> `amcl` logs `Lookup would require extrapolation into the future` →
> `controller_server: Failed to make progress`. **Not thermal** — `vcgencmd
> get_throttled` was `0x0`. robot_app's share is now explained: about half of it is
> rclpy executor spin, not robot_app's own callbacks — see the CPU notes near the
> end of this file, including the two "optimizations" that measurably did nothing.
>
> **robot_app now self-heals a base stack restart it did not perform.** Running
> `stop_session.sh` + `start_session_robot.sh` by hand used to leave it permanently
> poseless — three separate bugs in one chain, all fixed 2026-08-29: its TF buffer
> was never rebuilt, its nav session was never restored (so no AMCL, no `map`
> frame), and the externally-killed session was recorded as "the operator went
> idle", which silently disabled the restore. None of it was findable until
> `_poll_pose` stopped swallowing its TF exception in silence — the log line
> naming `LookupException` vs `ExtrapolationException` is what separated "stale
> buffer" from "no AMCL at all". Verified end to end: `base stack reappeared` →
> `restoring the nav session` → `resuming navigation on saved map` → `localized` →
> `map -> base_footprint resolved again`, in ~23 s with no robot_app restart.
>
> **Still untested:** `robot_app` on the Gazebo sim, and Docker.
>
> **Controller note:** navigation runs on `RegulatedPurePursuitController`, not DWB.
> DWB was replaced after its own `/evaluation` output showed two failures internal to
> it — the Oscillation critic banning an entire rotation direction, and a scoring local
> minimum where standing still outscored driving. See the CPU/tuning notes near the end
> of this file.
>
> Migration steps: **[bonicbot_a2_restructure_plan.md](./bonicbot_a2_restructure_plan.md)**.

---

## Overview

ROS2 stack for the BonicBot A2 Pro. Single colcon workspace, packages grouped into
`hardware` (owns `/dev/*`) and `nav` (no hardware access) — the same split
[bonicOS-m1-ros](../bonicOS-m1-ros/bonicos_m1_ros_overview.md) uses, so the two series
stay structurally readable side by side.

**Where A2 fundamentally differs from M1:** on M1 the Jetson drives all motion hardware
directly (ODrive over CAN, QDD actuators over native CAN, Feetech servos over USB), and
the ESP32 is a thin relay handling only IMU/Wi-Fi/ping. **On A2 the ESP32 *is* the motion
controller** — wheels (PWM + encoders), all 7 servos, and the IMU sit behind it, reached
over one USB CDC link. There is no CAN bus on A2. So A2's `ros2_control` hardware
interface is an *active CDC master*, not a relay, and A2 needs no per-bus driver packages
(`odrive_ros2_control`, `robstride_hardware_interface`, `feetech_ros2_driver` are all
M1-only).

---

## Robot Hardware — A2 Pro

| Component | Detail |
|---|---|
| Processor | Raspberry Pi 4 — 8GB RAM |
| OS | Ubuntu 22.04 (64-bit) |
| ROS2 | Humble |
| ESP MCU | Custom Autobonics controller board — ESP32-S3, USB CDC-ACM |
| Drive | H-bridge PWM, closed-loop in ESP firmware — **ESP-mediated** |
| Encoders | Wheel encoders → ESP (`encoder_cpr` 14040, wheel radius 0.06 m, separation 0.2895 m) |
| Servos | Waveshare serial servos — **ESP-mediated**, 7 fitted (see registry below) |
| IMU | On ESP board — polled over CDC |
| LiDAR | RPLIDAR C1M1 — USB, **Pi-direct** (`/dev/lidar`) |
| Camera | CSI camera module in the head — v4l2, **Pi-direct** (`/dev/video0`), published as `face_camera` |
| Battery | 4400 mAh — SOC/voltage/current from ESP |
| Phone | Android — Flutter app (BonicOS UI layer), BLE to ESP + WSS to robot_app |

> **A2 Lite** has no RPi4, no LiDAR, no ROS2 — phone + ESP only, manual drive. This repo
> is A2 **Pro** only.

### Actuator registry — A2 fitment

A2 fits **7 of the 18** canonical actuator IDs from
[bonicbot_actuator_naming.md](../bonicOS-flutter/docs/bonicbot_actuator_naming.md)
(active A-series BLE IDs `0, 4, 7, 8, 11, 15, 16`). All are standard serial servos — no
QDD actuators on A2.

| ID | Registry name | ROS joint | A2 limit |
|---|---|---|---|
| 0 | `rightGripper` | `right_gripper_finger1_joint` | −45° … 60° |
| 4 | `rightElbow` | `right_elbow_joint` | −50° … 0° |
| 7 | `rightShoulderPitch` | `right_shoulder_pitch_joint` | −45° … 180° |
| 8 | `leftShoulderPitch` | `left_shoulder_pitch_joint` | −45° … 180° |
| 11 | `leftElbow` | `left_elbow_joint` | −50° … 0° |
| 15 | `leftGripper` | `left_gripper_finger1_joint` | −45° … 60° |
| 16 | `neckYaw` | `neck_yaw_joint` | −90° … 90° |

**Gripper finger numbering matches M1 deliberately:** `finger1` is the driven joint,
`finger2` mimics it (URDF `mimic`, never commanded), and `finger3` is A2's fixed jaw —
A2 has three fingers where M1 has two. They were renumbered on 2026-08-23 (A2 previously
drove `finger2`) so that robot_app and the `bonicos` SDK can share ONE registry→joint
table. While they disagreed, gripper commands silently no-op'd on A2 and
`get_servo_angles()` returned a missing key that renders as `0.0deg`, because the joint
those tools named was A2's *fixed* jaw — not actuated, and absent from `/joint_states`.
**Keep this aligned with M1.**

> **These registry IDs go on the wire directly.** The firmware indexes servos by the
> canonical registry, so `CMD_SERVO_MULTI`'s Servo ID byte carries the ID from the table
> above — no translation layer. This **replaces** the old ROS-side scheme (`servo_id`
> 1/3/6/7/9/12/13 → `id − 1`), which matched the pre-migration firmware only.

---

## Repository Structure

The repo root **is** the colcon workspace. Motion hardware runs through a **ros2_control
hardware-interface plugin**, not a standalone driver node.

```
bonicbot-a2-ros/
├── CLAUDE.md
├── bonicbot_a2_restructure_plan.md
├── Dockerfile.ros                 # one image — hardware + nav (RPi4 constraint)
├── docker-compose.yml
├── config/udev/99-bonicbot.rules
├── maps/
├── start_session.sh               # sim base stack, backgrounded + preflight
├── start_session_robot.sh         # real-robot base stack
├── stop_session.sh                # sweep + verify (Ctrl+C is not enough)
├── scripts/setup_udev.sh
│
└── src/
    ├── hardware/                                  # owns EVERY /dev/* device
    │   └── bonicbot_a2_hardware/                  # ament_cmake
    │       ├── hardware/esp_hardware_interface.cpp    # ros2_control SystemInterface
    │       ├── hardware/include/bonicbot_a2_hardware/
    │       │   ├── esp_hardware_interface.hpp
    │       │   └── cdc_protocol.hpp               # AA 55 framing, opcode constants
    │       ├── bonicbot_a2_hardware.xml           # pluginlib descriptor
    │       ├── config/{controllers.yaml, twist_mux.yaml}
    │       └── launch/
    │           ├── hardware.launch.py             # controller_manager + spawners,
    │           │                                  # includes the two below
    │           ├── rplidar.launch.py              # /dev/lidar  → /scan
    │           └── camera.launch.py               # /dev/video0 → /face_camera/image_raw
    │
    ├── nav/                                       # no /dev/* access at all
    │   ├── bonicbot_a2_description/               # URDF/xacro + meshes, rsp launch
    │   └── bonicbot_a2_nav/
    │       ├── config/{nav2_params.yaml, ekf.yaml, ekf_imu.yaml, joystick.yaml,
    │       │           mapper_params_online_async.yaml}
    │       ├── launch/{bringup, slam, navigation, mapping, joystick}.launch.py
    │       ├── rviz/
    │       └── scripts/{vision_pipeline.py, object_follower.py}
    │
    └── sim/                                       # dev only — never on the robot
        └── bonicbot_a2_sim/
            ├── config/{gazebo_params.yaml, nav2_params_sim.yaml}   # nav2_params_sim.yaml UNUSED — to be deleted
            ├── launch/sim.launch.py
            └── worlds/*.sdf
```

**Package naming:** it's `bonicbot_a2_hardware`, not `..._esp_bridge`, because it owns the
LiDAR and camera too — both are `/dev/*` devices, and those have nothing to do with the
ESP. M1 can name its package `esp_bridge` because there it genuinely only handles the ESP
(lidar/cameras live in `bonicbot_m1_hardware_bringup`). The ESP-specific code is the
`esp_hardware_interface.*` files inside.

> **App-layer logic is not in this repo** — orchestration (nav goals, mapping lifecycle,
> map management, sessions, cloud) belongs to `bonicOS-robot-app`, which is shared with
> M1 and already has a `ROBOT_SERIES=A` config. `robot_manager.py` and `robot_agent.py`
> therefore **leave the build**: they now sit in `reference/` (with `agent_config.yaml`),
> out of every package, ready to move to robot_app as inputs for its A-series
> implementation — exactly as M1 did with its own `robot_manager.py`. They are kept
> rather than deleted because robot_app has not absorbed their functionality yet.
>
> `vision_pipeline.py` and `object_follower.py` **stay** — they consume camera topics and
> publish `/vision/*` and `/cmd_vel`, which is ROS-layer perception, not orchestration.
> M1 keeps the same two scripts in its nav package for the same reason.

### Why one ESP package, not M1's two

M1 splits `bonicbot_m1_hardware_bringup` (C++ ros2_control) from
`bonicbot_m1_esp_bridge` (Python IMU/Wi-Fi relay) because it has several distinct physical
buses needing separate driver packages, with the ESP handling only leftovers. A2 has
exactly one bus — the ESP does everything — so one package named for what it actually is:
**the bridge to the ESP**.

### Why the Wi-Fi relay is C++, inside the plugin

M1's Wi-Fi relay is a standalone Python node because there it is the *only* process
touching the CDC port. On A2 the ros2_control plugin already reads/writes that same port
continuously at 50 Hz for motors/servos/IMU. **Two processes sharing one CDC byte stream
would interleave and split frames**, so the relay cannot be a separate node. Its topics
are served from the internal `rclcpp::Node` the plugin already creates and spins for IMU
publishing (`SystemInterface` has no node handle of its own, so it makes one).

**One process owns `/dev/esp`. No exceptions.**

---

## Container Architecture

Two containers — **not** M1's three. The RPi4 can't justify separating hardware from nav,
so both package groups build into a single `bonicbot_ros` image. The `src/hardware`
vs `src/nav` source split is still enforced (see Key Design Rules); it just isn't a
container boundary.

```
RPi4 — A2 Pro

┌──────────────────────────────────────────────┐
│ bonicbot_ros            ros:humble-ros-base  │  privileged, restart: always
│                                              │
│  bonicbot_a2_hardware        — owns /dev/*   │
│   ├── controller_manager + controllers.yaml  │
│   │    ├── esp_hardware_interface (/dev/esp) │  ← wheels, 7 servos, IMU over USB CDC
│   │    ├── diff_cont → /diff_cont/odom       │
│   │    ├── joint_broad → /joint_states       │
│   │    └── arm/head/gripper position ctrls   │
│   ├── twist_mux  (/cmd_vel, /cmd_vel_joy)    │
│   ├── rplidar_ros  (/dev/lidar)  → /scan     │
│   └── v4l2_camera  (/dev/video0) → /camera/* │
│                                              │
│  bonicbot_a2_description → robot_state_pub   │
│  bonicbot_a2_nav             — no /dev/*     │
│   ├── slam_toolbox | nav2_amcl (never both)  │
│   ├── Nav2, EKF (robot_localization)         │
│   └── vision_pipeline / object_follower      │
└──────────────────────────────────────────────┘

┌──────────────────────────────────────────────┐
│ robot_app               autobonics/robot-app │  restart: always
│ ROBOT_SERIES=A          ← owns orchestration │
│  WSS (phone + BonicAI) · Firebase · LiveKit  │
│  nav goals · mapping · map mgmt · sessions   │
│  rclpy ROS2 bridge · nmcli · update manager  │
└──────────────────────────────────────────────┘
```

`robot_app` is **shared with M1** — same image, same core, series-dispatched by
`ROBOT_SERIES`. Its A-series config already exists (`app/config.py`). M-series adds
features on top (depth camera, on-device LLM); the core is common. That repo is not this
repo's concern beyond the topic contract below.

---

## ROS2 Topic Map

### hardware → nav (sensor data)

| Topic | Type | Rate | Source |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | RPLIDAR C1M1 (Pi-direct) |
| `/imu/data` | `sensor_msgs/Imu` | ~10 Hz (polled) | ESP IMU (`CMD_IMU_REQUEST` → `RESP_IMU`) |
| `/diff_cont/odom` | `nav_msgs/Odometry` | 50 Hz | diff_drive_controller (ESP encoders) |
| `/joint_states` | `sensor_msgs/JointState` | 50 Hz | joint_state_broadcaster (wheels + 7 servos) |
| `/face_camera/image_raw` | `sensor_msgs/Image` | ~6 fps | CSI head camera (v4l2) |
| `/odometry/filtered` | `nav_msgs/Odometry` | 10 Hz | EKF — owns `odom→base_link` TF |
| `/battery_state` | `sensor_msgs/BatteryState` | on `RESP_BATTERY` | ESP battery (`percentage` is 0..1; the frame's SOC is 0-100) |

> A2 publishes IMU on **`/imu/data`**, not M1's `/esp/imu` — matches `robot_app`'s
> existing A-series topic config and `ekf.yaml`'s `imu0`.

### nav → hardware (commands)

| Topic | Type | Destination |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | twist_mux (prio 10) → `/diff_cont/cmd_vel_unstamped` |
| `/cmd_vel_joy` | `geometry_msgs/Twist` | twist_mux (prio 100 — joystick always wins) |
| `/{left,right}_arm_controller/commands` | `std_msgs/Float64MultiArray` | shoulder_pitch + elbow |
| `/head_controller/commands` | `std_msgs/Float64MultiArray` | neck_yaw |
| `/{left,right}_gripper_controller/commands` | `std_msgs/Float64MultiArray` | gripper finger1 (driven; finger2 mimics, finger3 is the fixed jaw) |

> A2 uses `position_controllers/JointGroupPositionController` (plain
> `Float64MultiArray` on `/…/commands`). M1 uses `JointTrajectoryController` for arms
> (`JointTrajectory` with timing). Do not copy M1's arm-command code onto A2 unchanged.

### Wi-Fi relay (`/esp/*`) — verified end to end on hardware 2026-09-01

| Topic | Type | Direction |
|---|---|---|
| `/esp/wifi_credentials` | `std_msgs/String` | `esp_hardware_interface` → robot_app (runs `nmcli`) |
| `/esp/wifi_status` | `std_msgs/String` | robot_app → `esp_hardware_interface` → ESP → BLE notify |

**Wire formats — both are fixed, and getting either wrong fails silently:**

- **Inbound** (`CMD_WIFI_CONFIG` 0x0B): the ESP forwards its `WifiConfigPayload`
  struct **verbatim** — `char ssid[32]` then `char password[64]`, null-PADDED at
  fixed offsets, 96 bytes. `handleWifiConfig()` must read offsets 0 and 32; the
  original null-SEPARATOR scan read the ssid correctly and then hit the zero
  padding immediately after it, yielding an **empty password** and an `nmcli`
  join that could never authenticate. Republished on the topic as
  `"ssid\npassword"` (what `ros_bridge.py` splits on).
- **Outbound** (`/esp/wifi_status`): `"state,ssid,rssi,ip"` — exactly 4
  comma-separated fields. `state` is a **tri-state** (0 idle/failed,
  1 connected, 2 connecting), NOT a bool; collapsing it to 0/1 here drops the
  "connecting" state robot_app sends the moment credentials arrive, which is
  what lets the tablet show progress during the ~15 s join.

**Status is pushed, not just polled.** A fresh `/esp/wifi_status` sets
`wifi_status_push_pending_`, and `write()` sends it on the control thread — the
ESP relays any payload-carrying `CMD_WIFI_STATUS` straight out as a BLE notify,
so a completed join (and the robot's new IP) reaches the phone unprompted. The
flag/drain split exists because **only the control thread may touch `/dev/esp`**;
the subscription callback runs on the spin thread and must never write.

**The cache is zero-filled on every `on_configure()`.** So after a stack restart
the tablet reads "disconnected, no SSID, no IP" until something republishes —
which is why `robot_app` republishes every 30 s rather than only on change.

> **Provisioning needs a polkit grant on bare-metal robots.** `robot_app` runs as
> a `systemd --user` service with no seat, so NetworkManager refuses its scans and
> connects with `Not authorized to control networking.` — surfacing as `nmcli`
> exit 4 (connect refused) and exit 10 (scan refused ⇒ stale cache ⇒ "no such
> AP"). Containerised robots run it as root and never hit this. See
> `bonicOS-robot-app/OVERVIEW.md` §Wi-Fi Provisioning.

---

## ESP ⇄ RPi4 USB CDC Protocol

Full spec: **[bonicbot_usb_cdc_protocol_spec.md](../bonicOS-m1-ros/bonicbot_usb_cdc_protocol_spec.md)**.
CDC-ACM, ~1 MB/s, framing `0xAA 0x55 | type(1B) | length(2B LE) | payload` — **identical
to BLE framing, no checksum byte**. `cdc_protocol.hpp` implements the parser/packer.

**A2 uses the full motion subset** — the opposite of M1, which uses only ping/IMU/Wi-Fi
because its motion hardware is Jetson-direct:

| Frame | Hex | Direction | Used for |
|---|---|---|---|
| `CMD_PING` | 0x01 | Pi → ESP | Link liveness |
| `CMD_MOTOR_MOVE` | 0x02 | Pi → ESP | Wheel velocities (m/s) + accel |
| `CMD_ENCODER_REQUEST` | 0x21 | Pi → ESP | Poll encoder ticks |
| `CMD_RESET_ENCODERS` | 0x23 | Pi → ESP | Zero encoders (on activate) |
| `CMD_STOP` | 0x26 | Pi → ESP | Emergency stop (on deactivate) |
| `CMD_SERVO_MULTI` | 0x0A | Pi → ESP | **Servo positions** — `N × [id, angle°, speed, accel]`, registry IDs |
| `CMD_SERVO_CONTROL` | 0x27 | Pi → ESP | Single-servo **action** — `[id, action]`, 2 B: 0 move / 1 set-mid / 2 torque-off / 3 torque-on |
| `CMD_IMU_REQUEST` | 0x29 | Pi → ESP | Poll one IMU sample |
| `CMD_SERVO_FEEDBACK_REQUEST` | 0x2A | Pi → ESP | Servo feedback — once / continuous / stop, + interval |
| `CMD_WIFI_CONFIG` | 0x0B | ESP → Pi | SSID+password relayed from phone BLE |
| `CMD_WIFI_STATUS` | 0x0C | both | ESP requests; Pi replies status/SSID/RSSI/IP |
| `RESP_ACK` / `RESP_NACK` | 0x50 / 0x51 | ESP → Pi | Command acknowledgement |
| `RESP_BATTERY` | 0x52 | ESP → Pi | 31 B: voltage, current, SOC%, active-servo count + online IDs |
| `RESP_ENCODERS` | 0x60 | ESP → Pi | Two int32 tick counts |
| `RESP_SERVO_FEEDBACK` | 0x61 | ESP → Pi | `N × [id, angle°]` — position only |
| `RESP_IMU` | 0x63 | ESP → Pi | 6 floats: accel m/s², gyro deg/s |

> **Servo positioning is `CMD_SERVO_MULTI` (0x0A), not `CMD_SERVO_CONTROL` (0x27).**
> Per [bonicbot_ble_protocol_spec.md](../bonicOS-flutter/docs/bonicbot_ble_protocol_spec.md)
> (Rev 2.0), 0x27 is a **2-byte** payload — `[Servo ID][Action]` — so it cannot carry an
> angle at all. It is the *single-servo action* command (move-to-target / set-middle /
> torque-off / torque-on). 0x0A is the positional command: `Count` + `N × [Servo ID (0-17),
> float angle°, uint16 speed, uint8 accel]`.
>
> A2 uses **both**: 0x0A every control cycle for positions, and 0x27 for torque-on at
> activate and torque-off at deactivate.
>
> ⚠️ The CDC spec's §4 describes 0x27 with an `1 + N×8` positional payload — that
> contradicts the BLE spec and reflects the pre-migration A/S firmware. **The BLE spec
> Rev 2.0 is authoritative** for the unified firmware; framing is identical across both
> transports, so its payload tables apply to CDC too.

---

## How this repo and robot_app fit together

`bonicOS-robot-app` is **one image shared with M1** — series behaviour comes from
`ROBOT_CONFIG` in its `app/config.py`, keyed by `ROBOT_SERIES=A`. It owns
everything outward-facing (WebRTC/app control, maps, cloud, Wi-Fi via nmcli) and
this repo owns everything robot-facing. They meet only at ROS2 topics and a
small launch-file contract.

**Division of ownership**

| | this repo | robot_app |
|---|---|---|
| `/dev/*`, controllers, sensors | ✅ | never |
| SLAM / Nav2 / EKF launch files | ✅ provides | ✅ starts + stops them |
| Which map, which mode, when | never | ✅ |
| App / cloud / WebRTC | never | ✅ |

**Startup order doesn't matter.** `start_session_robot.sh` brings up the base
stack (hardware + rsp + EKF); robot_app is a separate persistent service that
adopts whatever session it finds and re-attaches if one starts later.

On a real robot robot_app **never starts the base stack itself** — `base_autostart`
is false off-sim. It cannot: the base stack here is two launches (hardware +
nav bringup) and `BaseSessionManager` spawns one. If nothing is running it logs
`no base stack running and autostart is off — staying down` and waits for its
health poll to find one. In sim it does self-start, because there
`sim.launch.py` genuinely is the whole base stack.

**The launch contract robot_app depends on** — identical for A2 and M1, which
is why one app drives both:

| robot_app calls | this repo provides |
|---|---|
| `<nav_pkg> mapping.launch.py use_sim_time:=…` | composite: slam_toolbox + Nav2 core (`slam:=true`) |
| `<nav_pkg> navigation.launch.py slam:=false maps_dir:=… map_name:=<name>.yaml` | map_server + AMCL + Nav2 core |
| base, **sim only**: `<sim_pkg> sim.launch.py` | Gazebo + rsp + controllers — the whole base stack in one file |

`<nav_pkg>` is `bonicbot_a2_nav` for series A and `bonicbot_m1_nav` for M —
resolved from the series table, not hardcoded. Note `map_name` **includes the
`.yaml` extension**.

Where A2 and M1 genuinely differ, robot_app reads it from the same table. Note odom is
**no longer** one of those differences: A2's entry used to be `/odom`, but nothing in this
repo publishes that topic — `diff_cont` has `enable_odom_tf: false` and publishes
`/diff_cont/odom`, and `ekf_node` runs with no remap so its output is
`/odometry/filtered`. robot_app's A-series odom telemetry was therefore silently dead
(the subscription succeeded and never fired). Corrected in `bonicOS-robot-app` commit
`6d6fa38`; **verified on real hardware 2026-08-29** — `/odometry/filtered` publishes
at ~10 Hz and robot_app's odom telemetry is live.

| | A2 | M1 |
|---|---|---|
| odom | `/odometry/filtered` | `/odometry/filtered` |
| imu | `/imu/data` | `/esp/imu` |
| arm commands | `Float64MultiArray` on `/…_controller/commands` | `JointTrajectory` on `/…/joint_trajectory` |
| cameras | `face` only | `face` + `docking` (+ depth) |

> **`Float64MultiArray` has a sharp edge robot_app has to work around.** It
> carries no joint names — array position IS joint identity — so it must be
> exactly `controllers.yaml`'s joint count for that group, every publish, or
> `ForwardCommandController` rejects it (repeatedly, once per control cycle:
> an `"update call ... returned an error"` flood, and the group never moves
> again until a correctly-sized command arrives). robot_app's
> `ROBOT_CONFIG["A"]["controller_joints"]` mirrors this repo's
> `controllers.yaml` group/joint order exactly so it can fill in any joint a
> caller didn't specify (from the last `/joint_states` sample) before
> publishing. **If a joint is ever added, removed, or reordered in
> `bonicbot_a2_hardware/config/controllers.yaml`, that table in robot_app's
> `app/config.py` must be updated to match, or arm/head commands break.**

---

## Navigation

```
LiDAR C1M1 → slam_toolbox → /map        (mapping)
           → nav2_amcl                   (localization on a saved map)
EKF        → wheel odom + ESP IMU → /odometry/filtered
Nav2       → NavigateToPose → /cmd_vel
CSI camera → vision_pipeline.py → YOLO / pose / face / gesture / aruco
```

**`map→odom` is owned by slam_toolbox OR AMCL, never both.** `bringup.launch.py`'s
`slam` arg (default `false`) selects which:

- `slam:=true` — mapping. `slam_toolbox` runs live and owns `map→odom`;
  `navigation.launch.py` skips AMCL/`map_server`.
- `slam:=false` — production. `nav2_amcl` + `map_server` run against a saved map and own
  `map→odom`; no SLAM.

A2 must also toggle this **at runtime** — the operator starts/stops mapping from the phone
app without relaunching. That orchestration belongs to **`robot_app`**, which starts and
stops these launch files and is responsible for upholding the never-both invariant by
tearing one down before bringing the other up. This repo's job is to expose the two modes
as cleanly separable launch files; it does not manage their lifecycle.

EKF (`ekf.yaml`) fuses `/diff_cont/odom` with the ESP IMU's yaw rate. `diff_cont` has
`enable_odom_tf: false` and inflated yaw covariance so the EKF trusts the gyro over
wheel-derived yaw — deliberate slip rejection. `ekf_imu.yaml` is a more aggressive
variant (IMU as *sole* rotation source); it is **not currently launched**.

---

## udev Rules

```bash
# Find IDs: udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"

# /etc/udev/rules.d/99-bonicbot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", SYMLINK+="esp",    GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="lidar",  GROUP="dialout", MODE="0660"

sudo udevadm control --reload-rules && sudo udevadm trigger
```

A2 has **one** ESP32-S3 board, so a plain vendor/product match suffices — no
serial-number disambiguation like M1 (three ESP boards sharing `303a:1001`).

---

## Production — Docker Compose

```yaml
services:
  bonicbot_ros:
    build: { context: ., dockerfile: Dockerfile.ros }
    restart: always
    privileged: true
    network_mode: host
    devices:
      - /dev/esp
      - /dev/lidar
      - /dev/video0
    volumes:
      - /home/pi/maps:/maps
    environment:
      - ROS_LOCALHOST_ONLY=1
      - ROS_DOMAIN_ID=0
      - BONICBOT_MAPS_DIR=/maps

  robot_app:
    image: autobonics/robot-app:latest
    restart: always
    network_mode: host
    mem_limit: 384m
    environment:
      - ROBOT_SERIES=A
      - ROS_DISTRO=humble
      - ROBOT_ID=${ROBOT_ID}
      - ROS_LOCALHOST_ONLY=1
    secrets: [firebase_credentials, livekit_api_key]
    volumes:
      - /home/pi/maps:/maps
      - /var/run/docker.sock:/var/run/docker.sock
    depends_on:
      bonicbot_ros: { condition: service_started }

secrets:
  firebase_credentials: { file: /home/pi/secrets/firebase.json }
  livekit_api_key:      { file: /home/pi/secrets/livekit.json }
```

---

## Simulation

**The ESP does not exist in simulation, and the CDC protocol is never used there.**
`ros2_control.xacro` branches on `sim_mode`: sim binds `gz_ros2_control/GazeboSimSystem`,
the real robot binds `bonicbot_a2/EspHardwareInterface`. Gazebo supplies the joints,
wheels, IMU, LiDAR and camera. Everything above `ros2_control` — the same
`controllers.yaml`, twist_mux, EKF, SLAM, Nav2 — is byte-identical between the two.

So `sim.launch.py` is the **sim-side replacement for `hardware.launch.py`**, and nav runs
on top of either one unchanged.

| | Real robot | Simulation |
|---|---|---|
| `ros2_control` plugin | `bonicbot_a2/EspHardwareInterface` | `gz_ros2_control/GazeboSimSystem` |
| Wheels / servos / IMU | ESP32-S3 over USB CDC | Gazebo physics + `ros_gz_bridge` |
| LiDAR, camera | `/dev/lidar`, `/dev/video0` | Gazebo sensors + bridge |
| `controllers.yaml` | same file | same file |
| EKF, SLAM, Nav2, twist_mux | same | same (`use_sim_time:=true`) |

### Running

```bash
# ── Simulation ──────────────────────────────────────────────
ros2 launch bonicbot_a2_sim sim.launch.py                      # world:=… headless:=…
ros2 launch bonicbot_a2_nav navigation.launch.py use_sim_time:=true
#   (or slam.launch.py use_sim_time:=true to map)

# ── Real robot ──────────────────────────────────────────────
ros2 launch bonicbot_a2_hardware hardware.launch.py
ros2 launch bonicbot_a2_nav bringup.launch.py use_sim_time:=false slam:=false
```

`sim.launch.py` brings up: rsp (`sim_mode:=true`) → Gazebo → spawn entity → `ros_gz`
bridges (clock, scan, imu, camera) → the same seven controller spawners → twist_mux →
joystick → EKF. Same composition M1's `bonicbot_m1_sim` uses.

> `sim.launch.py` already carries a `use_real_camera` arg — `False` bridges Gazebo's
> camera, `True` runs the real `v4l2_camera` node instead (a dev laptop's webcam). Both
> publish `/camera/image_raw`, so `vision_pipeline.py` doesn't care which is running.

---

## Development Flow (Bare Metal)

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# Terminal 1 — hardware (controller_manager, ESP link, lidar, camera)
ros2 launch bonicbot_a2_hardware hardware.launch.py

# Terminal 2 — navigation
ros2 launch bonicbot_a2_nav bringup.launch.py
```

---

## Key Design Rules

1. `src/hardware/` packages never import from `src/nav/`, and vice versa — communication
   is via ROS2 topics only. (Enforced even though both ship in one container.)
2. `bonicbot_a2_hardware` is the sole owner of every `/dev/*` device.
3. **Exactly one process opens `/dev/esp`.** The CDC stream cannot be shared.
4. `ROS_LOCALHOST_ONLY=1` always — ROS2 topics never visible on school/lab WiFi.
5. Maps mounted as a volume — survive container rebuilds.
6. `restart: always` — self-healing during student demos.
7. Phone is always the primary UI; ESP BLE is ground truth for IP/status/Wi-Fi config.
8. `robot_app` owns no hardware — only ROS2 topics, WSS, Firebase.
9. Humble uses `Twist`, not `TwistStamped` (`diff_cont.use_stamped_vel: false`).

---

## Open Items

- **Consider migrating arm/head/gripper controllers from
  `JointGroupPositionController` (`Float64MultiArray`) to
  `JointTrajectoryController` (`JointTrajectory`)**, matching M1. Named joints
  make partial commands safe by construction — no array-position/joint-count
  coupling, no "update call ... returned an error" flood class of bug (see the
  robot_app integration section above), and timed/interpolated motion instead
  of instant setpoints. Not urgent — the current fill-from-`/joint_states`
  workaround in robot_app works — but the sturdier pattern if A2's arms need
  smoother motion later. Would touch `controllers.yaml`, `ros2_control.xacro`,
  and robot_app's `publish_joint_command`/`ROBOT_CONFIG["A"]` together.
- **rosbridge removal** — `rosbridge_websocket` (:9090) stays behind an opt-in launch arg
  until `robot_app`'s WSS is confirmed to cover the same surface (map, pose, nav goals,
  joint states) for the Flutter app. Deleting it before that parity check would break the
  phone app.
- **~~`robot_app` A-series servo gate~~ — RESOLVED.** `ROBOT_SERIES=A` now has
  `moveit: True`, a populated `controllers` map and a `controller_joints` order matching
  this repo's `controllers.yaml`. Arm/head/gripper commands are servable. `robot_app`
  now runs on the real A2 (2026-08-29), but only drive and mapping were exercised —
  **arm/head/gripper commands through robot_app are still untested on hardware.**
- **`ekf_imu.yaml`** — an alternate EKF config (IMU as sole yaw source), present but never
  launched. Either wire it behind a launch arg or delete it.
- **RPi4 CPU budget.** Profiled on real hardware 2026-08-25 during a live SLAM + Nav2
  session. With the IDE remote server closed and `robot_app` stopped, the full stack
  leaves **53% idle**; `rplidar_composition` is the largest single consumer at **~33-44%
  of a core**, ahead of every Nav2 node.
- **`angle_compensate` is NOT a CPU lever at all — properly measured 2026-08-29.** This
  file previously claimed "essentially all" of the LiDAR's cost was `angle_compensate`.
  That was an assumption written up as a measurement. A first attempt to correct it
  reported 17%, but that comparison was invalid — both runs had the flag on (the launch
  argument had not been pulled) and one had the camera running.
  A valid A/B, with `ros2 param get /rplidar angle_compensate` verified True then False
  and the camera off in both:

  | | `rplidar_composition` |
  |---|---|
  | `angle_compensate: true` | 35.3%, 23.5% |
  | `angle_compensate: false` | 27.8%, 33.3% |

  **The ranges overlap completely** — run-to-run variance exceeds any difference between
  conditions, so there is no detectable saving. The cost, however, is immediate and
  visible: with it off, slam_toolbox logs `LaserRangeScan contains 508 range readings,
  expected 512` on essentially every scan, counts varying 502-511, because the raw
  revolution is no longer resampled into fixed evenly-spaced bins.
  **Leave it true.** It is exposed as a launch argument
  (`hardware.launch.py angle_compensate:=false`) for measurability only.
- **Where the CPU actually went:** the camera, not the LiDAR. `v4l2_camera_node` was
  running at 30 fps instead of 6 and cost **105% of a core**; fixing that took it to
  17.6%. See the camera note above. Two things that were *not* the problem
  and had already been trimmed by then: `ekf.yaml` (15 Hz → 10 Hz) and slam_toolbox
  (`throttle_scans` 1 → 2, `transform_publish_period` 50 Hz → 20 Hz), both now ~5-11%.
  **The real CPU thieves were off-stack:** the Antigravity IDE remote server peaked at
  **90%** of a core. Never benchmark this robot with an editor session attached — SSH
  from a plain terminal.
- **`robot_app` costs ~80% of a core, not the "~17%" this file used to claim.**
  That 17% was measured with `robot_app` **stopped** — it was never a like-for-like
  figure. Profiled properly with `py-spy` on 2026-08-29, with a nav session up and
  a peer connected, `robot_app` sits at **80-94%** of a core and the Pi at ~22% idle.
  Roughly half of that is not robot_app's own work at all: rclpy's
  `_wait_for_ready_callbacks` rebuilds its waitset **in Python** on every iteration
  of every executor thread, showing 36-47% own time with GIL contention at 43-68%.
- **Two "optimizations" that measurably did NOT help — do not redo them.** Both are
  correct changes and worth keeping for other reasons; neither reduced total CPU:
  | change | effect on robot_app CPU |
  |---|---|
  | Idling the WebRTC camera encoder when no one is watching | none (still ~85%). Encoder work fell from ~32% to ~11% in the profile, and the executor simply spun into the gap. Keeps WiFi bandwidth down, so still worth having. |
  | Throttling `/joint_states` 50 Hz → 10 Hz into robot_app | none. Removed ~40 callbacks/sec and total CPU did not move. |
  | `MultiThreadedExecutor(num_threads=2)` instead of the default 4 | **~10 points** (94% → 80%), reproduced three times. The only lever that worked. |

  The reason the first two fail is the same: `_wait_for_ready_callbacks` is a **spin
  loop**, so it absorbs whatever you free up. Cutting message volume or encoder work
  just gives it more room to spin. Only reducing the number of spinners helped.
  Further gains are architectural (fewer subscriptions, or an `rclcpp` bridge), not
  configuration.
- **The camera was running at 30 fps, not 6 — fixed 2026-08-28.** `v4l2_camera_node` has
  **no frame-rate parameter** (`ros2 param list` shows `image_size`, `pixel_format`,
  `output_encoding` and the V4L2 controls, nothing for timing), so the
  `time_per_frame: [1, 6]` that sat in `camera.launch.py` was silently ignored —
  `ros2 param get` on it answered "Parameter not set". The camera free-ran at its 30 fps
  default and cost **105.6% of a core**, more than every Nav2 node combined, saturating
  the Pi to 23% idle and making SSH sluggish. At the intended 6 fps the same node costs
  **17.6%**. `camera.launch.py` now applies the rate to the DEVICE with `v4l2-ctl
  --set-parm`, chained ahead of the node via `OnProcessExit`; the node does not override
  it once running. Requires `v4l-utils`. Lesson worth keeping: a parameter in a launch
  file is not evidence the node accepts it — check `ros2 param list` on the running node.
- **`nav2_params_sim.yaml` — unused, slated for deletion.** No launch file references it:
  `navigation.launch.py` defaults `params_file` to `bonicbot_a2_nav/config/nav2_params.yaml`,
  `bringup.launch.py` never forwards `params_file`, and neither does robot_app's
  `NavModeManager`. **Simulation runs on `nav2_params.yaml`, the same file the real robot
  uses** — verified end to end in Gazebo on 2026-08-25 (mapping + `navigate_to_pose`
  returning SUCCEEDED). That is the desired arrangement, not an accident: sim exists to be
  a reference for the real robot, and the sim profile's faster tuning (`max_vel_x` 0.8 vs
  0.22, `acc_lim_x` 3.5 vs 0.5, `controller_frequency` 20 vs 10) would validate motion the
  hardware cannot reproduce. The two files have already drifted — fixes applied to
  `nav2_params.yaml` were not mirrored into it — which is the usual cost of keeping two
  files that must stay in sync. Kept for now only as a record of the faster tuning; delete
  it once nobody wants that profile back. The only other mention is
  `reference/robot_manager.py:532`, which is out of the build.
- **~~`/battery_state`~~ — DONE.** `esp_hardware_interface.cpp` creates the
  `sensor_msgs/BatteryState` publisher and publishes voltage, current and SOC (as
  `percentage`, 0..1 — `RESP_BATTERY` reports 0-100, so it is divided on the way out).
  The firmware does emit 0x52 over CDC, contrary to the CDC spec's channel table, which
  lists battery as BLE-only and predates the unified firmware.
  Still unused: the frame's trailing `active servo count + online IDs`, which is a free
  health check — a fitted servo dropping off the bus would become detectable.
  **robot_app subscribes to `/battery_state` and broadcasts a `battery` telemetry event;
  the BonicAI frontend does not yet render it over the WebRTC lane** (its battery UI is
  fed from the BLE path). Surfacing it in the robot UI is a frontend task, not a
  robot-side one — the data is already on the wire.
- **Orchestration gap during migration** — `robot_manager.py` and `robot_agent.py` leave
  this repo (see below), but `robot_app` does not yet implement every service they
  provided: the precise-move queue, patrol/waypoint following, the named-location store,
  and per-detector vision toggles (`/vision/control`) have no robot_app equivalent found.
  Those need building in `robot_app` before A2 has feature parity with the current stack.

---

*bonicbot-a2-ros | Autobonics Pvt Ltd*
*Confidential and proprietary. All rights reserved.*
