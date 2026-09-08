# BonicBot A2 — Manual Control & Direction Verification Reference

Runnable `ros2 topic pub`/`echo` commands for driving, moving every joint through
its full range, and reading sensors directly — bypassing `robot_app` entirely.

This document is a **test matrix, not a sample sheet**. Every command is complete
and can be pasted verbatim. The same commands run against **both** Gazebo and real
hardware, which is the point: §3 compares the two side by side to pin down joint
and drive **direction**, which is not yet verified on hardware.

Sensor/telemetry sections (§4–§7) were verified against real A2 Pro hardware on
2026-08-24. Motion **direction** (§1–§3) is what this pass exists to establish.

---

## 0. Bringing the stack up

Both variants below are manual `ros2 launch` calls. Do **not** use
`start_session.sh` / `start_session_robot.sh` for this pass — those background the
stack into log files, which hides the controller-spawner output you need to see.

### 0a. Simulation (Gazebo)

```bash
cd ~/bonic/bonicbot-a2-ros
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch bonicbot_a2_sim sim.launch.py world:=obstacle_world.sdf
```

`sim.launch.py` also starts the EKF and joystick teleop, so `/odometry/filtered`
is available here without a second launch.

### 0b. Real hardware

```bash
cd ~/bonic/bonicbot-a2-ros
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch bonicbot_a2_hardware hardware.launch.py \
    use_lidar:=false use_camera:=false use_joystick:=false
```

LiDAR, camera and joystick are off so the terminal stays readable and nothing
else can command `/cmd_vel`. Turn them back on once §1–§2 pass.

`hardware.launch.py` does **not** start the EKF — `/odometry/filtered` does not
exist until you also run `bringup.launch.py`. Use `/diff_cont/odom` for the base
tests on hardware.

### 0c. Readiness gate — check before sending anything

```bash
# All seven controllers must read "active", not merely "loaded".
# A spawner that timed out leaves a controller loaded-but-unconfigured: the node
# exists and the graph looks healthy, but it has no command subscription.
ros2 control list_controllers

# Expect seven entries, all "active":
#   diff_cont, joint_broad, left_arm_controller, right_arm_controller,
#   head_controller, left_gripper_controller, right_gripper_controller

# State feedback is flowing (both sim and hardware):
ros2 topic hz /joint_states       # expect ~50 Hz

# SIM ONLY — exactly one /clock publisher, or ROS time steps backwards and
# every tf2 buffer wipes itself:
ros2 topic info /clock            # expect "Publisher count: 1"
```

If any controller is not `active`, stop here — `./stop_session.sh`, then relaunch.
Nothing below will behave.

---

## 1. Base / drive

`linear.x` is m/s, `angular.z` is rad/s, on `/cmd_vel`.

**Speed ceiling — corrected 2026-08-25.** The hardware interface clamps to
`max_velocity` (**4.5** rad/s) × `wheel_radius` (0.06 m) ≈ **0.27 m/s**. The
earlier figure of 0.63 m/s in this document came from `max_velocity: 10.47`,
which was lowered because 100 RPM was too aggressive on real hardware. Gazebo has
no such clamp (its command interface allows ±30 rad/s), so **the sim will out-run
the robot** for the same `/cmd_vel` — expected, not a bug.

A single `--once` publish decelerates back to stop almost immediately. Every
command below therefore uses `-r 10` for a sustained move; stop it with `Ctrl+C`
followed by the explicit stop command.

### 1a. Stop — know this before you start anything

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### 1b. The six direction tests

Run each for ~2 seconds, `Ctrl+C`, then publish the stop command from §1a.
On hardware: **wheels off the ground for the first pass of all six.**

```bash
# 1. FORWARD  — expect: robot moves in the +X body direction (face/head leading)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 2. BACKWARD — expect: robot moves in -X (backs away from its face direction)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1}, angular: {z: 0.0}}"

# 3. SPIN LEFT (CCW, +yaw) — expect: robot rotates counter-clockwise viewed from ABOVE
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 4. SPIN RIGHT (CW, -yaw) — expect: robot rotates clockwise viewed from ABOVE
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"

# 5. ARC FORWARD-LEFT  — expect: drives forward while curving to its left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.4}}"

# 6. ARC FORWARD-RIGHT — expect: drives forward while curving to its right
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: -0.4}}"
```

### 1c. Reading it back — the numeric check

While a move is running, in a second terminal:

```bash
# Raw wheel odometry — available on BOTH sim and hardware
ros2 topic echo /diff_cont/odom --field twist.twist

# Fused EKF odometry — sim always; hardware only if bringup.launch.py is running
ros2 topic echo /odometry/filtered --field twist.twist

# Wheel joint velocities, rad/s. Order matches the `name` array.
ros2 topic echo /joint_states --field velocity

# IMU yaw rate, rad/s — the independent check on turn direction
ros2 topic echo /imu/data --field angular_velocity
```

Expected signs:

| Test | `odom` twist.linear.x | `odom` twist.angular.z | `/imu/data` angular_velocity.z |
|---|---|---|---|
| 1 forward | **+** ≈ 0.1 | ≈ 0 | ≈ 0 |
| 2 backward | **−** ≈ −0.1 | ≈ 0 | ≈ 0 |
| 3 spin left | ≈ 0 | **+** ≈ 0.5 | **+** |
| 4 spin right | ≈ 0 | **−** ≈ −0.5 | **−** |
| 5 arc fwd-left | **+** | **+** | **+** |
| 6 arc fwd-right | **+** | **−** | **−** |

A sign that disagrees between `/cmd_vel` and `odom` means the wheels are swapped
or reversed. A sign where `odom` and the IMU disagree with **each other** means
the IMU mounting orientation is wrong, not the drive.

---

## 2. Joints — arms, grippers, neck

All arm/gripper/neck controllers take `std_msgs/msg/Float64MultiArray` on
`/<controller>/commands`, in **radians**.

> **The array must contain every joint in that controller's group, on every
> publish.** Partial arrays are silently rejected — the message carries no joint
> names, so array position *is* joint identity. Every command below is already
> full-length; do not shorten them.

### 2a. Group reference

| Controller | Topic | Array order | Joint | min | mid | max |
|---|---|---|---|---|---|---|
| Right arm | `/right_arm_controller/commands` | `[shoulder_pitch, elbow]` | `right_shoulder_pitch_joint` | −0.785 (−45°) | 1.1775 (67.5°) | 3.14 (180°) |
| | | | `right_elbow_joint` | −0.873 (−50°) | −0.4365 (−25°) | 0.0 (0°) |
| Left arm | `/left_arm_controller/commands` | `[shoulder_pitch, elbow]` | `left_shoulder_pitch_joint` | −0.785 (−45°) | 1.1775 (67.5°) | 3.14 (180°) |
| | | | `left_elbow_joint` | −0.873 (−50°) | −0.4365 (−25°) | 0.0 (0°) |
| Head/neck | `/head_controller/commands` | `[neck_yaw]` | `neck_yaw_joint` | −1.571 (−90°) | 0.0 (0°) | 1.571 (90°) |
| Right gripper | `/right_gripper_controller/commands` | `[finger1]` | `right_gripper_finger1_joint` | −0.785 (−45°) | 0.131 (7.5°) | 1.047 (60°) |
| Left gripper | `/left_gripper_controller/commands` | `[finger1]` | `left_gripper_finger1_joint` | −0.785 (−45°) | 0.131 (7.5°) | 1.047 (60°) |

Source of truth: `bonicbot_a2_description/urdf/ros2_control.xacro` (hardware
limits) and `body.xacro` / `gripper.xacro` / `head.xacro` (URDF limits). They
agree today; if they ever diverge, ros2_control's are the ones enforced.

> **Safety on hardware.** Elbow range is one-sided (−50°…0°) — a hard stop at
> each end and no slack. Torque auto-engages on the first position command, so
> the very first publish moves the joint immediately, from wherever it happens to
> be resting. Run §2b (home) **first**, keep a hand near the power switch, and
> step min → mid → max rather than jumping between extremes.

### 2b. Home — all seven joints to a neutral pose

Run this before and after every joint test.

```bash
ros2 topic pub /right_arm_controller/commands     std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
ros2 topic pub /left_arm_controller/commands      std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
ros2 topic pub /head_controller/commands          std_msgs/msg/Float64MultiArray "{data: [0.0]}"      --once
ros2 topic pub /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"      --once
ros2 topic pub /left_gripper_controller/commands  std_msgs/msg/Float64MultiArray "{data: [0.0]}"      --once
```

### 2c. Right shoulder pitch — elbow held at 0.0

```bash
# MIN  -0.785 rad / -45 deg  — expect: arm swings BACKWARD past vertical
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.785, 0.0]}" --once

# MID   1.1775 rad / 67.5 deg — expect: arm roughly horizontal, forward
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1775, 0.0]}" --once

# MAX   3.14 rad / 180 deg   — expect: arm raised fully overhead
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.14, 0.0]}" --once

# back to home
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
```

### 2d. Right elbow — shoulder held at 0.0

```bash
# MIN  -0.873 rad / -50 deg — expect: forearm folded IN toward the body
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.873]}" --once

# MID  -0.4365 rad / -25 deg — expect: forearm half-bent
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.4365]}" --once

# MAX   0.0 rad / 0 deg     — expect: arm fully STRAIGHT
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
```

### 2e. Left shoulder pitch — elbow held at 0.0

```bash
# MIN  -0.785 rad / -45 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.785, 0.0]}" --once

# MID   1.1775 rad / 67.5 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1775, 0.0]}" --once

# MAX   3.14 rad / 180 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.14, 0.0]}" --once

# back to home
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
```

The left arm must **mirror** the right — same command, mirrored motion. If one
arm raises while the other lowers, that is exactly the double-inversion class of
bug that emptied `isServoInverted()` (see §3).

### 2f. Left elbow — shoulder held at 0.0

```bash
# MIN  -0.873 rad / -50 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.873]}" --once

# MID  -0.4365 rad / -25 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.4365]}" --once

# MAX   0.0 rad / 0 deg
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once
```

### 2g. Neck yaw

```bash
# MIN  -1.571 rad / -90 deg — expect: head turns to the robot's RIGHT
ros2 topic pub /head_controller/commands std_msgs/msg/Float64MultiArray "{data: [-1.571]}" --once

# MID   0.0 rad / 0 deg     — expect: head faces straight forward
ros2 topic pub /head_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}" --once

# MAX   1.571 rad / 90 deg  — expect: head turns to the robot's LEFT
ros2 topic pub /head_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.571]}" --once
```

Neck yaw axis is `0 0 1` (+Z), so by the right-hand rule **positive = the robot's
left**. This is the one joint whose expected direction is unambiguous from the
URDF alone — a mismatch here is firmware, not modelling.

### 2h. Right gripper

```bash
# MIN  -0.785 rad / -45 deg — expect: fingers CLOSED / clamped
ros2 topic pub /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.785]}" --once

# MID   0.131 rad / 7.5 deg
ros2 topic pub /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.131]}" --once

# MAX   1.047 rad / 60 deg  — expect: fingers fully OPEN
ros2 topic pub /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.047]}" --once
```

Open/closed may turn out inverted from the guess above — that is one of the
things this pass determines. Record what actually happens.

### 2i. Left gripper

```bash
# MIN  -0.785 rad / -45 deg
ros2 topic pub /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.785]}" --once

# MID   0.131 rad / 7.5 deg
ros2 topic pub /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.131]}" --once

# MAX   1.047 rad / 60 deg
ros2 topic pub /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.047]}" --once
```

### 2j. Both arms together — mirror check

```bash
# Both arms to mid shoulder, half-bent elbow
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1775, -0.4365]}" --once
ros2 topic pub /left_arm_controller/commands  std_msgs/msg/Float64MultiArray "{data: [1.1775, -0.4365]}" --once
```

Both arms must end up in visually mirrored poses.

### 2k. Reading joint positions

```bash
# Full state — `position` is radians, in the same order as the `name` array
ros2 topic echo /joint_states --once

# Names and positions separately, easier to read against the table in §2a
ros2 topic echo /joint_states --field name --once
ros2 topic echo /joint_states --field position --once
```

`name` includes both wheels alongside the 7 servos. `effort` is always `nan` on
hardware — CDC servo feedback is position-only, no load/torque data.

**Commanded vs. achieved — this is what localizes a sign bug:**

- Position converges to the **negative** of what you sent → the flip is in the
  hardware interface (`isServoInverted()` / firmware).
- Position converges **correctly** but the robot moved the wrong way → the flip
  is in the URDF axis or the mesh orientation.

### 2l. Releasing the servos

Torque auto-engages on the first position command — there is no separate enable
step, and **no ROS topic exposes torque-off or the SET_MIDDLE calibration**.
`CMD_SERVO_CONTROL` (0x27) is implemented in `cdc_protocol.hpp` but never
published to ROS; the only subscriptions the hardware interface creates are
`/face/matrix_action` and `/esp/wifi_status`.

To release all servos (arms go loose), tear the session down:

```bash
./stop_session.sh
```

---

## 3. Direction verification protocol

This is why §1 and §2 are written as literal commands: **run the identical
command in Gazebo and on hardware, and compare.**

Gazebo is the reference. It has no inversion table at all, so it shows the
"true" direction implied by the URDF — the same URDF the frontend's 3D rig and
the whole nav stack assume. Hardware must be made to match it, never the reverse.

### Why direction is not yet trustworthy — four independent sign sources

1. **URDF joint axes.** Shoulders are `0 -1 ±0.265`, elbows `0 -0.9725 ∓0.2328`,
   grippers `0 -1 0`, neck `0 0 1`. The compound, mirrored axes on the shoulders
   and elbows mean the sign is not obvious by inspection.
2. **`isServoInverted()` is now empty** — it returns `false` for every servo.
   Elbows were removed 2026-08-24, then grippers and neck on 2026-08-25, each
   time because the table was double-flipping a sign the unified firmware
   already handles. Hardware direction now rests entirely on firmware behaviour.
3. **Wheel `axis_z`** is `+1` left / `−1` right — but that is sim-only geometry.
   On hardware `CMD_MOTOR_MOVE` carries raw m/s with no sign handling on the ROS
   side at all.
4. **Frontend `JOINT_AXIS_SIGN`** (`jointMapping.ts`) carries its own per-joint
   axis and sign for the 3D rig, with a comment saying to verify it on a real
   robot.

Any one of these being wrong produces a plausible-looking, mirrored robot.

### Procedure

1. Run the full §1 + §2 matrix in **Gazebo**. Fill the "Sim observed" column.
   This becomes the reference.
2. Run the identical matrix on **hardware**. Fill the "Real observed" column.
3. Every row where Sim ≠ Real is a bug. Fix it on the **hardware** side —
   normally by re-adding that servo's registry ID to `isServoInverted()` in
   `esp_hardware_interface.cpp`. Never fix it by editing the URDF to match
   hardware: that breaks sim, nav and the frontend rig together.
4. Re-run the changed rows on hardware to confirm.

### Results table

| # | Command | Expected | Sim observed | Real observed | Match? |
|---|---|---|---|---|---|
| 1 | `/cmd_vel` linear.x +0.1 | moves forward | | | |
| 2 | `/cmd_vel` linear.x −0.1 | moves backward | | | |
| 3 | `/cmd_vel` angular.z +0.5 | spins left / CCW | | | |
| 4 | `/cmd_vel` angular.z −0.5 | spins right / CW | | | |
| 5 | `/cmd_vel` +0.1 / +0.4 | arcs forward-left | | | |
| 6 | `/cmd_vel` +0.1 / −0.4 | arcs forward-right | | | |
| 7 | right shoulder −0.785 | arm back | | | |
| 8 | right shoulder 3.14 | arm overhead | | | |
| 9 | right elbow −0.873 | forearm folded in | | | |
| 10 | right elbow 0.0 | arm straight | | | |
| 11 | left shoulder −0.785 | arm back (mirrors #7) | | | |
| 12 | left shoulder 3.14 | arm overhead (mirrors #8) | | | |
| 13 | left elbow −0.873 | forearm folded in | | | |
| 14 | left elbow 0.0 | arm straight | | | |
| 15 | neck yaw −1.571 | head turns right | | | |
| 16 | neck yaw +1.571 | head turns left | | | |
| 17 | right gripper −0.785 | fingers closed | | | |
| 18 | right gripper +1.047 | fingers open | | | |
| 19 | left gripper −0.785 | fingers closed | | | |
| 20 | left gripper +1.047 | fingers open | | | |
| 21 | both arms mirror (§2j) | mirrored poses | | | |

### Known sim/hardware differences that are NOT direction bugs

- **Speed.** Sim allows ±30 rad/s at the wheel command interface; hardware clamps
  to 4.5 rad/s (≈0.27 m/s). Same `/cmd_vel`, faster sim robot.
- **Gripper mimic joints.** Sim declares `*_gripper_finger2_joint` as mimic
  joints in ros2_control; hardware does not — it drives `finger1` only and the
  second finger is mechanically linked. `/joint_states` therefore lists more
  joints in sim than on hardware.
- **`effort`.** `nan` on hardware, populated in sim.
- **EKF availability.** Sim starts it inside `sim.launch.py`; hardware needs
  `bringup.launch.py` as well.
- **Battery and face display.** Hardware only — neither is simulated.

---

## 4. Battery

```bash
ros2 topic echo /battery_state --once
```

Hardware only. Polled once per second internally. Meaningful fields: `voltage`
(V), `current` (A, negative while discharging), `percentage` (0.0–1.0 SOC).
Everything else (`temperature`, `charge`, `capacity`, cell-level fields) is
unavailable over CDC and stays at defaults.

---

## 5. IMU

```bash
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data
```

Polled at ~25 Hz (decimated from the 50 Hz control loop). Linear acceleration in
m/s², angular velocity in rad/s (converted from the wire's deg/s). Present in
both sim (via the Gazebo bridge, same topic name) and hardware.

`orientation_covariance[0]` is `−1` — there is no orientation estimate, only
rates and accelerations. The EKF consumes `angular_velocity.z` alone.

---

## 6. Face / LED matrix display

Hardware only — there is no simulated face display.

Raw pass-through topic — publish the exact `CMD_MATRIX_ACTION` byte payload
(action code + that action's own fields) as a `std_msgs/msg/UInt8MultiArray`. ROS
does not interpret or name any of this; it's a dumb pipe straight to the ESP.
Fire-and-forget — nothing is sent unless you publish, so this costs nothing at
idle (unlike the polled sensors above).

```bash
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [<action_code>, <payload...>]}" --once
```

| Code | Action | Payload | Notes |
|---|---|---|---|
| 1 | `SET_TEXT` | ASCII bytes | e.g. `[1, 72, 73]` = "HI" |
| 2 | `SET_COLOR` | R, G, B | **Sets the text/animation tint only — does NOT fill the whole matrix by itself.** See note below. |
| 3 | `SET_ANIMATION` | 1B anim ID | ID → animation mapping isn't documented anywhere available to this repo; ask firmware owner |
| 4 | `SET_BRIGHTNESS` | 1B (or 4B) 0–255 | |
| 5 | `SET_SPEED` | 1B (or 4B) | |
| 9 | `PLAY` | — | resume animation |
| 10 | `PAUSE` | — | **stop the animation loop before manual pixel control, or your pixels get overwritten on the next animation frame** |
| 11 | `GET_STATUS` | — | triggers `RESP_MATRIX_STATUS` (0x58) reply — **not parsed/exposed in ROS**, command sends fine but the response goes nowhere |
| 12 | `SET_PIXEL` | X, Y, R, G, B | verified working — single-pixel writes land correctly |
| 13 | `CLEAR` | — | |
| 14 | `SET_FRAME` | 384B RGB (16×8 grid) | too large to hand-type via `ros2 topic pub`; needs a small script |

```bash
# Pause any running animation first (required before SET_PIXEL or SET_TEXT)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [10]}" --once

# Play a specific built-in animation ID (e.g., Animation ID 1)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [3, 1]}" --once

# Adjust animation playback speed (e.g., speed=50)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [5, 50]}" --once

# Set text / animation tint color to Green (R=0, G=255, B=0)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [2, 0, 255, 0]}" --once

# Show "HI" text
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [1, 72, 73]}" --once

# Set a single pixel to red (X=0, Y=0, R=255, G=0, B=0)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [12, 0, 0, 255, 0, 0]}" --once

# Adjust brightness (0-255, e.g. 128)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [4, 128]}" --once

# Resume / activate animation engine first if CLEAR doesn't respond
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [9]}" --once

# Clear display (works reliably after PLAY [9] activates the matrix engine)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [13]}" --once

# Send a full solid RED frame (Action 14 = SET_FRAME, 384 bytes RGB for 16x8 grid)
python3 -c "import rclpy, time; from std_msgs.msg import UInt8MultiArray; rclpy.init(); n=rclpy.create_node('f'); p=n.create_publisher(UInt8MultiArray, '/face/matrix_action', 10); time.sleep(0.5); m=UInt8MultiArray(); m.data=[14] + [255, 0, 0]*128; p.publish(m); print('Full RED frame published'); rclpy.shutdown()"
```

> **Full-display color change — important finding:** `SET_COLOR` (action 2) does
> **not** paint the whole matrix a solid color. Bench-testing on 2026-08-24 showed
> it has no visible effect on its own — sending it changes nothing on the display.
> This matches `RESP_MATRIX_STATUS`'s field naming, which calls the equivalent
> state **"Text Color"**: `SET_COLOR` is almost certainly just the tint used when
> text or an animation is actively rendering, not a fill command. `SET_PIXEL` was
> confirmed working (individual pixels update correctly), so the transport and
> firmware are both fine — to get an actual full-red screen, use `SET_FRAME` with
> all 128 pixels set to the same color in one 384-byte packet (see python one-liner above),
> or drive `SET_PIXEL` across every coordinate.

---

## 7. Recovery

- **Controllers wedged / commands not landing:** `./stop_session.sh` then relaunch
  — `Ctrl+C` alone leaves orphaned processes, and a stale `robot_state_publisher`
  or a second `/clock` bridge breaks the next session in ways that look like
  anything except a stale process.
- **ESP unplugged/replugged mid-session:** no action needed — reconnects
  automatically on a backoff (1s → 2s → 4s → 5s cap) once `/dev/esp` reappears.
  Watch the launch terminal for `Reconnected; re-running handshake`.
- **Robot won't stop:** publish the §1a stop command. If it is being overridden,
  check what else is publishing — `ros2 topic info /cmd_vel --verbose`. twist_mux
  gives the joystick lane (priority 100) precedence over navigation (10), so a
  connected joystick at rest can hold the base still against `/cmd_vel`.
