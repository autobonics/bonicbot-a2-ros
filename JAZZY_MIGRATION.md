# bonicbot-a2-ros — Humble → Jazzy

This branch (`jazzy`) moves the A2 ROS stack from ROS 2 Humble (Ubuntu 22.04) to
ROS 2 Jazzy (Ubuntu 24.04), with Gazebo Harmonic for the simulation.

**Status:** the code is ported but has **not been built or run yet**. The plan is:

1. Simulation on the dev PC, Jazzy in Docker (below)
2. The demo A2 on a **Raspberry Pi 5**, bare metal, Ubuntu 24.04 + Jazzy (below)
3. The S-series ROS repos
4. `bonicOS-robot-app` last

robot_app is **not** part of steps 1–2. This port follows the standard Jazzy setup and
does not keep robot_app's current interface working. It breaks in two places until step 4
(see the end of this file).

---

## What changed, and why

| Area | Change | Why |
|---|---|---|
| Base image | `ros:humble-ros-base` → `ros:jazzy-ros-base`, all `ros-humble-*` → `ros-jazzy-*` | — |
| `v4l2_camera` in the image | replaced with `camera_ros` | The package already depended on `camera_ros`; the apt list was stale |
| tf2 deadlock guard | **removed** from `Dockerfile.ros` | Humble shipped the bug (#966) in 0.25.23 and the fix (#982) later. Jazzy got both in the same release (0.36.23), so no Jazzy tf2 has the bug without the fix |
| **Velocity commands** | **TwistStamped end to end.** twist_mux `use_stamped: true` and outputs straight into `/diff_cont/cmd_vel`; Nav2 `enable_stamped_cmd_vel: true` (controller_server, velocity_smoother, behavior_server); teleop_twist_joy `publish_stamped_twist: true`; `object_follower.py` publishes TwistStamped; diff_cont's `use_stamped_vel` removed | Jazzy's diff_drive_controller only accepts TwistStamped, and twist_mux 4.5 defaults to stamped. Kilted makes stamped the Nav2 default too |
| **ESP hardware plugin** | Moved to the Jazzy hardware-component API: `on_init(HardwareComponentInterfaceParams)`; the framework creates and owns the state/command interfaces (no more `export_*_interfaces()`); handles are resolved once in `on_configure()` and accessed non-blocking in `read()`/`write()` | The old API is deprecated on Jazzy |
| ESP plugin node | Uses the node ros2_control gives each component (`get_node()`, spun by the controller_manager's executor). The plugin's own node, executor and spin thread are gone | One thread fewer, and the standard place for a component's topics. The node is now `/bonicbot_a2_esp`, after the renamed `<ros2_control>` block (was `RealRobot`) |
| ESP plugin, logging and time | Uses the component's `get_logger()` and `get_clock()` | Standard; stamps follow the controller_manager's clock |
| ESP plugin, wheel resend cache | Made a member and reset on activate and on reconnect (it was a function `static`) | A rebooted ESP could otherwise miss an unchanged wheel command after a reconnect |
| controller_manager URDF | No longer passed as a parameter | Jazzy's controller_manager reads the URDF from the `/robot_description` **topic** (robot_state_publisher already publishes it) |
| **Spawner** | **One** spawner for all seven controllers, with `--param-file controllers.yaml` (plus the calibration overrides file, after it) | One process and one controller_manager handshake instead of seven. `--param-file` is the documented Jazzy way to give controllers their parameters. The spawner forwards a file only to controllers that have a section in it, so the overrides reach diff_cont alone |
| **Nav2 composition** | All Nav2 servers run as components in one `component_container_isolated` process (`nav2_container`). `use_composition:=false` gives one process per server | nav2_bringup's default on Jazzy. On the Pi: one process and one DDS participant instead of ten |
| Nav2 recoveries | behavior_server's `cmd_vel` → `cmd_vel_nav`, so spin/backup/drive_on_heading pass through the velocity smoother | Same as nav2_bringup. Recoveries are now acceleration-limited like path following |
| Sim gripper mimic joints | `finger2` joints: mimic params and command interface removed from the sim `<ros2_control>` tag | Jazzy reads mimic from the URDF `<mimic>` tag (already in `gripper.xacro`) and **refuses to start the whole system** if a mimic joint has a command interface |
| **slam_toolbox** | Launched as a `LifecycleNode`, then configured and activated | On Jazzy it is a lifecycle node. As a plain `Node` it stays unconfigured: the process runs, but there is no `/map` and no `map->odom` |
| Nav2 `bt_navigator` | `plugin_lib_names` list removed | Jazzy loads all built-in BT plugins itself and *appends* this list, so listing built-ins registers them twice and bt_navigator fails |
| Nav2 plugin names | `nav2_navfn_planner/NavfnPlanner` → `::`, `nav2_behaviors/X` → `nav2_behaviors::X` | Jazzy's plugin XMLs only register the `::` names |
| Nav2 controller_server | `progress_checker_plugin` → `progress_checker_plugins: ["progress_checker"]` | Renamed (now a list) |
| Nav2 behavior_server | `costmap_topic`/`footprint_topic` → `local_costmap_topic`/`local_footprint_topic`; `global_frame: odom` → `local_frame: odom` + `global_frame: map` | Renamed; behaviors now work in `local_frame` |
| Nav2 RPP | `use_interpolation` removed | The parameter no longer exists (RPP always interpolates) |
| Nav2 `*_rclcpp_node` sections | removed | Those helper nodes no longer exist |
| Gazebo | `IGN_GAZEBO_RESOURCE_PATH` dropped | Harmonic reads `GZ_SIM_RESOURCE_PATH` only. Worlds and sensors already used Harmonic names |
| Discovery | `ROS_LOCALHOST_ONLY=1` → `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in the session scripts and the ROS services in `docker-compose.yml` | Jazzy deprecates the old variable |
| `stop_session.sh` | Also sweeps `camera_ros/camera_node` and `component_container` | Both run from `/opt/ros`, so no other pattern caught them; a surviving camera node holds the sensor |

`nav2_params_sim.yaml` received the same Nav2 changes as `nav2_params.yaml`.

### What did NOT change

- **The ESP firmware and CDC protocol:** every byte on the wire is the same, and so is
  the plugin's control logic (handshake, reconnect, servo mapping, dead-IMU gate, Wi-Fi
  relay, face display). Only its ROS-facing API moved.
- **IMU and battery** are still published by the plugin itself (`/imu/data`,
  `/battery_state`), not through `imu_sensor_broadcaster`. The dead-IMU gate stops
  publishing when the ESP reports no gravity, and a broadcaster would publish those
  zeros to the EKF anyway.
- **Topic names, launch file names and arguments, the Nav2 action names**, and the
  slam_toolbox `paused_new_measurements` parameter.
- **Tuning:** all Nav2, EKF and SLAM values are unchanged.

---

## Step 1 — simulation on the dev PC (Jazzy in Docker)

Docker keeps Jazzy apart from the Humble setup S1 still uses. Use a **separate clone**:
building Jazzy into the Humble checkout's `install/` would break it.

```bash
git clone -b jazzy <repo-url> ~/pojects/bonic-jazzy/bonicbot-a2-ros

xhost +local:docker
docker run -it --rm --net=host --gpus all \
  -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/pojects/bonic-jazzy/bonicbot-a2-ros:/a2 \
  osrf/ros:jazzy-desktop-full bash
```

- `--gpus all` needs the NVIDIA Container Toolkit on the dev PC.
- `ROS_DOMAIN_ID=42` keeps this sim apart from any Humble robot_app or S1 session on
  the same machine (`--net=host` would otherwise put them on one graph).

Inside the container:

```bash
cd /a2
apt-get update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch bonicbot_a2_sim sim.launch.py world:=obstacle_world.sdf
# second shell (docker exec), same sourcing:
ros2 launch bonicbot_a2_nav mapping.launch.py use_sim_time:=true
#   or: ros2 launch bonicbot_a2_nav navigation.launch.py use_sim_time:=true map_name:=<map>.yaml
```

## Step 2 — demo A2 on a Raspberry Pi 5, bare metal

### OS and ROS

1. Flash **Ubuntu Server 24.04 LTS (64-bit)** for the Pi 5 with Raspberry Pi Imager.
2. Install ROS 2 Jazzy:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep v4l-utils
sudo rosdep init && rosdep update
```

3. Permissions: `sudo usermod -aG dialout,input,video $USER`, then log out and back in.

### Build

```bash
mkdir -p ~/bonic && cd ~/bonic
git clone -b jazzy <repo-url> bonicbot-a2-ros
cd bonicbot-a2-ros
sudo ./scripts/setup_udev.sh          # /dev/esp, /dev/lidar
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/hardware src/nav --ignore-src -r -y
colcon build --packages-up-to bonicbot_a2_hardware bonicbot_a2_nav \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The sim package is skipped on purpose, because Gazebo is never needed on the robot.

**Calibration:** `start_session_robot.sh` reads wheel radius, wheel separation and
encoder CPR from `../bonicOS-robot-app/robot_config.yaml`. Without that checkout it uses
the defaults. To test with the demo robot's real values, either copy its
`robot_config.yaml` there, or `export WHEEL_RADIUS=… WHEEL_SEPARATION=… ENCODER_CPR=…`
before starting.

### Camera on the Pi 5 — the most likely snag

The Pi 5 has a different camera pipeline from the Pi 4 (rp1-cfe + PiSP instead of
unicam + bcm2835-isp), so:

1. **The camera id differs.** The default in `camera.launch.py`
   (`/base/soc/i2c0mux/i2c@1/ov5647@36`) is the Pi 4 path. Run the camera once, read
   the id it reports ("no camera selected, using default: …"), and pass it:
   `camera:=<id>`.
2. **ROS's libcamera may not drive the Pi 5's ISP.** `ros-jazzy-libcamera` is the
   plain upstream build, and camera_ros's README warns that it may lack full Raspberry
   Pi support. If `camera_node` finds no camera, build the Raspberry Pi fork into the
   workspace:

```bash
sudo apt install -y python3-colcon-meson meson ninja-build pkg-config cmake \
  libyaml-dev python3-yaml python3-ply python3-jinja2 libudev-dev libyuv-dev \
  libboost-dev libgnutls28-dev openssl
sudo apt remove -y ros-jazzy-libcamera ros-jazzy-camera-ros   # so the workspace copies are the only ones
cd ~/bonic/bonicbot-a2-ros/src
git clone https://github.com/raspberrypi/libcamera.git
git clone https://github.com/christianrauch/camera_ros.git
cd .. && rosdep install --from-paths src --ignore-src -r -y --skip-keys=libcamera
colcon build --packages-up-to camera_ros bonicbot_a2_hardware bonicbot_a2_nav \
  --meson-args -Dpipelines=rpi/pisp -Dipas=rpi/pisp
```

The libcamera build takes a while on the Pi.

Leave these two clones out of any commit. They exist only for the Pi 5 bench test.

### Run

```bash
./start_session_robot.sh               # hardware + EKF, logs in ./logs/
./stop_session.sh                      # when done
```

For mapping and navigation without robot_app:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch bonicbot_a2_nav mapping.launch.py
ros2 launch bonicbot_a2_nav navigation.launch.py map_name:=<map>.yaml
```

Every shell you run `ros2` commands from needs that `export`, or it sees an empty graph.

---

## Test checklist

Each line checks one of the Jazzy changes above. Run in sim first, then on the Pi 5.

| # | Check | Command / what to look for |
|---|---|---|
| 1 | All 7 controllers **active** | `ros2 control list_controllers`. Read every line: the scripts' own check does not catch `inactive` (see Open items) |
| 2 | URDF reached the controller manager | No repeating "Waiting for data on 'robot_description' topic" in the hardware log |
| 3 | Stamped chain | `ros2 topic info -v /diff_cont/cmd_vel`: type `TwistStamped`, publisher `twist_mux` |
| 4 | Driving through twist_mux | `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"` moves the base; it stops ~0.5 s after Ctrl+C. A zero header stamp is accepted: diff_cont replaces it with the current time |
| 5 | Joystick wins over `/cmd_vel` | Hold the enable button while #4 is publishing |
| 6 | Calibration overrides applied | `ros2 param get /diff_cont wheel_radius` equals the robot's value, not 0.06 |
| 7 | Arms, head and grippers move | Same commands as on Humble; in sim, `finger2` follows `finger1` |
| 7b | Plugin node | `ros2 node list` shows `/bonicbot_a2_esp`; `/imu/data` and `/battery_state` publish; unplug and replug the ESP USB and driving resumes |
| 8 | slam_toolbox active | `ros2 lifecycle get /slam_toolbox` → `active`; `/map` publishes |
| 9 | Nav2 comes up, composed | `ros2 component list` shows every server under `/nav2_container`; no "already registered" from bt_navigator; `ros2 lifecycle get /bt_navigator` → `active` |
| 10 | A navigation goal completes | Including one recovery (spin/backup) |
| 11 | Camera | `ros2 topic hz /face_camera/image_raw` ≈ 6 Hz |
| 12 | Nothing leaks to Wi-Fi | From a laptop on the same network with default ROS settings, `ros2 topic list` shows none of the robot's topics |

---

## What breaks until robot_app is ported (step 4)

- **Driving from the app:** robot_app publishes plain Twist on `/cmd_vel`, and
  twist_mux now only accepts TwistStamped. Set `cmd_vel_stamped: True` for series A
  in `app/config.py` (the flag already exists) and make sure the publisher fills the
  header stamp.
- **The robot_app image itself:** it is built on this image, so it moves to Python 3.12
  and Ubuntu 24.04, whose pip refuses system-wide installs without
  `--break-system-packages`. Every pinned wheel needs checking on 3.12/arm64, along with
  the `ros_distro` defaults in `app/config.py` and the bonicOS-host fleet compose
  environment.

## Open items

- **Controller check in the session scripts:** `grep -v 'active'` also drops
  `inactive` lines, so a configured-but-not-activated controller goes unreported. This
  predates the port.
- **Fleet cost:** a new base image means every robot downloads the whole image again
  once. Ship it as one release together with the robot_app change.
