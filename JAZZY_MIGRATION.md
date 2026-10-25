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
| **Lidar driver** | `rplidar_ros` built from source: Slamtec's `ros2` branch, pinned to commit `24cc9b6d` in `external.repos`, imported into `src/external/` (gitignored). `ros-jazzy-rplidar-ros` removed from the image | Jazzy's apt package (2.1.0) bundles SDK 1.12, which cannot start an RPLIDAR C1 scan (`Cannot start scan: 80008000`, A2 Pi 5). Slamtec says no newer Jazzy release is coming soon ([#164](https://github.com/Slamtec/rplidar_ros/issues/164)). The pinned commit has SDK 2.1 and C1 support; verified on the A2: 10 Hz, 720 points/rev |
| **Camera library** | `libcamera` (Raspberry Pi fork `v0.7.2+rpt20260817`) and `camera_ros` (`0.7.0`) built from source, pinned in `external.repos`. `ros-jazzy-camera-ros` removed from the image; `camera.launch.py` picks the sensor id per board | Upstream libcamera, which ROS packages, finds no CSI camera on a Pi 5 under Raspberry Pi's kernel (node names differ; see "Camera on the Pi 5") |
| `v4l2_camera` in the image | replaced with `camera_ros` | The package already depended on `camera_ros`; the apt list was stale |
| tf2 deadlock guard | **removed** from `Dockerfile.ros` | Humble shipped the bug (#966) in 0.25.23 and the fix (#982) later. Jazzy got both in the same release (0.36.23), so no Jazzy tf2 has the bug without the fix |
| **Velocity commands** | **TwistStamped end to end.** twist_mux `use_stamped: true` and outputs straight into `/diff_cont/cmd_vel`; Nav2 `enable_stamped_cmd_vel: true` (controller_server, velocity_smoother, behavior_server); teleop_twist_joy `publish_stamped_twist: true`; `object_follower.py` publishes TwistStamped; diff_cont's `use_stamped_vel` removed | Jazzy's diff_drive_controller only accepts TwistStamped, and twist_mux 4.5 defaults to stamped. Kilted makes stamped the Nav2 default too |
| **ESP hardware plugin** | Moved to the Jazzy hardware-component API: `on_init(HardwareComponentInterfaceParams)`; the framework creates and owns the state/command interfaces (no more `export_*_interfaces()`); handles are resolved once in `on_configure()` and accessed non-blocking in `read()`/`write()` | The old API is deprecated on Jazzy |
| ESP plugin node | Uses the node ros2_control gives each component (`get_node()`, spun by the controller_manager's executor). The plugin's own node, executor and spin thread are gone | One thread fewer, and the standard place for a component's topics. The node is now `/bonicbot_a2_esp`, after the renamed `<ros2_control>` block (was `RealRobot`) |
| ESP plugin, IMU/battery publishing | `/imu/data` and `/battery_state` go through `realtime_tools::RealtimePublisher` (`try_publish`) | They are published from inside the 50 Hz control loop. The loop now only hands the message over; a background thread publishes, so the loop never waits on ROS. A sample is dropped only if the previous one is still in flight. The power-button and Wi-Fi publishers stay normal, since they must never drop |
| ESP plugin, logging and time | Uses the component's `get_logger()` and `get_clock()` | Standard; stamps follow the controller_manager's clock |
| ESP plugin, wheel resend cache | Made a member and reset on activate and on reconnect (it was a function `static`) | A rebooted ESP could otherwise miss an unchanged wheel command after a reconnect |
| controller_manager URDF | No longer passed as a parameter | Jazzy's controller_manager reads the URDF from the `/robot_description` **topic** (robot_state_publisher already publishes it) |
| **Spawner** | **One** spawner for all seven controllers, with `--param-file controllers.yaml` (plus the calibration overrides file, after it) | One process and one controller_manager handshake instead of seven. `--param-file` is the documented Jazzy way to give controllers their parameters. The spawner forwards a file only to controllers that have a section in it, so the overrides reach diff_cont alone |
| **Nav2 composition** | All Nav2 servers run as components in one `component_container_isolated` process (`nav2_container`). `use_composition:=false` gives one process per server | nav2_bringup's default on Jazzy: one process and one DDS participant instead of ten. One process per server was tried on the Pi 5 (2026-10-24) to avoid a dropped `load_node` reply; the same race then hit the lifecycle manager's `change_state` instead. The fix was the DDS setup ("ROS traffic pinned to 127.0.0.1"), and fewer participants helps it |
| Navigation start | `navigation_autostart` launch argument for `lifecycle_manager_navigation` only (default: `autostart`). robot_app passes `false` and starts the navigation servers through `manage_nodes` STARTUP once AMCL has a pose; map_server + AMCL still autostart | Jazzy's costmaps wait for `map → base_link` at most 60 s (`initial_transform_timeout`), then the whole bringup aborts; started after localization, they never wait |
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
| Discovery | `ROS_LOCALHOST_ONLY=1` → `config/fastdds_localhost.xml` (UDP pinned to 127.0.0.1 + shared memory, as Humble's setting built it) loaded through `FASTRTPS_DEFAULT_PROFILES_FILE`, with `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT`, in the session scripts and the image | Jazzy's `LOCALHOST` range does not pin the transport (a Wi-Fi change strands running nodes) and discovers more slowly in a busy graph. See "ROS traffic pinned to 127.0.0.1" |
| DDS shared memory | `<segment_size>` 4 MB in `config/fastdds_localhost.xml` (Fast DDS default 512 KB); `ipc: host` on every service in `docker-compose.yml` | A message bigger than the segment falls back to UDP fragments that drop under load: on the S1 the docking camera's raw frames reached the AprilTag detector at 3.4 Hz instead of 10 and detection failed. With 4 MB, 0.9 MB and 1.6 MB messages arrive at 9.8 of 10 Hz. Containers need the host's /dev/shm to share it |
| `stop_session.sh` | Also sweeps `camera_ros/camera_node` and `component_container` | Both run from `/opt/ros`, so no other pattern caught them; a surviving camera node holds the sensor |

The unused `bonicbot_a2_sim/config/nav2_params_sim.yaml` was deleted: nothing loads it, and navigation uses `nav2_params.yaml` in simulation too.

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
- **Tuning:** all Nav2, EKF and SLAM values are unchanged (the fleet's Humble values).

### Localization: tried on the Pi 5 and reverted (robot 038, 2026-10-13 to 10-23)

Heading in the app over-rotated ~5% per turn (wheel slip), so the EKF was switched to
take heading from the gyro only (wheels as velocities, `odom0` yaw off). Spin tests
looked right (gyro within 1% over 720°), but **AMCL got much worse**: it alternated
between poses ~20° apart and corrected by up to 0.58 m / 29°. Raising AMCL particles to
500–2000 and lowering `alpha1`/`alpha2` to 0.05 reduced AMCL's *reported* uncertainty
but not the real errors. Going back to the fleet's `ekf.yaml` and AMCL values made
localization much better at once. All three changes are reverted.

Lessons: judge localization by where the robot really is (scan-on-map alignment, or the
operator's eye), never by AMCL's own covariance, which lower noise settings simply
shrink; and a configuration proven on the Humble fleet beats a change that only passes
an odometry test. The plugin's gyro zero-offset tracking (subtracting the bias learned
while the wheels are stopped) is kept: it only removes bias from `/imu/data`.
`wheel_separation` stays the measured 0.319 m.

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
vcs import src/external < external.repos   # on the Mac, before syncing (or clone by hand, see the file)
rosdep install --from-paths src/hardware src/nav src/external --ignore-src -r -y --skip-keys libcamera
colcon build --packages-up-to bonicbot_a2_hardware bonicbot_a2_nav \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The sim package is skipped on purpose, because Gazebo is never needed on the robot.

**Calibration:** `start_session_robot.sh` reads wheel radius, wheel separation and
encoder CPR from `../bonicOS-robot-app/robot_config.yaml`. Without that checkout it uses
the defaults. To test with the demo robot's real values, either copy its
`robot_config.yaml` there, or `export WHEEL_RADIUS=… WHEEL_SEPARATION=… ENCODER_CPR=…`
before starting.

### ROS traffic pinned to 127.0.0.1 (`config/fastdds_localhost.xml`)

**Setup.** `config/fastdds_localhost.xml` is the complete Fast DDS configuration: built-in
transports off, UDPv4 whitelisted to `127.0.0.1`, shared memory, default discovery. That
is what Humble's `ROS_LOCALHOST_ONLY=1` built inside rmw_fastrtps (`participant.cpp`,
humble). Every ROS process loads it through `FASTRTPS_DEFAULT_PROFILES_FILE`, with
`ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` so ROS adds nothing of its own:

- the session scripts export both;
- `Dockerfile.ros` copies the profile to `/ws/config/` and sets both as `ENV`, which the
  stack, robot_app's image (built on this one) and `docker exec … ros2` inherit. The
  compose files must not set the range: it would override the image;
- robot_app's `install-service.sh` writes both into the bare-metal env file. Without the
  file it falls back to `LOCALHOST` and warns, because `SYSTEM_DEFAULT` without the
  profile would discover across the subnet;
- interactive shells need both too (`~/.bashrc`). A shell with a different setup sees a
  partial graph.

**Why not Jazzy's `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`** (all found on the A2 Pi 5):

1. *Wi-Fi changes* (2026-10-23). `LOCALHOST` only turns off multicast and adds
   `127.0.0.1` as a discovery peer; transports stay default, so each node advertises the
   addresses the machine had **at startup**. After the robot moved from a phone hotspot to
   the home router (new IP), nodes started before the change could not be reached by
   nodes started after it: robot_app lost Nav2. Fast DDS never updates a participant's
   locators ([Clearpath](https://docs.clearpathrobotics.com/docs/ros2humble/ros/networking/ros2_communication/),
   [Fast-DDS #6055](https://github.com/eProsima/Fast-DDS/issues/6055)). With the profile,
   the same switch (full stack and robot_app running) kept all 40 nodes visible.
2. *A layered profile is worse* (2026-10-24). rmw_fastrtps (8.4.4) applies `LOCALHOST`
   **after** loading the XML and appends its own SHM and UDP transports. A profile used
   with `LOCALHOST` therefore gives a mixed setup. On the robot that coincided with a
   dropped `load_node` reply (Nav2 half-loaded), a dropped `change_state` reply (Nav2's
   lifecycle manager waits forever on it), and robot_app hanging inside Fast DDS.
3. *Discovery in a busy graph* (2026-10-24 benchmark). A Nav2-style bring-up (map_server,
   AMCL and four other servers plus a lifecycle manager, all new processes) was repeated
   50 times per setup, alone and with 20 other ROS processes running:

| Setup | Alone | With 20 other processes |
|---|---|---|
| Jazzy `LOCALHOST` | 50/50, 2.6 s | 50/50, median 14.3 s |
| `LOCALHOST` + profile (mixed) | 50/50, 2.6 s | **49/50: one dropped reply, stuck**; median 18.3 s |
| **This profile + `SYSTEM_DEFAULT`** | 50/50, 2.6 s | **50/50, median 8.1 s** |
| Same without shared memory | 50/50, 2.6 s | 50/50, median 14.7 s |

The dropped reply is the known ROS 2 service race ([rclcpp #2760](https://github.com/ros2/rclcpp/issues/2760),
[rmw_fastrtps #749](https://github.com/ros2/rmw_fastrtps/issues/749)): the server sends
before it has discovered the caller's reply reader. The code is identical in Humble and
Jazzy; slower discovery widens the window. Humble's setup discovers fastest here, which
matches the Humble fleet rarely seeing it.

Shared memory stays (as on Humble). robot_app's teardown no longer SIGKILLs nodes that
are still shutting down after SIGINT, which is what left a shared-memory lock held when
robot_app hung.

### Camera on the Pi 5

Confirmed on the A2 Pi 5 on 2026-10-23: the kernel finds the ov5647 (`rp1-cfe … Using
sensor ov5647 10-0036`), but `ros-jazzy-libcamera` reports "no cameras available".
libcamera's rpi/pisp pipeline looks the CSI receiver's video nodes up by name. Raspberry
Pi's kernel driver, which Ubuntu's raspi kernel uses, names them `rp1-cfe-fe_image0`;
upstream libcamera (what ROS packages) expects mainline's `rp1-cfe-fe-image0`, so it
never matches ("Unable to acquire a CFE instance" at debug level). Raspberry Pi's fork
matches their kernel, and camera_ros's README gives building it in the workspace as the
only way to use it on Ubuntu. Any CSI camera on a Pi 5 hits this; a USB camera does not.

So `libcamera` (Raspberry Pi fork, `v0.7.2+rpt20260817`) and `camera_ros` (`0.7.0`) are
built from source, pinned in `external.repos`, and the apt `ros-jazzy-libcamera` and
`ros-jazzy-camera-ros` are not installed. `camera.launch.py` picks the sensor's
libcamera id for the board (Pi 4 or Pi 5); `camera:=<id>` still overrides it.

On the Pi 5, after the step 2 workspace setup:

```bash
sudo apt install -y python3-colcon-meson meson ninja-build pkg-config python3-jinja2 \
  python3-ply python3-yaml libyaml-dev libssl-dev openssl libudev-dev nlohmann-json3-dev
sudo apt remove -y ros-jazzy-camera-ros ros-jazzy-libcamera   # the workspace copies replace them
colcon build --packages-select libcamera camera_ros bonicbot_a2_hardware \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --meson-args -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=disabled -Dgstreamer=disabled -Dpycamera=disabled -Drpi-awb-nn=disabled \
    -Dcam=disabled -Dqcam=disabled -Dlc-compliance=disabled -Ddocumentation=disabled -Dtest=false
```

`external.repos` is imported on the Mac (with libcamera's `subprojects/libpisp` at
v1.5.0) and rsynced, so nothing is cloned on the robot.

**When to go back to apt.** The custom build is temporary. Recheck when a new
`ros-jazzy-libcamera` or Ubuntu raspi kernel arrives: install `ros-jazzy-camera-ros` on a
Pi 5 and run `ros2 run camera_ros camera_node`. If it logs "Registered camera … rpi/pisp"
rather than "no cameras available", remove the libcamera and camera_ros entries from
`external.repos`, put `ros-jazzy-camera-ros` back in `Dockerfile.ros`, and drop the
meson packages and `--meson-args` there. The rotation line in `config.txt` stays either
way.

**Upside-down mounting on the Pi 5.** The head camera is mounted inverted, and the fix
is still the device tree, as on the Pi 4, but the line differs. On the A2 the ribbon is
in the Pi 5's **CAM0** connector, and the overlay defaults to CAM1, so `cam0` is
required. `camera_auto_detect` must be off, or it loads its own overlay without the
rotation. In `/boot/firmware/config.txt`:

```
camera_auto_detect=0
dtoverlay=ov5647,rotation=180,cam0
```

Without `cam0` the sensor is looked for on the other port and its probe fails
(`ov5647 11-0036: ... i2c read error ... probe failed with error -121`). Check with
`v4l2-ctl -d /dev/v4l-subdevN -l | grep camera_sensor_rotation` (should read 180).
bonicOS-image writes the Pi 4 line (`dtoverlay=ov5647,rotation=180`) for series A; a
Pi 5 image needs the line above. Verified on the A2 Pi 5 on 2026-10-23: upright, 640x480
at 6.0 Hz.

### Run

```bash
./start_session_robot.sh               # hardware + EKF, logs in ./logs/
./stop_session.sh                      # when done
```

For mapping and navigation without robot_app:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_localhost.xml
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

## robot_app

robot_app's `jazzy` branch is the matching port: it publishes TwistStamped, reads Jazzy's
Nav2 results, builds on this image with Python 3.12, and is Jazzy-only. Its
`JAZZY_MIGRATION.md` has the details.

## Open items

- **Controller check in the session scripts:** `grep -v 'active'` also drops
  `inactive` lines, so a configured-but-not-activated controller goes unreported. This
  predates the port.
- **Fleet cost:** a new base image means every robot downloads the whole image again
  once. Ship it as one release together with the robot_app change.
