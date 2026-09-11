#!/usr/bin/env bash
# Starts the BonicBot A2 REAL-ROBOT base stack — the ESP32 CDC link,
# controllers, LiDAR, camera and EKF — backgrounded with log files.
#
# Two launches, not one (mirrors the hardware/nav package split):
#   hardware.launch.py  — controller_manager on /dev/esp, lidar, camera, twist_mux
#   bringup.launch.py   — rsp + EKF (no SLAM, no Nav2)
#
# The nav stack (mapping OR navigation) is NOT started here: robot_app brings
# it up on demand and resumes the last map at boot. robot_app itself is a
# persistent service that must outlive this stack — install it once with
# bonicOS-robot-app/deploy/install-service.sh.
#
# Simulation: use ./start_session.sh instead.
#
# Usage: ./start_session_robot.sh [--clean]
#          --clean   tear down a previous session first instead of refusing
# Stop everything: ./stop_session.sh

set -e

A2_ROS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BONIC_DIR="$(dirname "$A2_ROS")"
ROBOT_APP="$BONIC_DIR/bonicOS-robot-app"
LOG_DIR="$A2_ROS/logs"
mkdir -p "$LOG_DIR"

CLEAN=false
[ "${1:-}" = "--clean" ] && CLEAN=true

# ── preflight: devices ───────────────────────────────────────────────────────
# Fail here with a clear message rather than inside on_configure(), where a
# missing /dev/esp surfaces as a controller_manager that never advertises its
# services and seven spawners timing out.
missing=()
[ -e /dev/esp ]   || missing+=(/dev/esp)
[ -e /dev/lidar ] || missing+=(/dev/lidar)
if [ ${#missing[@]} -gt 0 ]; then
    echo "ERROR: missing device(s): ${missing[*]}" >&2
    echo "Run: sudo ./scripts/setup_udev.sh   (and check the hardware is plugged in)" >&2
    exit 1
fi

# ── preflight: no previous session ───────────────────────────────────────────
# See stop_session.sh's header — a stale robot_state_publisher or controller
# manager breaks the next session in ways that look like anything but a stale
# process.
if [ -n "$("$A2_ROS/stop_session.sh" --check 2>/dev/null)" ]; then
    if $CLEAN; then
        echo "previous session found — tearing it down first..."
        "$A2_ROS/stop_session.sh"
    else
        echo "ERROR: a previous session is still running." >&2
        "$A2_ROS/stop_session.sh" --check >&2
        echo >&2
        echo "Run ./stop_session.sh first, or ./start_session_robot.sh --clean." >&2
        exit 1
    fi
fi

source /opt/ros/humble/setup.bash
source "$A2_ROS/install/setup.bash"
export ROS_LOCALHOST_ONLY=1

export BONICBOT_MAPS_DIR="$A2_ROS/maps"
export MAPS_DIR="$A2_ROS/maps"
export USE_SIM_TIME=false
export ROBOT_SERIES=A

# Per-robot hardware calibration lives in bonicOS-robot-app's robot_config.yaml
# (hardware: encoder_cpr / wheel_radius), NOT here —
# a calibration number is a fact about this one physical robot, not about
# a2-ros's code. Read it and export what ros2_control.xacro's $(optenv...)
# and hardware.launch.py's controller override are wired to pick up. Missing
# file, missing block, or missing key all mean the same thing: leave the env
# var unset and let a2-ros's own defaults apply, exactly as before this
# existed.
ROBOT_CFG="$ROBOT_APP/robot_config.yaml"
if [ -f "$ROBOT_CFG" ]; then
    HW_JSON=$(python3 -c "
try:
    import yaml
    with open('$ROBOT_CFG') as f:
        hw = (yaml.safe_load(f) or {}).get('hardware') or {}
except Exception as e:
    import sys
    print(f'# robot_config.yaml hardware block unreadable: {e}', file=sys.stderr)
    hw = {}
for k in ('encoder_cpr', 'wheel_radius'):
    if k in hw and hw[k] is not None:
        print(f'{k}={hw[k]}')
")
    # Errors go to the session log (this script's stdout/stderr are already
    # redirected there by whatever invokes it), NOT /dev/null — a PyYAML
    # missing from this python3 must be visible, not indistinguishable from
    # "no calibration provisioned".
    while IFS='=' read -r key val; do
        case "$key" in
            encoder_cpr)  export ENCODER_CPR="$val" ;;
            wheel_radius) export WHEEL_RADIUS="$val" ;;
        esac
    done <<< "$HW_JSON"
    [ -n "${ENCODER_CPR:-}${WHEEL_RADIUS:-}" ] &&         echo "hardware calibration from $ROBOT_CFG: ENCODER_CPR=${ENCODER_CPR:-default} WHEEL_RADIUS=${WHEEL_RADIUS:-default}"

# No camera flip key, and nothing to export for one. It was parsed here and
# exported as CAMERA_VERTICAL_FLIP, which no launch file has read since the
# camera moved to libcamera/camera_ros — the old v4l2_camera node parameter it
# fed does not exist on this stack. Rotation is declared in
# /boot/firmware/config.txt by the base OS image (bonicOS-image), where
# libcamera reads it at boot and compensates for it, Bayer phase included.
# See camera.launch.py for why no runtime parameter can do this job.
fi

start() {
    local name="$1"; shift
    echo "starting $name..."
    nohup setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    echo $! > "$LOG_DIR/$name.pid"
    disown
}

cd "$A2_ROS"

start hardware ros2 launch bonicbot_a2_hardware hardware.launch.py
echo "waiting for the ESP link and controllers..."
# on_configure() opens the CDC port and settles it for 500ms before the
# controller_manager can serve anything, so the spawners need a moment.
sleep 12

start nav_bringup ros2 launch bonicbot_a2_nav bringup.launch.py \
    use_sim_time:=false use_nav2:=false
sleep 5

# Controllers must be ACTIVE, not merely loaded — a spawner that timed out
# leaves its controller loaded-but-unconfigured, which looks fine in the node
# graph but accepts no commands.
inactive="$(timeout 10 ros2 control list_controllers 2>/dev/null \
            | grep -v 'active' || true)"
if [ -n "$inactive" ]; then
    echo
    echo "WARNING: these controllers are not active:" >&2
    echo "$inactive" >&2
    echo "Check $LOG_DIR/hardware.log — most likely the ESP link failed to open." >&2
fi

# The ESP must actually be talking, or the robot is deaf while looking healthy.
if ! timeout 10 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    echo
    echo "WARNING: no /joint_states within 10s — the ESP32 link may be down." >&2
    echo "Check $LOG_DIR/hardware.log for CDC read/write errors." >&2
fi

echo
if curl -sf --max-time 5 http://127.0.0.1:8080/health >/dev/null 2>&1; then
    echo "robot_app is up (http://127.0.0.1:8080) — it will pick up this base stack."
else
    echo "NOTE: robot_app is not responding on :8080." >&2
    echo "It is a service, not part of this script. Install/start it with:" >&2
    echo "  $ROBOT_APP/deploy/install-service.sh \\" >&2
    echo "      --ros-workspace $A2_ROS --maps-dir $MAPS_DIR" >&2
    echo "  systemctl --user status bonic-robot-app" >&2
fi

echo
echo "Logs: $LOG_DIR/{hardware,nav_bringup}.log"
echo "Stop: ./stop_session.sh"
