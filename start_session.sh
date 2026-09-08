#!/usr/bin/env bash
# Starts the BonicBot A2 SIM base stack — Gazebo + controllers + EKF + teleop,
# backgrounded with a log file. That is all this script owns.
#
# The nav stack (mapping OR navigation) is NOT started here: robot_app brings
# a mapping session (slam_toolbox + Nav2 core) or a navigation session
# (map_server + AMCL + Nav2) up on demand and resumes the last map at boot.
# See bonicOS-robot-app/app/managers/nav_mode_manager.py.
#
# robot_app is not started here either — it is a persistent service that
# supervises this stack, so it has to outlive it. Install it once with
# bonicOS-robot-app/deploy/install-service.sh.
#
# Real robot: use ./start_session_robot.sh instead.
#
# Usage: ./start_session.sh [--clean]
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

# ── preflight: never start on top of a previous session ──────────────────────
# Starting twice does not fail loudly, it fails SILENTLY. Two Gazebo clock
# bridges make ROS time run backwards, which wipes every tf2 buffer and stops
# Nav2 planning while the dashboard still looks healthy. A stale
# robot_state_publisher is worse still: gz_ros2_control can read ITS URDF
# instead of this one and load the wrong robot's hardware plugins, so every
# controller fails to activate with "Not existing" interfaces.
# See the header of stop_session.sh.
if [ -n "$("$A2_ROS/stop_session.sh" --check 2>/dev/null)" ]; then
    if $CLEAN; then
        echo "previous session found — tearing it down first..."
        "$A2_ROS/stop_session.sh"
    else
        echo "ERROR: a previous session is still running." >&2
        "$A2_ROS/stop_session.sh" --check >&2
        echo >&2
        echo "Run ./stop_session.sh first, or ./start_session.sh --clean." >&2
        exit 1
    fi
fi

source /opt/ros/humble/setup.bash
source "$A2_ROS/install/setup.bash"
export ROS_LOCALHOST_ONLY=1

# nav's map loading (BONICBOT_MAPS_DIR) and robot_app's save/list/load
# (MAPS_DIR) must agree on one directory.
export BONICBOT_MAPS_DIR="$A2_ROS/maps"
export MAPS_DIR="$A2_ROS/maps"
# robot_app passes this through to the nav launch files it spawns.
export USE_SIM_TIME=true
# Series A, so robot_app picks the bonicbot_a2_* packages out of ROBOT_CONFIG.
export ROBOT_SERIES=A

start() {
    local name="$1"; shift
    echo "starting $name..."
    nohup setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    echo $! > "$LOG_DIR/$name.pid"
    disown
}

cd "$A2_ROS"
start sim ros2 launch bonicbot_a2_sim sim.launch.py world:=obstacle_world.sdf
echo "waiting for Gazebo + controllers (cold start can take ~20-30s)..."
sleep 20

# Exactly one publisher on /clock, or the whole graph's TF is unusable. Cheap,
# and it catches the stray whose command line matches no sweep pattern.
clock_pubs="$(timeout 10 ros2 topic info /clock 2>/dev/null \
              | sed -n 's/^Publisher count: //p')"
if [ "$clock_pubs" != "1" ]; then
    echo
    echo "ERROR: /clock has ${clock_pubs:-0} publishers, expected exactly 1." >&2
    echo "Interleaved clocks make ROS time step backwards, wiping every tf2" >&2
    echo "buffer and stopping Nav2 planning. Aborting." >&2
    "$A2_ROS/stop_session.sh" || true
    exit 1
fi

# Controllers must be ACTIVE, not merely loaded. A spawner that timed out
# leaves its controller loaded-but-unconfigured: the node exists and the graph
# looks healthy, but it accepts no commands.
inactive="$(timeout 10 ros2 control list_controllers 2>/dev/null \
            | grep -v 'active' || true)"
if [ -n "$inactive" ]; then
    echo
    echo "WARNING: these controllers are not active:" >&2
    echo "$inactive" >&2
    echo "The robot will not respond to those commands." >&2
fi

echo
if curl -sf --max-time 5 http://127.0.0.1:8080/health >/dev/null 2>&1; then
    echo "robot_app is up (http://127.0.0.1:8080) — it will pick up this base stack."
else
    echo "NOTE: robot_app is not responding on :8080." >&2
    echo "It is a service, not part of this script. Install/start it with:" >&2
    echo "  $ROBOT_APP/deploy/install-service.sh --sim \\" >&2
    echo "      --ros-workspace $A2_ROS --maps-dir $MAPS_DIR" >&2
    echo "  systemctl --user status bonic-robot-app" >&2
fi

echo
echo "Logs: $LOG_DIR/sim.log"
echo "Stop: ./stop_session.sh"
