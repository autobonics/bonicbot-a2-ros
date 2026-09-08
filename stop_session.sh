#!/usr/bin/env bash
# Fully stop a BonicBot A2 session (sim or real) and verify nothing survived.
#
# WHY THIS EXISTS — Ctrl+C is not enough.
#
# A `ros2 launch` parent does NOT reliably take its nodes down with it. Ctrl+C
# sends SIGINT to the foreground process group, but nodes that ignore it, or
# that were re-parented, keep running as orphans: robot_state_publisher,
# ros_gz_bridge, ign gazebo, twist_mux, ekf_node, joy/teleop, the Nav2 servers.
#
# Those orphans then break the NEXT session in ways that look like anything
# except a stale process:
#
#   * A leftover robot_state_publisher still serves ITS robot_description over
#     the parameter service. ROS2 permits duplicate node names, so the new
#     session now has two nodes called /robot_state_publisher, and
#     gz_ros2_control's `get_parameters` call can resolve to the WRONG one.
#     It then loads hardware plugins from a URDF that is not the one you
#     launched — every plugin fails, no command/state interfaces are
#     registered, and EVERY controller dies with "Not existing" interfaces.
#     Reads as a controller bug. Is not a controller bug.
#
#   * Two ros_gz_bridge processes bridging Gazebo's /clock publish interleaved
#     samples, so ROS time steps backwards ~40x/second. Every tf2 buffer wipes
#     itself ("Detected jump back in time"), map->base_link lookups fail, and
#     every Nav2 goal fails to plan while AMCL still looks healthy.
#
# So: run this between sessions. Exits non-zero if anything survived, rather
# than letting you start a second session on top of the first.
#
# Usage:
#   ./stop_session.sh           stop everything, verify, report
#   ./stop_session.sh --check   list what is running and exit; kill nothing
#                               (used by the start scripts' preflight)

set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CHECK_ONLY=false
[ "${1:-}" = "--check" ] && CHECK_ONLY=true

# Command-line patterns for processes an A2 session can leave behind.
# Scoped to this workspace's install tree where possible; the rest are node
# executables that run out of /opt/ros and can only be matched by name.
PATTERNS=(
  "$WS/install"                      # anything launched out of this workspace
  "ros2 launch bonicbot_a2"          # the launch parents themselves
  "ign gazebo"                       # Gazebo (sim)
  "gz sim"
  "ruby .*ign gazebo"                # Gazebo's ruby wrapper
  "robot_state_publisher"
  "ros_gz_bridge|parameter_bridge|image_bridge"
  "ros2_control_node|controller_manager|spawner"
  "twist_mux"
  # topic_tools relays (joint_states throttle). MUST be listed explicitly:
  # the binary lives in /opt/ros/humble/lib/topic_tools/, so neither the
  # "$WS/install" nor the "ros2 launch bonicbot_a2" pattern matches it, and a
  # surviving throttle is invisible until you notice /joint_states_throttled
  # running at a multiple of 10 Hz — N orphans publish N interleaved streams
  # onto the same topic, which reads as "the throttle isn't working".
  "topic_tools/throttle|joint_states_throttle"
  "ekf_node"
  "joy_node|teleop_node"
  "slam_toolbox|async_slam_toolbox_node"
  "amcl|map_server|planner_server|controller_server|bt_navigator"
  "behavior_server|smoother_server|waypoint_follower|velocity_smoother"
  "lifecycle_manager"
  "rplidar|v4l2_camera"
  "vision_pipeline.py|object_follower.py"
)

find_strays() {
  local pat
  for pat in "${PATTERNS[@]}"; do
    pgrep -af "$pat" 2>/dev/null
  done | sort -u -k1,1n
}

sweep() {
  local sig="$1" pat
  for pat in "${PATTERNS[@]}"; do
    pkill -"$sig" -f "$pat" 2>/dev/null
  done
  return 0
}

before="$(find_strays)"

# --check: report only. Prints nothing when clean, so callers can test with
# `[ -n "$(./stop_session.sh --check)" ]`.
if $CHECK_ONLY; then
  [ -n "$before" ] && echo "$before" | sed 's/^/  /'
  exit 0
fi

echo "Stopping BonicBot A2 session processes..."

if [ -z "$before" ]; then
  echo "Nothing running."
  exit 0
fi
echo "$before" | sed 's/^/  /'

# SIGINT first — ros2 launch and Gazebo shut down cleanly on it.
sweep INT
sleep 4

# SIGTERM for whatever ignored SIGINT.
sweep TERM
sleep 2

# SIGKILL for the stubborn (Gazebo in particular can wedge on exit).
sweep KILL
sleep 1

remaining="$(find_strays)"
if [ -n "$remaining" ]; then
  echo
  echo "WARNING: these survived the sweep:" >&2
  echo "$remaining" | sed 's/^/  /' >&2
  echo >&2
  echo "Kill them by hand before starting a new session." >&2
  exit 1
fi

echo "Clean — all session processes stopped."
