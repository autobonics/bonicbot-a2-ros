#!/usr/bin/env bash
# Source ROS2 and the workspace overlay, then exec whatever was asked for.
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

exec "$@"
