#!/usr/bin/env bash
# Usage:
#   source tools/source_humble_ros2_ws.bash
#
# Purpose:
#   Enter a clean ROS 2 Humble + this workspace overlay environment.
#   This avoids stale/non-existent entries (e.g. old package install prefixes)
#   lingering in AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH across repeated 'source'.

# Prevent mixing overlays from previous terminal sessions.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# Base ROS 2 distro.
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/humble/setup.bash"
else
  echo "ERROR: /opt/ros/humble/setup.bash not found" 1>&2
  return 1
fi

# Workspace overlay (this repo).
_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "${_ws_root}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_ws_root}/install/setup.bash"
else
  echo "ERROR: ${_ws_root}/install/setup.bash not found (build first?)" 1>&2
  return 1
fi

unset _ws_root
