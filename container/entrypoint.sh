#!/usr/bin/env bash
set -eo pipefail

# Source ROS 2 environment
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

# Source built workspace if present
if [ -f "/opt/muto_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/muto_ws/install/setup.bash"
fi

LAUNCH_FILE=${MUTO_LAUNCH:-/work/launch/muto.launch.py}
LAUNCH_ARGS=${MUTO_LAUNCH_ARGS:-}

# If a command is passed, run it directly
if [ $# -gt 0 ]; then
  exec "$@"
fi

# Default: launch ROS 2 app
if [[ "$LAUNCH_FILE" == *.py ]]; then
  exec ros2 launch "$LAUNCH_FILE" ${LAUNCH_ARGS}
else
  # Allow shell-based startup as an alternative
  exec bash -lc "$LAUNCH_FILE ${LAUNCH_ARGS}"
fi
