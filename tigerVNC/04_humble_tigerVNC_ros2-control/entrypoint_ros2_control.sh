#!/usr/bin/env bash
set -e

# Source ROS 2 environment
if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay if present
WS_DIR="${HOME}/ros2_ws"
if [ -d "${WS_DIR}" ]; then
  cd "${WS_DIR}"
  if [ -f install/setup.bash ]; then
    source install/setup.bash
  fi
fi

# Hand over control to the original TurboVNC entrypoint so VNC/noVNC start
exec /bin/bash /entrypoint_base.sh "$@"
