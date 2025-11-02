# Derived from GHCR TurboVNC/noVNC base image; adds Gazebo and ros2_control stacks
FROM ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-fixed-v2

ENV DEBIAN_FRONTEND=noninteractive

# Add Gazebo (ros-gz) desktop stack and ros2_control packages
RUN apt-get update -q && \
  apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-gz-ros2-control \
  ros-${ROS_DISTRO}-plotjuggler-ros \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers && \
  rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && apt-get install -y --no-install-recommends curl && \
  curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list && \
  apt-get update -q && \
  apt-get install -y --no-install-recommends gz-harmonic && \
  rm -rf /var/lib/apt/lists/*


# Setup XDG_RUNTIME_DIR for the non-root user 
ENV USER=ubuntu
RUN mkdir -p /tmp/runtime-${USER} && chown -R ${USER}:${USER} /tmp/runtime-${USER} && chmod 700 /tmp/runtime-${USER}
ENV XDG_RUNTIME_DIR=/tmp/runtime-${USER}
ENV QT_X11_NO_MITSHM=1


# --- software GL for headless/VNC (remove if using host GPU) ---
RUN apt-get update -q && apt-get install -y --no-install-recommends \
  libgl1-mesa-dri mesa-utils && \
  rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1


# Set Gazebo resource paths
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz:/usr/share/gz-harmonic:/usr/share/gazebo:/opt/ros/${ROS_DISTRO}/share:/workspaces/ws/src

# Ensure interactive shells also export the paths (covers derived users)
RUN cat <<'EOF' > /etc/profile.d/gazebo_env.sh
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${HOME}/.gz/models:${HOME}/.gz/fuel/models:${HOME}/.fuel/models
EOF
RUN chmod 644 /etc/profile.d/gazebo_env.sh
