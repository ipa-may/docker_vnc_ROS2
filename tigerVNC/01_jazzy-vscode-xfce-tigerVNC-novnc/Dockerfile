# ROS 2 Jazzy + XFCE + VNC/noVNC with stability helpers for Gazebo/RViz
FROM ubuntu:noble-20251001

ARG INSTALL_PACKAGE=ros-base   # 'ros-base' (lean) or 'desktop' (includes rviz/rqt)
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# OS + XFCE + VNC + noVNC + tooling (NO ROS tools yet)
RUN apt-get update -q && apt-get install -y --no-install-recommends \
    xfce4 xfce4-terminal dbus-x11 x11-xserver-utils \
    tigervnc-standalone-server tigervnc-common tigervnc-tools \
    novnc websockify \
    supervisor gosu tini sudo bash-completion \
    git wget curl vim lsb-release locales tzdata \
    python3 python3-pip \
    # ---- Stability helpers for Gazebo/RViz over VNC ----
    mesa-utils libgl1-mesa-dri libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && locale-gen
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 Jazzy repo + install (use ros-dev-tools to get rosdep/colcon/vcstool)
ENV ROS_DISTRO=jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -q && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (ok if already inited)
RUN rosdep init || true && rosdep update

# Force the system Python to be the default (avoid /usr/local/python/current)
RUN rm -rf /usr/local/python || true && \
    ln -sf /usr/bin/python3 /usr/local/bin/python3 && \
    ln -sf /usr/bin/pip3 /usr/local/bin/pip3

# Hint CMake/colcon to use the system interpreter
ENV CMAKE_ARGS="-DPython3_EXECUTABLE=/usr/bin/python3"

# Default environment (can be overridden by compose)
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6080 \
    VNC_GEOMETRY=1600x900 \
    VNC_DEPTH=24 \
    USER=ubuntu \
    PASSWORD=ubuntu

# Copy your startup files (keep same filenames you already use)
COPY entrypoint.xfce.sh /entrypoint.sh
COPY supervisord.xfce.conf /etc/supervisor/conf.d/supervisord.conf
RUN chmod +x /entrypoint.sh

EXPOSE 5901 6080
ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
