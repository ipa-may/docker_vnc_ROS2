# ROS 2 Jazzy + XFCE + TurboVNC + VirtualGL
# GPU acceleration optional (enable with Docker --gpus all or compose GPU override)
FROM ubuntu:noble-20251001

ARG INSTALL_PACKAGE=desktop   # 'ros-base' (lean) or 'desktop' (includes rviz/rqt)
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Base OS + XFCE + TurboVNC/VirtualGL + essentials
RUN apt-get update -q && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg2 lsb-release locales tzdata \
      xfce4 xfce4-terminal dbus-x11 x11-xserver-utils \
      turbovnc virtualgl \
      supervisor gosu tini sudo bash-completion \
      git wget vim build-essential python3 python3-pip \
      # stability helpers / diagnostics
      mesa-utils libgl1-mesa-dri libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && locale-gen && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 Jazzy
ENV ROS_DISTRO=jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -q && \
    apt-get install -y --no-install-recommendS \
      ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
      ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# rosdep init/update
RUN rosdep init || true && rosdep update

# Defaults (overridable)
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    VNC_GEOMETRY=1600x900 \
    VNC_DEPTH=24 \
    USER=ubuntu \
    PASSWORD=ubuntu \
    # Stability envs for VNC X11
    QT_X11_NO_MITSHM=1 \
    LIBGL_DRI3_DISABLE=1

# Add entrypoint & supervisor
COPY entrypoint.sh /entrypoint.sh
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
RUN chmod +x /entrypoint.sh

EXPOSE 5901

ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
