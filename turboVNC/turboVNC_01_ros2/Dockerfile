# ROS 2 Jazzy + XFCE + TurboVNC + VirtualGL + noVNC
# GPU acceleration optional (enable with Docker --gpus all or compose GPU override)
FROM ubuntu:noble-20251001

ARG INSTALL_PACKAGE=desktop   # 'ros-base' (lean) or 'desktop' (includes rviz/rqt)
ARG TURBOVNC_VERSION=3.1.4
ARG VIRTUALGL_VERSION=3.1.1
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Base OS + XFCE + TurboVNC/VirtualGL + noVNC + essentials
RUN apt-get update -q && apt-get upgrade -y && \
  apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg gnupg2 lsb-release locales tzdata \
  iputils-ping dnsutils net-tools software-properties-common \
  xfce4 xfce4-terminal dbus-x11 x11-xserver-utils x11-apps tigervnc-tools \
  novnc websockify \
  supervisor gosu tini sudo bash-completion \
  git wget vim build-essential python3 python3-pip \
  # stability helpers / diagnostics
  mesa-utils libgl1-mesa-dri libglu1-mesa \
  && curl -fsSL -o /tmp/turbovnc.deb https://github.com/TurboVNC/turbovnc/releases/download/${TURBOVNC_VERSION}/turbovnc_${TURBOVNC_VERSION}_amd64.deb \
  && curl -fsSL -o /tmp/virtualgl.deb https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb \
  && apt-get install -y --no-install-recommends /tmp/turbovnc.deb /tmp/virtualgl.deb \
  && rm -f /tmp/turbovnc.deb /tmp/virtualgl.deb \
  && rm -rf /var/lib/apt/lists/*

# Enable Ubuntu universe repository (needed for several packages)
RUN add-apt-repository -y universe

# Configure official ROS 2 apt source
RUN set -eux; \
  apt-get update -q; \
  apt-get install -y --no-install-recommends curl ca-certificates gnupg lsb-release; \
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}'); \
  CODENAME=$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}}); \
  curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"; \
  dpkg -i /tmp/ros2-apt-source.deb; \
  rm -f /tmp/ros2-apt-source.deb; \
  apt-get update -q

# Locale
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && locale-gen && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 Jazzy
ENV ROS_DISTRO=jazzy
RUN apt-get update -q && \
  apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*

# rosdep init/update
RUN rosdep init || true && rosdep update

# Defaults (overridable)
ENV DISPLAY=:1 \
  VNC_PORT=5901 \
  NO_VNC_PORT=6080 \
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

EXPOSE 5901 6080

ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
