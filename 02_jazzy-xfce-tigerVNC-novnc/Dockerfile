# Minimal ROS 2 Jazzy + XFCE desktop with VNC & noVNC
FROM ubuntu:noble-20251001

ARG TARGETPLATFORM
ARG TARGETARCH
ARG INSTALL_PACKAGE=ros-base   # ros-base (default) or desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Base OS
RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 lsb-release \
    locales tzdata \
    # Desktop + VNC/noVNC + supervisor + helpers
    xfce4 xfce4-terminal dbus-x11 x11-xserver-utils \
    tigervnc-standalone-server tigervnc-common tigervnc-tools \
    novnc websockify \
    supervisor gosu tini \
    sudo bash-completion \
    python3 python3-pip \
    # Utilities
    vim git wget \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Install ROS 2 Jazzy
ENV ROS_DISTRO=jazzy
RUN apt-get update -q && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    ros-${ROS_DISTRO}-ros-gz \
    python3-argcomplete python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update

# Ensure apt completion works after apt-get update inside container
RUN rm /etc/apt/apt.conf.d/docker-clean || true

# Default environment ports & settings (can be overridden at runtime)
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6080 \
    VNC_GEOMETRY=1600x900 \
    VNC_DEPTH=24 \
    USER=ubuntu \
    PASSWORD=ubuntu

# Add entrypoint & supervisor config
COPY entrypoint.xfce.sh /entrypoint.sh
COPY supervisord.xfce.conf /etc/supervisor/conf.d/supervisord.conf
RUN chmod +x /entrypoint.sh

EXPOSE 5901 6080

ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
