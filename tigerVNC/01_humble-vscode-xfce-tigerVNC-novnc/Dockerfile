# ROS 2 Humble + XFCE + VNC/noVNC with stability helpers for Gazebo/RViz
FROM ubuntu:jammy

ARG INSTALL_PACKAGE=ros-base   # 'ros-base' (lean) or 'desktop' (includes rviz/rqt)
ARG USERNAME=ros
ARG USER_ID=1000
ARG GROUP_ID=1000
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

# ROS 2 Humble repo + install (use ros-dev-tools to get rosdep/colcon/vcstool)
ENV ROS_DISTRO=humble
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

# Create ros user (UID/GID can be overridden via build args/compose)
RUN set -eux; \
    existing_group=$(getent group "${GROUP_ID}" | cut -d: -f1 || true); \
    if [ -n "$existing_group" ]; then \
        group_name="$existing_group"; \
    elif getent group "${USERNAME}" >/dev/null 2>&1; then \
        groupmod --gid "${GROUP_ID}" "${USERNAME}"; \
        group_name="${USERNAME}"; \
    else \
        groupadd --gid "${GROUP_ID}" "${USERNAME}"; \
        group_name="${USERNAME}"; \
    fi; \
    existing_user=$(getent passwd "${USER_ID}" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
        usermod --login "${USERNAME}" --home /home/"${USERNAME}" --move-home "$existing_user"; \
        usermod --uid "${USER_ID}" --gid "${GROUP_ID}" "${USERNAME}"; \
    elif id -u "${USERNAME}" >/dev/null 2>&1; then \
        usermod --uid "${USER_ID}" --gid "${GROUP_ID}" "${USERNAME}"; \
    else \
        useradd --uid "${USER_ID}" --gid "${GROUP_ID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/"${USERNAME}"; \
    chmod 0440 /etc/sudoers.d/"${USERNAME}"; \
    mkdir -p /home/"${USERNAME}"/.config /home/"${USERNAME}"/ros2_ws/src; \
    chown -R "${USER_ID}":"${GROUP_ID}" /home/"${USERNAME}"

# Default environment (can be overridden by compose)
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6080 \
    VNC_GEOMETRY=1600x900 \
    VNC_DEPTH=24 \
    USER=${USERNAME} \
    HOME=/home/${USERNAME} \
    PASSWORD=ubuntu

# Copy your startup files (keep same filenames you already use)
COPY entrypoint.xfce.sh /entrypoint.sh
COPY supervisord.xfce.conf /etc/supervisor/conf.d/supervisord.conf
COPY bashrc /tmp/ros_bashrc
RUN chmod +x /entrypoint.sh && \
    cp /tmp/ros_bashrc /home/${USERNAME}/.bashrc && \
    chown ${USER_ID}:${GROUP_ID} /home/${USERNAME}/.bashrc

EXPOSE 5901 6080
ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
