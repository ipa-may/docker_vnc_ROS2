#!/usr/bin/env bash
set -e

# Defaults (allow overrides from env)
: "${USER:=ubuntu}"
: "${PASSWORD:=ubuntu}"
: "${DISPLAY:=:1}"
: "${VNC_PORT:=5901}"
: "${NO_VNC_PORT:=6080}"
: "${VNC_GEOMETRY:=1600x900}"
: "${VNC_DEPTH:=24}"

# Resolve home
HOME_DIR="/home/${USER}"
if [ "$USER" = "root" ]; then
  HOME_DIR="/root"
fi

# Create user if not root
if [ "$USER" != "root" ] && ! id -u "$USER" >/dev/null 2>&1; then
  echo "[entrypoint] Creating user: $USER"
  useradd --create-home --shell /bin/bash --user-group --groups adm,sudo "$USER"
  echo "$USER:$PASSWORD" | chpasswd
fi

# Ensure passwordless sudo for the target user
if [ "$USER" != "root" ]; then
  SUDOERS_SNIPPET="/etc/sudoers.d/${USER}"
  if [ ! -f "$SUDOERS_SNIPPET" ]; then
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > "$SUDOERS_SNIPPET"
    chmod 440 "$SUDOERS_SNIPPET"
  fi
fi

# Ensure X11 socket dir exists with proper ownership
mkdir -p /tmp/.X11-unix
chown root:root /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# Touch Xauthority in advance to silence xauth warnings
touch "${HOME_DIR}/.Xauthority"
chown "${USER}:${USER}" "${HOME_DIR}/.Xauthority"

# Prepare VNC password
mkdir -p "${HOME_DIR}/.vnc"
echo "${PASSWORD}" | vncpasswd -f > "${HOME_DIR}/.vnc/passwd"
chmod 600 "${HOME_DIR}/.vnc/passwd"
chown -R "${USER}:${USER}" "${HOME_DIR}/.vnc"

# xstartup for XFCE
XSTART="${HOME_DIR}/.vnc/xstartup"
cat > "${XSTART}" <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
EOF
chown "${USER}:${USER}" "${XSTART}"
chmod 755 "${XSTART}"

# vnc run script
VNC_RUN="${HOME_DIR}/.vnc/vnc_run.sh"
cat > "${VNC_RUN}" <<EOF
#!/bin/sh
if [ "\$(uname -m)" = "aarch64" ]; then
  LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
else
  vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
fi
EOF
chown "${USER}:${USER}" "${VNC_RUN}"
chmod 755 "${VNC_RUN}"

# Auto-source ROS 2 in .bashrc
BASHRC="${HOME_DIR}/.bashrc"
grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" "$BASHRC" || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$BASHRC"
chown "${USER}:${USER}" "$BASHRC"

# Fix rosdep permission for non-root
mkdir -p "${HOME_DIR}/.ros"
if [ -d "/root/.ros/rosdep" ] && [ "$HOME_DIR" != "/root" ]; then
  cp -r /root/.ros/rosdep "${HOME_DIR}/.ros/rosdep" || true
  chown -R "${USER}:${USER}" "${HOME_DIR}/.ros"
fi

# Make a default workspace
mkdir -p "${HOME_DIR}/ros2_ws/src"
chown -R "${USER}:${USER}" "${HOME_DIR}/ros2_ws"

echo "[entrypoint] VNC ${DISPLAY} on port ${VNC_PORT}, noVNC on ${NO_VNC_PORT}, geometry ${VNC_GEOMETRY}, depth ${VNC_DEPTH}"
exec /bin/tini -- supervisord -n -c /etc/supervisor/conf.d/supervisord.conf
