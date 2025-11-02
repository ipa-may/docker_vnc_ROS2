#!/usr/bin/env bash
set -e

: "${USER:=ubuntu}"
: "${PASSWORD:=ubuntu}"
: "${DISPLAY:=:1}"
: "${VNC_PORT:=5901}"
: "${VNC_GEOMETRY:=1600x900}"
: "${VNC_DEPTH:=24}"

HOME_DIR="/home/${USER}"
if [ "$USER" = "root" ]; then HOME_DIR="/root"; fi

# Create user if needed
if [ "$USER" != "root" ] && ! id -u "$USER" >/dev/null 2>&1; then
  echo "[entrypoint] Creating user: $USER"
  useradd --create-home --shell /bin/bash --user-group --groups adm,sudo "$USER"
  echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
  echo "$USER:$PASSWORD" | chpasswd
fi

# Ensure X11 socket directory exists for non-root VNC sessions
mkdir -p /tmp/.X11-unix
chown root:root /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# Prevent xauth warnings by creating the authority file ahead of time
touch "${HOME_DIR}/.Xauthority"
chown "${USER}:${USER}" "${HOME_DIR}/.Xauthority"

# VNC password
mkdir -p "${HOME_DIR}/.vnc"
echo "${PASSWORD}" | vncpasswd -f > "${HOME_DIR}/.vnc/passwd"
chmod 600 "${HOME_DIR}/.vnc/passwd"
chown -R "${USER}:${USER}" "${HOME_DIR}/.vnc"

# XFCE xstartup
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
if [ -e /tmp/.X1-lock ]; then rm -f /tmp/.X1-lock; fi
if [ -e /tmp/.X11-unix/X1 ]; then rm -f /tmp/.X11-unix/X1; fi

if [ "\$(uname -m)" = "aarch64" ]; then
  LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
else
  vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
fi
EOF
chown "${USER}:${USER}" "${VNC_RUN}"
chmod 755 "${VNC_RUN}"

# Source ROS in .bashrc
BASHRC="${HOME_DIR}/.bashrc"
grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" "$BASHRC" || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$BASHRC"
chown "${USER}:${USER}" "$BASHRC"

# Workspace
mkdir -p "${HOME_DIR}/ros2_ws/src"
chown -R "${USER}:${USER}" "${HOME_DIR}/ros2_ws"

echo "[entrypoint] Starting TigerVNC on ${DISPLAY} (port ${VNC_PORT}), geometry ${VNC_GEOMETRY}, depth ${VNC_DEPTH}"
exec /bin/tini -- supervisord -n -c /etc/supervisor/conf.d/supervisord.conf
