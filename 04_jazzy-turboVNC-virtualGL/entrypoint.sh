#!/usr/bin/env bash
set -e

: "${USER:=ubuntu}"
: "${PASSWORD:=ubuntu}"
: "${DISPLAY:=:1}"
: "${VNC_PORT:=5901}"
: "${VNC_GEOMETRY:=1600x900}"
: "${VNC_DEPTH:=24}"

HOME_DIR="/home/${USER}"
[ "$USER" = "root" ] && HOME_DIR="/root"

# Create user if needed
if [ "$USER" != "root" ] && ! id -u "$USER" >/dev/null 2>&1; then
  echo "[entrypoint] Creating user: $USER"
  useradd --create-home --shell /bin/bash --user-group --groups adm,sudo "$USER"
  echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
  echo "$USER:$PASSWORD" | chpasswd
fi

# TurboVNC password
mkdir -p "${HOME_DIR}/.vnc"
echo "${PASSWORD}" | /opt/TurboVNC/bin/vncpasswd -f > "${HOME_DIR}/.vnc/passwd"
chmod 600 "${HOME_DIR}/.vnc/passwd"
chown -R "${USER}:${USER}" "${HOME_DIR}/.vnc"

# TurboVNC xstartup (TurboVNC looks for xstartup.turbovnc)
XSTART="${HOME_DIR}/.vnc/xstartup.turbovnc"
cat > "${XSTART}" <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
startxfce4 &
EOF
chown "${USER}:${USER}" "${XSTART}"
chmod 755 "${XSTART}"

# vnc run script
VNC_RUN="${HOME_DIR}/.vnc/vnc_run.sh"
cat > "${VNC_RUN}" <<EOF
#!/bin/sh
if [ -e /tmp/.X1-lock ]; then rm -f /tmp/.X1-lock; fi
if [ -e /tmp/.X11-unix/X1 ]; then rm -f /tmp/.X11-unix/X1; fi

/opt/TurboVNC/bin/vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT} -SecurityTypes None
EOF
chown "${USER}:${USER}" "${VNC_RUN}"
chmod 755 "${VNC_RUN}"

# Auto-source ROS
BASHRC="${HOME_DIR}/.bashrc"
grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" "$BASHRC" || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$BASHRC"
chown "${USER}:${USER}" "$BASHRC"

# Default workspace
mkdir -p "${HOME_DIR}/ros2_ws/src"
chown -R "${USER}:${USER}" "${HOME_DIR}/ros2_ws"

export HOME="${HOME_DIR}"

echo "[entrypoint] TurboVNC on ${DISPLAY} (port ${VNC_PORT}), geometry ${VNC_GEOMETRY}, depth ${VNC_DEPTH}"
echo "[entrypoint] Use 'vglrun <app>' to run GPU-accelerated OpenGL apps (when GPU is available)."
exec /bin/tini -- supervisord -n -c /etc/supervisor/conf.d/supervisord.conf
