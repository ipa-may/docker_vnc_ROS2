#!/usr/bin/env bash
set -euo pipefail

: "${USER:=ros}"
: "${PASSWORD:=ubuntu}"
: "${DISPLAY:=:1}"
: "${VNC_PORT:=5901}"
: "${VNC_GEOMETRY:=1600x900}"
: "${VNC_DEPTH:=24}"

HOME_DIR="${HOME:-/home/${USER}}"
VNC_DIR="${HOME_DIR}/.vnc"
XSTART="${VNC_DIR}/xstartup"
VNC_RUN="${VNC_DIR}/vnc_run.sh"

log() {
  echo "[entrypoint] $*"
}

prepare_home() {
  mkdir -p "${HOME_DIR}" "${HOME_DIR}/ros2_ws/src"
  touch "${HOME_DIR}/.Xauthority"
  mkdir -p "${HOME_DIR}/.ros/log"
}

prepare_x11_socket() {
  mkdir -p /tmp/.X11-unix
  chmod 1777 /tmp/.X11-unix 2>/dev/null || true
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
}

ensure_vnc_password() {
  mkdir -p "${VNC_DIR}"
  if [ ! -f "${VNC_DIR}/passwd" ]; then
    echo "${PASSWORD}" | vncpasswd -f > "${VNC_DIR}/passwd"
    chmod 600 "${VNC_DIR}/passwd"
  fi
}

write_xstartup() {
  cat > "${XSTART}" <<'EOS'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
EOS
  chmod 755 "${XSTART}"
}

write_vnc_runner() {
  cat > "${VNC_RUN}" <<'EOS'
#!/bin/sh
if [ -e /tmp/.X1-lock ]; then rm -f /tmp/.X1-lock; fi
if [ -e /tmp/.X11-unix/X1 ]; then rm -f /tmp/.X11-unix/X1; fi

if [ "$(uname -m)" = "aarch64" ]; then
  LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
else
  vncserver ${DISPLAY} -fg -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -rfbport ${VNC_PORT}
fi
EOS
  chmod 755 "${VNC_RUN}"
}

prepare_home
prepare_x11_socket
ensure_vnc_password
write_xstartup
write_vnc_runner

log "Starting TigerVNC on ${DISPLAY} (port ${VNC_PORT}), geometry ${VNC_GEOMETRY}, depth ${VNC_DEPTH}"
exec /bin/tini -- supervisord -n -c /etc/supervisor/conf.d/supervisord.conf
