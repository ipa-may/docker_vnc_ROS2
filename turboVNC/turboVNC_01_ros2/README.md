# ROS 2 Jazzy + XFCE + TurboVNC + VirtualGL + noVNC

```sh
docker tag ros2-turbovnc-novnc:jazzy-fixed-v2 ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-fixed-v2


docker tag ros2-turbovnc-novnc:jazzy-gazebo ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-gazebo
docker push ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-gazebo

```

This variant adds **noVNC** (browser access) while keeping **TurboVNC + VirtualGL** for maximum performance and optional GPU acceleration.

## Files

- `Dockerfile` — TurboVNC + VirtualGL + **noVNC**
- `entrypoint.sh` — creates user, configures TurboVNC, launches XFCE
- `supervisord.conf` — runs TurboVNC and websockify/noVNC
- `docker-compose.yml` — base (CPU/software rendering) with 5901 and 6080 exposed
- `docker-compose.gpu.override.yml` — enable NVIDIA GPU
- `.devcontainer/devcontainer.json` & `.devcontainer/devcontainer.gpu.json` — open in VS Code
- `ros2_ws/` — your workspace

## Quick start (Windows PowerShell)

```powershell
docker compose up --build
# Browser noVNC: http://localhost:8080  (proxied to container 6080)
# TurboVNC Viewer: localhost:5901
# Password: change-me  (set in docker-compose.yml)
```

## Enable GPU (optional)

```powershell
docker compose -f docker-compose.yml -f docker-compose.gpu.override.yml up --build
# inside the container, run accelerated apps with vglrun:
vglrun rviz2
vglrun gz sim shapes
```

## Notes

- **noVNC** proxies browser traffic to the TurboVNC server via `websockify`.
- **VirtualGL** accelerates individual apps when launched with `vglrun`.
- Keep `shm_size: "2gb"` for Gazebo cameras/physics.
- If you see blank windows:
  ```bash
  export QT_X11_NO_MITSHM=1
  export LIBGL_DRI3_DISABLE=1
  # export LIBGL_ALWAYS_SOFTWARE=1  # last resort
  ```

## Sanity checks

```bash
glxinfo | grep -E 'OpenGL renderer|OpenGL version'   # llvmpipe in desktop session
vglrun glxinfo | grep -E 'OpenGL renderer|OpenGL version'  # should show NVIDIA GPU with GPU override
```
