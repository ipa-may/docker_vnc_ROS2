# ROS 2 Jazzy + XFCE + TurboVNC + VirtualGL

Windows-friendly pack with optional GPU acceleration.

## VS Code Dev Containers

This pack includes ready-to-use Dev Container configurations to make development easy inside the same container used for visualization.

| File                                  | Description                                        |
| ------------------------------------- | -------------------------------------------------- |
| `.devcontainer/devcontainer.json`     | Standard (CPU / software rendering) container      |
| `.devcontainer/devcontainer.gpu.json` | GPU-enabled variant (uses NVIDIA runtime override) |

## Using on Windows with VS Code

1. Install the VS Code Dev Containers extension.

2. Open the project folder in VS Code.

3. When prompted, select **Reopen in Container**.

4. To use GPU acceleration:

- Press `Ctrl+Shift+P` → search “**Dev Containers: Open Folder in Container...**”.

- Select `devcontainer.gpu.json`.

This automatically launches your ROS 2 + TurboVNC + XFCE desktop environment, ready to:

- Code in VS Code (`/home/ubuntu/ros2_ws`)

- Visualize in TurboVNC (localhost:5901)

- Run accelerated apps (if GPU-enabled):

Inside the container, run GPU-accelerated apps with **vglrun**:

```bash
vglrun rviz2
vglrun gz sim shapes
```

## Files

- `Dockerfile` — ROS 2 Jazzy + XFCE + **TurboVNC + VirtualGL**
- `entrypoint.sh` — creates user, configures TurboVNC, launches XFCE
- `supervisord.conf` — keeps TurboVNC running
- `docker-compose.yml` — base (CPU / software rendering)
- `docker-compose.gpu.override.yml` — enable NVIDIA GPU
- `ros2_ws/` — your workspace

## Quick start (Windows PowerShell)

```powershell
docker compose up --build
# Then connect TurboVNC Viewer to: localhost:5901
# Password: set with PASSWORD (default: change-me)
```

## Enable GPU (optional)

Requirements: NVIDIA GPU + Windows NVIDIA driver + Docker Desktop (WSL2 GPU support)

```powershell
docker compose -f docker-compose.yml -f docker-compose.gpu.override.yml up --build
```

## Notes

- TurboVNC uses `/opt/TurboVNC/bin/vncserver` and `xstartup.turbovnc`.
- **VirtualGL** accelerates individual apps via `vglrun`. The desktop itself stays software-rendered.
- Keep `shm_size: "2gb"` for Gazebo cameras/physics.
- If a window is blank, try:
  ```bash
  export QT_X11_NO_MITSHM=1
  export LIBGL_DRI3_DISABLE=1
  # export LIBGL_ALWAYS_SOFTWARE=1  # last resort
  ```

## Sanity checks

```bash
# 1) VNC/desktop is up
glxinfo | grep -E 'OpenGL renderer|OpenGL version'   # will show llvmpipe by default

# 2) With GPU enabled + vglrun
vglrun glxinfo | grep -E 'OpenGL renderer|OpenGL version'  # should show your NVIDIA GPU

# 3) ROS 2
ros2 run demo_nodes_cpp talker
vglrun rviz2
```
