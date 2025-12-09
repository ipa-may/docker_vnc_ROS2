# VS Code + VNC/noVNC Pack (ROS 2 Humble + XFCE)

Use TigerVNC/noVNC for RViz2, Gazebo, and rqt, and **VS Code Dev Containers** for coding. Bundles ROS 2 Humble, XFCE, TigerVNC with a web client, and Mesa helpers for stable GL over VNC.

## Files

- `Dockerfile` - ROS 2 Humble + XFCE + TigerVNC/noVNC
- `entrypoint.xfce.sh` - creates user, configures VNC/noVNC, launches XFCE
- `supervisord.xfce.conf` - runs the VNC server and websockify
- `docker-compose.yml` - local build/run (plus a prebuilt registry service)
- `.devcontainer/devcontainer.json` - attaches VS Code
- `ros2_ws/` - your workspace

## Run

```bash
docker compose up --build
```

- Browser (noVNC): http://localhost:8080
- VNC client: localhost:5901
- Password: value of `PASSWORD` in `docker-compose.yml` (default `change-me`)

### VS Code Dev Containers

1. Install the Dev Containers extension.
2. Open this folder and choose "Reopen in Container".
3. You will land in `/home/ros/ros2_ws` (mounted from `./ros2_ws`).

## Customize

- Resolution/depth: `VNC_GEOMETRY` (e.g. 1920x1080), `VNC_DEPTH` (e.g. 24)
- ROS install: build arg `INSTALL_PACKAGE=ros-base` or `desktop`
- User/pass: `USER`, `PASSWORD`
- UID/GID: export `ROS_USER_ID` / `ROS_GROUP_ID` before `docker compose up` (or edit compose build args) to match your host IDs
- Ports: host bindings for 6080/5901 in compose; remove `8080:6080` if you do not want noVNC exposed
- DDS/ROS graph: `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`
- Stability toggles:
  - `QT_X11_NO_MITSHM=1` avoids X11 shared-memory issues
  - `LIBGL_DRI3_DISABLE=1` dodges some mesa black screens
  - `LIBGL_ALWAYS_SOFTWARE=1` forces software rendering (slower but very robust)
- Shared memory: `shm_size: "2gb"` in compose for Gazebo cameras/physics
- GPU (optional): uncomment the NVIDIA section in `docker-compose.yml` when the NVIDIA Container Toolkit is available

## Security

- Change `PASSWORD` before exposing beyond localhost.
- Prefer SSH tunnels or VPN if remote.

## Notes for Windows hosts (Docker Desktop + WSL2)

Software rendering over VNC is expected. For faster 3D, switch to a GPU-enabled stack (for example, VirtualGL/TurboVNC) and ensure Docker Desktop has GPU sharing enabled. Keep your workspace under the WSL2 Linux filesystem (or a named volume) for better I/O than mounting from NTFS.
