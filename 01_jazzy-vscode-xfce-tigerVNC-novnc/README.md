# VS Code + VNC Pack (ROS 2 Jazzy + XFCE, no noVNC)

Use **VNC** for visualization (RViz2, Gazebo, rqt) and **VS Code Dev Containers** for coding.
This image excludes browsers/IDEs and **noVNC** to stay lean.

## Files

- `Dockerfile` — ROS 2 Jazzy + XFCE + TigerVNC
- `entrypoint.sh` — creates user, configures VNC, launches XFCE
- `supervisord.conf` — runs the VNC server
- `docker-compose.yml` — builds/runs the container
- `.devcontainer/devcontainer.json` — attaches VS Code
- `ros2_ws/` — your workspace

## Run

```bash
docker compose up --build
# Connect your VNC viewer to: localhost:5901
# Password = 'PASSWORD' from docker-compose.yml
```

In **VS Code**:

- Install **Dev Containers** extension
- Reopen the folder **in container** (you'll be attached to /home/ubuntu/ros2_ws)

## Customize

- Resolution: set `VNC_GEOMETRY` (e.g. 1920x1080)
- Depth: set `VNC_DEPTH` (e.g. 24)
- User/Pass: set `USER` and `PASSWORD`
- ROS install: build with `INSTALL_PACKAGE=ros-base` or `desktop`

## GPU (optional)

Uncomment the GPU section in `docker-compose.yml` and ensure NVIDIA drivers + toolkit are installed.

## Notes

- Port 5901 is exposed only; no browser client is included.
- For remote access, consider SSH tunneling your VNC connection.

About the compose flags:

- `QT_X11_NO_MITSHM=1` avoids shared-memory X11 issues in virtual displays.
- `LIBGL_DRI3_DISABLE=1` prevents some DRI3/mesa black-screen quirks under VNC.
- `LIBGL_ALWAYS_SOFTWARE=1` forces software rendering (slower but bulletproof if needed).
- `shm_size: "2gb"` gives Gazebo camera/physics plenty of shared memory.

## Notes for Windows hosts (Docker Desktop + WSL2)

This remains software rendering via VNC (expected). It’s fine for most dev/testing.

We can use hardware acceleration by switching to a GPU-enabled stack (e.g., use a VirtualGL/TurboVNC base and enable NVIDIA in Docker Desktop).

Keep your workspace under the WSL2 Linux filesystem (or a named volume) for better I/O than mounting from NTFS.
