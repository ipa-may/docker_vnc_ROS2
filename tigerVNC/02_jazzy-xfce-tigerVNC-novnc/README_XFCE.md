# Minimal ROS 2 Jazzy + XFCE + VNC/noVNC (Docker + Compose Pack)

This pack provides a lean **ROS 2 Jazzy** desktop you can access from a **web browser** (noVNC) or any **VNC client**.
It uses **XFCE** (lightweight), **TigerVNC**, and **noVNC**. Ideal for Windows/macOS/Linux hosts via Docker Desktop.

## Files

- `Dockerfile` — minimal image with ROS 2 Jazzy (default `ros-base`) + XFCE + VNC/noVNC
- `entrypoint.xfce.sh` — creates user, configures VNC/noVNC, launches XFCE
- `supervisord.xfce.conf` — runs VNC (`:1`) and noVNC proxy
- `docker-compose.yml` — one-command local run with volumes/ports
- `ros2_ws/` — your workspace (created on first run)

## Quick start

```bash
docker compose up --build
```

Open http://localhost:8080 (noVNC)

Or VNC viewer → localhost:5901

Default creds: `USER=ubuntu`, `PASSWORD=change-me` (set in compose file).

## Customize

- **Resolution / depth**: edit in `docker-compose.yml`
  ```yaml
  environment:
    VNC_GEOMETRY: 1920x1080
    VNC_DEPTH: "24"
  ```
- **Ports**: change host ports if occupied
  ```yaml
  ports:
    - "8081:6080" # browser
    - "5902:5901" # VNC
  ```
- **ROS 2 package set**: build with `desktop` for rviz/rqt preinstalled
  ```yaml
  args:
    INSTALL_PACKAGE: desktop
  ```
- **User/password**:
  ```yaml
  environment:
    USER: alice
    PASSWORD: s3cur3!
  ```

## Using ROS 2

In the XFCE desktop (browser or VNC):

1. Open **XFCE Terminal**.
2. ROS 2 Jazzy is already sourced.
3. Build your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## GPU (optional)

If you have an NVIDIA GPU and the NVIDIA Container Toolkit installed:

- Uncomment the GPU section in `docker-compose.yml`.
- On Windows/macOS, ensure Docker Desktop supports GPU sharing.

## Security tips

- Change `PASSWORD` before exposing beyond `localhost`.
- Keep ports bound to your machine or protect with firewall/TLS.
- Consider removing the password autofill tweak if you add it later for noVNC.
