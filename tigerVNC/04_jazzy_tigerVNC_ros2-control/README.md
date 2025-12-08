# TigerVNC ROS 2 Jazzy + ros2_control

This variant layers the ros2_control / Gazebo tooling onto the existing `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-vscode-xfce` desktop while keeping the Turbo/TigerVNC entrypoint intact. Multi-stage building keeps the VNC stack untouched and copies the heavy ROS dependencies from a separate builder stage. Highlights:

- TigerVNC + noVNC + XFCE desktop identical to the upstream image.
- ros2_control, ros2_controllers, PlotJuggler, RViz2, ros-gz/gz-harmonic and friends preinstalled.
- Workspace volumes mounted under `/home/ros/ros2_ws` (see compose file) so you can drop source packages under `src`.
- VS Code attaches as the `ros` user automatically via the `devcontainer.metadata` label, and ROS environments are pre-sourced before the VNC entrypoint starts.

## Build & run

```bash
cd tigerVNC/04_jazzy_tigerVNC_ros2-control
docker compose build    # or: docker compose build --no-cache
docker compose up -d
```

Access options once the container is running:

- Browser: `http://localhost:6080` (noVNC). Password defaults to `change-me`.
- Native VNC client: connect to `localhost:5901`.
- CLI shell as `ros`: `docker exec -it -u ros ros2-jazzy-tigervnc-ros2-control bash`.

The workspace mount points in `docker-compose.yaml` map:

- `./ros2_ws` → `/home/ros/ros2_ws` (colcon workspace state: build/install/logs).
- `../../src` → `/home/ros/ros2_ws/src` (shared source tree).

If you need a different layout, adjust the `volumes` section accordingly.

## VS Code Dev Containers

Thanks to the `devcontainer.metadata` label in `docker-compose.yaml`, you can attach with **Dev Containers: Attach to Running Container…** and VS Code will:

1. Connect to the `ros2-jazzy-tigervnc-ros2-control` service.
2. Open `/home/ros/ros2_ws` as the workspace.
3. Use the non-root `ros` user automatically.

No additional `.devcontainer` folder is required, but you can add one if you want custom extensions or settings.

## Extending the image

- Drop additional apt packages in the final `runtime` stage if they must exist in the running desktop.
- Add custom ros2_control packages by cloning them into `ros2_ws/src` (host bind mount), then build with `colcon build` inside the container.
- Keep the entrypoint wrapper (`entrypoint_ros2_control.sh`) intact so ROS setup scripts run before handing off to the original VNC entrypoint.
