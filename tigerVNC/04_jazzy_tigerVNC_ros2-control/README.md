# TigerVNC ROS 2 Jazzy + ros2_control

A TigerVNC/noVNC desktop with the ros2_control stack preinstalled on top of `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-vscode-xfce`. The multi-stage build copies the heavy ROS bits from a separate builder stage, so the VNC environment stays identical to the upstream image while adding controllers, RViz2, PlotJuggler, MoveIt, and common dependencies.

- TigerVNC + noVNC + XFCE desktop with the original entrypoint preserved.
- ros2_control, ros2_controllers, hardware interfaces, RViz2, MoveIt, PlotJuggler, and supporting libraries baked in.
- Dev Containers label points VS Code at `/home/ros/ros2_ws` as `ros`.
- Only the source tree is bind-mounted by default; add a workspace volume if you want to persist build artifacts.

## Build & run

```bash
cd tigerVNC/04_jazzy_tigerVNC_ros2-control
docker compose build    # add --no-cache if you change apt packages
docker compose up -d
```

Endpoints after start:

- Browser (noVNC): `http://localhost:6080` (password: `change-me`).
- TigerVNC client: connect to `localhost:5901`.
- Shell as `ros`: `docker exec -it -u ros ros2-jazzy-tigervnc-ros2-control bash`.

## Volumes and workspace layout

- `../../src` → `/home/ros/ros2_ws/src` (shared source tree).
- Add `./ros2_ws:/home/ros/ros2_ws` to `volumes` if you want build/install/logs to persist on the host.

If you need host UID/GID alignment, pass build args: `docker compose build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)`.

## VS Code Dev Containers

The `devcontainer.metadata` label lets **Dev Containers: Attach to Running Container…** connect directly to the `ros2-jazzy-tigervnc-ros2-control` service, open `/home/ros/ros2_ws`, and use the `ros` user without adding a `.devcontainer` folder. Add one if you want extra extensions or settings.

## Extending

- Add extra apt packages to the final `runtime` stage in `Dockerfile-vnc-ros2-control`.
- Clone additional ros2_control packages into `ros2_ws/src` (host bind mount) and build with `colcon build` inside the container.
- Keep `entrypoint_ros2_control.sh` intact so the ROS environment is sourced before handing off to the VNC startup.
