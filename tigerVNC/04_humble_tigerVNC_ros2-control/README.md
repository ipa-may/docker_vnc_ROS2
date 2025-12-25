# TigerVNC ROS 2 Humble + ros2_control

A TigerVNC/noVNC desktop with the ros2_control stack preinstalled on top of `ghcr.io/ipa-may/ros2-tigervnc-novnc:humble-vscode-xfce`. The multi-stage build keeps the upstream VNC desktop intact while adding controllers, RViz2, PlotJuggler, and supporting dependencies.

- TigerVNC + noVNC + XFCE desktop with the original entrypoint preserved.
- ros2_control, ros2_controllers, hardware interfaces, RViz2, PlotJuggler baked in.
- Dev Containers label points VS Code at `/home/ros/ros2_ws` as `ros`.
- Only the source tree is bind-mounted by default; add a workspace volume if you want to persist build artifacts.

## Build & run

```bash
cd tigerVNC/04_humble_tigerVNC_ros2-control
docker compose build    # or: docker compose build --no-cache
docker compose up -d
```

Access once running:

- Browser (noVNC): `http://localhost:6082` (password: `change-me`).
- TigerVNC client: connect to `localhost:5902`.
- Shell as `ros`: `docker exec -it -u ros ros2-humble-tigervnc-ros2-control bash`.

## Volumes and workspace layout

- `../../src` → `/home/ros/ros2_ws/src` (shared source tree).
- Add `./ros2_ws:/home/ros/ros2_ws` to `volumes` if you want build/install/logs to persist on the host.

UID/GID alignment: set `ROS_USER_ID` and `ROS_GROUP_ID` before `docker compose build/up` (they feed into `USER_ID`/`GROUP_ID` build args and the service `user:` directive).

## VS Code Dev Containers

The `devcontainer.metadata` label lets **Dev Containers: Attach to Running Container…** connect directly to the `ros2-humble-tigervnc-ros2-control` service, open `/home/ros/ros2_ws`, and use the `ros` user without adding a `.devcontainer` folder. Add one if you want extra extensions or settings.

## Extending

- Add extra apt packages to the final `runtime` stage in `Dockerfile-vnc-ros2-control`.
- Clone additional ros2_control packages into `ros2_ws/src` (host bind mount) and build with `colcon build` inside the container.
- Keep `entrypoint_ros2_control.sh` intact so the ROS environment is sourced before handing off to the VNC startup.
