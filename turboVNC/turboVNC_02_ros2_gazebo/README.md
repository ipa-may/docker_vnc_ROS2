# ROS 2 Jazzy TurboVNC/noVNC + Gazebo + ros2_control (GHCR base)

Builds on the published image `ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy`, adding Gazebo (`ros-gz`) desktop tools and the ros2_control stacks.

## Files
- `Dockerfile` — pulls the GHCR image and installs Gazebo + ros2_control packages
- `docker-compose.yml` — base runtime (CPU/software rendering) exposing 5901/6080
- `docker-compose.gpu.override.yml` — optional NVIDIA GPU support
- `ros2_ws/` — bind-mounted workspace inside the container

## Usage
```bash
docker compose up --build
# Browser noVNC → http://localhost:8080
# TurboVNC viewer → localhost:5901
```

To use GPU acceleration (VirtualGL/vglrun):
```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.override.yml up --build
```

## Quick checks inside the container
```bash
gz sim shapes               # Gazebo (ros-gz) demo world
ros2 control list_controllers
```

The base image already sets up TurboVNC + noVNC + XFCE; this layer only adds the requested simulation/control packages.
