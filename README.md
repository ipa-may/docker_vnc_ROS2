# Docker images for ROS 2 VNC

| Status                                                                                                                                                                                                                | Docker Image Name                                   |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| [![Build & Push turboVNC 01 ROS2 Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_01_ros2.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_01_ros2.yml) | `ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy`         |
| [![Build & Push turboVNC 02 ROS2 Gazebo Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_02_ros2_gazebo.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_02_ros2_gazebo.yml)   | `ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-gazebo` |

## Repository layout

- `tigerVNC/` — TigerVNC-based development desktops (`01_…`, `02_…`, `03_…` variants).
- `turboVNC/turboVNC_01_ros2/` — TurboVNC + VirtualGL + noVNC base image (publishes `jazzy`, `jazzy-fixed-v2`).
- `turboVNC/turboVNC_02_ros2_gazebo/` — TurboVNC base with Gazebo + ros2_control additions.

## Build the docker image

```sh
cd tigerVNC/01_jazzy-vscode-xfce-tigerVNC-novnc/
docker compose build
```

## Run the container

```sh
docker compose up
```

## Utilizing VNC on Ubuntu

### Using tigervnc

**Ubuntu**

```sh
sudo apt install tigervnc-viewer
```

```sh
vncviewer localhost:5901
```

### Using remmina

```sh
sudo apt install remmina
```

open remmina

```sh
# on a terminal
remmina
```

When it opens, choose VNC as protocol and connect to:

```sh
localhost:5901
```

<figure style="text-align: center;">
    <img src="doc/remmina_remote_co_profile.png" alt="plot" width="1000">
    <figcaption>Remmina remote connexion profile</figcaption>
</figure>

### Using noVNC

In the browser connect to http://localhost:8080/

<figure style="text-align: center;">
    <img src="doc/noVNC_login.png" alt="plot" width="1000">
    <figcaption>Connect via noVNC</figcaption>
</figure>

## Utilizing VNC on Windows

### Using RealVNC Viewer

1. Download: https://www.realvnc.com/en/connect/download/viewer/
2. Install & open VNC Viewer
3. in the address bar enter:

```sh
localhost:5901
```

4. Connect → enter VNC password

## Troubleshooting

On windows:

- Firewall : "Allow app through Windows Firewall".

```

```
