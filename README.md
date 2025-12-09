# Docker images for ROS 2 VNC

| Status                                                                                                                                                                                                                | Docker Image Name                                        |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| [![Build & Push turboVNC 01 ROS2 Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_01_ros2.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_01_ros2.yml) | `ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy`              |
| [![Build & Push turboVNC 02 ROS2 Gazebo Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_02_ros2_gazebo.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-turboVNC_02_ros2_gazebo.yml)   | `ghcr.io/ipa-may/ros2-turbovnc-novnc:jazzy-gazebo`      |
| [![Build & Push tigerVNC 01 Humble VSCode XFCE Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_01_humble_vscode_xfce.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_01_humble_vscode_xfce.yml) | `ghcr.io/ipa-may/ros2-tigervnc-novnc:humble-vscode-xfce` |
| [![Build & Push tigerVNC 01 Jazzy VSCode XFCE Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_01_jazzy_vscode_xfce.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_01_jazzy_vscode_xfce.yml) | `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-vscode-xfce`  |
| [![Build & Push tigerVNC 02 Jazzy XFCE Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_02_jazzy_xfce.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_02_jazzy_xfce.yml) | `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-xfce`         |
| [![Build & Push tigerVNC 03 Jazzy MATE Firefox Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_03_jazzy_mate_firefox.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_03_jazzy_mate_firefox.yml) | `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-mate-firefox` |
| [![Build & Push tigerVNC 04 Jazzy ros2_control Image](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_04_jazzy_ros2_control.yml/badge.svg)](https://github.com/ipa-may/docker_vnc_ROS2/actions/workflows/build-and-push-tigerVNC_04_jazzy_ros2_control.yml) | `ghcr.io/ipa-may/ros2-tigervnc-novnc:jazzy-ros2-control` |

## Quick image differences

- `ros2-turbovnc-novnc:jazzy` — TurboVNC + VirtualGL base (GPU-accelerated RViz/Gazebo), lean XFCE desktop.
- `ros2-turbovnc-novnc:jazzy-gazebo` — TurboVNC base plus Gazebo + ros2_control preinstalled.
- `ros2-tigervnc-novnc:humble-vscode-xfce` — TigerVNC XFCE desktop for ROS 2 Humble with VS Code Dev Container attach metadata.
- `ros2-tigervnc-novnc:jazzy-vscode-xfce` — TigerVNC XFCE desktop tuned for VNC-first workflows and VS Code Dev Container attach.
- `ros2-tigervnc-novnc:jazzy-xfce` — Minimal TigerVNC XFCE desktop with VNC and browser noVNC.
- `ros2-tigervnc-novnc:jazzy-mate-firefox` — Full TigerVNC MATE desktop with Firefox and browser noVNC access.
- `ros2-tigervnc-novnc:jazzy-ros2-control` — TigerVNC desktop with staged ros2_control + Gazebo tooling and VS Code attach metadata.

| Image tag                                | VNC stack            | Desktop | Extras / intent                           |
| ---------------------------------------- | -------------------- | ------- | ----------------------------------------- |
| `ros2-turbovnc-novnc:jazzy`              | TurboVNC + VirtualGL | XFCE    | GPU-accelerated base for RViz/Gazebo      |
| `ros2-turbovnc-novnc:jazzy-gazebo`       | TurboVNC + VirtualGL | XFCE    | Gazebo + ros2_control preinstalled        |
| `ros2-tigervnc-novnc:humble-vscode-xfce` | TigerVNC             | XFCE    | Humble desktop with VS Code attach config |
| `ros2-tigervnc-novnc:jazzy-vscode-xfce`  | TigerVNC             | XFCE    | VNC-first; VS Code Dev Container friendly |
| `ros2-tigervnc-novnc:jazzy-xfce`         | TigerVNC             | XFCE    | Minimal TigerVNC with VNC + noVNC         |
| `ros2-tigervnc-novnc:jazzy-mate-firefox` | TigerVNC             | MATE    | Browser-ready noVNC + Firefox             |
| `ros2-tigervnc-novnc:jazzy-ros2-control` | TigerVNC             | XFCE    | ros2_control stack + VS Code friendly     |

## Repository layout

- `tigerVNC/` — TigerVNC-based development desktops (`01_…`, `02_…`, `03_…`, `04_…` variants).
- `turboVNC/turboVNC_01_ros2/` — TurboVNC + VirtualGL + noVNC base image (publishes `jazzy`, `jazzy-fixed-v2`).
- `turboVNC/turboVNC_02_ros2_gazebo/` — TurboVNC base with Gazebo + ros2_control additions.

## Build the docker image

```sh
# Jazzy VSCode XFCE
(cd tigerVNC/01_jazzy-vscode-xfce-tigerVNC-novnc/ && docker compose build)

# Humble VSCode XFCE
(cd tigerVNC/01_humble-vscode-xfce-tigerVNC-novnc/ && docker compose build)
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
