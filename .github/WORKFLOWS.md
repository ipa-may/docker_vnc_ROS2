# GitHub Actions Workflows

This repository publishes the ROS 2 VNC images to GHCR using six workflows located in `.github/workflows/`.

**`build-and-push-turboVNC_01_ros2.yml` — base TurboVNC image**
- Watches `turboVNC/turboVNC_01_ros2/**` and its workflow file on pushes to `main`, and supports manual runs.
- Calls the shared `docker-metadata.yml` workflow to generate branch/commit/stable tags, then executes `docker/build-push-action@v5` to push `ghcr.io/<owner>/ros2-turbovnc-novnc` with the `jazzy` and `jazzy-fixed-v2` aliases.

**`build-and-push-turboVNC_02_ros2_gazebo.yml` — Gazebo/ros2_control layer**
- Triggers on changes under `turboVNC/turboVNC_02_ros2_gazebo/**`, on updates to its workflow file, or automatically whenever `Build & Push turboVNC 01 ROS2 Image` completes successfully (via `workflow_run`), ensuring the dependent image rebuilds after base changes.
- Reuses `docker-metadata.yml` to apply the same metadata rules but with a `-gazebo` suffix, then pushes the Gazebo variant using `docker/build-push-action@v5`. The Dockerfile consumes the tag produced by the 01 workflow.

**`build-and-push-tigerVNC_01_jazzy_vscode_xfce.yml` — TigerVNC + VS Code helper desktop**
- Listens for changes under `tigerVNC/01_jazzy-vscode-xfce-tigerVNC-novnc/**` or its workflow file on `main`, plus manual dispatch.
- Publishes `ghcr.io/<owner>/ros2-tigervnc-novnc` with branch/sha tags suffixed `-vscode-xfce` and the stable tag `jazzy-vscode-xfce`.

**`build-and-push-tigerVNC_02_jazzy_xfce.yml` — TigerVNC XFCE desktop**
- Triggers on updates to `tigerVNC/02_jazzy-xfce-tigerVNC-novnc/**` or the workflow file on `main`, plus manual dispatch.
- Publishes `ghcr.io/<owner>/ros2-tigervnc-novnc` with branch/sha tags suffixed `-xfce` and the stable tag `jazzy-xfce`.

**`build-and-push-tigerVNC_03_jazzy_mate_firefox.yml` — TigerVNC MATE desktop with Firefox**
- Listens for changes under `tigerVNC/03_jazzy-tigerVNC-matedesktop-firefox/**` or the workflow file on `main`, plus manual dispatch.
- Publishes `ghcr.io/<owner>/ros2-tigervnc-novnc` with branch/sha tags suffixed `-mate-firefox` and the stable tag `jazzy-mate-firefox`.

**`docker-metadata.yml` — reusable metadata generator**
- Exposes a `workflow_call` interface that accepts an image name plus newline-separated tag rules.
- Runs `docker/metadata-action@v5` and returns the resolved tags and labels so the build workflows can inject them into `docker/build-push-action`.

This structure keeps tagging logic centralized while giving each image its own entry point and automatic rebuild chain. Adjust tag schemes in `docker-metadata.yml` for shared changes, or in the caller workflows for image-specific overrides.
