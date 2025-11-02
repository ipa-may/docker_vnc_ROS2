# GitHub Actions Workflows

This repository publishes the ROS 2 VNC images to GHCR using three workflows located in `.github/workflows/`.

**`build-and-push-05.yml` — base image**
- Watches `05_jazzy-turboVNC-virtualGL-noVNC/**` and its workflow file on pushes to `main`, and supports manual runs.
- Calls the shared `docker-metadata.yml` workflow to generate branch/commit/stable tags, then executes `docker/build-push-action@v5` to push `ghcr.io/<owner>/ros2-turbovnc-novnc` with the `jazzy` and `jazzy-fixed-v2` aliases.

**`build-and-push-07.yml` — Gazebo/ros2_control layer**
- Triggers on changes under `07_jazzy-turboVNC-virtualGL-noVNC-gazebo-ros2control/**`, on updates to its workflow file, or automatically whenever `Build & Push 05 TurboVNC Image` completes successfully (via `workflow_run`), ensuring the dependent image rebuilds after base changes.
- Reuses `docker-metadata.yml` to apply the same metadata rules but with a `-gazebo` suffix, then pushes the Gazebo variant using `docker/build-push-action@v5`. The Dockerfile consumes the tag produced by the 05 workflow.

**`docker-metadata.yml` — reusable metadata generator**
- Exposes a `workflow_call` interface that accepts an image name plus newline-separated tag rules.
- Runs `docker/metadata-action@v5` and returns the resolved tags and labels so the build workflows can inject them into `docker/build-push-action`.

This structure keeps tagging logic centralized while giving each image its own entry point and automatic rebuild chain. Adjust tag schemes in `docker-metadata.yml` for shared changes, or in the caller workflows for image-specific overrides.
