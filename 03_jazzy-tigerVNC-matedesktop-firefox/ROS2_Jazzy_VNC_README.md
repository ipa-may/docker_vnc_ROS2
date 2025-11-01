# ROS 2 Jazzy Desktop (Ubuntu 24.04) with VNC & noVNC
A reproducible Docker setup to run a **full Ubuntu MATE desktop** with **ROS 2 Jazzy** inside a container, accessible via:

- **VNC** (TigerVNC server on `:1` → TCP **5901**)
- **noVNC** (browser client proxied by `websockify` on TCP **80**)

This README matches the **Dockerfile + `entrypoint.sh`** you shared and shows how to build, run, and customize it.

---

## ✅ What you get
- Ubuntu **24.04 (Noble)** base with **ROS 2 Jazzy** (`desktop` by default).
- Full **MATE** desktop session served over **TigerVNC**.
- **noVNC** so you can use the desktop **in a browser tab**.
- Developer tools (Firefox, VSCodium, build-essentials, Python, etc.).
- A clean startup via **tini** + **supervisord**.
- A sensible non-root user you can define at runtime.

> Note: The provided `entrypoint.sh` starts **noVNC on port 80** (not 6080) and VNC on **5901**.

---

## 🧱 Prerequisites
- Docker Engine or **Docker Desktop** (Windows/macOS/Linux).
- (Optional) An NVIDIA GPU + **NVIDIA Container Toolkit** for hardware acceleration.

---

## 📦 Build the image
From the folder containing your `Dockerfile` and `entrypoint.sh`:

```bash
docker build -t ros2-jazzy-desktop-vnc .
```

### Optional: choose a smaller ROS 2 install
The Dockerfile supports `INSTALL_PACKAGE` (`desktop` or `ros-base`). For a leaner image:
```bash
docker build --build-arg INSTALL_PACKAGE=ros-base -t ros2-jazzy-rosbase-vnc .
```

---

## ▶️ Run the container

### Easiest (browser only + VNC)
```bash
docker run --rm -it \
  -p 80:80 \        # noVNC in your browser
  -p 5901:5901 \    # VNC (TigerVNC) optional
  -e USER=ubuntu \
  -e PASSWORD=ubuntu \
  --name jazzy-desktop \
  ros2-jazzy-desktop-vnc
```

Now open **http://localhost/** in your browser.  
Or connect a VNC client to **localhost:5901** (password = `PASSWORD` value).

> The entrypoint injects the same password into noVNC’s UI by editing `/usr/lib/novnc/app/ui.js` at runtime.

### With a persistent ROS workspace
```bash
# Linux/macOS
docker run --rm -it \
  -p 80:80 -p 5901:5901 \
  -e USER=ubuntu -e PASSWORD=ubuntu \
  -v "$HOME/ros2_ws:/home/ubuntu/ros2_ws" \
  --name jazzy-desktop \
  ros2-jazzy-desktop-vnc

# Windows PowerShell
docker run --rm -it ^
  -p 80:80 -p 5901:5901 ^
  -e USER=ubuntu -e PASSWORD=ubuntu ^
  -v "$Env:USERPROFILE\ros2_ws:/home/ubuntu/ros2_ws" ^
  --name jazzy-desktop ^
  ros2-jazzy-desktop-vnc
```

### With NVIDIA GPU acceleration (optional)
```bash
docker run --rm -it \
  --gpus all \
  -p 80:80 -p 5901:5901 \
  -e USER=ubuntu -e PASSWORD=ubuntu \
  --name jazzy-desktop \
  ros2-jazzy-desktop-vnc
```
> On some environments, Ubuntu Jammy/Noble based images require:
> `--security-opt seccomp=unconfined`
> (see https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56)

---

## 🚪 Log in & use ROS 2
- **Browser**: go to **http://localhost/** → you should see the MATE desktop (noVNC).
- **VNC client**: connect to **localhost:5901** (display `:1`).

Inside the desktop:
1. Open **Terminator** (desktop shortcut provided) or **MATE Terminal**.
2. ROS 2 is auto-sourced via `.bashrc`:
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash   # done by entrypoint if missing
   ```
3. Test quickly:
   ```bash
   ros2 run demo_nodes_cpp talker
   # New terminal:
   ros2 run demo_nodes_cpp listener
   ```

---

## 🛠️ Customization

### 1) Users & passwords
- `USER` (env): non-root username to create (default: `root` → stays root).
- `PASSWORD` (env): Linux user password **and** VNC/noVNC password (default: `ubuntu`).

> Security: Don’t expose ports **80/5901** beyond localhost without changing the password and adding network protections.

### 2) Screen resolution & color depth
The current script hardcodes **1920x1080, depth 24** in `~/.vnc/vnc_run.sh`:
```sh
vncserver :1 -fg -geometry 1920x1080 -depth 24
```
You can change these values or make them **configurable via env**. Example modification in `entrypoint.sh`:
```sh
VNC_GEOMETRY="${VNC_GEOMETRY:-1920x1080}"
VNC_DEPTH="${VNC_DEPTH:-24}"
# ...
vncserver :1 -fg -geometry "$VNC_GEOMETRY" -depth "$VNC_DEPTH"
```

Then run with:
```bash
-e VNC_GEOMETRY=1600x900 -e VNC_DEPTH=24
```

### 3) noVNC port
The entrypoint starts websockify like this:
```ini
[program:novnc]
command=gosu '$USER' bash -c "websockify --web=/usr/lib/novnc 80 localhost:5901"
```
Change **80** to another port (e.g. 6080) and map it:
```bash
-p 6080:6080
```
*(Remember to update your docs and links.)*

### 4) Desktop environment
You are using **MATE** (`mate-session`). To switch to **XFCE** for a lighter image:
- In the Dockerfile, install `xfce4 xfce4-terminal dbus-x11 x11-xserver-utils` instead of `ubuntu-mate-desktop`.
- In `entrypoint.sh`, change the xstartup to:
  ```sh
  echo -e '#!/bin/sh\nunset DBUS_SESSION_BUS_ADDRESS\nstartxfce4' > "$XSTARTUP_PATH"
  ```

### 5) ROS 2 meta package
You can select a smaller ROS 2 install at **build-time**:
```bash
--build-arg INSTALL_PACKAGE=ros-base
```
or keep `desktop` for full GUI tooling (rviz2/rqt, etc.).

### 6) Clipboard & prefilled password in noVNC
The entrypoint modifies `/usr/lib/novnc/app/ui.js` to inject the password:
```sh
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" ...
```
- ✅ Convenient for demo/dev.
- ⚠️ If you prefer to **disable auto-filled passwords**, remove this line.

### 7) Architecture handling
On **aarch64**, the script sets:
```sh
LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver ...
```
This is kept for better stability. You generally don’t need to change it.

---

## 🧰 Troubleshooting

- **Blank/no desktop in browser**  
  Ensure container is running and port **80** is mapped. Try a hard refresh. Check logs:
  ```bash
  docker logs jazzy-desktop
  ```

- **“Only loopback connections are allowed”**  
  Ensure `websockify` is binding on `0.0.0.0` via Docker port mapping. Using `-p 80:80` exposes it on the host.

- **GLX/OpenGL errors in RViz/Gazebo**  
  Use `--gpus all` with NVIDIA drivers + NVIDIA Container Toolkit, or run headless features if GPU is unavailable.

- **“Address already in use” on port 80**  
  Map a different host port, e.g. `-p 8080:80` and open `http://localhost:8080/`.

- **Permission issues with rosdep**  
  The entrypoint copies `/root/.ros/rosdep` into the user’s home to avoid permission problems. If you change usernames or volumes, ensure `~/.ros` is writable by the user.

---

## 🧪 docker-compose (optional)
`docker-compose.yml` example with a persistent workspace and custom resolution:

```yaml
services:
  jazzy-desktop:
    image: ros2-jazzy-desktop-vnc
    build:
      context: .
      args:
        INSTALL_PACKAGE: desktop
    container_name: jazzy-desktop
    environment:
      USER: ubuntu
      PASSWORD: change-me
      VNC_GEOMETRY: 1600x900
      VNC_DEPTH: "24"
    ports:
      - "8080:80"     # noVNC in browser
      - "5901:5901"   # VNC client
    volumes:
      - ./ros2_ws:/home/ubuntu/ros2_ws
    # For NVIDIA GPU support (uncomment next two lines):
    # deploy:
    #   resources:
    #     reservations: {devices: [{capabilities: ["gpu"]}]}
    # runtime: nvidia
```

Then:
```bash
docker compose up --build
# Open http://localhost:8080/
```

---

## 🔐 Security tips
- Always change the default `PASSWORD` for any network-exposed container.
- Keep ports bound to `127.0.0.1` (Docker default) or use a firewall/reverse proxy with TLS if remote access is needed.
- For teams, consider disabling the password injection in the noVNC UI and rely on proper auth.

---

## 📎 Quick reference
| Thing | Where/How |
|---|---|
| Browser desktop | `http://localhost/` (container 80 → host 80) |
| VNC viewer | `localhost:5901` (display `:1`) |
| Default credentials | `USER=ubuntu`, `PASSWORD=ubuntu` (change these) |
| ROS 2 setup | Auto-sourced in `.bashrc` (Jazzy) |
| Change resolution | Edit `entrypoint.sh` (or add `VNC_GEOMETRY`, `VNC_DEPTH` envs as shown) |
| Swap to XFCE | Replace MATE install and xstartup command |

---

Happy hacking with **ROS 2 Jazzy** in a browser-ready desktop! 🚀
