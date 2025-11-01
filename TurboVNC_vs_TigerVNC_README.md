# TurboVNC vs TigerVNC in ROS 2 Docker Environments

## Overview

When running graphical tools like **Gazebo** or **RViz** inside a Docker container, your choice of VNC server greatly affects rendering performance. The two most common options are **TigerVNC** and **TurboVNC**.

---

## Key Differences

| Feature             | **TigerVNC**                               | **TurboVNC**                                     |
| ------------------- | ------------------------------------------ | ------------------------------------------------ |
| Origin              | Based on RealVNC / TightVNC                | Fork of TightVNC optimized for 3D + OpenGL       |
| Focus               | Lightweight general-purpose remote desktop | High-performance OpenGL rendering                |
| Rendering           | Software-rendered framebuffer only         | Hardware-accelerated OpenGL via VirtualGL        |
| Typical use         | Headless servers, light desktops           | 3D visualization (Gazebo, RViz, CAD, simulation) |
| Speed (3D apps)     | Slow (software rasterizer)                 | Much faster (passes GPU-rendered frames)         |
| Color compression   | Basic (Tight/Zlib)                         | Optimized JPEG compression for large frames      |
| Integration         | Standard Linux repos                       | Works with **VirtualGL**                         |
| noVNC compatibility | Full (websocket bridge works)              | Works, but may need minor tweaks                 |
| Package name        | `tigervnc-standalone-server`               | `turbovnc`                                       |

---

## Rendering Paths

### TigerVNC (current default)

```
RViz → OpenGL calls → Mesa software renderer (llvmpipe)
→ framebuffer → sent via VNC to client
```

- All rendering happens **on CPU**.
- GPU on your host is **not used**.
- Low frame rate and high CPU usage.

### TurboVNC + VirtualGL

```
RViz → OpenGL calls → intercepted by VirtualGL → GPU (NVIDIA)
→ off-screen render → JPEG-compressed by TurboVNC → sent to VNC client
```

- **Real GPU acceleration**.
- Smoother 3D, lower latency.
- Optimized for remote rendering workloads.

---

## Why This Matters on Windows Hosts

Running Linux containers on **Windows via WSL2** means:

- GPU access is **limited** unless using NVIDIA container runtime.
- Even with access, **TigerVNC** can’t leverage it.
- **TurboVNC + VirtualGL** can, giving real GPU-powered 3D rendering.

| Use case                        | Recommended                     |
| ------------------------------- | ------------------------------- |
| Just a GUI terminal or editor   | TigerVNC (simpler)              |
| Gazebo or RViz visualizations   | TurboVNC + VirtualGL            |
| Windows + Docker Desktop (WSL2) | TurboVNC for smoother streaming |
| Linux host with NVIDIA GPU      | TurboVNC + VirtualGL            |

---

## VirtualGL Role

**VirtualGL** intercepts OpenGL calls from your 3D app, renders using the GPU, and passes the frame to TurboVNC.

```
[Container app] → VirtualGL → GPU → TurboVNC → Your viewer
```

Result: hardware-accelerated frames over VNC.

---

## 🧠 Why TigerVNC is Default

- Ships with Ubuntu (simple `apt install`)
- Works headlessly with no GPU setup
- Integrates easily with noVNC (browser-based desktop)
- Ideal for lightweight ROS tools or quick visualization

---

## 🏁 TL;DR

| Scenario                                   | Best Choice              |
| ------------------------------------------ | ------------------------ |
| Headless server, terminals only            | **TigerVNC**             |
| Heavy visualization (RViz, Gazebo, MoveIt) | **TurboVNC + VirtualGL** |
| Want maximum performance on Windows host   | **TurboVNC + VirtualGL** |
| No GPU, basic debugging                    | **TigerVNC**             |

---

## 💡 Next Steps

If you want to upgrade your current **TigerVNC** container to **TurboVNC + VirtualGL**, you’ll need to:

1. Install `turbovnc` and `virtualgl` packages.
2. Add the NVIDIA runtime (`--gpus all`) if you have an NVIDIA GPU.
3. Start the VNC session with `vglrun`.
