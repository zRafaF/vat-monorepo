# VAT — Volumetric Asynchronous Teleoperation

VAT streams a live **360° point cloud** and **robot pose** from a
[Unitree Go2-W](https://www.unitree.com/) carrying a **RICOH Theta X** panoramic
camera, over [Zenoh](https://zenoh.io), to a GPU **mapping server** running the
**PRISM-VGGT** reconstruction engine, and renders it in a 3D viewer on any machine
across the internet or a VPN.

The design goal is *asynchronous* telepresence: the robot never waits for the cloud.
It sends a light real-time stream for online mapping, keeps a full-resolution local
archive for heavy offline reconstruction later, and stays authoritative for its own
pose. The cloud contributes a slow, drift-free correction; the client predicts motion
between updates like multiplayer game netcode.

!!! note "Status"
    Research prototype, in staged bring-up. The reconstruction engine, live streaming,
    remote periscope, and teleoperation work today. Autonomous navigation (point-and-click
    + Nav2) is planned — see the [Roadmap](roadmap.md).

## The system in one picture

```
RICOH Theta X ──USB (UVC)──▶ theta_camera (robot, in Docker)
                               ├─ downscale + encode → go2/prism/camera/frame ──▶ server
                               └─ full-res 4K twin  → local rolling archive (offline recon)

Unitree state (IMU, wheel odometry) ──ROS→Zenoh bridge──▶ pose_fuser (robot)

     ┌─────────────────────────── cloud (GPU server) ───────────────────────────┐
     │  mapping_server → PRISM-VGGT: pano → cubemap → nvblox TSDF → metric map    │
     │      point-cloud deltas + trajectory  ─────────────────────────────▶ client│
     │      slow VGGT pose correction  ───────────────────────────────────▶ robot │
     └───────────────────────────────────────────────────────────────────────────┘

pose path:     robot pose_fuser (authoritative, 30 Hz) ─▶ Zenoh router ─▶ client predicts
control path:  client keyboard → cmd_vel ─▶ teleop_bridge ─▶ Go2 Move API (deadman + e-stop)
render:        prism_viewer (client) — point cloud + predicted robot avatar (VisPy 3D)
```

Three machines cooperate:

| Role | Machine | Runs |
|---|---|---|
| 🤖 **Robot** | Jetson on the Go2-W | Theta X UVC feed + a Docker container (ROS↔Zenoh bridge, camera, pose fuser, teleop) |
| ☁️ **Server** | GPU workstation | Zenoh router + PRISM-VGGT mapping server (CUDA 12.8) |
| 💻 **Client** | your laptop | VisPy 3D viewer + diagnostic tools |

## Where to go next

- **[Architecture](architecture.md)** — how the data and pose paths fit together, and why the robot (not the cloud) owns its pose.
- **[Reconstruction Engine](reconstruction_engine.md)** — what PRISM-VGGT does: panorama → cubemap → nvblox, metric grounding, and the design choices behind it.
- **[Remote Periscope](periscope.md)** — the directable HD video slice out of the 360° stream.
- **[Robot Data Sources](robot_data_sources.md)** — which Go2-W sensors actually work (measured), and what that forces on the estimator.
- **[Roadmap](roadmap.md)** — what's planned (autonomous navigation) and what we tried that didn't work.

## Getting it running

If you just want to bring the system up, start at **[Set up & run](setup/index.md)**. It
assumes no prior Linux experience and walks each machine end to end, then hands off to the
**[Bring-up Runbook](bringup.md)**, which drives the staged `make steps` sequence until
every stage is green.

Everything is driven by a single `makefile` and one config file, `vat.env`. On any machine:

```bash
make help    # list every target, grouped by machine
make steps   # print the staged Stage 0 → Stage 4 bring-up runbook
```
