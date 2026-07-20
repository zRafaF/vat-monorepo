# Roadmap

What VAT does today, what is planned next, and — just as importantly — what we tried that
didn't work, so it isn't re-attempted blindly.

## Works today

- **Live reconstruction** — PRISM-VGGT builds a metric point cloud from the 360° stream and
  streams changed blocks to the client. See the [Reconstruction Engine](reconstruction_engine.md).
- **Robot-authoritative pose** — the onboard ESKF fuses wheel odometry + IMU + leg-FK height,
  re-anchored by slow VGGT corrections; the client predicts between samples. See the
  [Architecture](architecture.md).
- **Remote periscope** — a directable HD video slice cut from the panorama. See
  [Remote Periscope](periscope.md).
- **Teleoperation** — keyboard `cmd_vel` to the Go2 Move API, with a deadman timeout and
  e-stop. See the [Bring-up Runbook](bringup.md).
- **Full-res offline archive** — every transmitted frame keeps a full-resolution twin on the
  robot for later heavy reconstruction (Gaussian splats, NeRF). See [Robot setup](setup/robot.md).

## Planned

### Autonomous navigation (point-and-click + Nav2)

The headline next feature: let the operator **click a destination in the 3D client** and have
the robot drive there autonomously, instead of only manual teleoperation.

The substrate already exists — the mapping server publishes an **ESDF slice** (Euclidean
Signed Distance Field; `COMPUTE_ESDF=1`, see the [Reconstruction Engine](reconstruction_engine.md)),
which is exactly what a planner needs to reason about free space and obstacles. The remaining
pieces, **not yet implemented**:

- **Global planner (cloud)** — turn a clicked goal + the ESDF into a sparse waypoint path,
  streamed down to the robot.
- **Local reactive navigation (robot)** — follow waypoints while dodging dynamic obstacles,
  most likely via a **[Nav2](https://nav2.org/)** stack, reusing the ESDF/costmap.
- **Client UX** — a click-to-goal interaction in the viewer and a rendered path.

This is planned work; the design intent is documented here so the ESDF output and the
robot-authoritative pose (which the local planner will consume) are understood as deliberate
groundwork.

### Loop closure in PRISM-VGGT

The shipped reconstruction pipeline is **non-looping**, and trajectory error grows on long
loops. The intended fix is a **GTSAM Sim(3) pose graph** that closes loops when the robot
re-observes a place. See [What holds up and what doesn't](reconstruction_engine.md#what-holds-up-and-what-doesnt).

### Re-enable visual odometry

A visual-odometry module exists to recover the lateral (strafe) motion wheel odometry misses,
but it is currently **disabled** (`VO_ENABLE=0`) because it wasn't reliable enough. Re-enabling
it (or replacing it) would improve dead-reckoning during sideways motion.

### On-robot Zenoh router

To keep robot-internal traffic off the VPN, the plan is to run a **local Zenoh router on the
robot** so only the camera frame and pose keys cross the network. Documented in
[Robot setup](setup/robot.md) but **not wired up yet**.

## What we tried that didn't work

Kept on record so these dead ends aren't silently rediscovered.

- **GTSAM pose-graph fuser on the robot** — an earlier estimator backend built on GTSAM was
  removed; it never loaded reliably on the Jetson. The robot now runs a pure-NumPy ESKF. See
  [Estimator: the ESKF](architecture.md#estimator-the-eskf).
- **SL(4) projective alignment as the default** — extra projective freedom fits clean overlaps
  slightly better but drifts non-rigidly on loops. Default is now Sim(3). See
  [Why Sim(3) is the default](reconstruction_engine.md#4-alignment-into-the-world-frame).
- **DINOv2 + SALAD loop closure on panoramas** — degraded on wide-FOV, repetitive panoramic
  imagery; the pipeline shipped non-looping instead.
- **Rerun and Open3D viewers** — the earlier Rerun-based viewer froze on the live stream and
  Open3D was finicky; the client is now a **VisPy** viewer (`prism_viewer.py`).
- **Online-accumulate mapping (no reset)** — accumulating across batches without the periodic
  reset produced thick, fuzzy, duplicated walls; the mapper now rebuilds per batch and
  re-anchors (`PRISM_RESET_EACH_BATCH=1`).
- **Insta360 camera** — retired in favour of the RICOH Theta X, which does dynamic stitching
  and zenith correction in-camera and exposes a clean equirectangular UVC stream. See the
  [Insta360 archive note](archive/insta360.md).
- **`SportModeState` velocity/body-height for odometry** — these fields are zeroed on the Go2-W,
  so translation is taken from wheel odometry and height from leg-FK instead. See
  [Robot Data Sources](robot_data_sources.md).
