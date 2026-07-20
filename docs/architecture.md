# System Architecture

This page describes the high-level architecture of the Volumetric Asynchronous
Teleoperation (VAT) project: remote operation of a quadruped robot using a 3D client, a
cloud-based global mapping system, and onboard state estimation on the robot.

## Overview

VAT uses a **hybrid edge–cloud** design. To get low-latency teleoperation *and*
high-fidelity global mapping at the same time, we decouple the high-frequency reactive
tasks (which stay on the robot) from the computationally heavy reconstruction (which runs
in the cloud).

Three components cooperate:

1. **The Client** — renders the world, predicts robot motion, and sends teleop commands.
2. **The Cloud** — builds the PRISM map, estimates a slow global pose, and routes state to the client.
3. **The Robot** — captures sensor data and owns the **authoritative global pose**.

Two principles drive everything below:

- **The robot is the authority on where it is.** The cloud produces a *slow, drift-free*
  global pose from the PRISM-VGGT map; the robot *fuses* that correction with its *fast*
  onboard odometry and is the single source of truth for its global pose. See
  [Pose & state estimation](#pose-state-estimation).
- **The client predicts, it does not wait.** Pose updates arrive intermittently and with
  latency. The client extrapolates the robot's motion between updates from the velocity and
  angular-velocity vectors in each pose message — the same trick online multiplayer games
  use to hide network latency.

---

## 1. The Client

The client renders the reconstructed world and the robot's live pose, and sends
teleoperation commands back.

!!! info "Current implementation vs. target"
    Today the client is a **VisPy 3D viewer** (`client/prism_viewer.py`, run with
    `make viewer`) — a single-process desktop app that renders the streamed point cloud
    plus a predicted robot avatar, with an optional live RGBD panel and the
    [periscope](periscope.md) video. The XR/VR stack described below (Unity + a headset) is
    the *target* operator interface; the VisPy viewer is the working proof of concept that
    exercises the same data path.

### Target: XR/VR interface

The intended operator interface is **tethered PC VR** (Unity on a PC, streamed to a
headset such as a Meta Quest). A Python sidecar would handle the heavy networking (PRISM
deltas, pose, video) and talk to Unity over gRPC/WebSockets. Native on-headset rendering is
avoided because the PRISM point clouds are large (tens to hundreds of MB) and a laptop
CPU/GPU deserialises and renders them far better than a mobile headset chip.

Planned UX elements:

- **Third-person "toy box" view** — first-person VR teleoperation tends to cause motion
  sickness from latency and robot bounce, so the map is viewed top-down as a point cloud
  with a ceiling plane-slice.
- **The periscope** — point clouds lack texture detail, so the operator steers a directable
  HD video slice cut from the 360° camera (the [Remote Periscope](periscope.md)), shown both
  in-scene and in a side panel.
- **1-frame RGBD "headlight"** — the raw RGBD frame is rendered as a local mesh in front of
  the avatar for instant collision awareness, before geometry is folded into the global map.
  (Implemented today as the client's RGBD panel.)

### Robot avatar & client-side prediction

The robot's avatar is positioned **entirely from the pose stream the robot publishes** (see
[Pose & state estimation](#pose-state-estimation)). Because that stream is intermittent and
network-delivered, the client never snaps the avatar to the last sample. It runs a **local
predictor**, exactly as a multiplayer game predicts other players between server snapshots:

- **State buffer** — each pose carries position, orientation (quaternion), linear velocity,
  angular velocity, and a capture timestamp; the client keeps a short ring buffer.
- **Extrapolation** — between updates the avatar advances by dead-reckoning:
  `p(t) = p₀ + v·Δt` for position and `q(t) = q₀ ⊗ Δq(ω, Δt)` for orientation.
- **Reconciliation** — when a fresh pose arrives the client blends toward it over a few
  frames (critically-damped smoothing / slerp) instead of popping.
- **Staleness handling** — if no pose arrives for a configurable horizon, velocity decays to
  zero so a disconnected robot coasts to a stop.

This is what makes the avatar feel responsive despite the multi-second latency of the heavy
mapping path: the *map* may lag, but the *robot's position within it* tracks in real time.

---

## 2. The Cloud

The cloud does the heavy reconstruction — **but it is not the authority on the robot's
pose.** Its jobs are mapping, slow global localization, and routing.

- **PRISM-VGGT mapping** — ingests camera frames from the robot and builds the dense global
  point cloud. See the [Reconstruction Engine](reconstruction_engine.md).
- **Delta streaming** — sends only the changed blocks of the point cloud to the client, so
  each update is small.
- **Slow global pose (VGGT)** — PRISM-VGGT produces a metrically-scaled camera trajectory as
  it integrates each submap. The latest keyframe pose in the map frame is the cloud's
  localization product: drift-free relative to the map, but slow (~one per submap) and
  arriving a few seconds after capture. The cloud sends this **down to the robot** as a
  correction — it does not forward it to the client directly.
- **Pose router** — the robot publishes its fused authoritative pose back up; the cloud's
  pure-Python Zenoh router (`server/router/`, `make router`) relays that stream straight
  through to the client with no fusion on the return path. This is the
  `robot pose → server router → client` path.
- **Navigation (planned)** — the mapper already emits an ESDF (Euclidean Signed Distance
  Field) slice; a global planner that turns it into waypoints is future work (see the
  [Roadmap](roadmap.md)).

!!! question "Why does the global pose go *down* to the robot instead of straight to the client?"
    The VGGT pose is drift-free but slow and laggy; on its own it would make the avatar lurch
    every few seconds. The robot's onboard odometry is the opposite — fast and smooth but
    drifting. Fusing them belongs *on the robot*, where the high-rate odometry lives and where
    the fused result is also needed for local navigation and fail-safe behaviour. The robot is
    the natural owner of the single fused pose, and the client consumes that one authoritative
    stream rather than two competing ones.

---

## 3. The Robot

The robot handles immediate survival, data capture, and **owning its own global pose**.

- **State fusion (the authoritative pose)** — the robot fuses the slow VGGT correction from
  the cloud with its fast onboard odometry into a single smooth, drift-corrected global pose,
  published at 30 Hz (`PUBLISH_HZ`). This runs in `robot/docker/pose_fuser.py`; see
  [Pose & state estimation](#pose-state-estimation) for the estimator.
- **Sensor streaming** — continually streams camera frames and telemetry to the cloud (for
  PRISM) and to the client (RGBD panel, periscope video).
- **Teleoperation** — receives `cmd_vel` from the client and relays it to the Go2 Move API,
  with a deadman timeout and e-stop (see the [Bring-up Runbook](bringup.md)).
- **Fail-safe** — because the robot owns its pose, it keeps producing a usable (if slowly
  drifting) estimate during a cloud outage.
- **Local reactive navigation (planned)** — see the [Roadmap](roadmap.md).

---

## Pose & state estimation

This is the core of the design: **the robot, not the cloud, is authoritative for the
robot's global pose.** The pose travels in a loop:

```
   ┌──────────────────────────── CLOUD ────────────────────────────┐
   │                                                                │
   │  mapping_server.py                        Zenoh router          │
   │  ┌───────────────────┐                  ┌──────────────────┐   │
   │  │ PRISM-VGGT         │  slow            │ relays robot     │   │
   │  │ global keyframe    │  pose            │ pose straight    │   │
   │  │ pose (drift-free,  │  correction      │ through to       │   │
   │  │ laggy, ~per submap)│      │           │ the client       │   │
   │  └───────────────────┘      │           └────────▲─────────┘   │
   └─────────────────────────────┼────────────────────┼────────────┘
                                  │ (DOWN)             │ (UP, 30 Hz)
                                  ▼                    │
   ┌──────────────────────────── ROBOT ───────────────┼────────────┐
   │                                                   │            │
   │  onboard odometry: IMU + wheel odom + leg-FK Z    │            │
   │  (from /lowstate, native ~500 Hz)                 │            │
   │            │                                      │            │
   │            ▼                                       │            │
   │   ┌─────────────────────────────────────┐         │            │
   │   │  STATE FUSER  (ESKF, NumPy)          │ ────────┘            │
   │   │  dead-reckons at high rate; the VGGT │  authoritative       │
   │   │  fix re-anchors the world←odom       │  fused pose +        │
   │   │  transform (slewed, not teleported)  │  velocity + rotation │
   │   └─────────────────────────────────────┘                      │
   └────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼  (relayed by cloud router)
   ┌──────────────────────────── CLIENT ───────────────────────────┐
   │  predictor: dead-reckons avatar between pose samples using     │
   │  the velocity + angular-velocity vectors (multiplayer netcode) │
   └────────────────────────────────────────────────────────────────┘
```

### The two pose sources and why they must be fused

| Source | Rate | Latency | Drift | Frame |
|---|---|---|---|---|
| **VGGT global pose** (cloud) | ~one per submap | a few seconds | none (locked to map) | global map |
| **Onboard odometry** (robot) | native ~500 Hz | ~ms | accumulates over time | local/odom |

Neither is usable alone: VGGT is far too slow and laggy to drive an avatar; odometry drifts
away from the map. The fuser uses the fast odometry to **propagate** the pose between VGGT
updates, and the slow VGGT pose to **re-anchor** the accumulated drift each time a global fix
lands — the standard "high-rate prediction + low-rate correction" structure.

Crucially, the Go2-W's onboard odometry is *not* what its ROS API nominally advertises. The
`SportModeState` velocity and body-height fields are dead (zeroed), so translation is derived
from **wheel odometry** (the four wheel motors' speeds), attitude from the **IMU**, and
vertical height from **leg forward-kinematics**. See [Robot Data Sources](robot_data_sources.md)
for the measured reality that forces this.

### Camera ≠ base: the kinematic offset

VGGT estimates the pose of the **camera**, but we want the pose of the robot **base**. On the
Go2-W the camera rides a selfie-stick on the body's back (and may later move to an actuated
arm), so the two differ by a real transform: roll/pitch the body and the stick swings the
camera even though the wheels never moved. The robot subtracts the mount transform,
`T_world_base = T_world_camera ∘ inverse(T_base_camera)`, before fusing — so the server sends
a *camera* correction and stays kinematics-agnostic, while the robot (which has the joint/body
state) owns the base solution.

The same kinematics yield the **camera height above the floor**, which PRISM uses to ground
metric scale. Height comes from one of two modes (`CAMERA_HEIGHT_MODE` in `vat.env`):

- `const` (default) — a fixed measured height, `CAMERA_HEIGHT_M` (≈1.15 m). Matches the
  known-good offline runs.
- `legs` — stance-aware height from leg forward-kinematics, for a quadruped that lies down and
  stands up.

For a fixed stick the mount transform is a constant; for the future arm it becomes forward
kinematics from a URDF — same interface, swapped implementation.

### What the robot publishes (the authoritative pose message)

Every fused pose carries the full state the client needs to predict motion: **position**
(xyz, metres, map frame), **orientation** (quaternion), **linear velocity** (m/s),
**angular velocity** (rad/s, body frame), a capture **timestamp** (ns), and a **fix-quality
flag** (whether this sample was just VGGT-corrected or is dead-reckoning on odometry only).

The exact byte layout and Zenoh keys are in the [Wire Protocol reference](reference/wire_protocol.md).

### Estimator: the ESKF

The fuser (`robot/docker/pose_fuser.py`) runs as a plain **Python process inside the robot's
Docker container** — not a ROS node. This is possible because everything it needs is already
on the Zenoh bus: `dynamic_bridge.py` bridges the Unitree ROS topics (IMU, wheel/leg state) to
Zenoh, and the cloud publishes the VGGT correction over Zenoh. So the fuser is "just another
Zenoh client": it subscribes to the bridged odometry and the pose correction, runs the filter
in NumPy, and publishes the authoritative pose.

The default backend is an **Error-State Kalman Filter** (`POSE_BACKEND=eskf`,
`eskf_estimator.py`):

- A 6-state filter over **position + velocity** in the odom frame. Velocity is a filtered
  state, propagated by IMU acceleration and nudged by wheel-odometry velocity with anisotropic
  noise (tight forward, soft lateral, since the wheels observe forward motion well and strafe
  poorly). A slow accelerometer bias is learned during zero-velocity updates.
- Attitude is taken directly from the IMU quaternion.
- Vertical height comes from leg-FK stance height (`VERTICAL_FROM_LEGS=1`) — the fix that stops
  a prone robot from appearing to float.
- The VGGT fix is applied *outside* the filter as a delayed-measurement rigid re-anchor of the
  `world←odom` transform, matched at the fix's capture time, and **slewed** toward the target
  over `CORRECTION_SLEW_TAU` rather than teleported (so the avatar doesn't jump).

!!! note "History: why not `fuse` / `robot_localization`, and where GTSAM went"
    The mature ROS fusion stacks (`fuse`, `robot_localization`) are ROS-native C++ nodes,
    configured through ROS parameters and launch files. Using either would mean adding a ROS
    node into the loop under the Jetson's ROS/Python constraints. Keeping the fuser as a pure
    NumPy Zenoh client avoids that. An earlier GTSAM pose-graph backend was tried and
    **removed** — it never loaded reliably on the Jetson — so the ESKF is the shipped default.
    If a graph optimiser or loop closure is needed later, the migration path is to swap the
    process for a ROS node publishing the same message on the same Zenoh key; nothing else in
    the system changes.

!!! info "Visual odometry is present but disabled"
    A visual-odometry module (`robot/docker/visual_odometry.py`) was added to recover lateral
    (strafe) motion the wheel odometry misses, then **disabled** (`VO_ENABLE=0` in `vat.env`)
    because it was not yet reliable enough. The code is intact and gated behind the flag; the
    fuser runs on wheel + IMU + leg-FK today. See the [Roadmap](roadmap.md).

---

## Summary of the data flow

1. **Robot → cloud** — streams camera frames and telemetry.
2. **Cloud (mapping)** — ingests frames, builds the PRISM global map, computes cloud deltas,
   and produces the slow VGGT global pose.
3. **Cloud → robot** — sends the VGGT **camera** pose down as a drift correction.
4. **Robot (kinematics + fusion)** — converts the camera correction to a **base** pose
   (subtracting the stick transform), fuses it with wheel + IMU + leg-FK odometry in the ESKF
   into a single authoritative 30 Hz pose (position, orientation, linear/angular velocity), and
   publishes it back up. It also stamps the camera height for PRISM's metric scale.
5. **Cloud (router) → client** — the Zenoh router relays the robot's pose stream straight
   through; in parallel the cloud streams PRISM map deltas.
6. **Client** — receives map deltas and the authoritative pose stream, renders the point cloud
   and the RGBD/periscope panels, and **predicts the avatar's motion between pose updates**
   from the velocity/rotation vectors.
