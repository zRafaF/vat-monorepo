# System Architecture

This document outlines the high-level architecture for the Volumetric Asynchronous Teleoperation (VAT) project. The pipeline is designed for remote control of a quadruped robot using an advanced XR client, a cloud-based global mapping system, and a local reactive navigation stack on the robot.

## Overview

The VAT architecture is based on a **Hybrid Edge-Cloud paradigm**. To achieve low-latency teleoperation and high-fidelity global mapping simultaneously, we decouple the high-frequency reactive tasks from the computationally heavy global planning tasks.

There are three main components:

1. **The Client (XR/VR Interface)** — renders the world, predicts robot motion, sends teleop commands.
2. **The Cloud (Global Mapping & Routing)** — builds the PRISM map, estimates the low-frequency global pose, and routes state to the client.
3. **The Robot (Local Autonomy, Sensors & State Fusion)** — captures sensor data and owns the **authoritative global pose** of the robot.

Two design principles drive everything below:

- **The robot is the authority on where it is.** The cloud produces a *slow, drift-free* global pose from the PRISM/VGGT map; the robot *fuses* that correction with its *fast* onboard odometry and is the single source of truth for its global pose. See [Pose & State Estimation](#pose-state-estimation).
- **The client predicts, it does not wait.** Pose updates arrive intermittently and with latency. The client extrapolates the robot's motion between updates using the velocity and rotation vectors carried in each pose message — the same technique online multiplayer games use to hide network latency.

---

## 1. The Client (XR/VR Interface)

The client provides the operator with an intuitive, lag-free teleoperation interface. We prioritize **Tethered PC VR (e.g., Unity running on a PC streaming to Meta Quest via AirLink or Virtual Desktop)**.

### Technology Stack

* **Engine:** **Unity** is the engine of choice. It offers the most robust XR ecosystem, excellent point cloud rendering plugins (such as PCX), and reliable networking libraries (like WebRTC).
* **Python Backend:** A Python sidecar runs alongside the Unity PC app to handle data gathering, heavy networking (e.g., PRISM delta updates), and cloud communication. Unity communicates with this Python layer via gRPC or WebSockets.
* **Why not Native Quest?** The massive PRISM point clouds (50–200 MB) require significant memory, vertex throughput, and network deserialization. A laptop CPU/GPU is vastly superior for these tasks compared to the mobile Snapdragon XR2 on the Quest headset.

### User Experience (UX)

* **Third-Person "Toy Box" View:** First-person VR teleoperation typically induces severe motion sickness due to latency and robot bouncing. We use a "God-Mode" perspective. The PRISM map is represented as a point cloud, and we apply a plane slice on the ceiling so the user views the environment top-down.
* **The "Beacon" (Virtual Monitor):** To compensate for the lack of texture detail in point clouds, the user can steer a virtual "beacon" or raycast. This projects a real-time, high-definition video stream (via WebRTC) onto a virtual screen at the point of interest for detailed inspections.
* **1-Frame RGBD Mesh:** To provide instantaneous collision awareness, the client renders the raw RGBD frame from the robot as a local mesh attached to the front of the 3D robot avatar. This single-frame display is updated at high frequency and acts as a "headlight," allowing the user to react to fast-moving dynamic obstacles before they are integrated into the global PRISM map.

### Robot Avatar & Client-Side Prediction

The robot's 3D avatar is positioned **entirely from the pose stream the robot publishes** (see [Pose & State Estimation](#pose-state-estimation)). Because that stream is intermittent and arrives over the network, the client never snaps the avatar directly to the last received sample. Instead it runs a **local predictor**, exactly as a multiplayer game client predicts other players between server snapshots:

* **State buffer:** Each incoming pose carries position, orientation (quaternion), linear velocity, angular velocity, and a capture timestamp. The client keeps a short ring buffer of recent states.
* **Extrapolation:** Between updates, the client advances the avatar by dead-reckoning: `p(t) = p₀ + v·Δt` for position and `q(t) = q₀ ⊗ Δq(ω, Δt)` for orientation, where `Δt` is the time since the last sample. This keeps the avatar moving smoothly at the render frame rate (72–90 Hz in VR) even when poses arrive at a fraction of that rate.
* **Reconciliation / smoothing:** When a fresh pose arrives, the predicted state will generally differ slightly from the new ground truth. Rather than teleporting, the client blends toward the corrected state over a few frames (critically-damped smoothing / slerp for rotation) to avoid visible popping.
* **Staleness handling:** If no pose has arrived for longer than a configurable horizon, the client decays velocity toward zero so a disconnected robot coasts to a stop instead of flying off on the last known velocity.

This predictor is what makes the avatar feel responsive despite the multi-second latency of the heavy mapping path: the *map* may lag, but the *robot's position within it* tracks in real time.

---

## 2. The Cloud (Global Mapping & Routing)

The cloud is the heavy-lifter for global environment context — **but it is not the authority on the robot's pose.** Its responsibilities are mapping, low-frequency global localization, planning, and routing state between robot and client.

* **PRISM Integration:** Receives sensory data from the robot and builds the dense, global point cloud map.
* **Delta Streaming:** Employs a robust delta streaming architecture to send only the updated portions of the point cloud to the client.
* **Low-Frequency Global Pose (VGGT):** PRISM-VGGT produces a metrically-scaled camera trajectory as it integrates each sub-window. The **latest keyframe pose in the global map frame** is the cloud's localization product. It is accurate and drift-free *relative to the map*, but slow (it lands once per sub-window, on the order of 0.3–3 Hz) and arrives ~2–4 s after capture. The cloud **sends this pose *down* to the robot** as a correction — it does not forward it to the client directly.
* **Pose Router:** The robot publishes its fused, authoritative pose stream back up. The cloud (specifically the Zenoh router process running on the cloud machine) **relays** that stream straight through to the client. The cloud does no fusion on the return path — it is a router for this data. This is the `server (pose) → dog → server (router) → client` path.
* **Global Pathfinding (future):** Computes the Euclidean Signed Distance Field (ESDF) and generates a global path. It streams a sequence of sparse waypoints down to the robot for navigation.

> **Why does the global pose go *down* to the robot instead of straight to the client?**
> The VGGT pose is drift-free but slow and laggy. On its own it would make the avatar lurch every few seconds. The robot's onboard odometry is the opposite: fast and smooth but drifts over time. Fusing them belongs *on the robot*, where the high-rate odometry lives and where the result is also needed for local navigation and fail-safe behaviour. The robot is therefore the natural owner of the single fused pose, and the client should consume that one authoritative stream — not two competing ones it would have to reconcile itself.

---

## 3. The Robot (Local Autonomy, Sensors & State Fusion)

The robot focuses on immediate survival, data collection, and **owning its own global pose**.

* **State Fusion (the authoritative pose):** The robot fuses the **low-frequency global pose correction from the cloud** (VGGT) with its **high-frequency onboard odometry** (leg odometry + IMU from `SportModeState`) to produce a single, smooth, drift-corrected global pose at high rate. This fused pose — together with its velocity and rotation vectors — is the value streamed up to the cloud router and on to the client. See [Pose & State Estimation](#pose-state-estimation) for the estimator design and the FUSE-vs-pure-Python decision.
* **Local Reactive Navigation (future):** The robot receives sparse global waypoints from the cloud. It uses a lightweight local planner to drive toward these waypoints while relying on immediate sensors to dodge dynamic obstacles.
* **Fail-Safe Mechanism:** If the cloud connection drops, the robot's local loop prevents it from blindly crashing into walls, as it can stop or maneuver using its onboard obstacle avoidance. Because the robot owns its pose, it also keeps producing a usable (if slowly drifting) pose estimate during a cloud outage.
* **Sensor Streaming:** Continually streams RGBD frames, odometry, and telemetry to the cloud (for PRISM) and the client (for the 1-frame RGBD mesh and Beacon video).

---

## Pose & State Estimation {#pose-state-estimation}

This is the core of the architecture change: **the robot, not the cloud, is authoritative for the robot's global pose.** The pose travels in a loop:

```
   ┌──────────────────────────── CLOUD ────────────────────────────┐
   │                                                                │
   │  prism_server.py                          Zenoh router          │
   │  ┌───────────────────┐                  ┌──────────────────┐   │
   │  │ PRISM-VGGT         │  low-freq        │ relays robot     │   │
   │  │ global keyframe    │  global pose     │ pose straight    │   │
   │  │ pose (drift-free,  │  correction      │ through to       │   │
   │  │ ~0.3–3 Hz, laggy)  │      │           │ the client       │   │
   │  └───────────────────┘      │           └────────▲─────────┘   │
   └─────────────────────────────┼────────────────────┼────────────┘
                                  │ (DOWN)             │ (UP, high-freq)
                                  ▼                    │
   ┌──────────────────────────── ROBOT ───────────────┼────────────┐
   │                                                   │            │
   │  high-freq odometry (leg odom + IMU, ~50–200 Hz)  │            │
   │            │                                      │            │
   │            ▼                                       │            │
   │   ┌─────────────────────────────────────┐         │            │
   │   │  STATE FUSER  (EKF / complementary)  │ ────────┘            │
   │   │  fuses slow global correction with   │  authoritative       │
   │   │  fast local odometry → smooth,       │  fused pose +        │
   │   │  drift-corrected global pose         │  velocity + rotation │
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
| **VGGT global pose** (cloud) | ~0.3–3 Hz | ~2–4 s | none (locked to map) | global map |
| **Onboard odometry** (robot) | 50–200 Hz | ~ms | accumulates over time | local/odom |

Neither is usable alone. VGGT alone is far too slow and laggy to drive an avatar; odometry alone drifts away from the map within seconds to minutes. The fuser uses the fast odometry to **propagate** the pose between VGGT updates, and the slow VGGT pose to **anchor/correct** the accumulated drift each time a new global pose lands. The result is a high-rate pose that is both smooth and globally consistent — the standard "high-rate prediction + low-rate correction" structure of any Kalman-style estimator.

### What the robot publishes (the authoritative pose message)

Every fused pose carries the full state the client needs to predict motion:

* **position** — `xyz`, metres, global map frame
* **orientation** — quaternion `(x, y, z, w)`, global map frame
* **linear velocity** — `xyz`, m/s
* **angular velocity** — `xyz`, rad/s, body frame
* **timestamp** — capture time (ns), so the client can compute `Δt` for extrapolation and order/interpolate samples
* **fix quality / source flag** — whether this sample has been corrected by a recent VGGT update or is currently dead-reckoning on odometry only

(See `docs/streaming_poc.md` → *Wire formats* for the exact byte layout and Zenoh keys.)

### Estimator choice: FUSE vs. pure Python

The robot's "FUSE" stage is a **sensor-fusion state estimator**. Two families of options exist, and they have very different integration costs:

* **`fuse` (locusrobotics) / `robot_localization`** — These are the mature ROS sensor-fusion stacks. `fuse` is a graph-based, fixed-lag smoother built on Ceres; `robot_localization` is a classic EKF/UKF node. Both are **ROS-native C++ frameworks**: they run as ROS nodes, are configured through ROS parameters / YAML + launch files, consume and produce ROS messages, and are *not* usable as a standalone Python library. Adopting either means introducing a ROS node into the loop and running it inside the ROS environment (ROS Foxy / Python 3.8 on the Jetson).
* **Pure-Python EKF** — A self-contained Extended Kalman Filter written against NumPy (optionally using [`filterpy`](https://filterpy.readthedocs.io/)). It runs as a plain Python process — **no ROS node required**. It can live in the existing `robot/docker/` container next to `dynamic_bridge.py`, subscribe over Zenoh to the inputs it needs, and publish the fused pose over Zenoh.

**Decision for VAT:** the fusion is done in **pure Python, as a process inside `robot/docker/`, not as a ROS node.** This is possible because the data it needs is *already on the Zenoh bus*: `dynamic_bridge.py` bridges every ROS topic (including `SportModeState`, IMU, leg odometry) to Zenoh as CDR, and the cloud publishes the VGGT correction over Zenoh. So the fuser is "just another Zenoh client" — it subscribes to the bridged odometry and the cloud's pose correction, runs the EKF in NumPy, and publishes the authoritative pose. The bridge itself does no computation; it only exposes the inputs the fuser consumes.

This keeps the robot side free of an extra ROS node and respects the Python 3.8 constraint (NumPy/`filterpy` are fine on 3.8). If the placeholder EKF later proves insufficient — e.g. we need graph optimization, multi-rate landmark constraints, or proper loop closure — the migration path is to swap the Python process for a real `fuse`/`robot_localization` ROS node, which would then publish the same pose message onto the same Zenoh key. The rest of the system (cloud router, client predictor, wire format) is unaffected by that swap.

> **POC note:** For the streaming POC the fuser is a **placeholder**. It can pass the VGGT pose through and use a trivial constant-velocity/complementary blend with odometry, or even publish odometry-only with the global pose stapled on. The point of the POC is to lock in the *data flow and message contract* (`server pose → dog → server router → client`, full pose+velocity payload, client prediction) so the real EKF can drop in later without touching the surrounding system.

---

## Summary of the Data Flow

1. **Robot → Cloud:** streams RGBD frames, telemetry, and high-frequency odometry (leg odom + IMU).
2. **Cloud (mapping):** ingests frames, builds the PRISM global map, computes deltas, and produces the **low-frequency VGGT global pose**.
3. **Cloud → Robot:** sends the VGGT global pose *down* to the robot as a drift correction.
4. **Robot (fusion):** fuses the slow global correction with fast onboard odometry into a single **authoritative, high-rate global pose** carrying position, orientation, linear velocity, and angular velocity, and publishes it back up.
5. **Cloud (router) → Client:** the Zenoh router relays the robot's pose stream straight through to the client; in parallel the cloud streams PRISM map deltas + high-res video.
6. **Client (Python backend):** receives map deltas, the authoritative pose stream, and video streams.
7. **Client (Unity frontend):** renders the "Toy Box" room and the 1-frame RGBD mesh, **predicts the robot avatar's motion between pose updates** using the velocity/rotation vectors, drives the Beacon video stream, and handles XR inputs to send teleoperation commands back to the cloud/robot.
