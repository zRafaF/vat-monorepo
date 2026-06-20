# PRISM Streaming POC

End-to-end live point cloud from the **RICOH Theta X** on the Go2 robot → PRISM-VGGT mapping server → Rerun 3D viewer on any machine. (The camera was switched from the Insta360 — see [archive](archive/insta360.md).)

This POC also carries the **robot pose** end-to-end. Per the [system architecture](architecture.md#pose-state-estimation), the robot — not the server — is authoritative for its global pose. The data path is:

```
server (VGGT pose) ──► dog (state fusion) ──► server (Zenoh router) ──► client (prediction)
```

The server computes a slow, drift-free VGGT pose and sends it **down** to the dog; the dog fuses it with fast onboard odometry into an authoritative pose (with velocity + rotation), and sends it **up**; the server's Zenoh router relays it to the client, which dead-reckons between samples like a multiplayer game. For the POC the fuser is a **placeholder** (see [Pose & state estimation (POC)](#pose-state-estimation-poc)) — the goal is to lock in the data flow and message contract, not to ship a production EKF.

---

## Architecture

```
┌─────────────────────────── Jetson (on robot) ────────────────────────────┐
│                                                                            │
│  RICOH Theta X  (in-camera stitched equirectangular, H.264 over UVC)       │
│      │ USB                                                                 │
│  libuvc-theta gst_loopback  →  /dev/video0   (host)                        │
│      │                                                                     │
│  theta_camera.py (docker)  ← OpenCV UVC capture                            │
│      │ best-of-N-frame sharpest + camera_height stamp + JPEG               │
│      │   (throttle_fps + window_size live via Zenoh)                       │
│  {robot}/prism/camera/frame       (VAT frame: ts + seq + camera_height + JPEG)│
│      │                                                                     │
│  dynamic_bridge.py  (docker)  — ROS odometry (/sportmodestate) → Zenoh     │
│      │                                                                     │
└──────┼─────────────────────────────────────────────────────────────────────┘
       │  Zenoh  (go2/prism/camera/frame)
       ▼
┌─────────────────────────── Cloud / Dev Machine ──────────────────────────┐
│                                                                            │
│  mapping_server.py                                                         │
│    ├─ CDR decode  (rosbags, no ROS install)                                │
│    ├─ JPEG decode (OpenCV)                                                 │
│    ├─ FrameInput accumulator (sliding window)                              │
│    ├─ PRISM-VGGT engine                                                    │
│    │    ├─ PanoVGGT  (depth + poses per sliding window)                    │
│    │    ├─ NvbloxPanoTSDF  (equirect → cubemap → C++ TSDF)                 │
│    │    └─ BlockColorCache (versioned blocks, delta API)                   │
│    └─ Zenoh publishers                                                     │
│         ├─ server/prism/pcd_delta      (binary VAT format)                 │
│         ├─ server/prism/pcd_snapshot   (queryable on demand)               │
│         ├─ server/prism/trajectory                                         │
│         └─ server/prism/status        (JSON heartbeat)                     │
│                                                                            │
└──────────────────────────────────────────────────────────────────────────┘
       │  Zenoh
       ▼
┌─────────────────────────── Client (any machine) ─────────────────────────┐
│                                                                            │
│  prism_rerun_viewer.py                                                     │
│    ├─ Subscribe  server/prism/pcd_delta                                    │
│    ├─ Subscribe  server/prism/pcd_snapshot                                 │
│    ├─ Subscribe  server/prism/trajectory                                   │
│    ├─ Subscribe  go2/prism/pose          (authoritative robot pose)        │
│    ├─ LocalCloud (versioned block accumulator)                             │
│    ├─ PosePredictor (dead-reckon between samples)                          │
│    └─ rr.Points3D + rr.Transform3D → Rerun 3D viewer                       │
│                                                                            │
└──────────────────────────────────────────────────────────────────────────┘
```

### Pose data path (`server → dog → server router → client`)

The point cloud and the pose travel on separate paths. The cloud owns the *map*; the **robot owns its *pose***. The VGGT pose is sent down to the dog, fused with fast odometry, and the authoritative result is sent back up and relayed to the client.

```
┌──────────────────────────── Cloud / Dev Machine ──────────────────────────┐
│  mapping_server.py                                                         │
│    └─ from PRISM trajectory → latest global keyframe pose                  │
│         │  publish (low-freq, ~0.3–3 Hz)                                   │
│         ▼                                                                  │
│       server/prism/pose_correction   (sent DOWN to the dog)               │
│                                                                            │
│   zenohd router  ◄── go2/prism/pose ──  relays straight through to client │
└──────────┬─────────────────────────────────────────────▲──────────┬───────┘
           │ (DOWN: correction)              (UP: relayed)│          │ (UP)
           ▼                                              │          │
┌──── Jetson (robot, robot/docker/) ─────────────────────┼──────────┘
│  pose_fuser.py   (PLACEHOLDER, pure-Python, no ROS node)│
│    ├─ sub  server/prism/pose_correction  (slow, drift-free global pose)    │
│    ├─ sub  go2/rt/<odom|imu>  (via bridge — fast leg odom + IMU ~50–200 Hz)│
│    ├─ EKF / complementary fuse (NumPy / filterpy)                          │
│    └─ pub  go2/prism/pose   (authoritative: pos + quat + v + ω + ts) ──────┘
└────────────────────────────────────────────────────────────────────────────┘
                  │
                  ▼   (relayed by the cloud's zenohd router)
┌──── Client ────────────────────────────────────────────────────────────────┐
│  prism_rerun_viewer.py → PosePredictor: dead-reckons the avatar between     │
│  samples using the linear + angular velocity vectors (multiplayer netcode)  │
└────────────────────────────────────────────────────────────────────────────┘
```

The bridge (`dynamic_bridge.py`) does **no computation** — it merely exposes the ROS odometry/IMU topics on Zenoh as CDR. `pose_fuser.py` is "just another Zenoh client" that subscribes to those bridged inputs plus the cloud's correction, runs the fuse in NumPy, and publishes the authoritative pose. No extra ROS node is required. See [Pose & state estimation (POC)](#pose-state-estimation-poc).

---

## Repository layout after this refactor

```
vat-monorepo/
├── pyproject.toml              ← uv workspace root (virtual, no deps of its own)
│
├── common/
│   └── vat_protocol.py         ← shared wire formats (robot/server/client import it)
│
├── server/                     ← multiple microservices, each its own uv project
│   ├── mapping/                ← vat-mapping (ISOLATED env; heavy CUDA deps)
│   │   ├── pyproject.toml
│   │   ├── mapping_server.py   ← PRISM mapping + VGGT pose-correction publisher
│   │   └── PRISM-VGGT/         ← git submodule (git submodule add … — see below)
│   └── router/                 ← vat-router (ISOLATED env, excluded from workspace)
│       ├── pyproject.toml      ← only eclipse-zenoh — no clash with the mapper
│       └── router.py           ← pure-Python Zenoh router (no zenohd binary)
│
├── client/
│   ├── pyproject.toml          ← vat-client package (zenoh, rerun-sdk)
│   └── prism_rerun_viewer.py   ← Rerun viewer + pose predictor + robot block
│
├── robot/
│   ├── theta/
│   │   └── theta_uvc.sh        ← Theta X UVC → /dev/video0 (libuvc-theta loopback)
│   ├── docker/                 ← single container alongside the host ROS stack
│   │   ├── Dockerfile          ← build from REPO ROOT (-f robot/docker/Dockerfile)
│   │   ├── run.sh              ← build + docker run helper (no compose)
│   │   ├── start.sh            ← supervisor: restarts each process independently
│   │   ├── dynamic_bridge.py   ← ROS2 → Zenoh bridge (odometry)  [only ROS node]
│   │   ├── theta_camera.py     ← Theta UVC capture + best-of-window + camera-height
│   │   ├── pose_fuser.py       ← PLACEHOLDER authoritative-pose fuser
│   │   └── kinematics.py       ← camera↔base transform + camera-height + body state
│   └── systemd/
│       ├── vat-theta-uvc.service     ← host: Theta UVC → /dev/video0
│       └── vat-robot-docker.service  ← the Docker container (docker run, no compose)
│
└── docs/
    ├── streaming_poc.md        ← this file
    └── archive/insta360.md     ← retired Insta360 camera setup (historical)
```

> The Jetson has **no docker-compose** — there is a single `Dockerfile`, built
> from the repo root and run via `run.sh` / the systemd unit. The camera is
> captured directly over UVC (no ROS camera node); the bridge (odometry),
> `theta_camera`, and `pose_fuser` are the three container processes.

---

## One-time setup

### 1. Add the PRISM-VGGT submodule

!!! warning "Manual step — cannot be scripted"
    `.gitmodules` requires a manual `git` command. Run this once from the repo root:

```bash
git submodule add https://github.com/zRafaF/PRISM-VGGT server/mapping/PRISM-VGGT
git commit -m "chore: add PRISM-VGGT submodule"
```

After cloning on a new machine:
```bash
git submodule update --init --recursive
```

### 2. Install Python deps (server)

```bash
# Install uv if you don't have it
curl -Lsf https://astral.sh/uv/install.sh | sh

# Each service is its own isolated uv project — its .venv + uv.lock live in its
# own folder, so running several on one machine never rewrites a shared lock.
cd client        && uv sync && cd -    # client viewer + bring-up tools
cd server/mapping && uv sync && cd -   # mapping server (GPU machine)
cd server/router  && uv sync && cd -   # Zenoh router
```

**PyTorch + CUDA:** `prism-vggt` requires CUDA-capable torch. If `uv sync` pulls
the CPU-only wheel, uncomment the `[[tool.uv.index]]` block in
`server/pyproject.toml` and set your CUDA version (e.g. `cu121`):

```toml
[[tool.uv.index]]
name = "pytorch-cu121"
url  = "https://download.pytorch.org/whl/cu121"
explicit = true

[tool.uv.sources]
torch = { index = "pytorch-cu121" }
```

### 3. Install Python deps (client)

```bash
# On the visualisation machine (no GPU required) — own client/.venv
cd client && uv sync && cd -
```

### 4. Bring up the camera (RICOH Theta X over UVC)

No ROS camera node — the Theta X streams in-camera-stitched equirectangular over
UVC. Follow [robot setup](setup/robot.md) for the one-time `libuvc-theta` +
`v4l2loopback` install, then expose it as `/dev/video0`:

```bash
make theta-uvc      # = bash robot/theta/theta_uvc.sh  (leave running)
```

Sanity-check the camera alone (on the robot, no Zenoh):

```bash
make test_frames_robot      # = python3 tools/view_theta.py
```

### 5. Build & run the robot Docker container (Jetson — NO compose)

The bridge (odometry) + `theta_camera` + pose fuser run in **one** container.
The Jetson has no docker-compose, so build from the repo root and run with
`run.sh` (or the systemd unit). The Theta `/dev/video0` is passed in:

```bash
# from the repo root — point it at your server's IP (start theta-uvc first)
bash robot/docker/run.sh <SERVER_IP>
# logs:
docker logs -f vat-robot       # expect "Theta stream open. Streaming…"
```

### 6. Install the systemd services (Jetson)

```bash
# host ROS camera stack
sudo cp robot/systemd/vat-robot.service        /etc/systemd/system/
# docker container (edit ZENOH_CONNECT inside first)
sudo cp robot/systemd/vat-robot-docker.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vat-robot.service vat-robot-docker.service

# Verify
sudo journalctl -fu vat-robot          # camera stack
sudo journalctl -fu vat-robot-docker   # bridge + theta_camera + fuser
```

After this, **the full VAT stack starts automatically on every Jetson boot**,
no SSH required. The container's `start.sh` restarts any of the three processes
independently if one dies, so a transient fault doesn't take the stack down.

---

## Running the POC

### Start the Zenoh router microservice (cloud server)

A pure-Python router node in its own isolated env — **no `zenohd` binary, no
Docker**.  It opens a `router`-mode session that listens and relays traffic for
every robot/server/client (including the `dog → router → client` pose relay):

```bash
cd server/router && uv sync && uv run python router.py     # or: make router
# listens on tcp/0.0.0.0:7447 — override with ZENOH_LISTEN, mesh with ZENOH_CONNECT
```

### Start the mapping server (cloud / dev machine)

```bash
# from the repo root (config comes from vat.env)
make mapping
# = cd server/mapping && uv run python mapping_server.py
```

The mapping server reads `ZENOH_ROUTER` from `vat.env` (the router's VPN IP),
so it connects to the router even from a different datacenter.

Key env vars for the server:

| Variable | Default | Description |
|---|---|---|
| `ZENOH_ROUTER` | `tcp/127.0.0.1:7447` | Zenoh router endpoint |
| `ROBOT_NAME` | `go2` | Zenoh key prefix |
| `WEIGHTS_PATH` | `server/mapping/PRISM-VGGT/checkpoints/model.pt` | PanoVGGT model weights |
| `CAMERA_HEIGHT` | `0.50` | Fixed camera height (m) for POC metric scale |
| `WINDOW_SIZE` | `10` | Frames per PRISM sub-window |
| `OVERLAP` | `3` | Overlapping frames between windows |
| `VOXEL_SIZE` | `0.02` | TSDF voxel size (m) |
| `MAX_DEPTH` | `4.5` | Maximum depth for TSDF integration (m) |
| `TARGET_WIDTH` | `1036` | Canonical image width fed to PRISM |
| `TARGET_HEIGHT` | `518` | Canonical image height fed to PRISM |

### Start the Rerun viewer (any machine)

```bash
# from the repo root (config from vat.env) — runs in client/.venv
make viewer
# or directly:  cd client && uv run python prism_rerun_viewer.py --snapshot
```

### Tune the frame rate live (from any machine)

```python
import zenoh
z = zenoh.open(zenoh.Config())
z.put("go2/rt/prism/config/throttle_fps", b"5.0")
```

Or with the zenoh CLI:
```bash
zenoh put --key go2/rt/prism/config/throttle_fps --payload 5.0
```

---

## Wire formats

### Point cloud  (`server/prism/pcd_delta`, `server/prism/pcd_snapshot`)

All values big-endian.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | `int32` | Magic = `0x50434400` (`"PCD\x00"`) |
| 4 | 4 | `int32` | `version` — monotonic TSDF map version |
| 8 | 4 | `int32` | `n_points` |
| 12 | 4 | `int32` | `is_snapshot` — 1 = full cloud, 0 = delta |
| 16 | 4 | `int32` | `since_version` — delta base version (0 if snapshot) |
| 20 | n×12 | `float32[n,3]` | XYZ positions |
| 20+n×12 | n×12 | `float32[n,3]` | RGB colours in [0, 1] |

### Trajectory  (`server/prism/trajectory`)

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | `int32` | `n` — number of poses |
| 4 | n×12 | `float32[n,3]` | Camera positions (XYZ) |

### Authoritative robot pose  (`go2/prism/pose`)

Published **by the robot** (`pose_fuser.py`), relayed by the cloud router to the
client. High-rate (target 50–200 Hz; placeholder runs at the correction rate).
Carries the full state the client needs to dead-reckon between samples. All
values big-endian.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | `int32` | Magic = `0x504F5345` (`"POSE"`) |
| 4 | 8 | `int64` | `timestamp_ns` — capture time of this state estimate |
| 12 | 4 | `uint32` | `seq` — monotonic sequence number |
| 16 | 12 | `float32[3]` | position XYZ (m, global map frame) |
| 28 | 16 | `float32[4]` | orientation quaternion (x, y, z, w), map frame |
| 44 | 12 | `float32[3]` | linear velocity XYZ (m/s, map frame) |
| 56 | 12 | `float32[3]` | angular velocity XYZ (rad/s, body frame) |
| 68 | 4 | `int32` | `fix_quality` — 0 = dead-reckon (odom only), 1 = VGGT-corrected |

Total: **72 bytes**, fixed size. `linear`/`angular velocity` are what the client
extrapolates with: `p(t) = p₀ + v·Δt`, `q(t) = q₀ ⊗ Δq(ω, Δt)`.

### VGGT pose correction  (`server/prism/pose_correction`)

Published **by the server**, sent **down** to the robot's `pose_fuser.py`. This
is the latest drift-free **camera** pose in the map frame, derived from the
PRISM trajectory — *not* the base pose. The robot converts it to a base pose
with its own kinematics (`T_world_base = T_world_camera ∘ inverse(T_base_camera)`),
keeping the server kinematics-agnostic. Slow (~0.3–3 Hz, once per submap) and
laggy (~2–4 s); no velocity — it is an anchor, not a motion source. Big-endian.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | `int32` | Magic = `0x50434F52` (`"PCOR"`) |
| 4 | 8 | `int64` | `timestamp_ns` — capture time of the keyframe this pose anchors |
| 12 | 4 | `int32` | `map_version` — PRISM map version this pose belongs to |
| 16 | 12 | `float32[3]` | **camera** position XYZ (m, map frame) |
| 28 | 16 | `float32[4]` | **camera** orientation quaternion (x, y, z, w), map frame |

Total: **44 bytes**, fixed size.

> The current server derives the camera orientation from the trajectory tangent
> (heading) as a placeholder; the robot fuser re-anchors orientation anyway.
> Replace with the true VGGT per-keyframe extrinsics when the engine exposes them.

### Camera frame  (`{robot}/prism/camera/frame`)

Published **by the robot** decimator → server. Big-endian header + JPEG body.
Carries the camera height at capture time (computed on the robot, see below).

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | `int32` | Magic = `0x46524D45` (`"FRME"`) |
| 4 | 8 | `int64` | `timestamp_ns` — capture time |
| 12 | 4 | `uint32` | `seq` — monotonic; lets the server detect & re-request drops |
| 16 | 4 | `float32` | `camera_height` (m above floor; **< 0 = unknown**, server falls back) |
| 20 | … | bytes | JPEG image |

Frames are published **reliably** (`RELIABLE` + `BLOCK`) so they aren't silently
dropped. The decimator keeps a ring buffer of recent frames and exposes a
queryable (`{robot}/prism/camera/frame/get?seq=N`); the mapping server detects a
`seq` gap and **re-requests** the missing frame before processing a window. It
also batches a window on **N new frames OR a timeout, whichever comes first**
(`WINDOW_TIMEOUT_S`, default 2 s) so a sparse stream never stalls the viewer.

All wire formats live in one place: **`common/vat_protocol.py`** (imported by
robot, server and client) so they can never drift apart.

### Live config  (`{robot}/rt/prism/config/throttle_fps`, `.../window_size`)

Plain UTF-8 string payloads (e.g. `"3.0"`, `"5"`). The `theta_camera.py`
process subscribes and applies the new value immediately.

---

## Robot kinematics: camera ↔ base, and camera height

PRISM-VGGT estimates the pose of the **camera**, but the camera is on a
selfie-stick on the back of the **Go2-W** (wheeled), and may later move to an
actuated arm. Two geometry problems follow, both solved on the **robot** (it has
the joint/body state) in `robot/docker/kinematics.py`:

**1. Camera → base.** The stick is rigid to the body, so `T_base_camera` is a
fixed transform — but *not* identity: when the body rolls/pitches (even with the
wheels planted) the stick swings the camera sideways/forward. To recover the
base pose we subtract it:

```
T_world_base = T_world_camera ∘ inverse(T_base_camera)
```

The `RobotModel` interface hides whether this is a fixed stick
(`SelfieStickModel`, used now — configure `STICK_OFFSET_X/Y/Z`) or, in the
future, forward kinematics of an arm from a URDF (`URDFArmModel`, placeholder;
set `ROBOT_MODEL=urdf` + `ROBOT_URDF=…`). For the Go2-W the *wheels* are
continuous joints — they don't change `T_base_camera`, only the body height.

**2. Camera height above the floor** (for PRISM metric scale). The Go2-W can lie
down / stand up, so this is **not** constant:

```
camera_height = body_height + (R_body · stick_offset).z
```

`body_height` comes from `SportModeState.body_height` (live, via the bridge),
`R_body` from the IMU quaternion (so body tilt is accounted for). The robot
computes this at frame-capture time and stamps it into every
`{robot}/prism/camera/frame` message; the server reads it straight from the
frame (falling back to its `CAMERA_HEIGHT` env var only if the value is `< 0`).

`RobotStateTracker` decodes `SportModeState` directly from the bridged CDR using
`rosbags` (no ROS install) via embedded `unitree_go` message definitions. If the
custom message can't be decoded (firmware layout differs), it falls back to a
configured constant and never crashes — verify the layout against your firmware.

---

## Pose & state estimation (POC)

Per the [system architecture](architecture.md#pose-state-estimation), **the robot
owns its global pose** — the server only produces a slow correction and routes
the result. This section describes how that is realised (and faked) in the POC.

### Roles

| Component | Where | Role |
|---|---|---|
| `server/mapping/mapping_server.py` | cloud | Derives the latest **camera** pose from the PRISM trajectory and publishes it on `server/prism/pose_correction` (DOWN to the dog). |
| `dynamic_bridge.py` | robot (docker) | Bridges ROS odometry/IMU (`SportModeState`, etc.) to Zenoh as CDR. **No computation.** |
| `pose_fuser.py` | robot (docker) | **NEW, placeholder.** Subscribes to the correction + bridged odometry, fuses them, publishes the authoritative pose on `go2/prism/pose` (UP). |
| `server/router/router.py` | cloud | Pure-Python Zenoh router; relays `go2/prism/pose` straight through to the client. |
| `prism_rerun_viewer.py` | client | Subscribes to `go2/prism/pose` and dead-reckons the avatar between samples. |

### Why the fuser is pure Python in `robot/docker/`, not a ROS node

`fuse` (locusrobotics) and `robot_localization` are the mature ROS sensor-fusion
stacks — but both are **ROS-native C++ frameworks**: they run as ROS nodes,
are configured through ROS params/YAML + launch files, and are *not* usable as a
standalone Python library. `fuse` in particular is a graph-based fixed-lag
smoother on top of Ceres.

We do **not** want another ROS node in the loop, and we are constrained to
Python 3.8 inside ROS Foxy. The inputs the fuser needs are already on the Zenoh
bus: `dynamic_bridge.py` exposes the ROS odometry/IMU as CDR, and the cloud
publishes the VGGT correction over Zenoh. So the fuser is **just another Zenoh
client** — a plain Python process in `robot/docker/` that subscribes to both,
runs the filter in NumPy (optionally [`filterpy`](https://filterpy.readthedocs.io/),
which is pure-Python and fine on 3.8), and publishes `go2/prism/pose`.

> **Migration path:** if the placeholder EKF proves insufficient (graph
> optimisation, landmark constraints, loop closure), swap the Python process for
> a real `fuse`/`robot_localization` ROS node that publishes the *same*
> `go2/prism/pose` message onto the *same* Zenoh key. The cloud router, the wire
> format, and the client predictor are all unaffected by that swap — which is
> the entire point of locking the contract now.

### Placeholder `pose_fuser.py` behaviour (POC)

The POC fuser is intentionally trivial — it exists to prove the data path, not
to estimate well:

1. Hold the last `pose_correction` (drift-free global anchor) from the server.
2. On each high-rate odometry tick, integrate the odometry delta on top of that
   anchor (a constant-velocity / complementary blend — *not* a real EKF).
3. Fill `linear`/`angular velocity` straight from the odometry twist.
4. Set `fix_quality = 1` for a short window after each correction, `0` once it is
   dead-reckoning on odometry alone.
5. Publish `go2/prism/pose` at the odometry rate.

A real EKF replaces only steps 2–3; the contract (inputs, outputs, key, wire
format) stays identical.

### Client-side prediction (POC)

`prism_rerun_viewer.py` adds a `PosePredictor`:

- Keep a small ring buffer of recent poses.
- Each render tick, extrapolate from the newest sample:
  `p(t) = p₀ + v·Δt`, `q(t) = q₀ ⊗ Δq(ω, Δt)`.
- On a fresh pose, blend toward it over a few frames (slerp for rotation) instead
  of snapping.
- If no pose arrives within a staleness horizon, decay velocity toward zero so a
  disconnected robot coasts to a stop.

In Rerun this is logged as an `rr.Transform3D` on the robot-avatar entity, updated
every render tick from the predictor rather than only when a pose arrives.

> **POC honesty:** the placeholder fuser will produce a pose that mostly tracks
> raw odometry (so it will drift), with periodic jumps when a VGGT correction
> lands. That is expected and acceptable for the POC — the deliverable is the
> *data flow and message contract*, not localisation accuracy.

---

## Timing analysis

Running at **3 Hz** with **10-frame windows** and **3 frames overlap**:

| Stage | Duration |
|---|---|
| Window accumulation (10 frames @ 3 Hz) | 3.3 s |
| PanoVGGT inference (parallel A/B mode) | ~1.6 s |
| TSDF integration | ~0.1 s |
| BlockColorCache delta generation | ~0.05 s |
| **Total server latency** | **~1.75 s** |

With 3.3 s of footage per sub-window and ~1.75 s processing time, there is
~1.5 s of headroom. The parallel A/B pipeline means one window is being
processed while the next is accumulating, so the effective latency experienced
by the viewer is ~3–4 s end-to-end. This is acceptable for a mapping POC.

---

## Known limitations (POC)

### Online streaming mode

`StreamingWindowEngine.process_sequence()` calls `self.reset()` at the start of
every invocation — it was designed for offline batch mode. The server works
around this by replaying the full accumulated frame list on every new sub-window.
This is O(N) in total frames processed, which becomes slow for long sessions.

**Fix (TODO):** extend `StreamingWindowEngine` with an `add_frame()` / `step()`
API that maintains internal state across calls without resetting. The
`NvbloxPanoTSDF` mapper already supports incremental integration; only the
sliding-window bookkeeping in `process_sequence()` needs restructuring.

### Delta streaming is approximate

The current server publishes per-submap deltas by calling
`engine.get_point_cloud_delta(version - 1)` — this gives only blocks that
changed in the last submap. Because the engine resets on each `process_sequence`
call, `version` restarts from 0 each time, so the client may receive duplicate
points. The `LocalCloud` accumulator on the client side merges by version key,
which limits visual artefacts.

This is resolved once the true online engine mode is implemented.

### No navigation or ESDF

`compute_esdf = False` in the server to save ~80 ms per submap. ESDF
(Euclidean Signed Distance Field) is needed for collision-aware path planning
(the full VAT navigation stack). It is not needed for the streaming POC.

---

## Task checklist

### POC milestone (this branch)

- [x] uv workspace root + `server/` + `client/` packages
- [x] `common/vat_protocol.py` — shared wire formats (robot/server/client)
- [x] PRISM-VGGT as git submodule (`server/mapping/PRISM-VGGT`)  — **manual** `git submodule add`
- [x] `server/mapping/mapping_server.py` — frame subscriber → PRISM engine → pcd + pose-correction publisher
- [x] `server/router/router.py` — standalone pure-Python Zenoh router (isolated uv env)
- [x] `client/prism_rerun_viewer.py` — pcd + pose subscriber → Rerun, with `PosePredictor` + robot block
- [x] `robot/docker/theta_camera.py` — Theta UVC capture + best-of-window + camera-height stamp
- [x] `robot/docker/kinematics.py` — camera↔base transform + camera height + body-state tracker
- [x] `robot/docker/pose_fuser.py` — PLACEHOLDER authoritative-pose fuser
- [x] `robot/docker/` single Dockerfile + `run.sh` + supervised `start.sh` (no compose)
- [x] `robot/systemd/{vat-robot,vat-robot-docker}.service` — auto-start on boot
- [ ] Smoke test: `journalctl -fu vat-robot-docker` clean, viewer renders cloud + robot block
- [ ] Tune `WINDOW_SIZE` (sharpness window) and `OVERLAP` for latency vs. quality
- [ ] Measure/set the real selfie-stick geometry (`STICK_OFFSET_X/Y/Z`)
- [ ] Verify the `unitree_go` SportModeState layout matches your firmware

#### Pose path (POC — locks the contract, fusion is a placeholder)

- [x] `mapping_server.py` — publish VGGT **camera** pose on `server/prism/pose_correction` (DOWN)
- [x] `pose_fuser.py` — convert camera→base via kinematics, fuse, publish `go2/prism/pose` (UP)
- [x] `prism_rerun_viewer.py` — subscribe pose, `PosePredictor` (dead-reckon + slerp + staleness decay)
- [ ] Verify the `server → dog → server router → client` path round-trips on hardware

### Phase 2 (after POC)

- [ ] Replace trajectory-tangent heading with true VGGT camera extrinsics in the correction
- [ ] Extend `StreamingWindowEngine` with `add_frame()` / `step()` for true online mode
- [ ] Proper per-block delta streaming (requires stable `version` across engine calls)
- [ ] **Real state estimator** — replace placeholder `pose_fuser.py` with a proper EKF (NumPy/`filterpy`) or migrate to a `fuse`/`robot_localization` ROS node publishing the same `go2/prism/pose` contract
- [ ] Wire real high-rate inputs into the fuser (leg odometry + IMU from the bridge); tune correction gating / outlier rejection
- [ ] Unity VR client as git submodule in `client/unity/` (port `PosePredictor` to the Unity avatar)
- [ ] ESDF-based global pathfinding (enable `compute_esdf = True`, wire to navigation)
- [ ] Loop closure feeding back into the VGGT correction
- [ ] Dog 3D reconstruction from protobuf joint states

---

## Troubleshooting

**Viewer shows no points after server starts**  
→ Check `journalctl -fu vat-robot-docker` on the Jetson — frames may not be reaching the bridge.  
→ `zenoh sub --key 'go2/prism/camera/frame'` from any machine to verify frames are flowing.

**PRISM server crashes with CUDA OOM**  
→ Lower `WINDOW_SIZE` (try 6) or `FACE_SIZE` (try 384).

**`prism-vggt` import error on server**  
→ Run `git submodule update --init server/mapping/PRISM-VGGT` then `cd server/mapping && uv sync`.

**Robot block doesn't move / no pose in viewer**  
→ `zenoh sub --key 'go2/prism/pose'` to confirm `pose_fuser.py` publishes.  
→ `zenoh sub --key 'server/prism/pose_correction'` to confirm the server sends corrections.  
→ `zenoh sub --key 'go2/rt/sportmodestate'` to confirm odometry reaches the fuser.

**Camera height looks wrong (map scale off)**  
→ Set the real stick geometry via `STICK_OFFSET_X/Y/Z`; check the `SportModeState`
   decode isn't silently falling back (look for the one-time decode-failure warning).

**No camera frames (`camera/frame` 0 Hz)**  
→ Preview the Theta on the robot first: `make test_frames_robot`
   (`tools/view_theta.py`). If that's blank, the UVC source is down — start
   `make theta-uvc` and confirm `ls -l /dev/video0`.  
→ If the robot preview works but the client sees 0 Hz, the container didn't get
   the device — start `make theta-uvc` **before** `make robot-docker` so `--device
   /dev/video0` is attached; check `docker logs vat-robot` for `theta_camera`.

**`ros2 topic list` is empty / `package not found` on the robot**  
→ Export the CycloneDDS fix first (the Go2 points at the wrong interface):
   `export CYCLONEDDS_URI='<CycloneDDS>…<NetworkInterface name="eth0"/>…</CycloneDDS>'`
   (the container handles this itself). This only affects the odometry bridge now.

**Throttle Zenoh update has no effect**  
→ Confirm `theta_camera`'s `ZENOH_CONNECT` matches your router, and that frames
   are flowing (`make test_frames_server`).

**Robot avatar doesn't move (no pose in viewer)**  
→ `zenoh sub --key 'go2/prism/pose'` to confirm `pose_fuser.py` is publishing.  
→ If silent, check `pose_fuser.py` is receiving odometry: `zenoh sub --key 'go2/rt/sport_mode_state'`.  
→ Confirm the server is sending corrections: `zenoh sub --key 'server/prism/pose_correction'`.

**Avatar jitters or jumps periodically**  
→ Expected with the placeholder fuser: it tracks raw odometry and snaps when a VGGT correction lands. The client `PosePredictor` smooths this; widen its blend window if jumps are visible. A real EKF removes the jumps.
