# PRISM Streaming POC

End-to-end live point cloud from the Insta360 on the Go2 robot → PRISM-VGGT mapping server → Rerun 3D viewer on any machine.

---

## Architecture

```
┌─────────────────────────── Jetson (on robot) ────────────────────────────┐
│                                                                            │
│  Insta360 SDK (C++)                                                        │
│      │ H.264 (USB)                                                         │
│      ▼                                                                     │
│  /dual_fisheye/image/compressed   (sensor_msgs/CompressedImage)            │
│      │                                                                     │
│  decoder node  (insta360_ros_driver)                                       │
│      │                                                                     │
│  /dual_fisheye/image              (sensor_msgs/Image, BGR8)                │
│      │                                                                     │
│  equirectangular node  (GPU PyTorch, 1920×960)                             │
│      │                                                                     │
│  /equirectangular/image           (sensor_msgs/Image, RGB8)                │
│      │                                                                     │
│  frame_publisher node  ← throttle_fps (default 3 Hz, tunable via Zenoh)   │
│      │ JPEG-compressed                                                      │
│  /prism/camera/frame              (sensor_msgs/CompressedImage)            │
│      │                                                                     │
│  DynamicZenohBridge  (demand-driven, CDR-serialized)                       │
│      │                                                                     │
└──────┼─────────────────────────────────────────────────────────────────────┘
       │  Zenoh  (go2/rt/prism/camera/frame)
       ▼
┌─────────────────────────── Cloud / Dev Machine ──────────────────────────┐
│                                                                            │
│  prism_server.py                                                           │
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
│    ├─ LocalCloud (versioned block accumulator)                             │
│    └─ rr.Points3D → Rerun 3D viewer                                        │
│                                                                            │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Repository layout after this refactor

```
vat-monorepo/
├── pyproject.toml              ← uv workspace root (virtual, no deps of its own)
│
├── server/
│   ├── pyproject.toml          ← vat-server package (zenoh, rosbags, prism-vggt)
│   ├── prism_server.py         ← PRISM streaming server  ← NEW
│   └── PRISM-VGGT/             ← git submodule  (git submodule add ... — see below)
│
├── client/
│   ├── pyproject.toml          ← vat-client package (zenoh, rerun-sdk)
│   └── prism_rerun_viewer.py   ← Rerun point cloud viewer  ← NEW
│
├── robot/
│   ├── insta360_ros_driver/    ← DO NOT MODIFY (sensitive hardware driver)
│   ├── bridge_node/            ← DO NOT MODIFY (DynamicZenohBridge)
│   ├── frame_publisher/        ← NEW ROS2 package (throttle + JPEG compress)
│   │   ├── frame_publisher/node.py
│   │   ├── launch/bringup.launch.xml
│   │   ├── package.xml
│   │   └── setup.py
│   ├── vat_bringup/            ← NEW master launch package
│   │   ├── launch/vat_bringup.launch.xml
│   │   ├── package.xml
│   │   └── setup.py
│   └── systemd/
│       └── vat-robot.service   ← NEW systemd unit for auto-bringup
│
└── docs/
    └── streaming_poc.md        ← this file
```

---

## One-time setup

### 1. Add the PRISM-VGGT submodule

!!! warning "Manual step — cannot be scripted"
    `.gitmodules` requires a manual `git` command. Run this once from the repo root:

```bash
git submodule add https://github.com/zRafaF/PRISM-VGGT server/PRISM-VGGT
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

# From the repo root — resolves the full workspace lockfile
uv sync

# Server only (on the GPU machine)
uv sync --package vat-server

# Activate venv
source .venv/bin/activate
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
# On the visualisation machine (no GPU required)
uv sync --package vat-client
```

### 4. Build robot ROS2 packages (Jetson)

```bash
# Create or update the ROS2 workspace
mkdir -p ~/vat_ws/src
# Symlink the robot packages (or copy them)
ln -sfn $(pwd)/robot/frame_publisher  ~/vat_ws/src/frame_publisher
ln -sfn $(pwd)/robot/vat_bringup      ~/vat_ws/src/vat_bringup
# insta360_ros_driver should already be there from the original setup

cd ~/vat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 5. Install the systemd service (Jetson)

```bash
# Edit the service file first — set ZENOH_ROUTER to your server's IP
nano robot/systemd/vat-robot.service

sudo cp robot/systemd/vat-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable vat-robot.service

# Start immediately (without rebooting)
sudo systemctl start vat-robot.service

# Verify
sudo systemctl status vat-robot
sudo journalctl -fu vat-robot   # follow live logs
```

After this, **the full VAT stack starts automatically on every Jetson boot**,
no SSH required.

---

## Running the POC

### Start the Zenoh router (cloud server)

```bash
# Install zenoh-router if not present
cargo install zenohd          # or: pip install eclipse-zenoh[router]

zenohd -l tcp/0.0.0.0:7447
```

### Start the PRISM server (cloud / dev machine)

```bash
source .venv/bin/activate
ZENOH_ROUTER=tcp/127.0.0.1:7447 \
ROBOT_NAME=go2 \
CAMERA_HEIGHT=0.50 \
python server/prism_server.py
```

Key env vars for the server:

| Variable | Default | Description |
|---|---|---|
| `ZENOH_ROUTER` | `tcp/127.0.0.1:7447` | Zenoh router endpoint |
| `ROBOT_NAME` | `go2` | Zenoh key prefix |
| `WEIGHTS_PATH` | `server/PRISM-VGGT/checkpoints/model.pt` | PanoVGGT model weights |
| `CAMERA_HEIGHT` | `0.50` | Fixed camera height (m) for POC metric scale |
| `WINDOW_SIZE` | `10` | Frames per PRISM sub-window |
| `OVERLAP` | `3` | Overlapping frames between windows |
| `VOXEL_SIZE` | `0.02` | TSDF voxel size (m) |
| `MAX_DEPTH` | `4.5` | Maximum depth for TSDF integration (m) |
| `TARGET_WIDTH` | `1036` | Canonical image width fed to PRISM |
| `TARGET_HEIGHT` | `518` | Canonical image height fed to PRISM |

### Start the Rerun viewer (any machine)

```bash
source .venv/bin/activate
ZENOH_ROUTER=tcp/<server-ip>:7447 python client/prism_rerun_viewer.py

# Request the full current snapshot immediately on startup
python client/prism_rerun_viewer.py --snapshot
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

### Live throttle config  (`go2/rt/prism/config/throttle_fps`)

Plain UTF-8 float string, e.g. `"3.0"`. The `frame_publisher` node subscribes
to this key and applies the new rate immediately.

---

## Camera height strategy

For metric scale estimation, PRISM needs to know the camera's height above the floor.

**POC (now):** fixed constant `CAMERA_HEIGHT` env var on the server, default 0.50 m.

**Phase 2 (planned):** dynamic height from the dog's body odometry.  
The decision to do this on the **server** (not the robot node) is deliberate:

- The robot already streams `SportModeState_.body_height` via the Zenoh bridge
  (it's captured in `Go2StreamFrame` protobuf, which goes to the server for 3D
  dog reconstruction anyway).
- The server computes `camera_height = body_height + CAMERA_MOUNT_OFFSET`.
- `CAMERA_MOUNT_OFFSET` is a fixed physical constant (measure from CAD or
  physically). Default in `prism_server.py`: 0.18 m.
- Keeps the robot node simple — it does not need limb kinematics.

When you wire this up, the `_on_sport_state()` callback in `prism_server.py`
already handles CDR decoding of `SportModeState` and updates `_camera_height`
automatically. Just ensure the bridge is forwarding `/{robot_name}/sport_mode_state`.

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
- [x] PRISM-VGGT as git submodule (`server/PRISM-VGGT`)  — **manual** `git submodule add`
- [x] `server/prism_server.py` — Zenoh frame subscriber → PRISM engine → pcd publisher
- [x] `client/prism_rerun_viewer.py` — pcd delta subscriber → Rerun 3D viewer
- [x] `robot/frame_publisher/` — throttle + JPEG-compress node with live Zenoh config
- [x] `robot/vat_bringup/` — master launch file composing the full robot stack
- [x] `robot/systemd/vat-robot.service` — auto-start on Jetson boot
- [ ] Smoke test: `journalctl -fu vat-robot` shows no errors, viewer renders cloud
- [ ] Tune `WINDOW_SIZE` and `OVERLAP` for latency vs. map quality tradeoff
- [ ] Verify JPEG quality 85 is sufficient for PRISM depth quality

### Phase 2 (after POC)

- [ ] Dynamic camera height from `SportModeState_.body_height` + mount offset
- [ ] Extend `StreamingWindowEngine` with `add_frame()` / `step()` for true online mode
- [ ] Proper per-block delta streaming (requires stable `version` across engine calls)
- [ ] Unity VR client as git submodule in `client/unity/`
- [ ] ESDF-based global pathfinding (enable `compute_esdf = True`, wire to navigation)
- [ ] Odometry tracking and loop closure
- [ ] Dog 3D reconstruction from protobuf joint states

---

## Troubleshooting

**Viewer shows no points after server starts**  
→ Check `journalctl -fu vat-robot` on the Jetson — frames may not be reaching the bridge.  
→ `zenoh sub --key 'go2/rt/prism/camera/frame'` from any machine to verify frames are flowing.

**PRISM server crashes with CUDA OOM**  
→ Lower `WINDOW_SIZE` (try 6) or `FACE_SIZE` (try 384).

**`prism-vggt` import error on server**  
→ Run `git submodule update --init server/PRISM-VGGT` then `uv sync --package vat-server`.

**`frame_publisher` not found in `ros2 launch`**  
→ Run `colcon build --symlink-install` in the ROS2 workspace and `source install/setup.bash`.

**Throttle Zenoh update has no effect**  
→ Confirm the `frame_publisher` node's `zenoh_router` param matches your router.  
→ Check the node log: `ros2 node log /frame_publisher`.
