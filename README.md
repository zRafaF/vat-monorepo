# VAT — Volumetric Asynchronous Teleoperation

Live **360° point-cloud + robot pose** from a **Unitree Go2-W** carrying a
**RICOH Theta X** camera, streamed over [Zenoh](https://zenoh.io) to a
**PRISM-VGGT** mapping server (GPU), and rendered in a **Rerun** 3D viewer on any
machine — across the public internet / VPN.

The robot sends a light, real-time stream for online mapping and keeps a
full-resolution local archive for heavy offline reconstruction (Gaussian splats,
NeRF) later. Pose is authoritative on the robot: the cloud computes a slow,
drift-free VGGT pose, the dog fuses it with fast onboard odometry, and the client
dead-reckons between samples like multiplayer netcode.

> Status: pre-POC bring-up. Walk the staged tests (`make steps`) until each is
> green. See `docs/` for the full design and setup.

---

## Data path

```
RICOH Theta X ──UVC──▶ theta_camera (robot, docker)
                         ├─ downscale → {robot}/prism/camera/frame  (1036×518 JPEG → server)
                         └─ full-res 4K twin → local SQLite archive (10 GB rolling, fetch by seq)

dynamic_bridge (robot) ── ROS odometry (/sportmodestate) ──▶ Zenoh
mapping_server (server, GPU) ── PanoVGGT depth+pose + Nvblox TSDF ──▶ point-cloud deltas + trajectory
pose path:  server VGGT pose ─▶ dog fuses w/ odometry ─▶ authoritative pose ─▶ client predicts
prism_rerun_viewer (client) ── point cloud + predicted robot block in Rerun 3D
```

Three machines:

| Role | Machine | Runs |
|---|---|---|
| 🤖 **ROBOT** | Jetson on the Go2-W | Theta UVC feed + Docker container (bridge, camera, pose fuser) |
| ☁️ **SERVER** | GPU box | Zenoh router + PRISM-VGGT mapping server |
| 💻 **CLIENT** | your laptop | diagnostic tools + Rerun viewer |

---

## Repository layout

```
vat-monorepo/
├── vat.env                 ← single source of public config (router IP, robot name, tuning)
├── makefile                ← the control file: `make help`, `make steps`
├── common/
│   └── vat_protocol.py     ← shared Zenoh key schema + wire formats (pack/unpack)
├── server/
│   ├── router/             ← Zenoh router microservice (isolated uv env)
│   └── mapping/            ← PRISM-VGGT mapping server + PRISM-VGGT/ submodule (CUDA)
├── client/                 ← Rerun viewer + bring-up tools env
├── robot/
│   ├── theta/theta_uvc.sh  ← Theta X UVC → /dev/video10 (gstthetauvc loopback)
│   ├── docker/             ← bridge + theta_camera + frame_archive + pose_fuser + Dockerfile
│   ├── unitree_go_msgs/    ← minimal unitree_go interfaces built into the image
│   └── systemd/            ← auto-start units
├── tools/                  ← view_frames / view_robot_state / view_poses / theta_pub / fetch_archive …
└── docs/                   ← mkdocs site (architecture, setup, bring-up runbook)
```

Each runtime piece is its **own isolated [uv](https://docs.astral.sh/uv/)
project** (`server/router`, `server/mapping`, `client`, `robot`) with its own
`.venv` — so heavy CUDA deps never clash with the light router/robot envs.

---

## Quick start

Everything is driven by the **Makefile** + **`vat.env`**. Edit `vat.env` once
(set `ROUTER_IP` to the router's reachable address), then per machine:

```bash
# ☁️ SERVER
make router            # the Zenoh hub; leave running
make mapping           # PRISM-VGGT mapping server (needs GPU)

# 🤖 ROBOT  (one-time camera setup: see docs/setup/robot.md)
make theta-uvc         # Theta X → /dev/video10 (leave running)
make robot-docker      # bridge + theta_camera + pose_fuser container

# 💻 CLIENT
make test_link         # transport alive?  bridge + rates
make viewer            # full POC: point cloud + predicted robot block
```

Walk it up in stages — `make steps` prints the ordered runbook, and
[`docs/bringup.md`](docs/bringup.md) explains each check:

| Stage | Command | What it proves |
|---|---|---|
| 0 | `make test_link` | router + bridge alive, non-zero rates |
| 1 | `make test_frames_server` | live 360° frames over Zenoh (decimated) |
| 1 | `make theta-stream` (robot) | headless camera → Zenoh, no display needed |
| 2 | `make test_robot_state` | body + leg lines + selfie-stick + live camera |
| 3 | `make test_poses` | camera trajectory + fused robot pose |
| 4 | `make viewer` | the full POC |
| — | `make fetch_frame SEQ=N` | pull one full-res archived frame by seq |

`make help` lists every target.

---

## The camera: real-time + full-res archive

The Theta X isn't a plain webcam — its H.264 UVC stream is decoded on the host by
[`gstthetauvc`](https://github.com/nickel110/gstthetauvc) into a v4l2 loopback
(`/dev/video10`), which `theta_camera.py` reads. For each transmitted frame it:

- **downscales** to `TRANSMIT_WIDTH×TRANSMIT_HEIGHT` (default `1036×518`) and
  publishes that to the mapping server — light and low-latency; and
- archives the **full-res 4K twin** locally (SQLite index + JPEG files, rolling
  `ARCHIVE_MAX_BYTES`, default `10 GB`), tagged with the **same seq / timestamp /
  camera-height** — 1:1 with the live frame.

The server (or you) can fetch any full-res frame on demand by seq via a Zenoh
queryable (`make fetch_frame SEQ=N`). See [`docs/setup/robot.md`](docs/setup/robot.md).

---

## Configuration (`vat.env`)

Key knobs (all documented inline):

- `ROUTER_IP` / `ROUTER_PORT` — the Zenoh hub everything dials.
- `ROBOT_NAME` — key-schema prefix (default `go2`).
- `NET_IFACE` — DDS interface for the bridge; empty = auto-detect (prefers the
  Go2's `192.168.123.x` subnet).
- `THETA_MODE` (`2K`/`4K`), `TRANSMIT_WIDTH/HEIGHT`, `THROTTLE_FPS`, `JPEG_QUALITY`.
- `ARCHIVE_*` — full-res archive location, size cap, quality.
- `STICK_OFFSET_X/Y/Z` — selfie-stick geometry (measure these).

---

## Why Zenoh

Zenoh is the transport because it fits robot→cloud over flaky links: a
decentralized fabric (no broker bottleneck) with **interest propagation** (data
with no subscriber is dropped at the source, saving uplink), native
**fragmentation** of large payloads, and **query/reply** — which we use for
retransmitting dropped frames and fetching full-res archive frames on demand.

---

## Documentation

The full design, per-machine setup, and the bring-up runbook live in `docs/`
(mkdocs):

```bash
make docs-serve      # live local docs site
make docs            # static build
```

Start with [`docs/architecture.md`](docs/architecture.md),
[`docs/streaming_poc.md`](docs/streaming_poc.md), and the setup guides under
[`docs/setup/`](docs/setup/).
