# Bring-up Runbook

A **staged** way to bring the system up. Each stage adds one piece and has its
own check, so when something breaks you know exactly where. Don't run everything
at once — walk the stages until each is green, then move on.

There are three machines:

| Badge | Machine | Role |
|---|---|---|
| 🤖 **ROBOT** | Jetson on the Go2-W | camera stack (host ROS Foxy) + Docker (bridge, decimator, pose fuser) |
| ☁️ **SERVER** | GPU box | Zenoh router + mapping server (PRISM-VGGT) |
| 💻 **CLIENT** | your laptop | diagnostic tools + Rerun viewer |

### The Makefile is the control file

All config lives in **`vat.env`** at the repo root (committed; VPN-internal, no
secrets) — the router IP, robot name, ports, and tuning. Edit it once and every
machine agrees:

```bash
ROUTER_IP=100.87.118.34       # the host running `make router`
ROBOT_NAME=go2
# ...ports, throttle fps, stick offsets, etc.
```

The **Makefile** reads `vat.env` and drives everything. `make help` lists the
targets; **`make steps`** prints this whole runbook. Note the mapping server
dials the router by its IP (`ZENOH_ROUTER`), which may be in a *different
datacenter* than the router — that's expected.

Install deps once per machine:

```bash
make sync-router     # ☁️ SERVER — isolated router env
make sync-mapping    # ☁️ SERVER — mapping server (CUDA)
make sync-client     # 💻 CLIENT — viewer + bring-up tools
#  🤖 ROBOT builds the Docker image (make robot-docker)
```

---

## Stage 0 — Transport is alive

**☁️ SERVER — start the Zenoh router microservice** (everything connects to it).
This is a pure-Python router node in its own isolated env — no `zenohd` binary,
no Docker:

```bash
make router
# = cd server/router && uv run python router.py   (binds ZENOH_LISTEN from vat.env)
```

**🤖 ROBOT — start the Theta UVC + container.** One-time prereqs (see
[robot setup](setup/robot.md)): `libuvc-theta` + `gstthetauvc` plugin built,
`v4l2loopback-dkms` installed, Theta X in **live-streaming** mode. Then, from the
repo (e.g. `~/Desktop/vat-monorepo`):

```bash
# expose the Theta X UVC stream as /dev/video10 — LEAVE RUNNING in its own shell
make theta-uvc

# in another shell: bridge (odometry) + theta_camera + pose fuser
make robot-docker
docker logs -f vat-robot          # expect "Theta stream open. Streaming…"
```

**💻 CLIENT — check the link:**

```bash
make test_link
```

✅ Expect: `robot bridge ALIVE`, `robot pose fuser ALIVE`, and non-zero Hz on
`camera/frame` and `pose`. (`/sportmodestate` needs the `unitree_go` msgs in the
container — Stage 2.) If `camera/frame` is 0 Hz, fix the camera first
(`make test_frames_robot` on the robot).

---

## Stage 1 — See the 360° frames

**💻 CLIENT:**

```bash
# [ROBOT] preview the Theta UVC directly (camera alone, no Zenoh) — sanity check
make test_frames_robot      # = python3 tools/view_theta.py

# [CLIENT] the decimated frames the server will actually consume — the real check
make test_frames_server     # small JPEG; shows camera_height + seq
```

!!! note
    If the robot preview works but `camera/frame` is 0 Hz on the client, check
    `docker logs vat-robot` for `theta_camera` errors (device not passed in?) and
    that `make theta-uvc` is running. The Theta capture is local to the robot —
    only the decimated JPEG crosses the link.

✅ Expect: a live equirectangular image at ~`throttle_fps` (decimated);
`camera/height_m` should be a sane number (≈ stand height
+ stick). Tune live if needed:

```bash
zenoh put -k go2/rt/prism/config/throttle_fps -v 3.0
zenoh put -k go2/rt/prism/config/window_size  -v 5     # best-of-5 sharpest
```

---

## Stage 2 — See the body & limbs

**💻 CLIENT:**

```bash
make test_robot_state
```

✅ Expect: the body frame tilts with the real robot, four feet (FR/FL/RR/RL)
move in real time, and `body_height` changes when the Go2-W stands/lies. If
decode fails, your firmware's `unitree_go/SportModeState` layout differs — fix
the embedded defs in `robot/docker/kinematics.py`.

---

## Stage 3 — Are the poses right?

**☁️ SERVER — start the mapping server** (needs the GPU + PRISM-VGGT). It dials
the router at `ROUTER_IP` from `vat.env`, even across datacenters:

```bash
make mapping
# = cd server/mapping && uv run python mapping_server.py  (its own isolated env)
```

**💻 CLIENT — watch the pose path (no heavy cloud yet):**

```bash
make test_poses
```

✅ Expect: the camera **trajectory** grows as the robot moves; **correction**
points appear each submap; the **fused robot pose** tracks the trajectory,
offset by the selfie-stick transform, and goes **green** briefly after each
correction (amber while dead-reckoning). If the robot box sits in the wrong
place relative to the camera path, re-measure `STICK_OFFSET_X/Y/Z`.

---

## Stage 4 — Full POC (map + robot block)

**☁️ SERVER:** mapping server already running (Stage 3).

**💻 CLIENT — the full viewer (point cloud + predicted robot block):**

```bash
make viewer
```

✅ Expect: the coloured point cloud builds incrementally and the robot block
moves smoothly (predicted between pose samples), green/amber by fix quality.

---

## Per-machine command reference

| Machine | `make` target | Stage |
|---|---|---|
| ☁️ SERVER | `make router` | 0+ |
| ☁️ SERVER | `make mapping` | 3+ |
| 🤖 ROBOT | `make theta-uvc`  (Theta X → /dev/video10) | 0+ |
| 🤖 ROBOT | `make robot-docker` | 0+ |
| 💻 CLIENT | `make test_link` | 0 |
| 🤖 ROBOT `make test_frames_robot` / 💻 CLIENT `make test_frames_server` | | 1 |
| 💻 CLIENT | `make test_robot_state` | 2 |
| 💻 CLIENT | `make test_poses` | 3 |
| 💻 CLIENT | `make viewer` | 4 |

(`make help` lists these; `make steps` prints the ordered runbook.)

### Handy live tuning / inspection

```bash
zenoh put -k go2/rt/prism/config/throttle_fps -v 4.0   # output frame rate
zenoh put -k go2/rt/prism/config/window_size  -v 5     # sharpness window (odd)
zenoh sub -k 'go2/prism/pose'                          # raw authoritative pose
zenoh sub -k 'server/prism/pose_correction'            # VGGT corrections (down)
docker logs -f vat-robot                               # robot container logs
journalctl -fu vat-robot-docker                        # if installed as a service
```

> The mapping server batches a window when **either** `WINDOW_SIZE-OVERLAP` new
> frames arrive **or** `WINDOW_TIMEOUT_S` (default 2 s) elapses, whichever first,
> and re-requests any dropped frames (by `seq`) from the decimator before
> processing — so a sparse/lossy stream neither stalls the viewer nor corrupts
> the pose estimate.
