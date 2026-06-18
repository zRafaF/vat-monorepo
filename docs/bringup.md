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

Set these once per shell (adjust the IP to your server):

```bash
# ☁️ SERVER and 💻 CLIENT
export ZENOH_ROUTER=tcp/<SERVER_IP>:7447
export ROBOT_NAME=go2

# 🤖 ROBOT (the container connects OUT to the server's router)
export SERVER_IP=<SERVER_IP>
export ROBOT_NAME=go2
```

Install deps once: mapping server `uv sync --package vat-mapping`; router (its
own isolated env) `cd server/router && uv sync`; client (incl. the bring-up
tools) `uv sync --package vat-client`; robot builds the Docker image.

---

## Stage 0 — Transport is alive

**☁️ SERVER — start the Zenoh router microservice** (everything connects to it).
This is a pure-Python router node in its own isolated env — no `zenohd` binary,
no Docker:

```bash
cd server/router && uv sync && uv run python router.py      # or: make router
# listens on tcp/0.0.0.0:7447 — override with ZENOH_LISTEN
```

**🤖 ROBOT — start the camera stack + container:**

```bash
# host ROS Foxy camera stack (Insta360 → equirectangular)
source /opt/ros/foxy/setup.bash && source ~/vat_ws/install/setup.bash
ros2 launch vat_bringup vat_bringup.launch.xml

# in another shell: bridge + decimator + pose fuser (no compose)
cd ~/vat-monorepo
./robot/docker/run.sh $SERVER_IP
docker logs -f vat-robot          # watch for "Registered Zenoh route ..."
```

**💻 CLIENT — check the link:**

```bash
python tools/check_link.py
```

✅ Expect: `robot bridge ALIVE`, ROS topics listed (incl. `/equirectangular/image`,
`/sportmodestate`), and non-zero Hz on `equirectangular/image` and
`camera/frame`. If a stream is 0 Hz, fix it here before continuing.

---

## Stage 1 — See the 360° frames

**💻 CLIENT:**

```bash
# raw equirectangular straight off the camera/bridge (tests the camera alone)
python tools/view_frames.py --raw

# the decimated frames the server will actually consume
# (also shows the stamped camera_height + seq, tests the decimator)
python tools/view_frames.py
```

✅ Expect: a live equirectangular image in Rerun at ~`throttle_fps` (decimated)
or camera rate (raw); `camera/height_m` should be a sane number (≈ stand height
+ stick). Tune live if needed:

```bash
zenoh put -k go2/rt/prism/config/throttle_fps -v 3.0
zenoh put -k go2/rt/prism/config/window_size  -v 5     # best-of-5 sharpest
```

---

## Stage 2 — See the body & limbs

**💻 CLIENT:**

```bash
python tools/view_robot_state.py
```

✅ Expect: the body frame tilts with the real robot, four feet (FR/FL/RR/RL)
move in real time, and `body_height` changes when the Go2-W stands/lies. If
decode fails, your firmware's `unitree_go/SportModeState` layout differs — fix
the embedded defs in `robot/docker/kinematics.py`.

---

## Stage 3 — Are the poses right?

**☁️ SERVER — start the mapping server** (needs the GPU + PRISM-VGGT):

```bash
source .venv/bin/activate
ZENOH_ROUTER=tcp/127.0.0.1:7447 ROBOT_NAME=go2 \
  python server/mapping/mapping_server.py
```

**💻 CLIENT — watch the pose path (no heavy cloud yet):**

```bash
python tools/view_poses.py
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
python client/prism_rerun_viewer.py --snapshot
```

✅ Expect: the coloured point cloud builds incrementally and the robot block
moves smoothly (predicted between pose samples), green/amber by fix quality.

---

## Per-machine command reference

| Machine | Command | Stage |
|---|---|---|
| ☁️ SERVER | `cd server/router && uv run python router.py` (or `make router`) | 0+ |
| ☁️ SERVER | `python server/mapping/mapping_server.py` | 3+ |
| 🤖 ROBOT | `ros2 launch vat_bringup vat_bringup.launch.xml` | 0+ |
| 🤖 ROBOT | `./robot/docker/run.sh $SERVER_IP` | 0+ |
| 💻 CLIENT | `python tools/check_link.py` | 0 |
| 💻 CLIENT | `python tools/view_frames.py [--raw]` | 1 |
| 💻 CLIENT | `python tools/view_robot_state.py` | 2 |
| 💻 CLIENT | `python tools/view_poses.py` | 3 |
| 💻 CLIENT | `python client/prism_rerun_viewer.py --snapshot` | 4 |

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
