# Robot setup

End-to-end setup for the **Unitree Go2-W**. The "robot" here is the **Jetson**
computer on board the dog. Everything below is run **on the Jetson**, in a
terminal, from the repo root (the `vat-monorepo` folder you cloned).

There are two parts:

* **Camera — RICOH Theta X over UVC.** The Theta X does dynamic stitching +
  zenith correction *in-camera* during live streaming, so it exposes a clean
  equirectangular **UVC** stream. We capture it directly with OpenCV
  (`theta_camera.py`) — **no ROS camera node, no host-side stitching**. (We moved
  here from the Insta360 driver; see [archive](../archive/insta360.md).)
* **Docker container** — the ROS↔Zenoh **bridge** (for odometry) + `theta_camera`
  + `pose_fuser` (`robot/docker/`), run with `make robot-docker`.

!!! note "What the pose fuser does"
    `pose_fuser` runs the robot's **ESKF** (error-state Kalman filter) over the
    IMU, wheel odometry, and leg-FK height to produce the authoritative pose. It
    is not a placeholder. You don't need to understand it to set up the robot —
    see [Architecture](../architecture.md) for how the pose is computed and
    corrected.

!!! tip "Quick start (after the one-time install below)"
    From the repo root on the robot:
    ```bash
    make theta-uvc      # Theta X UVC → /dev/video10 (leave running in its own shell)
    make robot-docker   # bridge + theta_camera + pose fuser
    ```

---

## 0. Prerequisites

Before the camera and container steps, make sure the base tooling is present on
the Jetson.

- **Base tools** — `git`, `make`, and `uv`, plus the cloned repo. If you haven't
  done this, follow the [setup overview](index.md#linux-from-zero-skip-if-you-know-this)
  and [clone the repo with submodules](index.md#get-the-code-clone-with-submodules)
  first.
- **Docker** — the container step needs Docker. On a Jetson flashed with NVIDIA
  **JetPack**, Docker is normally preinstalled. Check with `docker --version`. If
  it's missing, install it (`sudo apt install docker.io`). ("Docker" runs the
  robot software inside an isolated container so it doesn't depend on the exact
  packages installed on the Jetson.)
- **Point the robot at the router** — in `vat.env` set `ROUTER_IP` to the VPN
  address of the SERVER (the same value the client uses). See
  [VPN setup](development/vpn.md).

!!! note "`sudo` and `apt`"
    Several steps use `sudo` (run as administrator — it asks for your password)
    and `apt` (Ubuntu's software installer). If those are new to you, see the
    [Linux primer](index.md#linux-from-zero-skip-if-you-know-this).

---

## 1. Camera setup (RICOH Theta X over UVC)

The Theta X is **not** a plain webcam. The mainline kernel `uvcvideo` driver
enumerates it (`dmesg` shows *"Found UVC 1.50 device RICOH THETA X"*) but then
reports *"No streaming interface found"* and exposes **no `/dev/videoN` capture
node** — the H.264 stream rides a vendor UVC 1.5 configuration the kernel driver
won't surface. (This is why plain `cv2.VideoCapture(0)` works on Windows, where
Ricoh ships a UVC driver, but never on stock Linux.) The stream must be pulled
in **userspace via `libuvc-theta`**. We decode it into a standard **v4l2
loopback** device that OpenCV reads as `/dev/video10`.

!!! note "New terms in this section"
    - **`/dev/videoN`** — Linux exposes cameras as files under `/dev/`; a webcam
      is typically `/dev/video0`.
    - **v4l2 loopback** — a *virtual* camera device. We decode the Theta stream
      and write it into this fake camera so ordinary software (OpenCV) can read
      it like any webcam.
    - **GStreamer** — a Linux media-pipeline toolkit. We use it to pull and
      decode the camera stream. Its command-line tool is `gst-launch-1.0`.

!!! warning "Loopback node is `/dev/video10`, not `/dev/video0`"
    This robot also runs an **Intel RealSense**, which claims the low-numbered
    `/dev/video0..N` nodes at boot. The Theta loopback therefore uses a dedicated
    high number — **`/dev/video10`** (`VIDEO_NR` in `theta_uvc.sh`, `THETA_DEVICE`
    in `vat.env`). If you put the loopback on a node the RealSense already owns,
    GStreamer fails with *"Device '/dev/videoN' is not a output device"*.

!!! warning "Use `gstthetauvc`, not stock `gst_loopback`"
    The `libuvc-theta-sample` `gst_loopback` binary identifies the camera by a
    **hardcoded product-ID table** (`0x2712` THETA V, `0x2715` Z1). The Theta X
    is `0x2717`, which isn't in the list, so it prints **`THETA not found`** and
    exits — *the camera is fine, the sample just doesn't know the X*. The
    [`gstthetauvc`](https://github.com/nickel110/gstthetauvc) plugin matches by
    **product name** (`strncmp(..., "RICOH THETA")`), so the Theta X — and any
    future model — works with **no source edits**. That's our default backend.
    (A patched `gst_loopback` is documented as a fallback at the end.)

**a) Put the camera in live-streaming mode** and update its firmware
([Ricoh guide](https://blog.ricoh360.com/en/12306)). Connect it to the Jetson by
USB-C, then on the camera switch to **Live Streaming** mode (Mode button →
`LIVE`). Confirm with `lsusb` (lists USB devices):

```bash
lsusb | grep -i ricoh
```

It **must** show **`05ca:2717`**. If you see **`05ca:0373`** (or any other id)
the camera is in normal/MTP mode, not streaming — `make theta-uvc` will refuse to
start until it reads `2717`. The camera can silently drop out of streaming mode
on reboot/idle, so re-check this if a previously-working setup stops.

**b) Build `libuvc-theta`** (the UVC1.5/H.264 fork) and remove any stray system
`libuvc` that would shadow it.

First install the build tools and libraries (`sudo apt install` fetches
system packages):

```bash
sudo apt install libjpeg-dev libusb-1.0-0-dev cmake \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

A system `libuvc` shadows the THETA fork, causing *"Found 1 Theta(s), but none
available"* / *"could not open"*. Remove it if present:

```bash
apt list --installed 2>/dev/null | grep -i libuvc   # if libuvc-dev/libuvc0 show up:
sudo apt purge -y libuvc-dev libuvc0 || true
```

Now clone and compile the fork. (`git clone` downloads the source;
`cmake .. && make` compiles it; `sudo make install` copies the built library into
the system; `sudo ldconfig` refreshes the system's list of libraries so it finds
the new one.)

```bash
git clone -b theta_uvc https://github.com/ricohapi/libuvc-theta
cd libuvc-theta && mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig
cd ~
```

A successful `make` ends with no errors and produces the `libuvc` shared library;
`sudo make install` prints the files it copied under `/usr/local`.

**c) Build + install the `gstthetauvc` plugin** (matches the camera by name → no
patch needed for the Theta X):

```bash
git clone https://github.com/nickel110/gstthetauvc
cd gstthetauvc/thetauvc && make
# put it where GStreamer finds it (adjust the triplet for your arch):
sudo cp gstthetauvc.so /usr/lib/$(uname -m)-linux-gnu/gstreamer-1.0/
cd ~
```

Sanity check — this should print the element's properties (mode/serial) with no
error:

```bash
gst-inspect-1.0 thetauvcsrc
```

If it prints *"No such element or plugin 'thetauvcsrc'"*, GStreamer didn't find
the plugin — see the note below.

!!! note "Plugin in a custom dir?"
    If you don't copy `gstthetauvc.so` into the system plugin dir, export
    `GST_PLUGIN_PATH=/path/to/gstthetauvc/thetauvc` instead; the helper script
    and the systemd unit both honour it. (An environment variable — a named value
    the shell passes to programs.)

**d) Install the v4l2 loopback kernel module** (this is what provides the virtual
camera device):

```bash
sudo apt install v4l2loopback-dkms
```

**e) Expose the Theta as `/dev/video10`** with the helper. `make theta-uvc` loads
the loopback module (via `sudo modprobe`, which the script calls for you) and
runs the `thetauvcsrc → v4l2sink` GStreamer pipeline:

```bash
make theta-uvc        # = bash robot/theta/theta_uvc.sh  (leave running)
# overrides: THETA_BACKEND (gstthetauvc|loopback), THETA_DECODER (auto|nv|sw),
#            VIDEO_NR, THETA_MODE (2K|4K), GST_PLUGIN_PATH
```

Leave this running in its own terminal — the container reads the device it feeds.
A healthy start prints `[theta] starting gstthetauvc … → /dev/video10` and then
sits quietly (no repeating errors).

!!! warning "Jetson: hardware decoder (`not negotiated` spam)"
    On the Jetson, GStreamer's `decodebin` auto-picks **`nvv4l2decoder`**,
    whose output lives in **NVMM** (GPU) memory. The CPU `videoconvert`/
    `v4l2sink` can't negotiate with NVMM, so the pipeline floods
    *"… capsfilter1: not negotiated"* and no frames reach `/dev/video10`.
    `theta_uvc.sh` fixes this with `THETA_DECODER=auto`, which uses an
    explicit **`nvv4l2decoder ! nvvidconv`** pair on the Jetson (force with
    `THETA_DECODER=nv`; use `sw` for x86/software decode).

Verify the device streams (run on the robot — camera alone, no Zenoh):

```bash
make test_frames_robot     # = python3 tools/view_theta.py  (THETA_DEVICE=/dev/video10)
```

!!! tip "Headless robot? View on the host over Zenoh (low latency)"
    `test_frames_robot` opens an **OpenCV window on the robot** — useless on a
    headless Go2 (one with no monitor attached). Instead, publish the loopback to
    Zenoh and view it on your laptop. No container, no decimation, ~one JPEG per
    frame:

    ```bash
    # on the robot (leave both running):
    make theta-uvc                  # feed /dev/video10
    make theta-stream               # uv run tools/theta_pub.py → Zenoh
    # on the host:
    make test_frames_server         # = tools/view_frames.py  (OpenCV window)
    ```

    `theta-stream` publishes on the **same** `{robot}/prism/camera/frame` key
    the container uses, so run **either** `theta-stream` **or** the full
    container — not both. Tune `PREVIEW_FPS` / `PREVIEW_SCALE` /
    `PREVIEW_QUALITY` (env) to trade latency vs. quality. Dependencies are a
    standalone **uv** project (`robot/pyproject.toml`: eclipse-zenoh +
    headless OpenCV + numpy); `make theta-stream` runs `make sync-robot`
    first, so the `robot/.venv` is created automatically on first run.

!!! note "Advanced: skip the loopback entirely"
    If your OpenCV is built **with GStreamer**, you can hand a pipeline straight
    to OpenCV via `THETA_GST_PIPELINE` (e.g. `thetauvcsrc mode=2K ! … ! appsink`,
    with `nvv4l2decoder` for Jetson HW decode) instead of going through
    `/dev/video10`. `theta_camera.py` and `tools/view_theta.py` both honour it.
    We keep the loopback as the default because the **pip OpenCV in the container
    has V4L but not GStreamer**.

??? note "Fallback: patched `gst_loopback` (only if you can't build the plugin)"
    The original `libuvc-theta-sample` route still works **if** you add the
    Theta X product ID. In `libuvc-theta-sample/gst/thetauvc.c`:

    ```c
    #define USBPID_THETAX_UVC 0x2717        // add near the other USBPID_ defines
    // …and accept it in thetauvc_find_devices():
    if (desc->idProduct == USBPID_THETAV_UVC
        || desc->idProduct == USBPID_THETAZ1_UVC
        || desc->idProduct == USBPID_THETAX_UVC) {
    ```

    Then `cd libuvc-theta-sample/gst && make`, and run with
    `THETA_BACKEND=loopback make theta-uvc` (set `GST_LOOPBACK_BIN` if the binary
    isn't at `~/libuvc-theta-sample/gst/gst_loopback`).

The Theta capture lives in the container's **`theta_camera.py`**: it reads the
device, picks the **sharpest frame in a small window** (live-tunable
`window_size`), stamps the **camera height**, and publishes
`{robot}/prism/camera/frame`. The raw stream never touches Zenoh — only the
decimated JPEG goes out.

### Full-res archive (offline reconstruction)

The live stream is **downscaled** to `TRANSMIT_WIDTH`×`TRANSMIT_HEIGHT`
(`1036×518` by default) for the mapping server. In parallel, `theta_camera.py`
writes the **full-resolution twin** of every transmitted frame to a local,
size-capped archive — same `seq` / timestamp / `camera_height`, so the live and
archived frames line up **1:1**. This is the source for offline, non-real-time
reconstruction (Gaussian splats, NeRF): the heavy work runs later against the
high-res frames, joined to the recorded pose trajectory by timestamp.

* **Storage** — a SQLite index (`<ARCHIVE_DIR>/index.sqlite`) plus JPEG files
  (`<ARCHIVE_DIR>/frames/<seq>.jpg`). Written on a **separate thread** with a
  bounded queue, so the full-res encode + disk I/O never stall the real-time
  publish (frames are dropped from the archive, never from the live stream, if
  the disk can't keep up).
* **Size cap** — `ARCHIVE_MAX_BYTES` (default `10GB`); oldest frames are evicted
  first (FIFO). It lives on a **bind-mounted host dir** (`ARCHIVE_DIR_HOST` →
  `ARCHIVE_DIR`) so it survives container restarts; `run.sh` creates and mounts
  it. (A "bind mount" shares a folder from the Jetson into the container so the
  files persist outside it.)
* **On-demand full-res fetch** — the container exposes a Zenoh queryable
  `{robot}/prism/camera/archive/get?seq=N`. Pull one full-res frame from the
  host or server by the seq the live stream showed:

    ```bash
    make fetch_frame SEQ=1234        # = tools/fetch_archive.py --seq 1234
    ```

!!! note "4K needs 4K hardware decode"
    The archive's detail comes from `THETA_MODE=4K`, which needs a Jetson that
    can HW-decode 4K H.264 (Xavier/Orin). If `make theta-uvc` stalls at 4K, set
    `THETA_MODE=2K` — the archive then stores 2K full-res and ~4× more history
    fits in the 10GB budget. (5.7K/8K aren't available over live USB — they'd
    need separate still captures.)

Config (all in `vat.env`): `TRANSMIT_WIDTH`/`TRANSMIT_HEIGHT`, `ARCHIVE_ENABLE`,
`ARCHIVE_DIR`, `ARCHIVE_DIR_HOST`, `ARCHIVE_MAX_BYTES`, `ARCHIVE_JPEG_QUALITY`.

---

## 2. Docker container (bridge + camera + fuser)

The container ships four processes (`robot/docker/`): the ROS↔Zenoh **bridge**
(odometry, e.g. `/sportmodestate`), **`theta_camera`** (above), the
**`pose_fuser`** (the ESKF), and a **teleop bridge**. The Go2 has no
docker-compose, so it's built from the repo root and run via `run.sh`, wrapped by
`make robot-docker`. The Theta `/dev/video10` is passed in with `--device` (which
hands one device from the Jetson into the container).

Start the camera **first** (`make theta-uvc`), then the container, both from the
repo root:

```bash
# config — router IP, robot name, THETA_* — comes from vat.env
make robot-docker
# = bash robot/docker/run.sh $ROUTER_IP   (build + docker run --network host --device /dev/video10 …)
```

`make robot-docker` builds the image (slow the first time) and starts the
container in the background. Watch its logs with:

```bash
docker logs -f vat-robot
```

(`docker logs -f` streams a container's output; press `Ctrl`+`C` to stop
watching — the container keeps running.)

!!! note "Docker permissions"
    If docker needs root, use `sudo make robot-docker` and `sudo docker logs -f
    vat-robot`. Better: add yourself to the `docker` group once so you don't need
    `sudo` — `sudo usermod -aG docker $USER`, then log out and back in.

!!! note "DDS matching (for the odometry bridge)"
    The Go2 host speaks **CycloneDDS** on `eth0`. The bridge container (ROS
    Humble) is built with `rmw_cyclonedds_cpp` and exports the matching
    `CYCLONEDDS_URI` at startup so it can see the host's Foxy topics — otherwise
    the bridge runs but bridges nothing. Override the interface with
    `NET_IFACE=eth1 make robot-docker`.

**Auto-start on boot:**

The robot uses two **systemd** services (background programs the system starts on
boot): `vat-theta-uvc.service` (the camera loopback) and
`vat-robot-docker.service` (the container). Install them by copying the unit files
into the system directory, then enabling them:

```bash
# Theta UVC loopback (host) + the container
sudo cp robot/systemd/vat-theta-uvc.service     /etc/systemd/system/   # edit User/paths
sudo cp robot/systemd/vat-robot-docker.service  /etc/systemd/system/   # edit ZENOH_CONNECT
sudo systemctl daemon-reload
sudo systemctl enable --now vat-theta-uvc.service vat-robot-docker.service
```

`systemctl enable --now` both enables the service at boot **and** starts it now.
Watch a service's logs with `sudo journalctl -fu vat-theta-uvc` (or
`vat-robot-docker`).

!!! warning "Edit the unit files before enabling"
    The shipped units assume the user `unitree` and the repo at
    `/home/unitree/Desktop/vat-monorepo`, and `vat-robot-docker.service` has
    `ZENOH_CONNECT=tcp/127.0.0.1:7447` as a placeholder. Open each file under
    `/etc/systemd/system/` and fix the `User`, `WorkingDirectory`, and
    `ZENOH_CONNECT` to match your robot and your router's address, then
    `sudo systemctl daemon-reload`.

### How the bridge works

* **Dynamic discovery** — polls the ROS graph every 2 s for new topics.
* **Smart routing (`MatchingListener`)** — only subscribes to a ROS topic when a
  remote Zenoh client is listening; stops when clients disconnect.
* **QoS matching** — probes each publisher's QoS and matches it (sensor topics
  are BEST_EFFORT; a default RELIABLE subscriber would get nothing).
* **Liveliness** — a heartbeat token so the server can detect a dropped robot.

??? failure "[Experimental History] Attempted bridge solutions"
    Before the current Python `rclpy` bridge we tried the
    `eclipse/zenoh-bridge-ros2dds` container (couldn't detect the Foxy nodes),
    middleware/loopback tweaks (`RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY=1`,
    multicast on `lo`), and building `zenoh-plugin-ros2dds` from source for
    `dds_shm` (cmake/Foxy dependency conflicts). The Humble Docker + `rclpy`
    bridge bypassed the discovery issues while keeping native compatibility with
    the robot's ROS graph.

---

## 3. Known issue — CycloneDDS interface

On our Go2, ROS fails at startup with `<iface>: does not match an available
interface` when CycloneDDS is bound to a NIC (network card) that doesn't exist —
this crash-loops the bridge (`rcl node's rmw handle is invalid`). **The container
now auto-detects the interface** (`start.sh` prefers the NIC on the Unitree
subnet `192.168.123.x`, then the default route, then the first UP NIC), so
`make robot-docker` should just work. If auto-detect picks the wrong one
(bridge logs `[no data]`), pin it: set `NET_IFACE=<iface>` in `vat.env`
(find it with `ip -br addr` on the robot, which lists network interfaces). For
ad-hoc `ros2` commands on the host, export it manually:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
```

!!! note
    The container (via `start.sh`) and the `vat-theta-uvc`/`vat-robot-docker`
    units handle this for you; you only need it manually for ad-hoc `ros2 topic …`
    commands. Tip: add it to `~/.bashrc` (a file of commands your shell runs at
    startup). Override with `NET_IFACE`.

---

## 4. Troubleshooting

**`tools/view_theta.py` / `theta_camera`: could not open the Theta stream**

1. Camera in **live-streaming mode** and connected (`lsusb | grep -i ricoh` →
   `05ca:2717`).
2. The loopback is up: `make theta-uvc` running, and `ls -l /dev/video10` exists
   (it is the loopback, not the RealSense).
3. `gstthetauvc` plugin found: `gst-inspect-1.0 thetauvcsrc` succeeds (set
   `GST_PLUGIN_PATH` if not in the system dir); `v4l2loopback-dkms` installed.
4. `THETA not found` from `make theta-uvc` → you're on the `loopback` backend
   with an **unpatched** `gst_loopback` (no Theta X `0x2717` PID). Switch to the
   default `gstthetauvc` backend, or apply the patch (see §1 fallback note).
5. `Found 1 Theta(s), but none available` / cannot open → a **stray system
   `libuvc`** is shadowing the fork: `sudo apt purge libuvc-dev libuvc0`, then
   rebuild and `sudo ldconfig`.
6. `Device '/dev/videoN' is not a output device` (caps `0x…0001` = capture-only)
   → the loopback node clashed with a **real capture device** (the RealSense owns
   `/dev/video0..N`). Use the dedicated `VIDEO_NR=10` (default) and keep
   `THETA_DEVICE=/dev/video10` in `vat.env`. Check with
   `cat /sys/class/video4linux/video10/name` → it should read `ThetaUVC`.
7. The camera dropped to `05ca:0373` (normal/MTP) → re-enter **Live Streaming**
   mode on the camera; `make theta-uvc` refuses to start until `lsusb` shows
   `05ca:2717`.
8. `… capsfilter1: not negotiated` repeating (you'll see `NvMMLiteOpen`,
   `BlockType = 261`) → Jetson `decodebin` picked the HW decoder
   (`nvv4l2decoder`, NVMM memory) and the CPU `v4l2sink` can't take it. The
   script's `THETA_DECODER=auto` handles this; if you overrode it, use
   `THETA_DECODER=nv` (Jetson) or `sw` (x86). See §1.
9. The container got the device: `--device /dev/video10` (run.sh adds it if the
   device exists — start `make theta-uvc` **before** `make robot-docker`).

!!! tip "Camera dropped after an idle/reboot? Restart just the feed"
    If the camera fell out of streaming mode and you rebooted it, kill the old
    feed and restart it without touching the container:
    ```bash
    make theta-uvc-kill    # stop the old gstthetauvc/gst_loopback feed
    make theta-uvc         # start a fresh feed
    ```
    The loopback `/dev/video10` stays loaded and the container's frozen-stream
    watchdog reconnects automatically once the fresh feed is up.

**Container `theta_camera` logs "could not open Theta stream"** — the device
isn't visible inside the container. Confirm `/dev/video10` exists on the host
*before* `make robot-docker`, or rerun it so `--device` is attached.

**`make robot-docker` → `run.sh: Permission denied`** — the Makefile calls `bash
robot/docker/run.sh`; if invoking directly, prefix with `bash`.

**`docker logs` → `permission denied … /var/run/docker.sock`** — add your user to
the `docker` group (`sudo usermod -aG docker $USER`, re-login) or use `sudo`.

**Container spams `AMENT_TRACE_SETUP_FILES: unbound variable`** — old `set -u`
bug, fixed; **rebuild** (`make robot-docker`).

**`make test_link` shows the bridge `absent`, or odometry won't bridge** — a DDS
mismatch. The container is built with `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI`;
**rebuild**. Verify inside: `sudo docker exec -it vat-robot bash -lc 'source
/opt/ros/humble/setup.bash && ros2 topic list'`. The bridge logs forwarded
counts every 10 s (`[forwarded] …`); `[no data]` means QoS/DDS/interface/domain.

!!! note "Custom Unitree types are built into the image (full unitree_ros2)"
    `/sportmodestate` is `unitree_go/msg/SportModeState`, a CUSTOM type — and
    because the Go2 (Foxy) types are **FINAL** extensibility, CycloneDDS only
    *delivers* samples when the subscriber's type matches the publisher's
    **exactly** (a trimmed subset connects but receives **0 msgs**). The image
    therefore clones the **full [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2)**
    package (`unitree_go` + `unitree_api`) and colcon-builds it; `start.sh`
    sources the overlay. Pin the version with `UNITREE_ROS2_REF` in `vat.env`.
    Confirm in the bridge logs: `Registered Zenoh route for: /sportmodestate`
    then `[forwarded] /sportmodestate=N` (N climbing).

    If it stays `[no data]` after a rebuild: (1) the publisher's **QoS** — the
    bridge now defaults a BEST_EFFORT subscriber when it can't read the
    publisher QoS, which fixes the common case; (2) your **firmware's layout**
    differs from the pinned `unitree_ros2` — run
    `ros2 interface show unitree_go/msg/SportModeState` **on the Go2 host** and
    pin `UNITREE_ROS2_REF` to the matching branch/tag (older defs carry extra
    fields like `path_point`).
---

## 5. Performance — keep the robot↔robot hops off the VPN (recommended)

The robot processes connect **directly to the cloud router** over the VPN. With
the Theta capture now local (no raw frames on Zenoh), the heaviest remaining
concern is keeping any robot-internal pub/sub off the link. The standard fix is
a **local Zenoh router on the robot** that `connect`s to the cloud router; the
robot processes point at `tcp/127.0.0.1:7447`. Zenoh only forwards keys with a
*remote* subscriber, so `{robot}/prism/camera/frame` (JPEG → mapping server) and
`{robot}/prism/pose` (→ client) cross the VPN, while anything robot-only stays
local.

!!! note "Unverified"
    This local-router optimisation is described as **not wired up yet** — a small
    change to `run.sh` + `vat.env`. Treat it as a design note, not a step you can
    run today.
