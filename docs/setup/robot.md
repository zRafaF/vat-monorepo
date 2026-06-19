# Robot setup

End-to-end setup for the **Unitree Go2-W**. Two parts:

* **Camera — RICOH Theta X over UVC.** The Theta X does dynamic stitching +
  zenith correction *in-camera* during live streaming, so it exposes a clean
  equirectangular **UVC** stream. We capture it directly with OpenCV
  (`theta_camera.py`) — **no ROS camera node, no host-side stitching**. (We moved
  here from the Insta360 driver; see [archive](../archive/insta360.md).)
* **Docker container** — the ROS↔Zenoh **bridge** (for odometry) + `theta_camera`
  + `pose_fuser` (`robot/docker/`), run with `make robot-docker`.

!!! tip "Quick start (after the one-time install below)"
    From the repo root on the robot:
    ```bash
    make theta-uvc      # Theta X UVC → /dev/video0 (leave running in its own shell)
    make robot-docker   # bridge + theta_camera + pose fuser
    ```

---

## 1. Camera setup (RICOH Theta X over UVC)

The Theta X is not a plain webcam — its UVC stream needs `libuvc-theta` to
decode. The cleanest path on the Jetson is to decode it into a standard **v4l2
loopback** device that OpenCV reads as `/dev/video0`.

**a) Put the camera in live-streaming mode** and update its firmware
([Ricoh guide](https://blog.ricoh360.com/en/12306)). Connect it to the Jetson by
USB-C. Confirm: `lsusb | grep -i ricoh`.

**b) Build `libuvc-theta` + `libuvc-theta-sample`** (provides `gst_loopback`):

```bash
sudo apt install libjpeg-dev libusb-1.0-0-dev cmake \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad

# libuvc fork for THETA
git clone -b theta_uvc https://github.com/ricohapi/libuvc-theta
cd libuvc-theta && mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig
cd ~

# sample apps (gst_loopback / gst_viewer)
git clone https://github.com/ricohapi/libuvc-theta-sample
cd libuvc-theta-sample/gst && make
```

**c) Install the v4l2 loopback module:**

```bash
sudo apt install v4l2loopback-dkms
```

**d) Expose the Theta as `/dev/video0`** with the helper (loads the loopback
module + runs `gst_loopback`):

```bash
make theta-uvc        # = bash robot/theta/theta_uvc.sh  (leave running)
# overrides: GST_LOOPBACK_BIN, VIDEO_NR, THETA_MODE (2K|4K)
```

Verify the device streams (run on the robot — camera alone, no Zenoh):

```bash
make test_frames_robot     # = python3 tools/view_theta.py  (THETA_DEVICE=/dev/video0)
```

!!! note "Advanced: skip the loopback with a GStreamer pipeline"
    If your OpenCV is built **with GStreamer** and you have the
    [`gstthetauvc`](https://github.com/nickel110/gstthetauvc) plugin, set
    `THETA_GST_PIPELINE` instead of `THETA_DEVICE` (e.g. a `thetauvcsrc mode=2K !
    … ! appsink` pipeline, with `nvv4l2decoder` for Jetson HW decode).
    `theta_camera.py` and `tools/view_theta.py` both honour it. The pip OpenCV in
    the container has **V4L but not GStreamer**, which is why the default is the
    `/dev/video0` loopback path.

The Theta capture lives in the container's **`theta_camera.py`**: it reads the
device, picks the **sharpest frame in a small window** (live-tunable
`window_size`), stamps the **camera height**, and publishes
`{robot}/prism/camera/frame`. The raw stream never touches Zenoh — only the
decimated JPEG goes out.

---

## 2. Docker container (bridge + camera + fuser)

The container ships three processes (`robot/docker/`): the ROS↔Zenoh **bridge**
(odometry, e.g. `/sportmodestate`), **`theta_camera`** (above), and
**`pose_fuser`**. The Go2 has no docker-compose, so it's built from the repo root
and run via `run.sh`, wrapped by `make robot-docker`. The Theta `/dev/video0` is
passed in with `--device`.

```bash
# from the repo root (config — router IP, robot name, THETA_* — comes from vat.env)
make robot-docker
# = bash robot/docker/run.sh $ROUTER_IP   (build + docker run --network host --device /dev/video0 …)

docker logs -f vat-robot
```

!!! note "Docker permissions"
    If docker needs root, use `sudo make robot-docker` and `sudo docker logs -f
    vat-robot`. Better: `sudo usermod -aG docker $USER`, then log out/in.

!!! note "DDS matching (for the odometry bridge)"
    The Go2 host speaks **CycloneDDS** on `eth0`. The bridge container (ROS
    Humble) is built with `rmw_cyclonedds_cpp` and exports the matching
    `CYCLONEDDS_URI` at startup so it can see the host's Foxy topics — otherwise
    the bridge runs but bridges nothing. Override the interface with
    `NET_IFACE=eth1 make robot-docker`.

**Auto-start on boot:**

```bash
# Theta UVC loopback (host) + the container
sudo cp robot/systemd/vat-theta-uvc.service     /etc/systemd/system/   # edit paths
sudo cp robot/systemd/vat-robot-docker.service  /etc/systemd/system/   # edit ZENOH_CONNECT
sudo systemctl daemon-reload
sudo systemctl enable --now vat-theta-uvc.service vat-robot-docker.service
```

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

On our Go2, ROS failed at startup with `ros2: eth1: does not match an available
interface`. Pin CycloneDDS to the real interface before any ROS command:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
```

!!! note
    The container (via `start.sh`) and the `vat-theta-uvc`/`vat-robot-docker`
    units handle this for you; you only need it manually for ad-hoc `ros2 topic …`
    commands. Tip: add it to `~/.bashrc`. Override with `NET_IFACE`.

---

## 4. Troubleshooting

**`tools/view_theta.py` / `theta_camera`: could not open the Theta stream**

1. Camera in **live-streaming mode** and connected (`lsusb | grep -i ricoh`).
2. The loopback is up: `make theta-uvc` running, and `ls -l /dev/video0` exists.
3. `gst_loopback` built and on `GST_LOOPBACK_BIN`; `v4l2loopback-dkms` installed.
4. The container got the device: `--device /dev/video0` (run.sh adds it if the
   device exists — start `make theta-uvc` **before** `make robot-docker`).

**Container `theta_camera` logs "could not open Theta stream"** — the device
isn't visible inside the container. Confirm `/dev/video0` exists on the host
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

!!! warning "Custom Unitree types need their message package in the container"
    `/sportmodestate` is `unitree_go/msg/SportModeState` (a custom type). The
    bridge can only forward it once the `unitree_go` messages are available inside
    the container — needed for Stage 2 (body/limbs) and the live camera-height.
    Standard types (the camera path) are unaffected. (Follow-up: add `unitree_go`
    msgs to the image.)

---

## 5. Performance — keep the robot↔robot hops off the VPN (recommended)

The robot processes connect **directly to the cloud router** over the VPN. With
the Theta capture now local (no raw frames on Zenoh), the heaviest remaining
concern is keeping any robot-internal pub/sub off the link. The standard fix is
a **local Zenoh router on the robot** that `connect`s to the cloud router; the
robot processes point at `tcp/127.0.0.1:7447`. Zenoh only forwards keys with a
*remote* subscriber, so `{robot}/prism/camera/frame` (JPEG → mapping server) and
`{robot}/prism/pose` (→ client) cross the VPN, while anything robot-only stays
local. (Not wired up yet — a small change to `run.sh` + `vat.env`; ask and it can
be added.)
