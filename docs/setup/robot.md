# Robot setup

End-to-end setup for the **Unitree Go2-W**: the Insta360 camera stack (host ROS
Foxy) and the Zenoh bridge / decimator / pose-fuser container.

The robot side has two parts:

* **Host ROS (Foxy)** — the `insta360_ros_driver` publishing
  `/equirectangular/image` etc. Built in `~/ros2_ws`.
* **Docker container** — the ROS↔Zenoh bridge + frame decimator + pose fuser
  (`robot/docker/`), run with `make robot-docker`.

!!! tip "Quick start (after the one-time install below)"
    From the repo root on the robot:
    ```bash
    make robot-ros      # camera (CycloneDDS fix + equirectangular)
    make robot-docker   # bridge + decimator + pose fuser
    ```

---

## 1. Workspace & dependencies (Foxy)

The Go2 runs Ubuntu 20.04 / ROS 2 Foxy. Enable the Unitree ROS 2 environment and
create a clean workspace.

```bash
# Enable the Unitree ROS 2 environment (publishes the Go2 topics)
source ~/unitree_ros2/setup.sh

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

On Foxy some dependencies are missing from `apt`, so build them from source:

```bash
cd ~/ros2_ws/src
# Foxy-compatible camera manager
git clone https://github.com/ros-perception/image_common.git -b foxy
# IMU tools (imu_filter_madgwick)
git clone https://github.com/ccny-ros-pkg/imu_tools.git -b foxy

cd ~/ros2_ws
rosdep update
# --skip-keys avoids apt-installing the packages we just cloned
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "imu_tools imu_filter_madgwick camera_info_manager"
```

---

## 2. Acquire the Insta360 SDK

You need an Insta360 account and must request SDK access from the
[Insta360 SDK Portal](https://www.insta360.com/sdk/record).

!!! warning
    Use the **latest** SDK. The driver requires the SDK posted **after April 23,
    2025**. See the driver's [install notes](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

!!! tip
    Right-click the download button → "Copy Link Address" to get a direct URL you
    can `wget`/`curl` onto the robot. It looks like
    `https://wassets.insta360.com/common/<key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip`.

```bash
cd ~
curl -O https://wassets.insta360.com/common/<your-key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
rm Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip

cd Linux_CameraSDK-2.1.1_MediaSDK-3.1.1
# Go2-W is ARM64 → use the jetson-linux tarball
tar -xzf CameraSDK-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu.tar.gz
```

---

## 3. Install & build the driver

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver

# Copy SDK headers + library into the driver (dir names vary by SDK version)
cp -r ~/Linux_CameraSDK-*/CameraSDK-*/include/* ~/ros2_ws/src/insta360_ros_driver/include/
cp    ~/Linux_CameraSDK-*/CameraSDK-*/lib/libCameraSDK.so ~/ros2_ws/src/insta360_ros_driver/lib/

# Clean up the SDK tarball
cd ~ && rm -rf Linux_CameraSDK-2.1.1_MediaSDK-3.1.1
```

Build (the CMake policy flag keeps Foxy compatible with newer build tools):

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --allow-overriding image_transport
source install/setup.bash
```

Verify: `ros2 pkg prefix camera_info_manager` should return a path under
`~/ros2_ws/install`.

---

## 4. Hardware configuration

**Camera settings (on the camera itself):**

1. **USB Mode = Android** — swipe down → Settings → General → USB Mode → Android
   (not Webcam). Required for the driver to detect the camera
   ([Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4)).
2. **Dual-Lens mode** — make sure the camera is in dual-lens mode.

**USB permissions (udev `/dev/insta`):** the SDK needs USB access. Create the
udev rule (camera connected + powered on so the device node exists):

```bash
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh

# Manual fallback if setup.sh fails ("device /dev/insta not found"):
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' \
  | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

---

## 5. Camera bringup (equirectangular)

Bring the camera up in **equirectangular** mode — that's what PRISM consumes.
The driver's `equirectangular` arg **defaults to `false`**, so it must be set.

From the repo root, `make robot-ros` wraps everything (CycloneDDS eth0 fix +
sourcing `~/ros2_ws` + the launch):

```bash
make robot-ros
# = bash robot/ros/bringup_camera.sh
#   → ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true
```

!!! warning "It goes quiet after startup — that's normal, don't Ctrl-C it"
    Success looks like: `Camera opened successfully` → `Live streaming started`
    → `Mapping matrices initialization complete`, and then **silence** (the
    driver doesn't log every frame). It is streaming. **Leave `make robot-ros`
    running in its own terminal** — if you Ctrl-C it, the camera stops and every
    downstream stream drops to 0 Hz. (A one-off `error info: 400 [unknown msg
    code.]` just before "Camera opened successfully" is a benign SDK quirk.)

Published topics: `/equirectangular/image`, `/dual_fisheye/image[/compressed]`,
`/imu/data[_raw]`. Verify in another shell (while `make robot-ros` keeps running):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
source ~/ros2_ws/install/setup.bash
ros2 topic hz /equirectangular/image
```

Env overrides for the bringup script: `ROS_DISTRO` (default `foxy`), `ROS2_WS`
(default `~/ros2_ws`), `NET_IFACE` (default `eth0`), `EQUIRECTANGULAR` (default
`true`).

---

## 6. Zenoh bridge container

The bridge no longer runs alone — it ships in **one** container alongside the
frame decimator and the pose fuser (`robot/docker/`). The Go2 has no
docker-compose, so it's built from the repo root and run via `run.sh`, wrapped
by `make robot-docker`.

```bash
# from the repo root (config — router IP, robot name — comes from vat.env)
make robot-docker
# = bash robot/docker/run.sh $ROUTER_IP
#   (docker build -f robot/docker/Dockerfile … ; docker run --network host …)

docker logs -f vat-robot       # expect "Registered Zenoh route ..." per topic
```

!!! note "Docker permissions"
    If docker needs root on your robot, use `sudo make robot-docker` and
    `sudo docker logs -f vat-robot`. (Better: add your user to the `docker`
    group: `sudo usermod -aG docker $USER`, then log out/in — after that no
    `sudo` is needed.)

!!! note "DDS matching (important)"
    The Go2 host speaks **CycloneDDS** on `eth0`. The bridge container (ROS
    Humble) is built with `rmw_cyclonedds_cpp` and exports the matching
    `CYCLONEDDS_URI` at startup so it can actually see the host's Foxy topics —
    otherwise the bridge runs but bridges nothing. Override the interface with
    `NET_IFACE=eth1 make robot-docker` if needed. The container uses
    `--network host`, so it shares the host's interfaces and ROS domain.

**Auto-start on boot:**

```bash
sudo cp robot/systemd/vat-robot-docker.service /etc/systemd/system/   # edit ZENOH_CONNECT
sudo systemctl enable --now vat-robot-docker.service
sudo journalctl -fu vat-robot-docker
```

### How the bridge works

* **Dynamic discovery** — polls the ROS graph every 2 s for new topics.
* **Smart routing (`MatchingListener`)** — only creates a ROS subscription when a
  remote Zenoh client is actually listening; stops it when clients disconnect.
* **Liveliness** — broadcasts a heartbeat token so the server can detect a
  dropped robot immediately.

| Variable | Description | Default |
|---|---|---|
| `ROBOT_NAME` | Zenoh key prefix (e.g. `go2/rt/topic`) | `go2` (from `vat.env`) |
| `ZENOH_CONNECT` | Endpoint of the remote Zenoh router | `tcp/$ROUTER_IP:7447` |
| `LOG_LEVEL` | `DEBUG` / `INFO` / `WARN` | `INFO` |

??? failure "[Experimental History] Attempted bridge solutions"
    We explored several official Zenoh-ROS 2 integration paths before settling on
    the current Python implementation.

    **Attempt 1: `eclipse/zenoh-bridge-ros2dds` container** — connected to the
    Zenoh network, but "No topics found" persisted; it couldn't detect the Foxy
    nodes despite sharing the host network.

    **Attempt 2: Middleware/loopback tweaks** — `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
    `ROS_LOCALHOST_ONLY=1`, multicast on `lo`. Still couldn't maintain a ROS graph.

    **Attempt 3: Build `zenoh-plugin-ros2dds` from source (dds_shm)** — dependency
    conflicts with the robot's `cmake`/Foxy build tools made it impractical.

    **Conclusion:** the Humble Docker + Python `rclpy` bridge bypassed the
    discovery issues while keeping native compatibility with the robot's ROS graph.

---

## 7. Known issue — CycloneDDS interface

On our Go2, ROS failed at startup with:

```
ros2: eth1: does not match an available interface.
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain
```

The fix is to pin CycloneDDS to the real interface (`eth0`) before any ROS
command:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
```

!!! note
    `make robot-ros` (via `robot/ros/bringup_camera.sh`) and the
    `vat-robot.service` unit already export this before launching, so you only
    need it manually for ad-hoc `ros2 topic …` commands. Tip: add it to your
    `~/.bashrc`. Override the interface with `NET_IFACE`.

---

## 8. Troubleshooting

**`insta360_ros_driver`: "No available camera devices found." (process dies, exit 255)**
The driver can't enumerate the camera. Work through:

1. Camera **powered on** and connected by USB; try a different cable/port.
2. **USB Mode = Android** and **Dual-Lens** mode set on the camera (§4).
3. The OS sees it: `lsusb | grep -i arashi` (Arashi Vision = Insta360).
4. The udev node exists: `ls -l /dev/insta`. If missing, (re)run `./setup.sh`
   or the manual udev rule, **replug** the camera, then `sudo chmod 777 /dev/insta`.
5. SDK is the **latest** (post Apr 23 2025) and the `libCameraSDK.so` + headers
   were copied into the driver before building (§3).
6. Sometimes the SDK only enumerates after the camera is replugged **once the
   USB/lens modes are set** — unplug, set modes, replug, relaunch.

The `decoder` and `equirectangular` nodes starting fine while the camera node
dies (as in the logs) is exactly this: no camera → no `/dual_fisheye/...` →
`imu_filter` also "Still waiting for data on /imu/data_raw".

**`make robot-docker` → `run.sh: Permission denied`**
Fixed: the Makefile now calls `bash robot/docker/run.sh`. If you invoke the
script directly, use `bash robot/docker/run.sh <router-ip>` or
`chmod +x robot/docker/run.sh` first.

**`docker logs` → `permission denied ... /var/run/docker.sock`**
Your user isn't in the `docker` group. Use `sudo docker logs -f vat-robot`, or
add yourself: `sudo usermod -aG docker $USER` then log out/in.

**Container logs spam `AMENT_TRACE_SETUP_FILES: unbound variable`**
Old bug: `start.sh` ran `set -u` while sourcing ROS (which isn't `set -u`
clean), so the bridge never started. Fixed — **rebuild the image**
(`make robot-docker` rebuilds) and the spam is gone.

**Bridge container runs but `make test_link` shows the bridge `absent` / 0 Hz**
The bridge can't see the host's ROS graph — almost always a **DDS mismatch**.
The container is now built with `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI` to match
the Go2, so **rebuild** (`make robot-docker`). Then verify, from inside:
`sudo docker exec -it vat-robot bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list'`
should list the Go2 topics. Also confirm the ROS domain matches (the Go2 uses
the default `0`; don't set `ROS_DOMAIN_ID` unless the robot does).

**Topic is bridged + advertised, but the decimator/viewer get 0 frames (`buf=0`)**
A **QoS mismatch**. Camera/sensor topics (`/equirectangular/image`, IMU,
point clouds) are published **BEST_EFFORT**; a default **RELIABLE** subscriber
receives *nothing* from a best-effort publisher (while `ros2 topic hz` "works"
because it adapts QoS). The bridge now **probes each publisher's QoS and matches
it** (best-effort sub is compatible with both), so **rebuild** (`make
robot-docker`). To watch data actually flow, the bridge logs forwarded counts
every 10 s:

```
[forwarded] /equirectangular/image=87
```

If a subscribed topic stays at `0`, it logs `[no data] subscribed to … but
received 0 msgs` — then it's still QoS/DDS/interface/domain, not the camera.

!!! tip "Raw frames are heavy over a VPN — prefer the decimated view"
    `/equirectangular/image` is ~**5.5 MB/frame** (1920×960×3). `make
    test_frames_robot` (raw) pulls that across the link; for normal checks use
    `make test_frames_server` (the **decimated JPEG**, tens of KB). See
    *Performance* below — a robot-local Zenoh router keeps the raw frames from
    crossing the VPN at all.

!!! warning "Custom Unitree types bridge only with their message package"
    The bridge resolves each topic's type with `get_message(type)`. **Standard**
    types — including the camera's `sensor_msgs/Image` (`/equirectangular/image`)
    — work out of the box, so Stage 0/1 (frames) is fine. But **custom** types
    like `unitree_go/msg/SportModeState` (`/sportmodestate`, needed for Stage 2
    body/limbs and the pose fuser's camera-height) won't bridge until the
    `unitree_go` message package is available inside the container. That's a
    follow-up (install/copy the `unitree_go` msgs into the image); the camera POC
    does not need it.

**`ros2 topic list` is empty / `package not found` (on the host)**
Export the CycloneDDS fix (§7) and source the workspace:
`source ~/ros2_ws/install/setup.bash`.

---

## 9. Performance — keep raw frames off the VPN (recommended)

Currently the robot processes (bridge, decimator, fuser) connect **directly to
the cloud router** over the VPN. That means the bridge→decimator hop also goes
over the VPN: the decimator pulls the **raw 5.5 MB equirectangular frame back
from the cloud router** just to compress it locally — ~16 MB/s round-trip that
never needed to leave the robot.

The fix is the standard Zenoh "router-per-site" topology: run a **local Zenoh
router on the robot**, point the robot processes at `tcp/127.0.0.1:7447`
(`ZENOH_CONNECT`), and have that local router `connect` to the cloud router.
Zenoh only forwards keys that have a *remote* subscriber, so:

* `go2/rt/equirectangular/image` (raw, consumed only by the local decimator) →
  **stays on the robot**, never crosses the VPN.
* `go2/prism/camera/frame` (small JPEG, subscribed by the cloud mapping server)
  and `go2/prism/pose` (subscribed by the client) → forwarded over the VPN.

This makes the only things crossing the link the decimated JPEG and the tiny
pose stream. (Not wired up yet — it's a small topology change to `run.sh` +
`vat.env`; ask and it can be added.)
