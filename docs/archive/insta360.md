# Archived — Insta360 camera setup

!!! warning "Archived — this is **not** the current camera"
    VAT moved away from the **Insta360** (via the `insta360_ros_driver`) to the
    **RICOH Theta X over UVC** — see the current [Robot setup](../setup/robot.md).
    This page is kept as a historical record: *why* we moved, and the full
    Insta360 bring-up, in case we revisit it (e.g. a different Insta360 model).

    **Why we moved away.** The Insta360 ROS driver's stream suffered from
    **motion artifacts** we could not resolve. They persisted with **uncompressed**
    frames and even with the **raw dual-fisheye** images — which pointed at the
    driver/node (stitching / rolling-shutter / frame timing), not the transport
    or our pipeline. The **Theta X** does dynamic stitching + zenith correction
    *in-camera* during live streaming and exposes a clean equirectangular **UVC**
    stream (no custom ROS node), so we switched.

    **Where the code lives.** The full Insta360 implementation at the point of
    removal is preserved on the
    **[`insta360` branch](https://github.com/zRafaF/vat-monorepo/tree/insta360)**.
    The current `main` no longer ships `robot/insta360_ros_driver/`, `robot/ros/`,
    `robot/docker/frame_decimator.py`, or the `make robot-ros` flow. The commands
    below are therefore **historical** (they work on the `insta360` branch).

    The Zenoh bridge, CycloneDDS fix, and DDS/QoS notes are *not* Insta360-specific
    and still apply — see the current [Robot setup](../setup/robot.md).

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
git clone https://github.com/ros-perception/image_common.git -b foxy
git clone https://github.com/ccny-ros-pkg/imu_tools.git -b foxy

cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "imu_tools imu_filter_madgwick camera_info_manager"
```

## 2. Acquire the Insta360 SDK

You need an Insta360 account and must request SDK access from the
[Insta360 SDK Portal](https://www.insta360.com/sdk/record).

!!! warning
    Use the **latest** SDK (the driver requires the SDK posted **after April 23,
    2025**). See the driver's
    [install notes](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

```bash
cd ~
curl -O https://wassets.insta360.com/common/<your-key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
rm Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
cd Linux_CameraSDK-2.1.1_MediaSDK-3.1.1
# Go2-W is ARM64 → use the jetson-linux tarball
tar -xzf CameraSDK-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu.tar.gz
```

## 3. Install & build the driver

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver

# Copy SDK headers + library into the driver (dir names vary by SDK version)
cp -r ~/Linux_CameraSDK-*/CameraSDK-*/include/* ~/ros2_ws/src/insta360_ros_driver/include/
cp    ~/Linux_CameraSDK-*/CameraSDK-*/lib/libCameraSDK.so ~/ros2_ws/src/insta360_ros_driver/lib/
cd ~ && rm -rf Linux_CameraSDK-2.1.1_MediaSDK-3.1.1
```

Build (the CMake policy flag keeps Foxy compatible with newer build tools):

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --allow-overriding image_transport
source install/setup.bash
```

## 4. Hardware configuration

1. **USB Mode = Android** — swipe down → Settings → General → USB Mode → Android
   ([Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4)).
2. **Dual-Lens mode** on the camera.

USB permissions (udev `/dev/insta`):

```bash
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
# Manual fallback if setup.sh fails:
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' \
  | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules && sudo udevadm trigger && sudo chmod 777 /dev/insta
```

## 5. Camera bringup (equirectangular)

The driver's `equirectangular` arg **defaults to `false`** — it must be set. On
the `insta360` branch this was wrapped by `make robot-ros`
(`robot/ros/bringup_camera.sh`), which applied the CycloneDDS fix + launched:

```bash
ros2 launch insta360_ros_driver bringup.launch.xml equirectangular:=true
```

!!! warning "It goes quiet after startup — that was normal"
    Success: `Camera opened successfully` → `Live streaming started` →
    `Mapping matrices initialization complete`, then **silence** (no per-frame
    logs). It is streaming — don't Ctrl-C it. A one-off `error info: 400
    [unknown msg code.]` just before "Camera opened successfully" is a benign SDK
    quirk. Published: `/equirectangular/image`, `/dual_fisheye/image[/compressed]`,
    `/imu/data[_raw]`.

## 6. Troubleshooting (Insta360-specific)

**`insta360_ros_driver`: "No available camera devices found." (exit 255)** — the
SDK can't enumerate the camera. Check, in order:

1. Camera powered on + connected by USB (try another cable/port).
2. USB Mode = Android, Dual-Lens mode set (§4).
3. `lsusb | grep -i arashi` (Arashi Vision = Insta360) shows the device.
4. `ls -l /dev/insta` exists; if not, re-run `./setup.sh` or the manual udev
   rule, **replug**, then `sudo chmod 777 /dev/insta`.
5. SDK is the latest (post Apr 23 2025) and `libCameraSDK.so` + headers were
   copied into the driver before building (§3).
6. The SDK often only enumerates after a replug *once the USB/lens modes are
   set* — unplug, set modes, replug, relaunch.

The decoder/equirectangular nodes starting while the camera node dies (and
`imu_filter` "Still waiting for data on /imu/data_raw") is exactly this symptom.

---

## Infrastructure that carried over to the Theta X

The bridge container, **CycloneDDS interface fix**, **DDS/RMW matching**, and
**QoS** handling were developed during the Insta360 era but are camera-agnostic
and remain in use. They now live in the current [Robot setup](../setup/robot.md):

* The ROS↔Zenoh **bridge** still runs (now only for odometry, e.g.
  `/sportmodestate`), in the same Humble container.
* The Go2 needs `CYCLONEDDS_URI` pinned to `eth0`; the container is built with
  `rmw_cyclonedds_cpp` to match the host.
* Sensor topics are **BEST_EFFORT** — the bridge probes and matches publisher QoS.
* Custom `unitree_go` types only bridge if their message package is in the
  container.
