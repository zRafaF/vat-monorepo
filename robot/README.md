# Setting up the Insta360 ROS 2 Driver (In-Depth Guide)

This guide provides a comprehensive walkthrough for setting up the Insta360 ROS 2 driver on a Unitree Go2 robot (Ubuntu 20.04 / ROS 2 Foxy) using a standard workspace

---

## 1. Environment and Workspace Setup
First, enable the robot's base ROS 2 system and create a clean development workspace.
```bash
# Enable the Unitree ROS 2 environment
source ~/unitree_ros2/setup.sh

# Create the standard workspace structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

## 2. Install Dependencies (Foxy Source Build)
On Ubuntu 20.04, ROS 2 Foxy binaries are often missing or return 404 errors via `apt`. You must build key dependencies from source to ensure `rosdep` can resolve them.

```bash
cd ~/ros2_ws/src

# 1. Clone the Foxy-compatible branch of the camera manager
git clone [https://github.com/ros-perception/image_common.git](https://github.com/ros-perception/image_common.git) -b foxy

# 2. Clone the IMU tools (required for imu_filter_madgwick)
git clone [https://github.com/ccny-ros-pkg/imu_tools.git](https://github.com/ccny-ros-pkg/imu_tools.git) -b foxy

# 3. Navigate to workspace root to install remaining system dependencies
cd ~/ros2_ws
rosdep update

# Use --skip-keys to prevent rosdep from trying to install the packages we just cloned via apt
rosdep install --from-paths src --ignore-src -r -y --skip-keys "imu_tools imu_filter_madgwick camera_info_manager"
```

---

## 3. Acquire the Insta360 SDK
You will need your Insta360 camera's SDK; you must have an account and request access from the [Insta360 SDK Portal](https://www.insta360.com/sdk/record).

![Picture of the download page](assets/image.png)

> [!TIP]
> You can right-click on the download button and select "Copy Link Address" to get the direct download link for the SDK, which you can use in the terminal with `wget` or `curl` to download it directly to your robot. It should look something like this: `https://wassets.insta360.com/common/<my_key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip`

```bash
cd ~
# Use your direct link here
curl -O [https://wassets.insta360.com/common/](https://wassets.insta360.com/common/)<my_key>/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip

# Extract and clean up zip
unzip Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip
rm Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip

cd Linux_CameraSDK-2.1.1_MediaSDK-3.1.1

# Pick the appropriate SDK for your system. 
# For Unitree Go2 (ARM64), use the jetson-linux tarball:
tar -xzf CameraSDK-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu.tar.gz
```

---

## 4. Install the Driver Source
Clone the driver and move the SDK files into the workspace directories.

```bash
# Clone the driver into your workspace
cd ~/ros2_ws/src
git clone -b humble [https://github.com/ai4ce/insta360_ros_driver](https://github.com/ai4ce/insta360_ros_driver)

# Copy SDK Headers (Note: directory names may vary slightly based on SDK version)
cp -r ~/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/CameraSDK-20251105_112855-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu/include/* ~/ros2_ws/src/insta360_ros_driver/include/

# Copy SDK Libraries
cp ~/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1/CameraSDK-20251105_112855-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu/lib/libCameraSDK.so ~/ros2_ws/src/insta360_ros_driver/lib/

# Clean up temp SDK files
cd ~
rm -rf Linux_CameraSDK-2.1.1_MediaSDK-3.1.1
```

---

## 5. Build the Final Driver
Compile the workspace. We use a CMake policy fix to ensure compatibility between Foxy and modern build tools.

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --allow-overriding image_transport
source install/setup.bash
```

> **Verification:** Run `ros2 pkg prefix camera_info_manager`. It should return a path inside `~/ros2_ws/install`.

---

## 6. Hardware Configuration
Before launching, the camera and system permissions must be configured.

### Camera Settings
1. **USB Mode:** Swipe down on the camera screen, go to **Settings > General**, and set USB Mode to **Android**.
2. **Lens Mode:** Ensure the camera is set to **Dual-Lens** mode.

### System Permissions (Udev Rules)
Run the automated setup script or apply the rules manually to grant the driver USB access. (Note: Remember to have the camera on and connected when applying these rules, as the device node must be created for permissions to apply correctly.)

```bash
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh

# Manual fallback if setup.sh fails:
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

---


## 7. Usage
**IMPORTANT:** Every time you open a new terminal, you must source both the Unitree environment and your local workspace.


```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash

# Launch the camera driver
ros2 launch insta360_ros_driver bringup.launch.xml
```


# Seting up Zenoh ROS bridge for Cloud Connectivity

This section provides a detailed guide for configuring the Zenoh ROS bridge to enable cloud connectivity for your robot. The Zenoh bridge allows you to seamlessly connect your robot's ROS 2 topics to a cloud-based router using the QUIC protocol. More info can be found in the [Zenoh ROS 2 DDS Plugin](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).


> As we are using an older version of ubuntu (20.04) on the Jetson Nano, we will need to build the `zenoh-bridge-ros2dds` plugin from source, as the pre-built binaries may not be compatible with our system.

``` bash
cd ~/ros2_ws/src
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds

cd ..
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select zenoh_bridge_ros2dds --cmake-args -DCMAKE_BUILD_TYPE=Release
```