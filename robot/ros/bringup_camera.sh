#!/bin/bash
# ============================================================================
# VAT robot — camera bringup (host ROS, run ON the Go2-W)
# ============================================================================
# Brings up the Insta360 driver in EQUIRECTANGULAR mode (what PRISM needs) and
# applies the CycloneDDS interface fix the Go2 requires for DDS discovery.
#
# Publishes (insta360_ros_driver):
#   /dual_fisheye/image[/compressed]   /equirectangular/image   /imu/data[_raw]
#
# Prereqs (one-time, see docs/setup/robot.md):
#   * insta360_ros_driver built in ~/ros2_ws  (colcon build --symlink-install)
#   * camera in Dual-Lens mode + USB mode = Android, udev /dev/insta created
#
# Usage:  make robot-ros        (or)   bash robot/ros/bringup_camera.sh
# Env overrides: ROS_DISTRO (default foxy), ROS2_WS (default ~/ros2_ws),
#                NET_IFACE (default eth0), EQUIRECTANGULAR (default true)
# ============================================================================
set -e

ROS_DISTRO="${ROS_DISTRO:-foxy}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
NET_IFACE="${NET_IFACE:-eth0}"
EQUIRECTANGULAR="${EQUIRECTANGULAR:-true}"

# The Go2 ships CycloneDDS pointed at the wrong interface, which breaks
# `ros2` discovery. Pin it to the real interface before any ROS command.
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>"

echo "[bringup] ROS $ROS_DISTRO  ws=$ROS2_WS  iface=$NET_IFACE  equirect=$EQUIRECTANGULAR"

# Base ROS + the workspace that contains insta360_ros_driver.
# (The Go2's own ROS2 graph is already running system-wide.)
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
source "$ROS2_WS/install/setup.bash"

# --- Preflight: the #1 failure is "No available camera devices found." -------
# The SDK enumerates the camera via the /dev/insta udev symlink; it also needs
# USB Mode = Android and Dual-Lens. Warn early (don't abort) so it's obvious.
if command -v lsusb >/dev/null 2>&1 && ! lsusb | grep -qi 'Arashi'; then
    echo "[bringup] WARNING: no Insta360 (Arashi Vision) device seen on USB."
    echo "          → connect/power the camera; set USB Mode = Android + Dual-Lens."
fi
if [ ! -e /dev/insta ]; then
    echo "[bringup] WARNING: /dev/insta is missing — the driver will likely report"
    echo "          'No available camera devices found'. Fix the udev rule:"
    echo "            (cd \"$ROS2_WS/src/insta360_ros_driver\" && ./setup.sh)"
    echo "          then REPLUG the camera and: sudo chmod 777 /dev/insta"
    echo "          (see docs/setup/robot.md → Troubleshooting)"
fi

# Camera node + decoder + equirectangular conversion (+ imu_filter by default).
exec ros2 launch insta360_ros_driver bringup.launch.xml \
    equirectangular:="$EQUIRECTANGULAR"
