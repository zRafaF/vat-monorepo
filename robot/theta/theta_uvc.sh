#!/bin/bash
# ============================================================================
# VAT robot — RICOH Theta X UVC → v4l2 loopback (run ON the Go2 host)
# ============================================================================
# The Theta X streams 360° (equirectangular, in-camera stitched) over UVC, but
# it isn't a plain webcam — it needs libuvc-theta to decode. This script uses
# the libuvc-theta-sample `gst_loopback` to decode the stream and expose it as a
# standard v4l2 device (e.g. /dev/video0), which the container's theta_camera.py
# then reads with OpenCV (THETA_DEVICE).
#
# One-time prerequisites (see docs/setup/robot.md):
#   * Theta X firmware up to date; put the camera in LIVE STREAMING mode.
#   * Build libuvc-theta + libuvc-theta-sample (provides `gst_loopback`).
#   * v4l2loopback-dkms installed (provides the loopback kernel module).
#
# Usage:   bash robot/theta/theta_uvc.sh
# Env:
#   GST_LOOPBACK_BIN  path to the gst_loopback binary
#                     (default ~/libuvc-theta-sample/gst/gst_loopback)
#   VIDEO_NR          v4l2loopback device number (default 0 → /dev/video0)
#   THETA_MODE        2K | 4K (passed to gst_loopback, default 2K)
# ============================================================================
set -euo pipefail

GST_LOOPBACK_BIN="${GST_LOOPBACK_BIN:-$HOME/libuvc-theta-sample/gst/gst_loopback}"
VIDEO_NR="${VIDEO_NR:-0}"
THETA_MODE="${THETA_MODE:-2K}"

if [ ! -x "$GST_LOOPBACK_BIN" ]; then
    echo "ERROR: gst_loopback not found at '$GST_LOOPBACK_BIN'."
    echo "       Build libuvc-theta-sample, or set GST_LOOPBACK_BIN."
    echo "       See docs/setup/robot.md → Camera setup (Theta X)."
    exit 1
fi

# Ensure the loopback device exists.
if [ ! -e "/dev/video${VIDEO_NR}" ]; then
    echo "[theta] loading v4l2loopback → /dev/video${VIDEO_NR}"
    sudo modprobe v4l2loopback video_nr="${VIDEO_NR}" exclusive_caps=1 \
        card_label="ThetaUVC" || {
        echo "ERROR: could not load v4l2loopback (install v4l2loopback-dkms)."; exit 1; }
fi

echo "[theta] starting gst_loopback (mode=${THETA_MODE}) → /dev/video${VIDEO_NR}"
echo "[theta] leave this running; the container reads THETA_DEVICE=/dev/video${VIDEO_NR}"
exec "$GST_LOOPBACK_BIN"
