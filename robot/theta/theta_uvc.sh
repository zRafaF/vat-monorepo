#!/bin/bash
# ============================================================================
# VAT robot — RICOH Theta X UVC → v4l2 loopback (run ON the Go2 host)
# ============================================================================
# The Theta X streams 360° (equirectangular, in-camera stitched) over UVC, but
# it isn't a plain webcam: the mainline kernel `uvcvideo` driver enumerates it
# ("Found UVC 1.50 device RICOH THETA X") yet reports "No streaming interface
# found" and exposes NO /dev/videoN capture node. The H.264 stream must be
# pulled in userspace via libuvc-theta. This script decodes it into a standard
# v4l2 loopback device (e.g. /dev/video0) that the container's theta_camera.py
# reads with OpenCV (THETA_DEVICE) — the container's pip OpenCV has V4L but not
# GStreamer, hence the loopback hop.
#
# Two backends (THETA_BACKEND):
#   * gstthetauvc  (DEFAULT) — the `thetauvcsrc` GStreamer element matches the
#     camera by PRODUCT NAME ("RICOH THETA"*), so the Theta X works with no
#     source edits. See https://github.com/nickel110/gstthetauvc
#   * loopback     (FALLBACK) — libuvc-theta-sample's `gst_loopback` binary.
#     NOTE: stock gst_loopback hardcodes product IDs (THETA V 0x2712, Z1 0x2715)
#     and does NOT include the Theta X (0x2717), so it prints "THETA not found"
#     unless you patch thetauvc.c. Only use this if you can't build the plugin.
#
# One-time prerequisites (see docs/setup/robot.md):
#   * Theta X firmware up to date; put the camera in LIVE STREAMING mode.
#   * Build libuvc-theta (the UVC1.5/H.264 fork) + the gstthetauvc plugin.
#   * v4l2loopback-dkms installed (provides the loopback kernel module).
#   * No stray system libuvc shadowing the fork: `sudo apt purge libuvc-dev`.
#
# Usage:   bash robot/theta/theta_uvc.sh
# Env:
#   THETA_BACKEND     gstthetauvc (default) | loopback
#   VIDEO_NR          v4l2loopback device number (default 0 → /dev/video0)
#   THETA_MODE        2K | 4K (default 2K)
#   GST_PLUGIN_PATH   dir holding gstthetauvc.so, if not in the system plugin
#                     dir (gstthetauvc backend only)
#   GST_LOOPBACK_BIN  path to the gst_loopback binary
#                     (loopback backend only; default ~/libuvc-theta-sample/gst/gst_loopback)
# ============================================================================
set -euo pipefail

THETA_BACKEND="${THETA_BACKEND:-gstthetauvc}"
VIDEO_NR="${VIDEO_NR:-0}"
THETA_MODE="${THETA_MODE:-2K}"
DEV="/dev/video${VIDEO_NR}"

# Ensure the loopback device exists (both backends sink into it).
if [ ! -e "$DEV" ]; then
    echo "[theta] loading v4l2loopback → ${DEV}"
    sudo modprobe v4l2loopback video_nr="${VIDEO_NR}" exclusive_caps=1 \
        card_label="ThetaUVC" || {
        echo "ERROR: could not load v4l2loopback (install v4l2loopback-dkms)."; exit 1; }
fi

case "$THETA_BACKEND" in
  gstthetauvc)
    # Confirm the plugin is discoverable before we try to stream.
    if ! gst-inspect-1.0 thetauvcsrc >/dev/null 2>&1; then
        echo "ERROR: GStreamer element 'thetauvcsrc' not found."
        echo "       Build/install the gstthetauvc plugin, or set GST_PLUGIN_PATH"
        echo "       to the directory containing gstthetauvc.so."
        echo "       See docs/setup/robot.md → Camera setup (Theta X)."
        echo "       (Or set THETA_BACKEND=loopback to use a patched gst_loopback.)"
        exit 1
    fi
    echo "[theta] starting gstthetauvc (mode=${THETA_MODE}) → ${DEV}"
    echo "[theta] leave this running; the container reads THETA_DEVICE=${DEV}"
    # thetauvcsrc decodes H.264 → I420 raw → v4l2loopback sink.
    exec gst-launch-1.0 -e thetauvcsrc mode="${THETA_MODE}" \
        ! queue ! h264parse ! decodebin ! videoconvert \
        ! video/x-raw,format=I420 ! identity drop-allocation=true \
        ! v4l2sink device="${DEV}" sync=false
    ;;

  loopback)
    GST_LOOPBACK_BIN="${GST_LOOPBACK_BIN:-$HOME/libuvc-theta-sample/gst/gst_loopback}"
    if [ ! -x "$GST_LOOPBACK_BIN" ]; then
        echo "ERROR: gst_loopback not found at '$GST_LOOPBACK_BIN'."
        echo "       Build libuvc-theta-sample, or set GST_LOOPBACK_BIN."
        echo "       Reminder: stock gst_loopback lacks the Theta X PID (0x2717)"
        echo "       and prints 'THETA not found' unless thetauvc.c is patched."
        echo "       See docs/setup/robot.md → Camera setup (Theta X)."
        exit 1
    fi
    echo "[theta] starting gst_loopback (mode=${THETA_MODE}) → ${DEV}"
    echo "[theta] leave this running; the container reads THETA_DEVICE=${DEV}"
    exec "$GST_LOOPBACK_BIN"
    ;;

  *)
    echo "ERROR: unknown THETA_BACKEND='${THETA_BACKEND}' (use 'gstthetauvc' or 'loopback')."
    exit 1
    ;;
esac
