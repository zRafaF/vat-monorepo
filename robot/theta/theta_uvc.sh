#!/bin/bash
# ============================================================================
# VAT robot — RICOH Theta X UVC → v4l2 loopback (run ON the Go2 host)
# ============================================================================
# The Theta X streams 360° (equirectangular, in-camera stitched) over UVC, but
# it isn't a plain webcam: the mainline kernel `uvcvideo` driver enumerates it
# ("Found UVC 1.50 device RICOH THETA X") yet reports "No streaming interface
# found" and exposes NO /dev/videoN capture node. The H.264 stream must be
# pulled in userspace via libuvc-theta. This script decodes it into a standard
# v4l2 loopback device that the container's theta_camera.py reads with OpenCV
# (THETA_DEVICE) — the container's pip OpenCV has V4L but not GStreamer, hence
# the loopback hop.
#
# NOTE on the loopback device number:
#   This robot also has an Intel RealSense, which grabs the low-numbered
#   /dev/video0..N nodes at boot. So the Theta loopback uses a DEDICATED high
#   number (default /dev/video10) to avoid colliding with the RealSense. Keep
#   THETA_DEVICE (vat.env) in sync with VIDEO_NR here.
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
#   * Theta X firmware up to date; put the camera in LIVE STREAMING mode
#     (lsusb must show 05ca:2717, NOT 05ca:0373 which is normal/MTP mode).
#   * Build libuvc-theta (the UVC1.5/H.264 fork) + the gstthetauvc plugin.
#   * v4l2loopback-dkms installed (provides the loopback kernel module).
#   * No stray system libuvc shadowing the fork: `sudo apt purge libuvc-dev`.
#
# Usage:   bash robot/theta/theta_uvc.sh
# Env:
#   THETA_BACKEND     gstthetauvc (default) | loopback
#   VIDEO_NR          v4l2loopback device number (default 10 → /dev/video10)
#   THETA_MODE        2K | 4K (default 2K)
#   CARD_LABEL        v4l2loopback card label (default ThetaUVC)
#   GST_PLUGIN_PATH   dir holding gstthetauvc.so, if not in the system plugin
#                     dir (gstthetauvc backend only)
#   GST_LOOPBACK_BIN  path to the gst_loopback binary
#                     (loopback backend only; default ~/libuvc-theta-sample/gst/gst_loopback)
# ============================================================================
set -euo pipefail

THETA_BACKEND="${THETA_BACKEND:-gstthetauvc}"
VIDEO_NR="${VIDEO_NR:-10}"
THETA_MODE="${THETA_MODE:-2K}"
CARD_LABEL="${CARD_LABEL:-ThetaUVC}"
DEV="/dev/video${VIDEO_NR}"

# ── Pre-flight: camera must be in LIVE STREAMING mode (USB id 05ca:2717) ──────
if ! lsusb 2>/dev/null | grep -qi '05ca:2717'; then
    if lsusb 2>/dev/null | grep -qi '05ca:'; then
        cur="$(lsusb | grep -i '05ca:' | head -1 | grep -oi '05ca:[0-9a-f]*')"
        echo "ERROR: RICOH Theta is connected as '${cur}', not live-streaming mode (05ca:2717)."
        echo "       On the camera, switch to LIVE STREAMING mode (Mode button → 'LIVE'),"
        echo "       wait for lsusb to show 05ca:2717, then re-run."
        echo "       See docs/setup/robot.md → Camera setup (Theta X)."
    else
        echo "ERROR: no RICOH Theta (05ca:*) found on USB. Connect it and enable live streaming."
    fi
    exit 1
fi

# ── Ensure a v4l2loopback device exists at VIDEO_NR and is actually OUR loopback
# (not a real camera like the RealSense, which grabs /dev/video0..N at boot). ──
node_name() { cat "/sys/class/video4linux/video${1}/name" 2>/dev/null || true; }

if [ "$(node_name "$VIDEO_NR")" = "$CARD_LABEL" ]; then
    echo "[theta] reusing existing loopback ${DEV} (label ${CARD_LABEL})"
elif [ -e "$DEV" ]; then
    echo "ERROR: ${DEV} is in use by '$(node_name "$VIDEO_NR")', not the ${CARD_LABEL} loopback."
    echo "       (On this robot the Intel RealSense claims the low-numbered nodes.)"
    echo "       Pick a free number and keep THETA_DEVICE in sync, e.g.:"
    echo "         VIDEO_NR=11 make theta-uvc      # and set THETA_DEVICE=/dev/video11 in vat.env"
    exit 1
else
    echo "[theta] loading v4l2loopback → ${DEV} (label ${CARD_LABEL})"
    # If the module is already loaded but didn't create our node, reload it.
    if lsmod | grep -q '^v4l2loopback'; then
        echo "[theta] v4l2loopback already loaded without ${DEV}; reloading at video_nr=${VIDEO_NR}"
        sudo modprobe -r v4l2loopback || true
    fi
    sudo modprobe v4l2loopback video_nr="${VIDEO_NR}" exclusive_caps=1 \
        card_label="${CARD_LABEL}" || {
        echo "ERROR: could not load v4l2loopback (install v4l2loopback-dkms)."; exit 1; }
    for _ in 1 2 3 4 5; do [ -e "$DEV" ] && break; sleep 0.3; done
    [ -e "$DEV" ] || { echo "ERROR: ${DEV} did not appear after modprobe."; exit 1; }
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
    if [ "$VIDEO_NR" != "1" ]; then
        echo "[theta] NOTE: stock gst_loopback writes to /dev/video1; this script's"
        echo "       VIDEO_NR=${VIDEO_NR} only governs the gstthetauvc backend. Adjust"
        echo "       gst_viewer.c (v4l2sink device=...) if you need a different node."
    fi
    echo "[theta] starting gst_loopback (mode=${THETA_MODE})"
    echo "[theta] leave this running; the container reads THETA_DEVICE accordingly"
    exec "$GST_LOOPBACK_BIN"
    ;;

  *)
    echo "ERROR: unknown THETA_BACKEND='${THETA_BACKEND}' (use 'gstthetauvc' or 'loopback')."
    exit 1
    ;;
esac
