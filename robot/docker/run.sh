#!/bin/bash
# VAT robot container — build & run helper (no docker-compose needed).
# Run from the REPO ROOT:  bash robot/docker/run.sh [SERVER_IP]
#
# Env overrides (all optional):
#   ROBOT_NAME      (default go2)
#   ZENOH_CONNECT   (default tcp/<SERVER_IP|127.0.0.1>:7447)
#   THROTTLE_FPS WINDOW_SIZE JPEG_QUALITY CAMERA_FPS PUBLISH_HZ LOSSLESS
#   STICK_OFFSET_X STICK_OFFSET_Y STICK_OFFSET_Z FALLBACK_BODY_HEIGHT
#   THETA_DEVICE        v4l2 device for the Theta UVC stream (default /dev/video10)
#   THETA_MODE          2K | 4K   (default 2K)
#   THETA_GST_PIPELINE  full GStreamer pipeline (advanced; needs GStreamer OpenCV)
set -euo pipefail

SERVER_IP="${1:-127.0.0.1}"
IMAGE="vat-robot-docker"
NAME="vat-robot"

ROBOT_NAME="${ROBOT_NAME:-go2}"
ZENOH_CONNECT="${ZENOH_CONNECT:-tcp/${SERVER_IP}:7447}"
THETA_DEVICE="${THETA_DEVICE:-/dev/video10}"

# Build from repo root (this script's grandparent) so /common is in context.
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

# Pass the Theta video device through, if it exists (skip for GStreamer mode).
DEVICE_ARGS=()
if [ -z "${THETA_GST_PIPELINE:-}" ] && [ -e "${THETA_DEVICE}" ]; then
    DEVICE_ARGS=(--device "${THETA_DEVICE}")
    echo "[run] passing camera device ${THETA_DEVICE} into the container"
elif [ -z "${THETA_GST_PIPELINE:-}" ]; then
    echo "[run] WARNING: ${THETA_DEVICE} not found. Start the Theta UVC source first"
    echo "[run]          (make theta-uvc → gstthetauvc) — see docs/setup/robot.md."
fi

# Full-res archive: bind-mount a host dir so the rolling archive survives
# container restarts and doesn't bloat the container's writable layer.
VOL_ARGS=()
if [ "${ARCHIVE_ENABLE:-true}" = "true" ]; then
    ARCHIVE_DIR_HOST="${ARCHIVE_DIR_HOST:-/data/vat-archive}"
    mkdir -p "${ARCHIVE_DIR_HOST}" 2>/dev/null || \
        echo "[run] WARNING: could not create ${ARCHIVE_DIR_HOST} (archive may fail)"
    VOL_ARGS=(--volume "${ARCHIVE_DIR_HOST}:${ARCHIVE_DIR:-/archive}")
    echo "[run] archive: ${ARCHIVE_DIR_HOST} → ${ARCHIVE_DIR:-/archive} (cap ${ARCHIVE_MAX_BYTES:-10GB})"
fi

echo "[run] building ${IMAGE} (context=${REPO_ROOT})"
docker build -f robot/docker/Dockerfile \
    --build-arg WITH_REALSENSE="${WITH_REALSENSE:-0}" \
    -t "${IMAGE}" .

echo "[run] (re)starting container ${NAME} → ${ZENOH_CONNECT}"
docker rm -f "${NAME}" 2>/dev/null || true
docker run -d --name "${NAME}" --restart unless-stopped \
    --network host --ipc host "${DEVICE_ARGS[@]}" "${VOL_ARGS[@]}" \
    -e ROBOT_NAME="${ROBOT_NAME}" \
    -e ZENOH_CONNECT="${ZENOH_CONNECT}" \
    -e NET_IFACE="${NET_IFACE:-eth0}" \
    -e THROTTLE_FPS="${THROTTLE_FPS:-3.0}" \
    -e WINDOW_SIZE="${WINDOW_SIZE:-5}" \
    -e JPEG_QUALITY="${JPEG_QUALITY:-85}" \
    -e CAMERA_FPS="${CAMERA_FPS:-30.0}" \
    -e LOSSLESS="${LOSSLESS:-}" \
    -e THETA_DEVICE="${THETA_DEVICE}" \
    -e THETA_MODE="${THETA_MODE:-2K}" \
    -e THETA_GST_PIPELINE="${THETA_GST_PIPELINE:-}" \
    -e TRANSMIT_WIDTH="${TRANSMIT_WIDTH:-0}" \
    -e TRANSMIT_HEIGHT="${TRANSMIT_HEIGHT:-0}" \
    -e ARCHIVE_ENABLE="${ARCHIVE_ENABLE:-true}" \
    -e ARCHIVE_DIR="${ARCHIVE_DIR:-/archive}" \
    -e ARCHIVE_MAX_BYTES="${ARCHIVE_MAX_BYTES:-10GB}" \
    -e ARCHIVE_JPEG_QUALITY="${ARCHIVE_JPEG_QUALITY:-92}" \
    -e PUBLISH_HZ="${PUBLISH_HZ:-50.0}" \
    -e SPORT_TOPIC="${SPORT_TOPIC:-lf/sportmodestate}" \
    -e WHEEL_RADIUS="${WHEEL_RADIUS:-0.085}" \
    -e TELEOP_RATE_HZ="${TELEOP_RATE_HZ:-20.0}" \
    -e TELEOP_TIMEOUT_S="${TELEOP_TIMEOUT_S:-0.3}" \
    -e TELEOP_MAX_VX="${TELEOP_MAX_VX:-0.3}" \
    -e TELEOP_MAX_VY="${TELEOP_MAX_VY:-0.2}" \
    -e TELEOP_MAX_VYAW="${TELEOP_MAX_VYAW:-0.6}" \
    -e STICK_OFFSET_X="${STICK_OFFSET_X:--0.20}" \
    -e STICK_OFFSET_Y="${STICK_OFFSET_Y:-0.0}" \
    -e STICK_OFFSET_Z="${STICK_OFFSET_Z:-0.55}" \
    -e FALLBACK_BODY_HEIGHT="${FALLBACK_BODY_HEIGHT:-0.30}" \
    -e PERISCOPE_ENABLE="${PERISCOPE_ENABLE:-1}" \
    -e PERISCOPE_CODEC="${PERISCOPE_CODEC:-mjpeg}" \
    -e PERISCOPE_RES="${PERISCOPE_RES:-480}" \
    -e PERISCOPE_ASPECT="${PERISCOPE_ASPECT:-1:1}" \
    -e PERISCOPE_MAX_FOV="${PERISCOPE_MAX_FOV:-130}" \
    -e PERISCOPE_MIN_FOV="${PERISCOPE_MIN_FOV:-20}" \
    -e PERISCOPE_FPS="${PERISCOPE_FPS:-15}" \
    -e PERISCOPE_FPS_DYNAMIC="${PERISCOPE_FPS_DYNAMIC:-1}" \
    -e PERISCOPE_FPS_MIN="${PERISCOPE_FPS_MIN:-8}" \
    -e PERISCOPE_FPS_MAX="${PERISCOPE_FPS_MAX:-24}" \
    -e PERISCOPE_BITRATE="${PERISCOPE_BITRATE:-1500000}" \
    -e PERISCOPE_JPEG_QUALITY="${PERISCOPE_JPEG_QUALITY:-80}" \
    -e PERISCOPE_IDR_INTERVAL_S="${PERISCOPE_IDR_INTERVAL_S:-2.0}" \
    -e PERISCOPE_VIEWER_TIMEOUT_S="${PERISCOPE_VIEWER_TIMEOUT_S:-5.0}" \
    -e PERISCOPE_SO_SNDBUF="${PERISCOPE_SO_SNDBUF:-262144}" \
    -e RGBD_ENABLE="${RGBD_ENABLE:-1}" \
    -e RGBD_DEFAULT_KIND="${RGBD_DEFAULT_KIND:-depth}" \
    -e RGBD_FPS="${RGBD_FPS:-20}" \
    -e RGBD_MAX_RANGE_M="${RGBD_MAX_RANGE_M:-4.0}" \
    -e RGBD_SEND_WIDTH="${RGBD_SEND_WIDTH:-424}" \
    -e RGBD_JPEG_QUALITY="${RGBD_JPEG_QUALITY:-70}" \
    -e RGBD_VIEWER_TIMEOUT_S="${RGBD_VIEWER_TIMEOUT_S:-5.0}" \
    -e RGBD_SO_SNDBUF="${RGBD_SO_SNDBUF:-262144}" \
    -e RGBD_DEPTH_TOPIC="${RGBD_DEPTH_TOPIC:-/camera/camera/depth/image_rect_raw}" \
    -e RGBD_COLOR_TOPIC="${RGBD_COLOR_TOPIC:-/camera/camera/color/image_raw}" \
    -e RGBD_DEPTH_INFO_TOPIC="${RGBD_DEPTH_INFO_TOPIC:-/camera/camera/depth/camera_info}" \
    -e RGBD_COLOR_INFO_TOPIC="${RGBD_COLOR_INFO_TOPIC:-/camera/camera/color/camera_info}" \
    "${IMAGE}"

echo "[run] done.  logs:  docker logs -f ${NAME}"
