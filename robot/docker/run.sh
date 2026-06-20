#!/bin/bash
# VAT robot container — build & run helper (no docker-compose needed).
# Run from the REPO ROOT:  bash robot/docker/run.sh [SERVER_IP]
#
# Env overrides (all optional):
#   ROBOT_NAME      (default go2)
#   ZENOH_CONNECT   (default tcp/<SERVER_IP|127.0.0.1>:7447)
#   THROTTLE_FPS WINDOW_SIZE JPEG_QUALITY CAMERA_FPS PUBLISH_HZ LOSSLESS
#   STICK_OFFSET_X STICK_OFFSET_Y STICK_OFFSET_Z FALLBACK_BODY_HEIGHT
#   THETA_DEVICE        v4l2 device for the Theta UVC stream (default /dev/video0)
#   THETA_MODE          2K | 4K   (default 2K)
#   THETA_GST_PIPELINE  full GStreamer pipeline (advanced; needs GStreamer OpenCV)
set -euo pipefail

SERVER_IP="${1:-127.0.0.1}"
IMAGE="vat-robot-docker"
NAME="vat-robot"

ROBOT_NAME="${ROBOT_NAME:-go2}"
ZENOH_CONNECT="${ZENOH_CONNECT:-tcp/${SERVER_IP}:7447}"
THETA_DEVICE="${THETA_DEVICE:-/dev/video0}"

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
    echo "[run]          (libuvc-theta gst_loopback) — see docs/setup/robot.md."
fi

echo "[run] building ${IMAGE} (context=${REPO_ROOT})"
docker build -f robot/docker/Dockerfile -t "${IMAGE}" .

echo "[run] (re)starting container ${NAME} → ${ZENOH_CONNECT}"
docker rm -f "${NAME}" 2>/dev/null || true
docker run -d --name "${NAME}" --restart unless-stopped \
    --network host --ipc host "${DEVICE_ARGS[@]}" \
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
    -e PUBLISH_HZ="${PUBLISH_HZ:-50.0}" \
    -e STICK_OFFSET_X="${STICK_OFFSET_X:--0.20}" \
    -e STICK_OFFSET_Y="${STICK_OFFSET_Y:-0.0}" \
    -e STICK_OFFSET_Z="${STICK_OFFSET_Z:-0.55}" \
    -e FALLBACK_BODY_HEIGHT="${FALLBACK_BODY_HEIGHT:-0.30}" \
    "${IMAGE}"

echo "[run] done.  logs:  docker logs -f ${NAME}"
