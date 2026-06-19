# VAT Robot Docker container
# ===========================
# Runs alongside the ROS Foxy stack on the Jetson.  Hosts three pure-Zenoh
# helpers + the ROS↔Zenoh bridge:
#   - dynamic_bridge.py   ROS2 topics → Zenoh (CDR)            [ROS node]
#   - frame_decimator.py  best-of-window frames → server      [Zenoh client]
#   - pose_fuser.py       authoritative robot pose → client   [Zenoh client]
# The container uses ROS Humble (Python 3.10) for modern Zenoh; the *host*
# stays on Foxy (Python 3.8).  DDS is wire-compatible across the two.
#
# IMPORTANT — the Jetson has NO docker-compose, only `docker`.
# Build from the REPO ROOT so the shared protocol module is in context:
#
#   docker build -f robot/docker/Dockerfile -t vat-robot-docker .
#
# Run (see robot/docker/run.sh or robot/systemd/vat-robot-docker.service):
#
#   docker run -d --name vat-robot --restart unless-stopped \
#     --network host --ipc host \
#     -e ROBOT_NAME=go2 -e ZENOH_CONNECT=tcp/SERVER_IP:7447 \
#     vat-robot-docker

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-runtime-py \
    ros-humble-rmw-cyclonedds-cpp \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Match the Go2 host's DDS (CycloneDDS) so the bridge can discover ROS topics.
# start.sh sets CYCLONEDDS_URI for the right interface at runtime.
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

COPY robot/docker/requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Shared wire protocol (single source of truth — lives in repo /common)
COPY common/vat_protocol.py     /app/
# Robot-side processes
COPY robot/docker/dynamic_bridge.py   /app/
COPY robot/docker/frame_decimator.py  /app/
COPY robot/docker/pose_fuser.py       /app/
COPY robot/docker/kinematics.py       /app/
COPY robot/docker/start.sh            /app/
RUN chmod +x /app/start.sh

# Defaults — override at runtime with -e flags
ENV ROBOT_NAME="go2" \
    ZENOH_CONNECT="tcp/127.0.0.1:7447" \
    LOG_LEVEL="INFO" \
    THROTTLE_FPS="3.0" \
    WINDOW_SIZE="5" \
    JPEG_QUALITY="85" \
    CAMERA_FPS="30.0" \
    PUBLISH_HZ="50.0" \
    ROBOT_MODEL="selfie_stick" \
    STICK_OFFSET_X="-0.20" \
    STICK_OFFSET_Y="0.0" \
    STICK_OFFSET_Z="0.55" \
    FALLBACK_BODY_HEIGHT="0.30"

ENTRYPOINT ["/bin/bash", "/app/start.sh"]
