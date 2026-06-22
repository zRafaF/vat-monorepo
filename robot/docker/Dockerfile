# VAT Robot Docker container
# ===========================
# Runs alongside the Go2 ROS Foxy stack on the Jetson. Hosts three processes:
#   - dynamic_bridge.py   ROS2 topics → Zenoh (CDR, for odometry)  [ROS node]
#   - theta_camera.py     RICOH Theta X UVC → best-of-window → server [Zenoh]
#   - pose_fuser.py       authoritative robot pose → client          [Zenoh]
# The container uses ROS Humble (Python 3.10) for modern Zenoh; the *host*
# stays on Foxy (Python 3.8). DDS is wire-compatible across the two.
#
# The camera no longer goes through ROS — theta_camera captures the Theta over
# UVC directly with OpenCV (see docs/setup/robot.md). The Insta360 ROS driver
# was retired (docs/archive/insta360.md).
#
# IMPORTANT — the Jetson has NO docker-compose, only `docker`.
# Build from the REPO ROOT so the shared protocol module is in context:
#
#   docker build -f robot/docker/Dockerfile -t vat-robot-docker .
#
# Run (see robot/docker/run.sh or robot/systemd/vat-robot-docker.service). The
# container needs the Theta video device, e.g.:
#   docker run -d --name vat-robot --restart unless-stopped \
#     --network host --ipc host --device /dev/video10 \
#     -e ROBOT_NAME=go2 -e ZENOH_CONNECT=tcp/SERVER_IP:7447 \
#     -e THETA_DEVICE=/dev/video10 vat-robot-docker

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    iproute2 \
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

# ── unitree_go / unitree_api messages (so the bridge can forward Go2 topics) ──
# /sportmodestate is unitree_go/msg/SportModeState — a CUSTOM type. rclpy can
# only subscribe to a type it can load, AND because the Go2's Foxy types are
# FINAL extensibility, CycloneDDS only DELIVERS samples when the subscriber's
# type matches the publisher's EXACTLY. So we build the REAL, complete message
# packages from unitree_ros2 — a hand-written subset connects but receives 0
# msgs. If your firmware's layout still differs (older defs with extra fields
# like path_point), pin the matching version:
#   UNITREE_ROS2_REF=<branch|tag> sudo make robot-docker
# Best-effort: if the clone/build fails the image still runs and just skips
# /sportmodestate (Stage 2); standard-typed topics are unaffected.
ARG UNITREE_ROS2_REF=master
RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-colcon-common-extensions \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-dds-idl \
        build-essential cmake \
    && rm -rf /var/lib/apt/lists/* || true
# Only unitree_go is needed — it carries SportModeState (odometry/limbs), the
# single custom type the fuser decodes. unitree_api (the /api/sport/* Request/
# Response types) is NOT used by VAT and is intentionally not built. unitree_go
# does not depend on unitree_api, so it builds standalone.
# NOTE: unitree_go's CMake does find_package(rosidl_generator_dds_idl) — hence
# ros-humble-rosidl-generator-dds-idl above. Without it the build fails and
# /sportmodestate silently never forwards. We therefore HARD-FAIL the build if
# the type can't be imported afterwards, so a broken image never ships quietly.
RUN git clone --depth 1 --branch "${UNITREE_ROS2_REF}" \
        https://github.com/unitreerobotics/unitree_ros2 /tmp/unitree_ros2 \
    && mkdir -p /opt/unitree_ws/src \
    && cp -r /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go \
            /opt/unitree_ws/src/ \
    && rm -rf /tmp/unitree_ros2 \
    && bash -lc 'source /opt/ros/humble/setup.bash && cd /opt/unitree_ws && \
        colcon build --packages-select unitree_go' \
    && bash -lc 'source /opt/ros/humble/setup.bash && \
        source /opt/unitree_ws/install/setup.bash && \
        python3 -c "from rosidl_runtime_py.utilities import get_message; \
        get_message(\"unitree_go/msg/SportModeState\"); \
        print(\"[build] unitree_go OK — SportModeState importable\")"'

# Shared wire protocol (single source of truth — lives in repo /common)
COPY common/vat_protocol.py     /app/
# Robot-side processes
COPY robot/docker/dynamic_bridge.py   /app/
COPY robot/docker/theta_camera.py     /app/
COPY robot/docker/frame_archive.py    /app/
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
    SPORT_TOPIC="lf/sportmodestate" \
    ROBOT_MODEL="selfie_stick" \
    STICK_OFFSET_X="-0.20" \
    STICK_OFFSET_Y="0.0" \
    STICK_OFFSET_Z="0.55" \
    FALLBACK_BODY_HEIGHT="0.30" \
    THETA_MODE="2K" \
    THETA_DEVICE="/dev/video10"

ENTRYPOINT ["/bin/bash", "/app/start.sh"]
