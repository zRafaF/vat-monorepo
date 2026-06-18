# VAT Robot Docker container
# Runs alongside the ROS Foxy stack on the Jetson.
# Contains: DynamicZenohBridge + FrameDecimator
# Requires Python 3.10+ for modern Zenoh — ROS Foxy only has Python 3.8.
#
# Build & run from the robot/docker/ directory:
#   docker compose up -d --build
#
# Or manually:
#   docker build -t vat-robot-docker .
#   docker run --rm --network host --ipc host \
#     -e ROBOT_NAME=go2 -e ZENOH_CONNECT=tcp/SERVER_IP:7447 \
#     vat-robot-docker

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-runtime-py \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY dynamic_bridge.py  .
COPY frame_decimator.py .
COPY start.sh           .
RUN chmod +x start.sh

# Defaults — override at runtime via docker-compose.yml or -e flags
ENV ROBOT_NAME="go2"
ENV ZENOH_CONNECT="tcp/127.0.0.1:7447"
ENV LOG_LEVEL="INFO"
ENV THROTTLE_FPS="3.0"
ENV WINDOW_SIZE="5"
ENV JPEG_QUALITY="85"
ENV CAMERA_FPS="30.0"

ENTRYPOINT ["/bin/bash", "start.sh"]
