# Use ROS 2 Humble for native Python 3.10 support
FROM ros:humble-ros-base

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-runtime-py \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Python deps: zenoh + frame decimator (numpy, opencv, rosbags)
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Source files
COPY dynamic_bridge.py  .
COPY ../frame_decimator.py .
COPY start.sh           .
RUN chmod +x start.sh

# Environment defaults
# Override these at docker run time or in docker-compose.yml
ENV ROBOT_NAME="go2"
ENV ZENOH_CONNECT="tcp/127.0.0.1:7447"
ENV LOG_LEVEL="INFO"
ENV THROTTLE_FPS="3.0"
ENV WINDOW_SIZE="5"
ENV JPEG_QUALITY="85"
ENV CAMERA_FPS="30.0"

# Both processes started by start.sh
ENTRYPOINT ["/bin/bash", "start.sh"]