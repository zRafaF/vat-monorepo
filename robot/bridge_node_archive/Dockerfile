# Use ROS 2 Humble for native Python 3.10 support
FROM ros:humble-ros-base

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# Install system dependencies and ROS-specific python tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-runtime-py \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements (now only contains zenoh)[cite: 4]
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy source code
COPY dynamic_bridge.py .

# Environment Defaults
ENV ROBOT_NAME="my_robot"
ENV ZENOH_CONNECT="tcp/100.125.156.19:7447"
ENV LOG_LEVEL="INFO"

# Default to bash to ensure setup.bash is sourced correctly
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 dynamic_bridge.py"]