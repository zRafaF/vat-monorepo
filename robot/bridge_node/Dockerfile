# Use ROS 2 Humble for native Python 3.10 support
FROM ros:humble-ros-base

WORKDIR /app

# Install pip and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy source code
COPY dynamic_bridge.py .

# Environment Defaults
ENV ROBOT_NAME="my_robot"
ENV ZENOH_CONNECT="tcp/100.125.156.19:7447"
ENV LOG_LEVEL="INFO"

# Run the bridge
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 dynamic_bridge.py"]