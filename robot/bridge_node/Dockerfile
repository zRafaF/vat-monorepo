# Use ROS 2 Humble for native Python 3.10 support
FROM ros:humble-ros-base

WORKDIR /app

# Install pip and clean up to keep the image slim
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first to leverage Docker cache
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy the rest of the source code
COPY . .

# Default log level can be overridden at runtime via docker run -e LOG_LEVEL=DEBUG
ENV LOG_LEVEL=INFO
ENV ENABLE_PER_MESSAGE_DEBUG=false

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 dynamic_bridge.py"]