# Use ROS 2 Humble which natively uses Python 3.10
FROM ros:humble-ros-base

# Set the working directory
WORKDIR /app

# Install Python pip and any necessary build tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install the latest Zenoh Python library (works perfectly on Python 3.10)
RUN pip3 install eclipse-zenoh

# Copy your bridge script into the container
COPY robot_bridge.py /app/robot_bridge.py

# When the container starts, source the ROS environment and run the script
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /app/robot_bridge.py"]