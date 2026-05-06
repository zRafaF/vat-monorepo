# Use ROS 2 Humble as the base for a full ROS environment
FROM ros:humble-ros-base

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install dependencies to add the Zenoh repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# 2. Add the Eclipse Zenoh private repository
RUN curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null

# 3. Install the standalone Zenoh ROS 2 bridge[cite: 14]
RUN apt-get update && apt-get install -y \
    zenoh-bridge-ros2dds \
    && rm -rf /var/lib/apt/lists/*

# 4. Environment Variables
# Ensure this matches your Robot's Domain ID (Foxy default is 0)[cite: 14]
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Default Router IP (overridden at runtime)
ENV ZENOH_ROUTER_IP="100.125.156.19"

# 5. Start the official bridge[cite: 14]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && zenoh-bridge-ros2dds -e tcp/${ZENOH_ROUTER_IP}:7447"]