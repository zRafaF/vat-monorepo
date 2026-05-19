# Robot setup

## Bridge node

To bridge ROS 2 topics from the robot to the remote Zenoh network, we utilize a dynamic bridge script. Because the latest Zenoh features require Python 3.10+ and our robot is locked to ROS Foxy (Python 3.8), we encapsulate the bridge inside a **ROS Humble Docker container**. This allows the bridge to run in a modern environment while communicating with the host robot via the shared network stack.

### Key Features

* **Dynamic Discovery:** The bridge automatically polls the ROS graph every 2 seconds to find new topics.
* **Smart Routing (`MatchingListener`):** To save bandwidth, the bridge only creates ROS subscriptions when a remote Zenoh client is actively listening. It automatically stops ROS subscriptions when clients disconnect.
* **Liveliness Monitoring:** The bridge broadcasts a "heartbeat" token. If the robot loses connection, the server can detect the drop immediately via a Zenoh Liveliness Subscriber.

---

### Deployment Instructions

#### 1. Build the Image

Run this command on the Jetson board to build the bridge container. It includes the official Zenoh library and the necessary ROS Humble components.

```bash
docker build -t zenoh-ros-bridge:latest .
```

#### 2. Run the Container

The container requires `--net=host` to "see" the Foxy nodes on the robot. Use the `-e` flags to configure your specific environment.

```bash
docker run -d \
  --name zenoh_bridge \
  --net=host \
  --restart unless-stopped \
  -e ROBOT_NAME="jetson_robot" \
  -e ZENOH_CONNECT="tcp/100.125.156.19:7447" \
  -e LOG_LEVEL="INFO" \
  zenoh-ros-bridge:latest
```

#### 3. Monitoring

To verify the bridge is discovering topics and activating routes based on demand, use the logs:

```bash
docker logs -f zenoh_bridge
```

---

### Technical Details

#### Configuration Variables

| Variable        | Description                                                         | Default              |
| :-------------- | :------------------------------------------------------------------ | :------------------- |
| `ROBOT_NAME`    | The prefix used for all Zenoh keys (e.g., `jetson_robot/rt/topic`). | `my_robot`           |
| `ZENOH_CONNECT` | The endpoint of the remote Zenoh router.                            | `tcp/127.0.0.1:7447` |
| `LOG_LEVEL`     | Verbosity of the ROS 2 and Zenoh logs (`DEBUG`, `INFO`, `WARN`).    | `INFO`               |

---

### Experimental History: Attempted Bridge Solutions

??? failure "[Experimental History] Attempted Bridge Solutions"
    We explored several official Zenoh-ROS 2 integration paths before settling on the current Python implementation. Below is a summary of what was attempted and the outcomes observed.
    
    ---

    **Attempt 1: Official Standalone Bridge (`zenoh-bridge-ros2dds`)**

    We attempted to run the pre-compiled `eclipse/zenoh-bridge-ros2dds` container. 
    *   **Setup:** Pointed the bridge to the remote router using the `-e` flag.
    *   **Result:** The bridge successfully connected to the Zenoh network, but the "No topics found" error persisted during discovery.
    *   **Observed Behavior:** The bridge was unable to detect active ROS Foxy nodes despite sharing the host network.

    ---

    **Attempt 2: Middleware & Loopback Configurations**
    
    We attempted to resolve the discovery issues by adjusting the DDS middleware environment.
    *   **Configurations tried:** 
        *   Forcing `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` inside the container.
        *   Setting `ROS_LOCALHOST_ONLY=1` to force local discovery.
        *   Manually enabling multicast on the loopback interface (`lo`).
    *   **Result:** Even with these configurations, the bridge could not reliably maintain a ROS graph of the Foxy nodes.

    ---

    **Attempt 3: Building from Source with Shared Memory**
    
    We attempted to build the `zenoh-plugin-ros2dds` from source to enable the `dds_shm` feature.
    *   **Setup:** Attempted to use the robot's native environment to ensure direct shared memory access.
    *   **Result:** This led to significant dependency conflicts with the robot's default `cmake` version and ROS Foxy build tools, making the maintenance of a source-built bridge impractical for this hardware.

    ---

    **Conclusion**
    
    The current **Humble Docker + Python Bridge** was chosen because it successfully bypassed these discovery issues. Since it utilizes the standard `rclpy` library, it maintains native compatibility with the robot's ROS graph while still leveraging Zenoh for long-distance transport.

## RGB-D Camera Setup

To start the RGB-D camera, you can run the following command on the robot:

```bash
roslaunch realsense2_camera rs_camera.launch
```

## Known Issues

For some reason on our unity of the go2 we were getting the following error when trying to first use ROS.

```
1779225848.796323 [0]       ros2: eth1: does not match an available interface.
[ERROR] [1779225848.796435099] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error

>>> [rcutils|error_handling.c:108] rcutils_set_error_state()
This error state is being overwritten:

  'error not set, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/node.c:276'

with this new error message:

  'rcl node's rmw handle is invalid, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/node.c:428'

rcutils_reset_error() should be called after error handling to avoid this.
<<<
[ERROR] [1779225848.796612987] [rcl]: Failed to fini publisher for node: 1
Unknown error creating node: rcl node's rmw handle is invalid, at /tmp/binarydeb/ros-foxy-rcl-1.1.14/src/rcl/node.c:428
```

I didn't try to fix it just used a workaround by running the follwing command before running any ROS commands:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
```