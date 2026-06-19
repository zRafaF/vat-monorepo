
```
py -3.10 -m venv venv-windows
```


# Zenoh for Robotics: Overview & Implementation

Moving from standard IoT protocols (like MQTT) to high-performance robotics (ROS/DDS) often introduces a "broker bottleneck." **Zenoh** is designed to solve this by providing a decentralized, high-throughput, and low-latency data fabric.

## 🚀 Why Zenoh?

In a robotics context, especially when sending heavy data like **Point Clouds** or **Video** to the cloud, Zenoh offers several critical advantages over traditional brokers:

1.  **P2P & Routed Mesh:** Devices can talk directly. A 5MB Point Cloud doesn't have to round-trip through a cloud broker if the server is on the same local network.
2.  **Unified Fabric (ROS + Native):** Use the `zenoh-bridge-ros2dds` to mirror ROS topics transparently, while allowing non-ROS sensors (like high-speed cameras) to stream data natively into the same ecosystem.
3.  **Interest Propagation:** If no one is subscribed to a topic (e.g., `/robot/logs`), the data is **dropped at the source**. It never touches the network interface, saving massive amounts of upload bandwidth.
4.  **Native Fragmentation:** Zenoh automatically handles the chunking and reassembly of large payloads (Lidar scans, images), removing the need for custom "chunking" logic in your code.

---

## 📊 The Big Picture: MQTT vs. Zenoh

| Feature | MQTT | Zenoh |
| :--- | :--- | :--- |
| **Topology** | Centralized Broker (Hub & Spoke) | Decentralized (P2P, Routed, or Mesh) |
| **Efficiency** | High overhead for large data | Ultra-low overhead (5-byte headers) |
| **Data Handling** | Pub/Sub only | Pub/Sub + Query/Reply + Storage |
| **Robotics** | Struggles with Point Clouds/Video | Designed for ROS2/Lidar/High-bandwidth |
| **Network** | Primarily TCP | TCP, UDP, **QUIC**, Serial, Bluetooth |

---

## 🌐 Cloud Deployment & QUIC Support

When deploying a robot over the public internet to a cloud server, Zenoh utilizes a **Router (`zenohd`)** to facilitate connection through NAT and firewalls.

### Why use QUIC?
For internet-facing links (LTE/5G/Starlink), **QUIC** is highly recommended over TCP:
*   **Connection Migration:** If the robot switches WiFi access points or cellular towers and its IP changes, the QUIC session stays alive.
*   **No Head-of-Line Blocking:** Unlike TCP, if one packet is lost in a massive Point Cloud, QUIC allows the rest of the data to keep flowing while the missing piece is re-transmitted.
*   **Performance:** Faster handshakes (0-RTT) for quicker reconnection after signal drops.



### Configuring the Router (Cloud)
Run the Zenoh daemon listening for QUIC connections:
```bash
zenohd --listen quic/0.0.0.0:7447
```

### Configuring the Robot (Client)
Connect via the QUIC protocol in your configuration:
```python
import zenoh

conf = zenoh.Config()
conf.insert_json5("connect/endpoints", '["quic/<CLOUD_IP>:7447"]')
session = zenoh.open(conf)
```

---

## 🛠 Implementation Checklist

### 1. Avoid Data Drops
Zenoh's discovery is asynchronous. To ensure you don't drop the first few packets, use explicit endpoints and verify subscribers before publishing:
```python
pub = session.declare_publisher('robot/telemetry')
while not pub.has_subscribers():
    time.sleep(0.1)
```

### 2. Parallel Computation
You can run multiple independent servers (one for Telemetry, one for Point Clouds). Because of Zenoh's source-side filtering, the robot will only send the specific data requested by each server, fanning it out at the router or via multicast.

### 3. Reliability & Late Joiners
Use a **Zenoh Storage** plugin. This allows a server that connects *after* an error has occurred to "Query" the last $N$ messages from the robot's local cache.

### 4. Security
Always wrap internet-facing connections in **TLS**.
*   **mTLS:** Use mutual TLS certificates to ensure only your specific robots can connect to your cloud router.
*   **ACLs:** Define policies so robots can only publish to their own specific namespaces.

---

## 🤖 Jetson Nano: The Multi-Process "Team" Approach

On the Jetson Nano, Zenoh operates as a layered "team" of processes. This architecture maximizes the hardware capabilities of the Nano (ARM64 + NVENC) while maintaining fault tolerance.

### 1. Layered Architecture
Instead of one massive script, the robot runs three distinct layers:

*   **Layer A: ROS 2 Nodes (DDS):** Your standard navigation and sensor nodes. They communicate locally using DDS.
*   **Layer B: Zenoh Bridge (`zenoh-bridge-ros2dds`):** A standalone Rust binary that "listens" to the local DDS chatter and mirrors selected topics (telemetry, point clouds) to the cloud via QUIC.
*   **Layer C: Video Encoder (Python + GStreamer):** A dedicated script that captures camera frames and uses the Jetson’s **NVENC** hardware encoder to compress video before sending it via the Zenoh Python API.



### 2. Hardware Acceleration with GStreamer
To keep CPU usage low on the Jetson Nano, the Video Encoder script should leverage the onboard hardware. 

**Recommended GStreamer Pipeline:**
```text
nvarguscamerasrc ! nvv4l2h264enc ! h264parse ! video/x-h264,stream-format=byte-stream ! appsink
```
The resulting byte-stream is then pushed to Zenoh using:
```python
# Minimal example within your video script
pub_video = session.declare_publisher("robot/video/h264")
pub_video.put(frame_bytes)
```

### 3. Why Multi-Process?
1.  **Fault Tolerance:** If the video script crashes due to a camera error, the Zenoh Bridge remains active, ensuring you never lose telemetry or the ability to send emergency stop commands.
2.  **Concurrency:** Each process manages its own resources. Zenoh efficiently multiplexes these different streams over a single QUIC connection to your cloud router.
3.  **Isolation:** You can update your video processing logic without touching the stable ROS-to-DDS bridge.

---

### A Quick Tip for the Jetson Nano:
Since the Nano's CPU can be a bottleneck, always run the Zenoh Bridge with an **allow list**. This prevents the bridge from "intercepting" internal ROS 2 chatter that you don't need in the cloud, keeping the overhead minimal.

```bash
./zenoh-bridge-ros2dds -e quic/<CLOUD_IP>:7447 --allow "/robot/telemetry|/robot/pc"
```

# Documentation

To run the Mkdocs documentation server, use the following command in the root of the repository:

```bash
mkdocs serve
```