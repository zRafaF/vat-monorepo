
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