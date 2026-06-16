# System Architecture

This document outlines the high-level architecture for the Volumetric Asynchronous Teleoperation (VAT) project. The pipeline is designed for remote control of a robot using an advanced XR client, a cloud-based global mapping system, and a local reactive navigation stack on the robot.

## Overview

The VAT architecture is based on a **Hybrid Edge-Cloud paradigm**. To achieve low-latency teleoperation and high-fidelity global mapping simultaneously, we decouple the high-frequency reactive tasks from the computationally heavy global planning tasks. 

There are three main components:

1. **The Client (XR/VR Interface)**
2. **The Cloud (Global Mapping & Planning)**
3. **The Robot (Local Autonomy & Sensors)**

---

## 1. The Client (XR/VR Interface)

The client provides the operator with an intuitive, lag-free teleoperation interface. We prioritize **Tethered PC VR (e.g., Unity running on a PC streaming to Meta Quest via AirLink or Virtual Desktop)**.

### Technology Stack
* **Engine:** **Unity** is the engine of choice. It offers the most robust XR ecosystem, excellent point cloud rendering plugins (such as PCX), and reliable networking libraries (like WebRTC).
* **Python Backend:** A Python sidecar runs alongside the Unity PC app to handle data gathering, heavy networking (e.g., PRISM delta updates), and cloud communication. Unity communicates with this Python layer via gRPC or WebSockets.
* **Why not Native Quest?** The massive PRISM point clouds (50-200MB) require significant memory, vertex throughput, and network deserialization. A laptop CPU/GPU is vastly superior for these tasks compared to the mobile Snapdragon XR2 on the Quest headset. 

### User Experience (UX)
* **Third-Person "Toy Box" View:** First-person VR teleoperation typically induces severe motion sickness due to latency and robot bouncing. We use a "God-Mode" perspective. The PRISM map is represented as a point cloud, and we apply a plane slice on the ceiling so the user views the environment top-down.
* **The "Beacon" (Virtual Monitor):** To compensate for the lack of texture detail in point clouds, the user can steer a virtual "beacon" or raycast. This projects a real-time, high-definition video stream (via WebRTC) onto a virtual screen at the point of interest for detailed inspections.
* **1-Frame RGBD Mesh:** To provide instantaneous collision awareness, the client renders the raw RGBD frame from the robot as a local mesh attached to the front of the 3D robot avatar. This single-frame display is updated at high frequency and acts as a "headlight," allowing the user to react to fast-moving dynamic obstacles before they are integrated into the global PRISM map.

---

## 2. The Cloud (Global Mapping & Planning)

The cloud acts as the heavy-lifter for the global environment context.

* **PRISM Integration:** Receives sensory data from the robot and builds the dense, global point cloud map.
* **Delta Streaming:** Employs a robust delta streaming architecture to send only the updated portions of the point cloud to the Client.
* **Global Pathfinding:** Computes the Euclidean Signed Distance Field (ESDF) and generates a global path. It streams a sequence of sparse waypoints down to the robot for navigation.
* **Odometry Tracking:** Receives the high-frequency odometry stream to continuously update the robot's 3D avatar position on the client map.

---

## 3. The Robot (Local Autonomy & Sensors)

The robot focuses on immediate survival and data collection, rather than heavy global reasoning.

* **FUSE (Odometry/Pose):** Calculates high-frequency odometry and pose estimation for the in-between times when a new dense point cloud pose isn't ready. This corrected pose is propagated back to the cloud.
* **Local Reactive Navigation:** The robot receives sparse global waypoints from the cloud. It uses a lightweight local planner to drive toward these waypoints while relying on immediate sensors to dodge dynamic obstacles.
* **Fail-Safe Mechanism:** If the cloud connection drops, the robot's local loop prevents it from blindly crashing into walls, as it can stop or maneuver using its onboard obstacle avoidance.
* **Sensor Streaming:** Continually streams RGBD frames, odometry, and telemetry to the cloud (for PRISM) and the client (for the 1-frame RGBD mesh and Beacon video).

---

## Summary of the Data Flow

1. **Robot** $\rightarrow$ Streams high-frequency odometry, RGBD frames, and telemetry.
2. **Cloud** $\rightarrow$ Ingests data, builds PRISM global map, computes ESDF, sends waypoints to Robot, and streams map deltas + global robot pose to the Client.
3. **Client (Python Backend)** $\rightarrow$ Receives map deltas, global pose, and high-res video streams.
4. **Client (Unity Frontend)** $\rightarrow$ Renders the "Toy Box" room, the 1-frame RGBD mesh, the Beacon video stream, and handles the XR inputs to send teleoperation commands back to the Cloud/Robot.
