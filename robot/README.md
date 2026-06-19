# Robot

The robot setup guide now lives in the project documentation (MkDocs), not here:

➡️ **[docs/setup/robot.md](../docs/setup/robot.md)** — Insta360 driver + SDK
install, build, hardware config, camera bringup, the Zenoh bridge container, the
CycloneDDS fix, and troubleshooting.

Quick start (after the one-time install in the docs), from the repo root on the
robot:

```bash
make robot-ros      # camera: CycloneDDS eth0 fix + insta360 equirectangular
make robot-docker   # ROS↔Zenoh bridge + frame decimator + pose fuser
```

Folders here:

- `robot/ros/` — `bringup_camera.sh` (the camera bringup) + the optional
  `vat_bringup/` launch wrapper.
- `robot/docker/` — the container (`dynamic_bridge.py`, `frame_decimator.py`,
  `pose_fuser.py`, `kinematics.py`) + `run.sh`.
- `robot/systemd/` — boot units for the camera stack and the container.
- `robot/insta360_ros_driver/` — **do not modify** (vendored hardware driver).
