# Robot

The robot setup guide lives in the project documentation (MkDocs):

➡️ **[docs/setup/robot.md](../docs/setup/robot.md)** — RICOH **Theta X** camera
over UVC (libuvc-theta → v4l2 loopback → OpenCV), the Zenoh bridge container, the
CycloneDDS fix, and troubleshooting.

> The camera was switched from the Insta360 (ROS driver, motion artifacts) to the
> Theta X (clean in-camera-stitched UVC stream). The retired Insta360 setup is
> archived at [docs/archive/insta360.md](../docs/archive/insta360.md) and on the
> [`insta360` branch](https://github.com/zRafaF/vat-monorepo/tree/insta360).

Quick start (after the one-time install in the docs), from the repo root on the
robot:

```bash
make theta-uvc      # Theta X UVC → /dev/video0 (leave running)
make robot-docker   # ROS↔Zenoh bridge (odometry) + theta_camera + pose fuser
```

Folders here:

- `robot/theta/` — `theta_uvc.sh` (Theta UVC → /dev/video0 loopback helper).
- `robot/docker/` — the container (`dynamic_bridge.py`, `theta_camera.py`,
  `pose_fuser.py`, `kinematics.py`) + `run.sh` + `start.sh`.
- `robot/systemd/` — boot units (`vat-theta-uvc.service`, `vat-robot-docker.service`).
