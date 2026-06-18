"""
VAT bring-up — Stage 3: are the poses right?
============================================
Renders the pose path WITHOUT the heavy point cloud, so you can sanity-check the
geometry on its own:
  * VGGT camera trajectory          (server/prism/trajectory)
  * VGGT camera-pose correction     (server/prism/pose_correction, sent DOWN)
  * fused authoritative robot pose  ({robot}/prism/pose, sent UP)

Watch that the camera trajectory grows sensibly and the robot pose tracks it
(offset by the selfie-stick transform). Run the mapping server + robot first.

    ZENOH_ROUTER=tcp/<server-ip>:7447 ROBOT_NAME=go2 python tools/view_poses.py

Deps: rerun-sdk, numpy.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import rerun as rr
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER     = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
K = proto.keys(ROBOT_NAME, SERVER_PREFIX)


def main():
    rr.init("VAT-poses", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    n_pose = {"v": 0}

    def on_traj(sample):
        try:
            pts = proto.unpack_trajectory(bytes(sample.payload))
            if pts.shape[0] >= 2:
                rr.log("world/camera_trajectory",
                       rr.LineStrips3D([pts], colors=[[255, 200, 60]], radii=0.01))
        except Exception as e:
            print(f"  traj decode: {e}")

    def on_correction(sample):
        try:
            c = proto.unpack_pose_correction(bytes(sample.payload))
            rr.log("world/camera_correction", rr.Points3D(
                [c.position], colors=[[255, 120, 255]], radii=0.05))
            print(f"  correction v{c.map_version}  cam_pos={np.round(c.position, 3)}")
        except Exception as e:
            print(f"  correction decode: {e}")

    def on_pose(sample):
        try:
            p = proto.unpack_pose(bytes(sample.payload))
            color = [80, 220, 120] if p.fix_quality == proto.FIX_CORRECTED else [255, 190, 60]
            rr.log("world/robot", rr.Transform3D(
                translation=p.position, rotation=rr.Quaternion(xyzw=p.quaternion)))
            rr.log("world/robot/body",
                   rr.Boxes3D(half_sizes=[[0.35, 0.16, 0.18]], colors=[color]))
            rr.log("pose/speed_mps", rr.Scalar(float(np.linalg.norm(p.linear_velocity))))
            rr.log("pose/fix_corrected", rr.Scalar(float(p.fix_quality)))
            n_pose["v"] += 1
            if n_pose["v"] % 50 == 0:
                print(f"  poses={n_pose['v']}  pos={np.round(p.position,3)}  "
                      f"fix={'CORR' if p.fix_quality else 'dead-reckon'}")
        except Exception as e:
            print(f"  pose decode: {e}")

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    z.declare_subscriber(K["trajectory"], on_traj)
    z.declare_subscriber(K["pose_correction"], on_correction)
    z.declare_subscriber(K["pose"], on_pose)
    print("Viewing pose path: trajectory + correction + fused pose (Ctrl+C to quit)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
