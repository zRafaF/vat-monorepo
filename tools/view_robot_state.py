"""
VAT bring-up — Stage 2: see the robot's body & limbs in real time
==================================================================
Decodes the bridged `SportModeState` (no ROS install needed) and renders, live
in Rerun:
  * the body coordinate frame (orientation from the IMU quaternion)
  * the four legs as lines (body origin → each foot) + foot markers
  * the selfie-stick as a line on the back, with the camera marker at its tip
  * the live 360° equirectangular camera image (camera/equirect)
  * scalars: body height, speed, yaw rate, camera height

This validates odometry/limb data BEFORE you trust the pose fuser or PRISM.

    ZENOH_ROUTER=tcp/<server-ip>:7447 ROBOT_NAME=go2 python tools/view_robot_state.py

Env: SPORT_TOPIC (default sportmodestate).  Deps: rerun-sdk, rosbags, numpy.

Note: this uses the embedded unitree_go message definitions from kinematics.py.
If your firmware's layout differs and decode fails, fix the defs there.
"""

from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np
import rerun as rr
import zenoh

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
import vat_protocol as proto  # noqa: E402
from kinematics import _UNITREE_MSG_DEFS  # noqa: E402

ROUTER      = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "go2")
SPORT_TOPIC = os.environ.get("SPORT_TOPIC", "sportmodestate")
KEY = f"{ROBOT_NAME}/rt/{SPORT_TOPIC}"

FOOT_LABELS = ["FR", "FL", "RR", "RL"]   # Unitree convention
FOOT_COLORS = [[255, 80, 80], [80, 255, 80], [80, 160, 255], [255, 220, 60]]

# Selfie-stick geometry (camera offset in the body frame, metres). Same env
# the robot uses, so the drawn stick matches the real rig. MEASURE THESE.
STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.20")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.55")),
], dtype=np.float32)
CAMERA_KEY = proto.keys(ROBOT_NAME)["camera_frame"]

_count = 0


def build_decoder():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    reg = {}
    for name, defn in _UNITREE_MSG_DEFS.items():
        reg.update(get_types_from_msg(defn, name))
    ts.register(reg)
    return lambda cdr: ts.deserialize_cdr(cdr, "unitree_go/msg/SportModeState")


def main():
    rr.init("VAT-robot-state", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/robot/body", rr.Boxes3D(half_sizes=[[0.35, 0.16, 0.10]],
                                           colors=[[120, 180, 255]]), static=True)
    # Selfie-stick on the back: a line from the body origin to the camera, plus
    # a marker at the camera. Logged under world/robot so it swings with the body.
    rr.log("world/robot/stick", rr.LineStrips3D(
        [np.vstack([[0, 0, 0], STICK])], colors=[[230, 230, 60]], radii=0.012),
        static=True)
    rr.log("world/robot/camera", rr.Points3D(
        [STICK], colors=[[255, 255, 0]], radii=0.05, labels=["theta"]),
        static=True)

    decode = build_decoder()

    def on_state(sample):
        global _count
        try:
            msg = decode(bytes(sample.payload))
        except Exception as e:
            print(f"  SportModeState decode failed: {e}  (check unitree_go layout)")
            return
        try:
            q_wxyz = np.asarray(msg.imu_state.quaternion, dtype=np.float64)
            quat_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
            quat_xyzw = proto.quat_normalize(quat_xyzw)

            rr.log("world/robot", rr.Transform3D(
                translation=[0, 0, float(msg.body_height)],
                rotation=rr.Quaternion(xyzw=quat_xyzw.astype(np.float32))))

            feet = np.asarray(msg.foot_position_body, dtype=np.float32).reshape(4, 3)
            rr.log("world/robot/feet", rr.Points3D(
                feet, colors=FOOT_COLORS, radii=0.03, labels=FOOT_LABELS))
            # legs: body origin → each foot
            rr.log("world/robot/legs", rr.LineStrips3D(
                [np.vstack([[0, 0, 0], f]) for f in feet], colors=FOOT_COLORS, radii=0.005))

            vel = np.asarray(msg.velocity, dtype=np.float64)
            rr.log("state/body_height_m", rr.Scalar(float(msg.body_height)))
            rr.log("state/speed_mps", rr.Scalar(float(np.linalg.norm(vel))))
            rr.log("state/yaw_speed_rps", rr.Scalar(float(msg.yaw_speed)))

            _count += 1
            if _count % 50 == 0:
                print(f"  states={_count}  body_h={msg.body_height:.3f}m  "
                      f"speed={np.linalg.norm(vel):.2f}m/s")
        except Exception as e:
            print(f"  render error: {e}")

    def on_frame(sample):
        try:
            _, _, cam_h, jpeg = proto.unpack_frame(bytes(sample.payload))
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            rr.log("camera/equirect", rr.Image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
            if cam_h >= 0:
                rr.log("state/camera_height_m", rr.Scalar(float(cam_h)))
        except Exception as e:
            print(f"  camera decode error: {e}")

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    z.declare_subscriber(KEY, on_state)
    z.declare_subscriber(CAMERA_KEY, on_frame)
    print(f"Viewing robot state on '{KEY}'  +  camera on '{CAMERA_KEY}'  (Ctrl+C to quit)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
