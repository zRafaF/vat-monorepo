"""
VAT bring-up — Stage 2 / 2.5: robot body, limbs & dead-reckoned motion
======================================================================
Renders, live in Rerun:
  * the robot avatar placed at the **dead-reckoned pose** streamed by the
    on-robot fuser (``{robot}/prism/pose``) — so it now MOVES and drifts as you
    drive, instead of sitting at the origin. Colour = fix quality (amber while
    dead-reckoning on odometry only, green after a VGGT correction).
  * a growing **trail** of the dead-reckoned path (watch the drift accumulate).
  * the four **legs** drawn from ``/lowstate`` joint angles via forward
    kinematics (the Go2-W does not populate ``SportModeState.foot_position_body``,
    so FK is the only way to draw limbs).
  * the selfie-stick + camera marker, and the live 360° camera image.
  * scalars: body height, speed, drift distance, fix quality.

    ZENOH_ROUTER=tcp/<router-ip>:7447 ROBOT_NAME=go2 python tools/view_robot_state.py

Env: SPORT_TOPIC (default lf/sportmodestate), LOWSTATE_TOPIC (default lowstate).
Deps: rerun-sdk, rosbags, numpy, opencv. Needs the robot container running
(bridge forwards /lowstate + sportmodestate; fuser publishes the pose).
"""

from __future__ import annotations

import os
import sys
import time
from collections import deque

import cv2
import numpy as np
import rerun as rr
import zenoh

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
import vat_protocol as proto  # noqa: E402
from vat_protocol import FIX_CORRECTED  # noqa: E402
from kinematics import RobotStateTracker, LowStateTracker, LEG_ORDER  # noqa: E402

ROUTER      = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "go2")
K = proto.keys(ROBOT_NAME)

FOOT_COLORS = {"FR": [255, 80, 80], "FL": [80, 255, 80],
               "RR": [80, 160, 255], "RL": [255, 220, 60]}
COLOR_DEADRECKON = [255, 180, 60]    # amber — drifting on odometry
COLOR_CORRECTED  = [80, 230, 120]    # green — anchored by VGGT

STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.20")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.55")),
], dtype=np.float32)


def main():
    rr.init("VAT-robot-state", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/robot/body", rr.Boxes3D(half_sizes=[[0.19, 0.05, 0.05]],
                                          colors=[[120, 180, 255]]), static=True)
    # selfie-stick + camera marker (children of world/robot → swing with the body)
    rr.log("world/robot/stick", rr.LineStrips3D(
        [np.vstack([[0, 0, 0], STICK])], colors=[[230, 230, 60]], radii=0.012),
        static=True)
    rr.log("world/robot/camera", rr.Points3D(
        [STICK], colors=[[255, 255, 0]], radii=0.04, labels=["theta"]), static=True)

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)

    # Trackers (subscribe + decode over Zenoh; both best-effort, never raise)
    body_tracker = RobotStateTracker(z, ROBOT_NAME)     # lf/sportmodestate
    leg_tracker = LowStateTracker(z, ROBOT_NAME)        # /lowstate → leg FK

    # Dead-reckoned authoritative pose from the on-robot fuser
    latest = {"pose": None}

    def on_pose(sample):
        try:
            latest["pose"] = proto.unpack_pose(bytes(sample.payload))
        except proto.ProtocolError:
            pass

    z.declare_subscriber(K["pose"], on_pose)

    def on_frame(sample):
        try:
            _, _, cam_h, jpeg = proto.unpack_frame(bytes(sample.payload))
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                rr.log("camera/equirect", rr.Image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
            if cam_h >= 0:
                rr.log("state/camera_height_m", rr.Scalar(float(cam_h)))
        except Exception as e:
            print(f"  camera decode error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)

    print(f"Viewing robot state: pose←'{K['pose']}'  legs←'{ROBOT_NAME}/rt/lowstate'  "
          f"body←'{ROBOT_NAME}/rt/lf/sportmodestate'  (Ctrl+C to quit)")

    trail = deque(maxlen=4000)
    legs_warned = False
    n = 0
    try:
        while True:
            pose = latest["pose"]
            body = body_tracker.get()
            legs, legs_valid = leg_tracker.get()
            z_h = float(body.body_height)

            # ── robot avatar at the dead-reckoned pose ──────────────────────
            if pose is not None:
                px, py = float(pose.position[0]), float(pose.position[1])
                quat = pose.quaternion.astype(np.float32)
                fix_corrected = (pose.fix_quality == FIX_CORRECTED)
            else:
                # fuser not up yet → sit at origin with the IMU attitude
                px, py = 0.0, 0.0
                quat = body.rotation.astype(np.float32)
                fix_corrected = False

            rr.log("world/robot", rr.Transform3D(
                translation=[px, py, z_h],
                rotation=rr.Quaternion(xyzw=quat)))

            # ── drift trail ────────────────────────────────────────────────
            trail.append([px, py, z_h])
            if len(trail) >= 2:
                col = COLOR_CORRECTED if fix_corrected else COLOR_DEADRECKON
                rr.log("world/trail", rr.LineStrips3D(
                    [np.asarray(trail, dtype=np.float32)], colors=[col], radii=0.004))

            # ── legs from /lowstate forward kinematics ─────────────────────
            if legs_valid and legs:
                for leg in LEG_ORDER:
                    p = legs[leg]
                    chain = np.vstack([p["hip"], p["thigh_root"], p["knee"], p["foot"]])
                    rr.log(f"world/robot/leg_{leg}", rr.LineStrips3D(
                        [chain.astype(np.float32)], colors=[FOOT_COLORS[leg]], radii=0.006))
                    rr.log(f"world/robot/foot_{leg}", rr.Points3D(
                        [p["foot"].astype(np.float32)], colors=[FOOT_COLORS[leg]],
                        radii=0.025, labels=[leg]))
            elif not legs_warned:
                print("  (no leg data yet — is /lowstate flowing? bridge running?)")
                legs_warned = True

            # ── scalars ─────────────────────────────────────────────────────
            speed = float(np.linalg.norm(body.linear_velocity))
            drift = float(np.hypot(px, py))   # distance from start (= drift, no GT)
            rr.log("state/body_height_m", rr.Scalar(z_h))
            rr.log("state/speed_mps", rr.Scalar(speed))
            rr.log("state/drift_m", rr.Scalar(drift))
            rr.log("state/fix_corrected", rr.Scalar(1.0 if fix_corrected else 0.0))

            n += 1
            if n % 60 == 0:
                print(f"  pos=({px:+.2f},{py:+.2f})m  drift={drift:.2f}m  "
                      f"speed={speed:.2f}m/s  h={z_h:.2f}m  "
                      f"fix={'CORR' if fix_corrected else 'dead-reckon'}  "
                      f"legs={'on' if legs_valid else 'off'}")
            time.sleep(1 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
