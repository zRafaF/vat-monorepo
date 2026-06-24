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
from kinematics import (  # noqa: E402
    RobotStateTracker, LowStateTracker, LEG_ORDER,
    base_height_above_ground, camera_height_above_ground)

ROUTER      = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "go2")
K = proto.keys(ROBOT_NAME)

FOOT_COLORS = {"FR": [255, 80, 80], "FL": [80, 255, 80],
               "RR": [80, 160, 255], "RL": [255, 220, 60]}
COLOR_DEADRECKON = [255, 180, 60]    # amber — drifting on odometry
COLOR_CORRECTED  = [80, 230, 120]    # green — anchored by VGGT

# Selfie-stick / camera mount offset in the BASE frame (x fwd, y left, z up).
# Calibrated from tape measurements on the dog (override via env):
#   • back-of-robot → camera  ≈ 0.65 m  (horizontal)  → STICK_OFFSET_X ≈ -0.65
#   • ground → camera (standing) ≈ 1.15 m            → STICK_OFFSET_Z ≈ 0.85
#     (1.15 m − ~0.30 m standing base height = ~0.85 m vertical stick reach).
# The HEIGHT now comes from the legs (ground plane), so only the stick's body-
# frame offset is fixed here; ground→camera is recomputed every frame.
STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.65")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.85")),
], dtype=np.float32)
CAM_GROUND_REF = float(os.environ.get("CAM_GROUND_REF_M", "1.15"))   # standing, for logging
FALLBACK_BASE_H = float(os.environ.get("FALLBACK_BODY_HEIGHT", "0.30"))


def _ground_grid(size=2.0, step=0.5, z=0.0):
    """LineStrips for a floor grid at world ``z`` — a visible ground reference."""
    strips, n = [], int(round(size / step))
    for i in range(-n, n + 1):
        x = i * step
        strips.append([[x, -size, z], [x, size, z]])
        strips.append([[-size, x, z], [size, x, z]])
    return strips


def main():
    rr.init("VAT-robot-state", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # ground plane at world Z=0 — the floor the leg-derived height sits the dog on
    rr.log("world/ground", rr.LineStrips3D(
        _ground_grid(), colors=[[70, 70, 85]], radii=0.003), static=True)
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
                # height the ROBOT stamped into the frame (current on-robot calc);
                # compare against state/camera_height_calc_m (the new leg-derived one)
                rr.log("state/camera_height_stamped_m", rr.Scalars(float(cam_h)))
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

            # ── leg-derived stance height (THE fix) ─────────────────────────
            # Base height comes from the FEET (ground plane) using the SAME
            # orientation as the avatar transform, so the feet land on the floor
            # (world Z=0) and the BODY lowers when the dog goes prone — instead of
            # the body staying put while the legs ride up. Falls back to the
            # SportModeState body_height (then a constant) if legs aren't flowing.
            base_h = base_height_above_ground(legs, quat)
            if base_h is None:
                base_h = float(body.body_height) or FALLBACK_BASE_H
            # camera height above the floor, recomputed every frame (stance + tilt)
            cam_h_calc, _bh, _coff = camera_height_above_ground(
                legs, quat, STICK, fallback_base_height=FALLBACK_BASE_H)

            rr.log("world/robot", rr.Transform3D(
                translation=[px, py, base_h],
                rotation=rr.Quaternion(xyzw=quat)))

            # ── drift trail ────────────────────────────────────────────────
            trail.append([px, py, base_h])
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
            rr.log("state/base_height_legs_m", rr.Scalars(base_h))
            rr.log("state/body_height_sportmode_m", rr.Scalars(float(body.body_height)))
            rr.log("state/camera_height_calc_m", rr.Scalars(cam_h_calc))
            rr.log("state/speed_mps", rr.Scalars(speed))
            rr.log("state/drift_m", rr.Scalars(drift))
            rr.log("state/fix_corrected", rr.Scalars(1.0 if fix_corrected else 0.0))

            n += 1
            if n % 30 == 0:
                print(f"  base_h(legs)={base_h:.2f}m  cam_h={cam_h_calc:.2f}m "
                      f"(ref~{CAM_GROUND_REF:.2f}m)  body_h(sport)={body.body_height:.2f}m  "
                      f"pos=({px:+.2f},{py:+.2f})  legs={'on' if legs_valid else 'OFF'}")
            time.sleep(1 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
