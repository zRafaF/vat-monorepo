"""
VAT bring-up — Stage 1: see the 360° frames
============================================
Confirms the camera → bridge → (decimator) → Zenoh path by rendering the live
equirectangular feed in Rerun.

    # decimated frames the server actually consumes (default) — also shows the
    # stamped camera_height and seq:
    python tools/view_frames.py

    # raw equirectangular straight off the bridge (tests the camera alone):
    python tools/view_frames.py --raw

Env: ZENOH_ROUTER, ROBOT_NAME, IMAGE_TOPIC (default equirectangular/image).
Deps: rerun-sdk, opencv-python-headless (decimated), rosbags (raw).
"""

from __future__ import annotations

import os
import sys
import time
import argparse
import threading

import numpy as np
import rerun as rr
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER      = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "go2")
IMAGE_TOPIC = os.environ.get("IMAGE_TOPIC", "equirectangular/image")
K = proto.keys(ROBOT_NAME)

_count = 0
_t0 = time.time()


def _tick(extra=""):
    global _count
    _count += 1
    if _count % 15 == 0:
        dt = time.time() - _t0
        print(f"  frames={_count}  {_count/dt:5.1f} Hz  {extra}")


def run_decimated(z):
    import cv2

    def on_frame(sample):
        try:
            ts_ns, seq, cam_h, jpeg = proto.unpack_frame(bytes(sample.payload))
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rr.log("camera/equirect", rr.Image(rgb))
            rr.log("camera/height_m", rr.Scalar(cam_h))
            rr.log("camera/seq", rr.Scalar(seq))
            _tick(f"seq={seq} cam_h={cam_h:.2f}m {rgb.shape[1]}x{rgb.shape[0]}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)
    print(f"Viewing DECIMATED frames on '{K['camera_frame']}' (Ctrl+C to quit)")


def run_raw(z):
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    key = f"{ROBOT_NAME}/rt/{IMAGE_TOPIC}"

    def on_image(sample):
        try:
            msg = ts.deserialize_cdr(bytes(sample.payload), "sensor_msgs/msg/Image")
            h, w = int(msg.height), int(msg.width)
            enc = str(msg.encoding).lower()
            data = np.frombuffer(bytes(msg.data), np.uint8)
            if enc in ("rgb8", "rgb"):
                img = data.reshape(h, w, 3)
            elif enc in ("bgr8", "bgr"):
                img = data.reshape(h, w, 3)[:, :, ::-1]
            else:
                print(f"  unsupported encoding {enc}"); return
            rr.log("camera/equirect", rr.Image(img))
            _tick(f"{w}x{h} {enc}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(key, on_image)
    print(f"Viewing RAW frames on '{key}' (Ctrl+C to quit)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", action="store_true", help="raw equirectangular off the bridge")
    args = ap.parse_args()

    rr.init("VAT-frames", spawn=True)
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)

    (run_raw if args.raw else run_decimated)(z)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
