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

    _prev_recv_ns = [None]        # mutable ref for closure
    _prev_seq     = [None]

    def on_frame(sample):
        recv_ns = time.time_ns()
        try:
            raw_payload = bytes(sample.payload)
            ts_ns, seq, cam_h, img_bytes = proto.unpack_frame(raw_payload)
            bgr = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rr.log("camera/equirect", rr.Image(rgb))
            rr.log("camera/height_m", rr.Scalar(cam_h))
            rr.log("camera/seq", rr.Scalar(seq))

            # ── Metrics ──────────────────────────────────────────────
            payload_kb = len(raw_payload) / 1024.0
            img_kb     = len(img_bytes) / 1024.0
            rr.log("metrics/frame_size_kB", rr.Scalar(img_kb))

            # Frame age: capture timestamp → local receive time.
            # ⚠ Only meaningful if robot and client clocks are NTP-synced.
            age_ms = (recv_ns - ts_ns) / 1e6
            rr.log("metrics/frame_age_ms", rr.Scalar(age_ms))

            # Inter-frame interval (client-side, always accurate).
            interval_ms = 0.0
            if _prev_recv_ns[0] is not None:
                interval_ms = (recv_ns - _prev_recv_ns[0]) / 1e6
                rr.log("metrics/interval_ms", rr.Scalar(interval_ms))
            _prev_recv_ns[0] = recv_ns

            # Detect dropped frames (seq gap).
            dropped = 0
            if _prev_seq[0] is not None:
                expected = (_prev_seq[0] + 1) & 0xFFFFFFFF
                if seq != expected:
                    dropped = (seq - expected) & 0xFFFFFFFF
                    rr.log("metrics/dropped_frames", rr.Scalar(dropped))
            _prev_seq[0] = seq

            _tick(f"seq={seq} cam_h={cam_h:.2f}m {rgb.shape[1]}x{rgb.shape[0]} "
                  f"img={img_kb:.0f}kB age={age_ms:.0f}ms Δ={interval_ms:.0f}ms"
                  + (f" DROP={dropped}" if dropped else ""))
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)
    print(f"Viewing DECIMATED frames on '{K['camera_frame']}' (Ctrl+C to quit)")
    print(f"  ⚠ frame_age_ms is only accurate if robot & client clocks are NTP-synced.")


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
