"""
VAT bring-up — Stage 1: see the 360° frames
============================================
Confirms the camera → bridge → (decimator) → Zenoh path by rendering the live
feed in an OpenCV window (lightweight, low-latency).

Modes
-----
  # decimated frames the server ingests (default):
  python tools/view_frames.py

  # raw equirectangular straight off the bridge:
  python tools/view_frames.py --raw

  # use Rerun instead of OpenCV (heavier, 3D panels):
  python tools/view_frames.py --rerun

Env: ZENOH_ROUTER, ROBOT_NAME, IMAGE_TOPIC (default equirectangular/image).
Deps: eclipse-zenoh, opencv-python, numpy, rosbags (raw mode only).
"""

from __future__ import annotations

import os
import sys
import time
import argparse

import cv2
import numpy as np
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER      = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "go2")
IMAGE_TOPIC = os.environ.get("IMAGE_TOPIC", "equirectangular/image")
K = proto.keys(ROBOT_NAME)


# ─────────────────────────────────────────────────────────────────────────────
# Metrics tracker
# ─────────────────────────────────────────────────────────────────────────────

class Metrics:
    def __init__(self):
        self._count = 0
        self._t0 = time.time()
        self._prev_recv_ns = None
        self._prev_seq = None
        self._last_print = time.time()

    def update(self, *, seq: int = -1, size_bytes: int = 0,
               capture_ns: int = 0, extra: str = "") -> str:
        recv_ns = time.time_ns()
        self._count += 1

        # inter-frame interval (always accurate)
        interval_ms = 0.0
        if self._prev_recv_ns is not None:
            interval_ms = (recv_ns - self._prev_recv_ns) / 1e6
        self._prev_recv_ns = recv_ns

        # frame age (only meaningful with synced clocks)
        age_ms = (recv_ns - capture_ns) / 1e6 if capture_ns > 0 else 0.0

        # dropped seq detection
        dropped = 0
        if seq >= 0 and self._prev_seq is not None:
            expected = (self._prev_seq + 1) & 0xFFFFFFFF
            if seq != expected:
                dropped = (seq - expected) & 0xFFFFFFFF
        if seq >= 0:
            self._prev_seq = seq

        # throttle console output to ~2 Hz
        now = time.time()
        if now - self._last_print >= 0.5:
            elapsed = now - self._t0
            avg_hz = self._count / elapsed if elapsed > 0 else 0
            size_kb = size_bytes / 1024.0
            parts = [
                f"n={self._count}",
                f"{avg_hz:.1f}Hz",
                f"Δ={interval_ms:.0f}ms",
            ]
            if size_bytes:
                parts.append(f"size={size_kb:.0f}kB")
            if capture_ns > 0:
                parts.append(f"age={age_ms:.0f}ms")
            if dropped:
                parts.append(f"DROP={dropped}")
            if extra:
                parts.append(extra)
            line = "  ".join(parts)
            self._last_print = now
            return line
        return ""


# ─────────────────────────────────────────────────────────────────────────────
# OpenCV viewer (lightweight, default)
# ─────────────────────────────────────────────────────────────────────────────

_latest_bgr = None  # shared between callback and main-thread display


def run_decimated_cv2(z):
    global _latest_bgr
    m = Metrics()

    def on_frame(sample):
        global _latest_bgr
        try:
            raw = bytes(sample.payload)
            ts_ns, seq, cam_h, img_bytes = proto.unpack_frame(raw)
            bgr = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            _latest_bgr = bgr
            line = m.update(seq=seq, size_bytes=len(img_bytes),
                            capture_ns=ts_ns,
                            extra=f"cam_h={cam_h:.2f}m {bgr.shape[1]}x{bgr.shape[0]}")
            if line:
                print(f"  {line}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)
    print(f"Viewing DECIMATED frames on '{K['camera_frame']}'")
    print(f"  Press 'q' or ESC to quit.  's' to save current frame.")
    print(f"  ⚠ age= is only accurate if robot & client clocks are NTP-synced.")


def run_raw_cv2(z):
    global _latest_bgr
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    key = f"{ROBOT_NAME}/rt/{IMAGE_TOPIC}"
    m = Metrics()

    def on_image(sample):
        global _latest_bgr
        try:
            raw = bytes(sample.payload)
            msg = ts.deserialize_cdr(raw, "sensor_msgs/msg/Image")
            h, w = int(msg.height), int(msg.width)
            enc = str(msg.encoding).lower()
            data = np.frombuffer(bytes(msg.data), np.uint8)
            if enc in ("rgb8", "rgb"):
                bgr = cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            elif enc in ("bgr8", "bgr"):
                bgr = data.reshape(h, w, 3).copy()
            else:
                print(f"  unsupported encoding {enc}")
                return
            _latest_bgr = bgr
            line = m.update(size_bytes=len(raw), extra=f"{w}x{h} {enc}")
            if line:
                print(f"  {line}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(key, on_image)
    print(f"Viewing RAW frames on '{key}'")
    print(f"  Press 'q' or ESC to quit.  's' to save current frame.")


def display_loop_cv2(window_name="VAT Frames"):
    """Main-thread loop: pulls latest frame and shows it.  ~60 Hz poll."""
    global _latest_bgr
    save_count = 0
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while True:
            bgr = _latest_bgr
            if bgr is not None:
                cv2.imshow(window_name, bgr)
            key = cv2.waitKey(16) & 0xFF  # ~60 Hz poll
            if key in (ord('q'), 27):      # q or ESC
                break
            if key == ord('s') and bgr is not None:
                fname = f"frame_{save_count:04d}.png"
                cv2.imwrite(fname, bgr)
                print(f"  saved {fname}")
                save_count += 1
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# Rerun viewer (optional, --rerun flag)
# ─────────────────────────────────────────────────────────────────────────────

def run_decimated_rerun(z):
    import rerun as rr
    m = Metrics()

    def on_frame(sample):
        try:
            raw = bytes(sample.payload)
            ts_ns, seq, cam_h, img_bytes = proto.unpack_frame(raw)
            bgr = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rr.log("camera/equirect", rr.Image(rgb))
            rr.log("camera/height_m", rr.Scalar(cam_h))
            rr.log("camera/seq", rr.Scalar(seq))

            recv_ns = time.time_ns()
            rr.log("metrics/frame_size_kB", rr.Scalar(len(img_bytes) / 1024.0))
            rr.log("metrics/frame_age_ms", rr.Scalar((recv_ns - ts_ns) / 1e6))
            rr.log("metrics/interval_ms", rr.Scalar(
                (recv_ns - (m._prev_recv_ns or recv_ns)) / 1e6))

            line = m.update(seq=seq, size_bytes=len(img_bytes),
                            capture_ns=ts_ns,
                            extra=f"seq={seq} cam_h={cam_h:.2f}m")
            if line:
                print(f"  {line}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)
    print(f"Viewing DECIMATED frames on '{K['camera_frame']}' (Ctrl+C to quit)")


def run_raw_rerun(z):
    import rerun as rr
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    key = f"{ROBOT_NAME}/rt/{IMAGE_TOPIC}"
    m = Metrics()

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
            line = m.update(extra=f"{w}x{h} {enc}")
            if line:
                print(f"  {line}")
        except Exception as e:
            print(f"  decode error: {e}")

    z.declare_subscriber(key, on_image)
    print(f"Viewing RAW frames on '{key}' (Ctrl+C to quit)")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="VAT frame viewer (Stage 1 bring-up)")
    ap.add_argument("--raw", action="store_true",
                    help="raw CDR images off the bridge (not decimated)")
    ap.add_argument("--rerun", action="store_true",
                    help="use Rerun viewer instead of OpenCV window")
    args = ap.parse_args()

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)

    if args.rerun:
        import rerun as rr
        rr.init("VAT-frames", spawn=True)
        (run_raw_rerun if args.raw else run_decimated_rerun)(z)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    else:
        (run_raw_cv2 if args.raw else run_decimated_cv2)(z)
        display_loop_cv2()

    z.close()


if __name__ == "__main__":
    main()
