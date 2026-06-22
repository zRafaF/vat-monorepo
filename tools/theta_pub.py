"""
VAT bring-up — headless Theta → Zenoh preview publisher (run ON the robot)
==========================================================================
The robot is headless, so we can't pop an OpenCV window on it. This reads the
Theta loopback (``/dev/video10``, fed by ``make theta-uvc``) and publishes JPEG
frames to Zenoh so you can view them on the host with ``make test_frames_server``
(tools/view_frames.py) — low latency, no best-of-window decimation, and without
running the full container (bridge + fuser).

This is the isolated "camera → cloud" smoke test. It publishes on the SAME key
the container's ``theta_camera.py`` uses (``{robot}/prism/camera/frame``), so run
EITHER this OR the full container — not both at once.

    # on the robot:
    make theta-uvc       # 1) feed /dev/video10   (leave running)
    make theta-stream    # 2) this publisher      (leave running)
    # on the host:
    make test_frames_server

Latency notes
-------------
* Reads every frame and drops to the latest (V4L2 buffersize 1), publishing at
  PREVIEW_FPS — so the stream stays fresh instead of building a backlog.
* PREVIEW_SCALE downsizes before JPEG to cut bandwidth/decode time.
* Decoding already happened on the host in ``theta-uvc`` (HW NVDEC on Jetson),
  so this process is just capture → JPEG → Zenoh.

Env:
  ZENOH_CONNECT   router endpoint (default tcp/127.0.0.1:7447; vat.env sets it)
  ROBOT_NAME      default go2
  THETA_DEVICE    v4l2 device to read (default /dev/video10)
  PREVIEW_FPS     publish-rate cap (default 15; 0 = uncapped)
  PREVIEW_SCALE   downscale factor 0<scale<=1 (default 0.5; 1 = full res)
  PREVIEW_QUALITY JPEG quality 1..100 (default 80)
Deps: eclipse-zenoh, opencv-python (or -headless), numpy. Install in the robot's
      env, e.g.  ``pip install eclipse-zenoh opencv-python-headless numpy``.
"""

from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np  # noqa: F401  (kept for parity / future use)
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ZENOH_CONNECT   = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
ROBOT_NAME      = os.environ.get("ROBOT_NAME", "go2")
THETA_DEVICE    = os.environ.get("THETA_DEVICE", "/dev/video10")
PREVIEW_FPS     = float(os.environ.get("PREVIEW_FPS", "15"))
PREVIEW_SCALE   = float(os.environ.get("PREVIEW_SCALE", "0.5"))
PREVIEW_QUALITY = int(os.environ.get("PREVIEW_QUALITY", "80"))

KEY = proto.keys(ROBOT_NAME)["camera_frame"]


def open_device() -> cv2.VideoCapture:
    dev = int(THETA_DEVICE) if THETA_DEVICE.isdigit() else THETA_DEVICE
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # keep latency low
    except Exception:
        pass
    return cap


def main() -> int:
    cap = open_device()
    if not cap or not cap.isOpened():
        print(f"ERROR: could not open {THETA_DEVICE}.")
        print("  - Is `make theta-uvc` running and feeding the loopback?")
        print("  - Is the camera in LIVE STREAMING mode (lsusb -> 05ca:2717)?")
        print("  - See docs/setup/robot.md -> Camera setup (Theta X).")
        return 1

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_CONNECT}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    pub = z.declare_publisher(KEY)

    print(f"[theta-pub] {THETA_DEVICE} -> Zenoh '{KEY}' @ {ZENOH_CONNECT}")
    print(f"[theta-pub] fps<={PREVIEW_FPS} scale={PREVIEW_SCALE} q={PREVIEW_QUALITY}")
    print("[theta-pub] view on the host:  make test_frames_server   (Ctrl+C to stop)")

    min_dt = 1.0 / PREVIEW_FPS if PREVIEW_FPS > 0 else 0.0
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), PREVIEW_QUALITY]
    seq = 0
    last_pub = 0.0
    n, t0, last_log = 0, time.time(), time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[theta-pub] read failed — is the loopback still fed? retrying…")
                cap.release()
                time.sleep(0.5)
                cap = open_device()
                continue

            now = time.time()
            if min_dt and (now - last_pub) < min_dt:
                continue                       # throttle, but keep draining buffer
            last_pub = now

            if 0.0 < PREVIEW_SCALE < 1.0:
                frame = cv2.resize(frame, None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE,
                                   interpolation=cv2.INTER_AREA)

            ok, jbuf = cv2.imencode(".jpg", frame, enc)
            if not ok:
                continue

            # camera_height = -1.0 → "unknown" (this preview has no body pose).
            payload = proto.pack_frame(time.time_ns(), seq & 0xFFFFFFFF, -1.0,
                                       jbuf.tobytes())
            try:
                pub.put(payload, encoding=proto.ENC_FRAME)
            except TypeError:
                pub.put(payload)

            seq += 1
            n += 1
            if now - last_log >= 1.0:
                hz = n / (now - t0) if now > t0 else 0.0
                h, w = frame.shape[:2]
                print(f"[theta-pub] n={n} {hz:.1f}Hz {w}x{h} {len(jbuf) // 1024}kB/frame")
                last_log = now
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        try:
            pub.undeclare()
        except Exception:
            pass
        z.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
