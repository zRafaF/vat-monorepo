"""
VAT bring-up — preview the RICOH Theta X UVC stream directly (run ON the robot)
==============================================================================
Opens the Theta UVC stream with OpenCV and shows it — no Zenoh, no container.
Use this to confirm the camera + UVC source work *before* running the pipeline.

    # via the v4l2 loopback device (libuvc-theta gst_loopback):
    THETA_DEVICE=/dev/video0 python3 tools/view_theta.py

    # via a GStreamer pipeline (needs OpenCV built with GStreamer + gstthetauvc):
    THETA_GST_PIPELINE='thetauvcsrc mode=2K ! queue ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink' \
        python3 tools/view_theta.py

Env: THETA_DEVICE, THETA_GST_PIPELINE, THETA_MODE (2K|4K).
Deps: opencv-python (+ GStreamer build for the pipeline mode).
Keys: 'q'/ESC quit, 's' save a PNG.
"""

from __future__ import annotations

import os
import time
import cv2

THETA_GST_PIPELINE = os.environ.get("THETA_GST_PIPELINE", "").strip()
THETA_DEVICE       = os.environ.get("THETA_DEVICE", "/dev/video0").strip()
THETA_MODE         = os.environ.get("THETA_MODE", "2K").strip()


def open_capture():
    if THETA_GST_PIPELINE:
        print(f"[theta] GStreamer: {THETA_GST_PIPELINE}")
        return cv2.VideoCapture(THETA_GST_PIPELINE, cv2.CAP_GSTREAMER)
    if THETA_DEVICE:
        dev = int(THETA_DEVICE) if THETA_DEVICE.isdigit() else THETA_DEVICE
        print(f"[theta] v4l2 device: {THETA_DEVICE}")
        return cv2.VideoCapture(dev, cv2.CAP_V4L2)
    pipe = (f"thetauvcsrc mode={THETA_MODE} ! queue ! h264parse ! decodebin ! "
            f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=2 sync=false")
    print(f"[theta] default gstthetauvc (mode={THETA_MODE})")
    return cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)


def sharpness(bgr) -> float:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main():
    cap = open_capture()
    if not cap or not cap.isOpened():
        print("ERROR: could not open the Theta UVC stream.")
        print("  • Is the camera in LIVE STREAMING mode?")
        print("  • Is the UVC source up (libuvc-theta gst_loopback → /dev/video0)?")
        print("  • See docs/setup/robot.md → Camera setup (Theta X).")
        return

    cv2.namedWindow("Theta UVC", cv2.WINDOW_NORMAL)
    n, t0, last = 0, time.time(), time.time()
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("  read failed — retrying…")
            time.sleep(0.2)
            continue
        n += 1
        cv2.imshow("Theta UVC", frame)
        now = time.time()
        if now - last >= 0.5:
            hz = n / (now - t0) if now > t0 else 0
            print(f"  n={n}  {hz:.1f}Hz  {frame.shape[1]}x{frame.shape[0]}  "
                  f"sharp={sharpness(frame):.0f}")
            last = now
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == ord('s'):
            fn = f"theta_{saved:04d}.png"
            cv2.imwrite(fn, frame)
            print(f"  saved {fn}")
            saved += 1
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
