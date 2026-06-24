"""
VAT — Record 360° frames for offline analysis
===============================================
Subscribes to the live decimated camera feed and saves every incoming JPEG
frame to a folder you pick at startup.  Each file is named by its capture
timestamp (nanoseconds) so the frames sort chronologically and can be
correlated with the recorded pose trajectory.

A ``metadata.csv`` sidecar is written alongside the frames with
``(timestamp_ns, seq, camera_height_m, filename)`` for every saved frame.

Usage
-----
  make record-frames                       # recommended (loads vat.env)
  cd client && uv run python ../tools/record_frames.py   # manual

Env: ZENOH_ROUTER, ROBOT_NAME  (defaults from vat.env).
Deps: eclipse-zenoh, numpy — no OpenCV needed (JPEG saved as-is).
"""

from __future__ import annotations

import csv
import os
import sys
import time
import signal
import threading

import numpy as np  # noqa: F401 — proto uses it transitively
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER     = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
K = proto.keys(ROBOT_NAME)


# ─────────────────────────────────────────────────────────────────────────────
# Folder picker (tkinter — bundled with CPython on Windows)
# ─────────────────────────────────────────────────────────────────────────────

def pick_folder() -> str | None:
    """Open a native folder-picker dialog.  Returns the chosen path or None."""
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()          # hide the empty root window
    root.attributes("-topmost", True)
    folder = filedialog.askdirectory(title="Choose folder to save 360° frames")
    root.destroy()
    return folder if folder else None


# ─────────────────────────────────────────────────────────────────────────────
# Metrics (same console-throttle pattern as view_frames.py)
# ─────────────────────────────────────────────────────────────────────────────

class Metrics:
    def __init__(self):
        self._count = 0
        self._bytes = 0
        self._t0 = time.time()
        self._prev_recv_ns = None
        self._prev_seq = None
        self._last_print = time.time()
        self._dropped = 0

    def update(self, *, seq: int, size_bytes: int, capture_ns: int) -> str:
        recv_ns = time.time_ns()
        self._count += 1
        self._bytes += size_bytes

        interval_ms = 0.0
        if self._prev_recv_ns is not None:
            interval_ms = (recv_ns - self._prev_recv_ns) / 1e6
        self._prev_recv_ns = recv_ns

        age_ms = (recv_ns - capture_ns) / 1e6 if capture_ns > 0 else 0.0

        if self._prev_seq is not None:
            expected = (self._prev_seq + 1) & 0xFFFFFFFF
            if seq != expected:
                gap = (seq - expected) & 0xFFFFFFFF
                self._dropped += gap
        self._prev_seq = seq

        now = time.time()
        if now - self._last_print >= 0.5:
            elapsed = now - self._t0
            avg_hz = self._count / elapsed if elapsed > 0 else 0
            mb = self._bytes / (1024 * 1024)
            parts = [
                f"saved={self._count}",
                f"{avg_hz:.1f}Hz",
                f"Δ={interval_ms:.0f}ms",
                f"size={size_bytes / 1024:.0f}kB",
                f"total={mb:.1f}MB",
            ]
            if capture_ns > 0:
                parts.append(f"age={age_ms:.0f}ms")
            if self._dropped:
                parts.append(f"DROP={self._dropped}")
            self._last_print = now
            return "  ".join(parts)
        return ""

    def summary(self) -> str:
        elapsed = time.time() - self._t0
        mb = self._bytes / (1024 * 1024)
        avg_hz = self._count / elapsed if elapsed > 0 else 0
        return (f"{self._count} frames saved  ({mb:.1f} MB)  "
                f"in {elapsed:.1f}s  ({avg_hz:.1f} Hz avg)"
                + (f"  [{self._dropped} dropped]" if self._dropped else ""))


# ─────────────────────────────────────────────────────────────────────────────
# Recorder
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    # ── folder picker ────────────────────────────────────────────────────────
    folder = pick_folder()
    if folder is None:
        print("No folder selected — exiting.")
        return 0
    os.makedirs(folder, exist_ok=True)
    print(f"Saving frames to: {folder}")

    # ── metadata CSV ─────────────────────────────────────────────────────────
    csv_path = os.path.join(folder, "metadata.csv")
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp_ns", "seq", "camera_height_m", "filename"])
    csv_lock = threading.Lock()

    # ── Zenoh ────────────────────────────────────────────────────────────────
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)

    metrics = Metrics()
    stop = threading.Event()

    def on_frame(sample):
        if stop.is_set():
            return
        try:
            raw = bytes(sample.payload)
            ts_ns, seq, cam_h, jpeg = proto.unpack_frame(raw)

            filename = f"{ts_ns}.jpg"
            filepath = os.path.join(folder, filename)

            with open(filepath, "wb") as f:
                f.write(jpeg)

            with csv_lock:
                writer.writerow([ts_ns, seq, f"{cam_h:.4f}", filename])
                csv_file.flush()

            line = metrics.update(seq=seq, size_bytes=len(jpeg),
                                  capture_ns=ts_ns)
            if line:
                print(f"  {line}")

        except proto.ProtocolError as e:
            print(f"  protocol error: {e}")
        except Exception as e:
            print(f"  save error: {e}")

    z.declare_subscriber(K["camera_frame"], on_frame)
    print(f"Subscribed to '{K['camera_frame']}'  (Ctrl+C to stop)")
    print(f"Metadata log: {csv_path}")

    # ── wait for Ctrl+C ─────────────────────────────────────────────────────
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        while not stop.is_set():
            stop.wait(timeout=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{metrics.summary()}")
        csv_file.close()
        z.close()
        print("Done.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
