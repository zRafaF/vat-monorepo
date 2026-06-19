"""
VAT bring-up — Stage 0: connectivity check
===========================================
Verifies the whole transport is alive before you debug anything visual:
  * Zenoh router reachable
  * robot bridge present (liveliness + topic discovery)
  * mapping server / pose stream present (liveliness)
  * measured rate (Hz) on the key streams

Run from any machine that can reach the Zenoh router:

    ZENOH_ROUTER=tcp/<server-ip>:7447 ROBOT_NAME=go2 python tools/check_link.py

No Rerun/OpenCV needed — text only.
"""

from __future__ import annotations

import os
import sys
import json
import time

import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER     = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
WATCH_S    = float(os.environ.get("WATCH_S", "4.0"))

K = proto.keys(ROBOT_NAME, SERVER_PREFIX)


def open_session():
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    return zenoh.open(conf)


def check_liveliness(z, key, label):
    try:
        alive = any(r.ok for r in z.liveliness().get(key, timeout=2.0))
    except Exception as e:
        print(f"  {label:24s} ERROR ({e})")
        return
    print(f"  {label:24s} {'ALIVE' if alive else 'absent'}   ({key})")


def discover_topics(z):
    print("\n[bridge] ROS topics advertised on Zenoh:")
    try:
        replies = list(z.get(f"{ROBOT_NAME}/system/get_topics", timeout=3.0))
        if not replies:
            print("  (no reply — is dynamic_bridge.py running?)")
            return
        for r in replies:
            if r.ok:
                topics = json.loads(bytes(r.result.payload).decode())
                if not topics:
                    print("  (bridge running, but no topics registered yet)")
                for name, typ in sorted(topics.items()):
                    print(f"  {name:40s} {typ}")
    except Exception as e:
        print(f"  ERROR querying topics: {e}")


def measure_rates(z):
    keys = {
        "equirectangular/image": f"{ROBOT_NAME}/rt/equirectangular/image",
        "sportmodestate":        f"{ROBOT_NAME}/rt/sportmodestate",
        "camera/frame (decim.)": K["camera_frame"],
        "pose (fused)":          K["pose"],
        "pose_correction":       K["pose_correction"],
        "pcd_delta":             K["pcd_delta"],
        "trajectory":            K["trajectory"],
    }
    counts = {label: 0 for label in keys}
    subs = []
    for label, key in keys.items():
        def cb(sample, label=label):
            counts[label] += 1
        subs.append(z.declare_subscriber(key, cb))

    print(f"\n[rates] sampling for {WATCH_S:.0f}s ...")
    time.sleep(WATCH_S)
    for label in keys:
        hz = counts[label] / WATCH_S
        bar = "█" * min(40, int(hz)) if hz >= 1 else ("·" if counts[label] else "")
        print(f"  {label:24s} {hz:6.1f} Hz  {bar}")


def main():
    print(f"VAT link check → router={ROUTER}  robot={ROBOT_NAME}")
    try:
        z = open_session()
    except Exception as e:
        print(f"FATAL: cannot open Zenoh session: {e}")
        sys.exit(1)
    print("Zenoh session open.\n")

    print("[liveliness]")
    check_liveliness(z, f"{ROBOT_NAME}/system/liveliness", "robot bridge")
    check_liveliness(z, K["live_server"],                  "mapping server")
    check_liveliness(z, K["live_pose"],                    "robot pose fuser")

    discover_topics(z)
    measure_rates(z)

    print("\nDone. (If a stream is 0 Hz, fix that stage before moving on.)")
    z.close()


if __name__ == "__main__":
    main()
