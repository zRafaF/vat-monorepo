"""
VAT — Zenoh probe
=================
Tiny diagnostic: subscribe to a set of keys on the router and print the byte
size of every sample that arrives (never the payload itself, so it can't flood
the terminal).  Used to localise "pose arrives but point cloud doesn't".

Run from the repo root in the CLIENT env:

    cd client && uv run python ../tools/zenoh_probe.py
    # or point at a specific router / keys:
    ZENOH_ROUTER=tcp/100.76.214.80:7447 cd client && uv run python ../tools/zenoh_probe.py
    cd client && uv run python ../tools/zenoh_probe.py server/prism/status server/prism/pcd_delta

Defaults to the VAT server keys (status, pcd_delta, pcd_snapshot, pose).
Ctrl+C to stop.
"""

from __future__ import annotations

import os
import sys
import time
import zenoh

ZENOH_ROUTER = os.environ.get("ZENOH_ROUTER", "tcp/100.76.214.80:7447")
ROBOT_NAME   = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")

DEFAULT_KEYS = [
    f"{SERVER_PREFIX}/status",        # tiny JSON — server-published
    f"{SERVER_PREFIX}/pcd_delta",     # small-ish binary — server-published
    f"{SERVER_PREFIX}/pcd_snapshot",  # large binary — server-published
    f"{ROBOT_NAME}/prism/pose",       # tiny — robot-published (known good)
]

keys = sys.argv[1:] or DEFAULT_KEYS

# Per-key counters (avoid printing payloads).
counts: dict[str, int] = {k: 0 for k in keys}
t_start = time.time()


def make_cb(key: str):
    def _cb(sample):
        counts[key] += 1
        n = len(bytes(sample.payload))
        print(f"[{time.time() - t_start:6.1f}s] GOT  {key:<28} "
              f"#{counts[key]:<4} {n:>10,} bytes", flush=True)
    return _cb


def main():
    print(f"Connecting to {ZENOH_ROUTER} (client mode)…", flush=True)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    z = zenoh.open(conf)
    print("Connected. Subscribing:", flush=True)
    subs = []
    for k in keys:
        subs.append(z.declare_subscriber(k, make_cb(k)))
        print(f"  - {k}", flush=True)
    print("Listening… (Ctrl+C to stop)\n", flush=True)
    try:
        while True:
            time.sleep(5.0)
            elapsed = time.time() - t_start
            summary = "  ".join(f"{k.split('/')[-1]}={counts[k]}" for k in keys)
            print(f"--- {elapsed:6.1f}s totals: {summary} ---", flush=True)
    except KeyboardInterrupt:
        print("\nStopping.", flush=True)
    finally:
        z.close()


if __name__ == "__main__":
    main()
