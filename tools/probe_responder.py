"""
VAT — Latency probe responder
==============================
Tiny echo service for the latency benchmark. Run it ON THE HOST whose link you
want to characterise (the ROBOT for the pose path, or the SERVER). It answers
``vat/probe/<name>`` queries by immediately echoing the query payload, so a remote
``latency_probe.py`` can measure the ZENOH round-trip to this host and compare it
against a raw ICMP ping (the medium) to separate the two.

The reply is sent at REAL_TIME priority so the echo itself isn't queued behind bulk
traffic — we want to measure the transport, not the responder.

Run (on the robot, inside its env / container that can reach the router):

    ZENOH_ROUTER=tcp/<router-ip>:7447 PROBE_NAME=robot python tools/probe_responder.py
    # or:  make probe-responder NAME=robot
"""

from __future__ import annotations

import os
import sys
import time

import zenoh

ZENOH_ROUTER = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
PROBE_NAME = os.environ.get("PROBE_NAME") or (sys.argv[1] if len(sys.argv) > 1 else "server")
KEY = f"vat/probe/{PROBE_NAME}"


def _on_query(query):
    try:
        payload = bytes(query.payload) if query.payload is not None else b""
        try:
            query.reply(query.key_expr, payload, priority=zenoh.Priority.REAL_TIME)
        except TypeError:
            query.reply(query.key_expr, payload)
    except Exception:
        pass


def main():
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    z = zenoh.open(conf)
    _q = z.declare_queryable(KEY, _on_query)
    print(f"[probe-responder] echoing '{KEY}' via {ZENOH_ROUTER}  (Ctrl+C to stop)", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("\n[probe-responder] stopping.")
    finally:
        z.close()


if __name__ == "__main__":
    main()
