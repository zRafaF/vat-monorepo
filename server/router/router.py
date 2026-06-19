"""
VAT — Zenoh Router microservice
===============================
A standalone Zenoh **router** node, run from pure Python (no `zenohd` binary and
no Docker).  All robot/server/client processes connect to it; it routes traffic
between them — including the `dog → router → client` pose relay.

Why a router (not just peers)?
------------------------------
Per the Zenoh deployment model, a node in **router mode** routes data on behalf
of other applications, so clients only keep a single session open to it.  This
is exactly the topology VAT uses (robot, mapping server and client all connect
to one router on the server host).

Note on `python -m zenoh.router`
--------------------------------
The eclipse-zenoh Python package does **not** ship a `zenoh.router` module
(that entry point only exists for the `zenohd` Rust binary).  The supported
pure-Python equivalent is to open a session in `router` mode with a `listen`
endpoint and keep the process alive — which is what this script does.

Run
---
    # isolated env (its own deps — does not clash with the mapping server)
    cd server/router && uv sync
    uv run python router.py
    # or, after `source .venv/bin/activate`:  python router.py

Environment
-----------
  ZENOH_LISTEN   listen endpoint(s), comma-separated   (default tcp/0.0.0.0:7447)
  ZENOH_CONNECT  optional peer routers to mesh with    (comma-separated)
  ZENOH_CONFIG   optional path to a full JSON5 config (overrides the above)
"""

from __future__ import annotations

import os
import sys
import json
import time
import signal
import logging

import zenoh

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("zenoh-router")

LISTEN  = os.environ.get("ZENOH_LISTEN", "tcp/0.0.0.0:7447")
CONNECT = os.environ.get("ZENOH_CONNECT", "")
CONFIG  = os.environ.get("ZENOH_CONFIG", "")


def _json5_list(csv: str) -> str:
    items = [e.strip() for e in csv.split(",") if e.strip()]
    return json.dumps(items)


def build_config() -> zenoh.Config:
    if CONFIG:
        log.info(f"Loading Zenoh config from {CONFIG}")
        return zenoh.Config.from_file(CONFIG)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"router"')
    conf.insert_json5("listen/endpoints", _json5_list(LISTEN))
    if CONNECT:
        conf.insert_json5("connect/endpoints", _json5_list(CONNECT))
    return conf


def main():
    conf = build_config()
    log.info(f"Starting Zenoh router (mode=router, listen={LISTEN}"
             + (f", connect={CONNECT}" if CONNECT else "") + ")")
    try:
        session = zenoh.open(conf)
    except Exception as e:
        log.error(f"Failed to start router: {e}")
        sys.exit(1)

    log.info(f"Router up. zid={session.zid()}  (Ctrl+C to stop)")

    stop = {"flag": False}

    def _sig(_s, _f):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        while not stop["flag"]:
            time.sleep(0.5)
    finally:
        log.info("Shutting down router.")
        session.close()


if __name__ == "__main__":
    main()
