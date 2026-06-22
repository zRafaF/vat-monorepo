"""
VAT — fetch a FULL-RES archived frame by seq from the robot (run on host/server)
================================================================================
The robot keeps a full-res twin of every transmitted frame in a local,
size-capped archive (SQLite index + JPEGs). The live stream the server ingests
is downscaled (e.g. 1036x518); when you want to *inspect* a moment in full
resolution, fetch that frame's full-res original by its ``seq`` — the same seq
the live frame carried.

    make fetch_frame SEQ=1234
    python tools/fetch_archive.py --seq 1234 [--out frame.jpg] [--timeout 5]

The reply is the full-res JPEG plus the original timestamp + camera height, so
you can line it up against the recorded pose trajectory.

Env: ZENOH_ROUTER (default tcp/127.0.0.1:7447), ROBOT_NAME (default go2).
Deps: eclipse-zenoh (the client env already has it).
"""

from __future__ import annotations

import os
import sys
import argparse

import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER     = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
K = proto.keys(ROBOT_NAME)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Fetch a full-res archived Theta frame by seq")
    ap.add_argument("--seq", type=int, required=True, help="frame seq to fetch")
    ap.add_argument("--out", default=None,
                    help="output file (default archive_seq<seq>.jpg)")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="reply timeout seconds (default 5)")
    args = ap.parse_args()

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)

    sel = f"{K['camera_archive_get']}?seq={args.seq}"
    payload = None
    err = None
    try:
        for reply in z.get(sel, timeout=args.timeout):
            if reply.ok:
                payload = bytes(reply.result.payload)
                break
            try:
                err = bytes(reply.err.payload).decode(errors="replace")
            except Exception:
                err = "error reply"
    finally:
        z.close()

    if payload is None:
        msg = f": {err}" if err else ""
        print(f"No archived frame for seq={args.seq}{msg}")
        print("  - Is the robot container running with ARCHIVE_ENABLE=true?")
        print("  - Is that seq still within the rolling archive window?")
        print(f"  - Router reachable at {ROUTER}?")
        return 1

    ts_ns, seq, cam_h, jpeg = proto.unpack_frame(payload)
    out = args.out or f"archive_seq{seq:09d}.jpg"
    with open(out, "wb") as f:
        f.write(jpeg)
    print(f"saved {out}  ({len(jpeg)//1024}kB)  seq={seq}  "
          f"ts={ts_ns}  cam_h={cam_h:.2f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
