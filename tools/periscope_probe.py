"""
VAT - Remote Periscope probe
============================
Localises "I see the frustum but no video": talks to the robot's periscope
WITHOUT the full VisPy client. It publishes a ViewRequest (and keeps it alive so
the robot's viewer-timeout doesn't stop the stream), subscribes to the encoded
frame stream, and prints codec / size / dimensions / rate for every frame.

If this prints frames but the viewer shows nothing -> the client can't DECODE
(most likely PyAV missing for H.26x; try PERISCOPE_CODEC=mjpeg on the robot).
If this prints nothing -> the robot isn't PUBLISHING (periscope service down, or
no viewer-active, or encoder failed) -> check the robot's [periscope] logs.

Run from the repo root in any env with eclipse-zenoh:

    ZENOH_ROUTER=tcp/100.76.214.80:7447 ROBOT_NAME=go2 python tools/periscope_probe.py
    # aim it too:
    python tools/periscope_probe.py --yaw 30 --pitch -10 --fov 60 --tier 720 --aspect 16:9
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402

ROUTER = os.environ.get("ZENOH_ROUTER", os.environ.get("ZENOH_CONNECT",
                        "tcp/127.0.0.1:7447"))
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
_CODEC = {proto.PSCOPE_CODEC_MJPEG: "mjpeg", proto.PSCOPE_CODEC_H264: "h264",
          proto.PSCOPE_CODEC_HEVC: "hevc"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--tier", type=int, default=480)
    ap.add_argument("--aspect", default="1:1")
    ap.add_argument("--secs", type=float, default=0.0, help="stop after N s (0=forever)")
    args = ap.parse_args()
    aw, ah = (args.aspect.split(":") + ["1"])[:2]

    K = proto.keys(ROBOT_NAME)
    print(f"Periscope probe -> router={ROUTER} robot={ROBOT_NAME}")
    print(f"  request : {K['periscope_request']}")
    print(f"  frames  : {K['periscope_frame']}")
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    z = zenoh.open(conf)

    state = {"n": 0, "bytes": 0, "t0": time.time(), "last": time.time(),
             "codecs": set(), "kf": 0}

    def on_frame(sample):
        try:
            f = proto.unpack_periscope_frame(bytes(sample.payload))
        except proto.ProtocolError as e:
            print(f"  [bad frame] {e}")
            return
        state["n"] += 1
        state["bytes"] += len(f.payload)
        state["codecs"].add(_CODEC.get(f.codec, f.codec))
        state["kf"] += 1 if f.is_keyframe else 0
        state["last"] = time.time()
        if state["n"] <= 5 or state["n"] % 15 == 0:
            print(f"  #{state['n']:<4} {_CODEC.get(f.codec, f.codec):5} "
                  f"{f.width}x{f.height} {'opt' if f.optical else 'dig'} "
                  f"kf={int(f.is_keyframe)} fov={f.hfov_deg:.0f} "
                  f"native_w={f.native_w} {len(f.payload)/1024:.1f}KB "
                  f"yaw={f.yaw_deg:.0f} pitch={f.pitch_deg:.0f}")

    z.declare_subscriber(K["periscope_frame"], on_frame)
    pub = z.declare_publisher(K["periscope_request"])

    def send(seq):
        v = proto.ViewRequest(yaw_deg=args.yaw, pitch_deg=args.pitch, hfov_deg=args.fov,
                              res_tier=args.tier, aspect_w=int(aw), aspect_h=int(ah),
                              seq=seq, timestamp_ns=time.time_ns())
        pub.put(proto.pack_view_request(v))

    print(f"Requesting yaw={args.yaw} pitch={args.pitch} fov={args.fov} "
          f"tier={args.tier} aspect={args.aspect}. Ctrl+C to stop.\n")
    seq = 0
    try:
        while True:
            seq += 1
            send(seq)                      # keepalive so the robot keeps streaming
            time.sleep(1.0)
            dt = time.time() - state["t0"]
            fps = state["n"] / dt if dt > 0 else 0
            kbps = (state["bytes"] / dt / 1024) if dt > 0 else 0
            age = time.time() - state["last"]
            if state["n"] == 0:
                print(f"  [{dt:5.0f}s] NO FRAMES yet - is the robot periscope up? "
                      f"(check robot [periscope] log; PERISCOPE_ENABLE=1)")
            else:
                print(f"  --- {dt:5.0f}s: {state['n']} frames  {fps:.1f} fps  "
                      f"{kbps:.0f} KB/s  codecs={state['codecs']} kf={state['kf']}  "
                      f"last {age:.1f}s ago ---")
            if args.secs and dt >= args.secs:
                break
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
