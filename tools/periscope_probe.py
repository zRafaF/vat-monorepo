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

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "common"))
sys.path.insert(0, os.path.join(_REPO, "client"))       # for vat_client._Decoder
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
    ap.add_argument("--decode", action="store_true",
                    help="also DECODE each frame with the viewer's exact decoder "
                         "(vat_client._Decoder) — proves the client machine can decode "
                         "this codec, separating 'robot not publishing' from 'client "
                         "can't decode' from a viewer-only display bug.")
    ap.add_argument("--save", default="",
                    help="with --decode: write the newest decoded frame to this path "
                         "(e.g. /tmp/peri.png) so you can eyeball the raw stream.")
    args = ap.parse_args()
    aw, ah = (args.aspect.split(":") + ["1"])[:2]

    decoder = None
    if args.decode:
        try:
            from vat_client.periscope_view import _Decoder
            decoder = _Decoder()
        except Exception as e:
            print(f"  [--decode] cannot load decoder: {e}\n"
                  f"  (H.26x needs PyAV: `pip install av`; MJPEG needs opencv-python)")
            decoder = None

    K = proto.keys(ROBOT_NAME)
    print(f"Periscope probe -> router={ROUTER} robot={ROBOT_NAME}")
    print(f"  request : {K['periscope_request']}")
    print(f"  frames  : {K['periscope_frame']}")
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    z = zenoh.open(conf)

    state = {"n": 0, "bytes": 0, "t0": time.time(), "last": time.time(),
             "codecs": set(), "kf": 0, "dec": 0, "dec_err": None, "saved": False}

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
        if decoder is not None:
            try:
                rgb = decoder.decode(f.codec, f.payload)
            except Exception as e:
                state["dec_err"] = str(e)[:80]
                rgb = None
            if rgb is not None:
                state["dec"] += 1
                if args.save and not state["saved"]:
                    try:
                        import cv2
                        cv2.imwrite(args.save, rgb[:, :, ::-1])   # RGB->BGR
                        print(f"  [--save] wrote decoded frame -> {args.save} "
                              f"({rgb.shape[1]}x{rgb.shape[0]})")
                        state["saved"] = True
                    except Exception as e:
                        print(f"  [--save] failed: {e}")
        if state["n"] <= 5 or state["n"] % 15 == 0:
            dec = f"  dec={state['dec']}" if decoder is not None else ""
            print(f"  #{state['n']:<4} {_CODEC.get(f.codec, f.codec):5} "
                  f"{f.width}x{f.height} {'opt' if f.optical else 'dig'} "
                  f"kf={int(f.is_keyframe)} fov={f.hfov_deg:.0f} "
                  f"native_w={f.native_w} {len(f.payload)/1024:.1f}KB "
                  f"yaw={f.yaw_deg:.0f} pitch={f.pitch_deg:.0f}{dec}")

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
                dec = ""
                if decoder is not None:
                    dec = f"  decoded={state['dec']}"
                    if state["dec"] == 0:
                        dec += (f" (0 DECODED — client can't decode {state['codecs']}; "
                                f"err={state['dec_err']})")
                print(f"  --- {dt:5.0f}s: {state['n']} frames  {fps:.1f} fps  "
                      f"{kbps:.0f} KB/s  codecs={state['codecs']} kf={state['kf']}  "
                      f"last {age:.1f}s ago{dec} ---")
            if args.secs and dt >= args.secs:
                break
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
