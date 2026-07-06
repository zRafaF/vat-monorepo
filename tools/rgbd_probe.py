"""
VAT - RGBD probe: validate the D435i single-frame stream WITHOUT the VisPy viewer.

Publishes an RgbdRequest (kind/fps/range) + keepalive, subscribes to the frames, and
prints kind/size/rate. With --decode it decodes with the viewer's exact decoder (depth
-> colormap / color -> rgb) and --save writes the newest frame to disk — cleanly
separating "relay/realsense not publishing" from "client can't decode".

    ZENOH_ROUTER=tcp/100.76.214.80:7447 ROBOT_NAME=go2 python tools/rgbd_probe.py --kind depth
    python tools/rgbd_probe.py --kind color --range 3 --decode --save /tmp/rgbd.png
"""
from __future__ import annotations
import argparse, os, sys, time
import zenoh

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "common"))
sys.path.insert(0, os.path.join(_REPO, "client"))
import vat_protocol as proto  # noqa: E402

ROUTER = os.environ.get("ZENOH_ROUTER", os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447"))
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
_KIND = {"off": proto.RGBD_KIND_OFF, "depth": proto.RGBD_KIND_DEPTH, "color": proto.RGBD_KIND_COLOR}
_KNAME = {v: k for k, v in _KIND.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=list(_KIND), default="depth")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--range", type=float, default=4.0, help="max range (m)")
    ap.add_argument("--gate", action="store_true", help="range-gate (only send when in range)")
    ap.add_argument("--decode", action="store_true", help="decode with the viewer's decoder")
    ap.add_argument("--save", default="", help="with --decode: write newest frame here")
    ap.add_argument("--secs", type=float, default=0.0)
    args = ap.parse_args()

    decoder = None
    if args.decode:
        try:
            from vat_client.rgbd_view import RgbdClient
            decoder = RgbdClient.__new__(RgbdClient)   # only need _decode()
        except Exception as e:
            print(f"  [--decode] cannot load decoder: {e} (need opencv-python)")
            decoder = None

    K = proto.keys(ROBOT_NAME)
    print(f"RGBD probe -> router={ROUTER} robot={ROBOT_NAME}")
    print(f"  request : {K['rgbd_request']}\n  frames  : {K['rgbd_frame']}")
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    z = zenoh.open(conf)
    st = {"n": 0, "bytes": 0, "t0": time.time(), "last": time.time(), "dec": 0, "saved": False}

    def on_frame(sample):
        try:
            f = proto.unpack_rgbd_frame(bytes(sample.payload))
        except proto.ProtocolError as e:
            print(f"  [bad frame] {e}"); return
        st["n"] += 1; st["bytes"] += len(f.payload); st["last"] = time.time()
        if decoder is not None:
            try:
                rgb = decoder._decode(f)
            except Exception as e:
                rgb = None; print(f"  [decode err] {e}")
            if rgb is not None:
                st["dec"] += 1
                if args.save and not st["saved"]:
                    import cv2
                    cv2.imwrite(args.save, rgb[:, :, ::-1]); st["saved"] = True
                    print(f"  [--save] wrote {args.save} ({rgb.shape[1]}x{rgb.shape[0]})")
        if st["n"] <= 5 or st["n"] % 20 == 0:
            print(f"  #{st['n']:<4} {_KNAME.get(f.kind, f.kind):5} {f.width}x{f.height} "
                  f"near={f.min_depth_mm/1000:.2f}m {len(f.payload)/1024:.1f}KB"
                  + (f" dec={st['dec']}" if decoder is not None else ""))

    z.declare_subscriber(K["rgbd_frame"], on_frame)
    pub = z.declare_publisher(K["rgbd_request"])
    flags = proto.RGBD_FLAG_RANGE_GATE if args.gate else 0
    seq = 0
    print(f"Requesting kind={args.kind} fps={args.fps} range={args.range}m gate={args.gate}. Ctrl+C to stop.\n")
    try:
        while True:
            seq += 1
            r = proto.RgbdRequest(kind=_KIND[args.kind], fps=args.fps,
                                  max_range_mm=int(args.range * 1000), flags=flags, seq=seq)
            pub.put(proto.pack_rgbd_request(r))
            time.sleep(1.0)
            dt = time.time() - st["t0"]; fps = st["n"] / dt if dt > 0 else 0
            kbps = st["bytes"] / dt / 1024 if dt > 0 else 0
            if st["n"] == 0:
                print(f"  [{dt:5.0f}s] NO FRAMES yet - relay up? realsense2_camera publishing? (RGBD_ENABLE=1)")
            else:
                dec = f" decoded={st['dec']}" if decoder is not None else ""
                print(f"  --- {dt:5.0f}s: {st['n']} frames {fps:.1f} fps {kbps:.0f} KB/s{dec} "
                      f"last {time.time()-st['last']:.1f}s ago ---")
            if args.secs and dt >= args.secs:
                break
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
