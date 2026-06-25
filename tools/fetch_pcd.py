#!/usr/bin/env python3
"""
VAT — fetch & inspect ONE PRISM point cloud  (diagnostic)
=========================================================
Queries the server's snapshot queryable over Zenoh, decodes the wire payload,
prints stats, and saves the cloud so you can open it in your *known-good*
``pano_viz.py`` (Open3D). This isolates the question:

    is a "messed up" cloud a STREAMING/CODEC problem, or a RENDERING problem?

If the saved cloud looks correct in pano_viz, the bytes on the wire are fine and
the bug is in the live viewer's rendering. If it's wrong here, the stats below
(NaN / outlier counts, bbox) point straight at the cause.

Usage
-----
  cd client && uv run python ../tools/fetch_pcd.py
  ZENOH_ROUTER=tcp/<ip>:7447 uv run python ../tools/fetch_pcd.py --out ~/Downloads/pcd
Outputs (timestamped):  <out>/pcd_<ts>.npz   (pano_viz: points + colors[0..255])
                        <out>/pcd_<ts>.ply   (if open3d present)
                        <out>/pcd_<ts>.bin   (raw wire bytes, for deep inspection)
"""
from __future__ import annotations

import os
import sys
import time
import struct
import argparse
import threading

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
import vat_protocol as proto  # noqa: E402
import vat_blockmap as bm  # noqa: E402
import zenoh  # noqa: E402

ROUTER        = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
CUBE_SIZE     = float(os.environ.get("CUBE_SIZE", "1.0"))
K = proto.keys(ROBOT_NAME, SERVER_PREFIX)
_ENC = {0: "RAW_F32", 1: "ZLIB_U8", 2: "ZLIB_QUANT"}


def _write_ply(path, xyz, rgb):
    """Minimal binary little-endian PLY (no deps)."""
    rgb8 = np.clip(rgb, 0, 1)
    rgb8 = (rgb8 * 255 + 0.5).astype(np.uint8)
    hdr = (f"ply\nformat binary_little_endian 1.0\nelement vertex {xyz.shape[0]}\n"
           "property float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           "end_header\n").encode()
    rec = np.empty(xyz.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                        ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec["r"], rec["g"], rec["b"] = rgb8[:, 0], rgb8[:, 1], rgb8[:, 2]
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(rec.tobytes())


def _stats(tag, xyz, rgb):
    n = xyz.shape[0]
    finite = np.isfinite(xyz).all(axis=1)
    far = int((np.abs(xyz).max(axis=1) > 50.0).sum()) if n else 0
    print(f"[{tag}] {n} pts   non-finite {int((~finite).sum())}   |coord|>50m {far}")
    fx = xyz[finite]
    if fx.shape[0]:
        print(f"       bbox {np.round(fx.min(0), 2)} .. {np.round(fx.max(0), 2)}  "
              f"span {np.round(fx.max(0) - fx.min(0), 2)} m")


def _save(out, tag, xyz, rgb):
    ts = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(out, f"pcd_{tag}_{ts}")
    rgb01 = rgb.astype(np.float32) / 255.0 if rgb.dtype != np.float32 or rgb.max() > 1.0 + 1e-6 else rgb
    np.savez(base + ".npz", points=xyz.astype(np.float32),
             colors=(np.clip(rgb01, 0, 1) * 255).astype(np.uint8))   # pano_viz format
    try:
        _write_ply(base + ".ply", xyz, np.clip(rgb01, 0, 1))
    except Exception as e:
        print(f"  (ply skipped: {e})")
    print(f"  saved {base}.npz / .ply   ← open in pano_viz.py")


def fetch_full(z, timeout):
    """The SERVER'S canonical full cloud (engine.get_point_cloud_snapshot)."""
    data = None
    for reply in z.get(K["pcd_snapshot"], timeout=timeout):
        try:
            if reply.ok:
                buf = bytes(reply.result.payload)
                if data is None or len(buf) > len(data):
                    data = buf
        except Exception:
            pass
    if not data or len(data) <= 20:
        print("✗ FULL: no snapshot reply (server mapping yet?)")
        return None
    enc = struct.unpack_from("!iiiiii", data, 0)[5]
    print(f"[full] wire {len(data)/1e6:.3f} MB  encoding {_ENC.get(enc, enc)}")
    v, xyz, rgb, *_ = proto.unpack_pcd(data)              # rgb in [0,1]
    return xyz, (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def fetch_blocks(z, timeout):
    """What the CLIENT reconstructs from the block-sync manifest + Draco bundles."""
    store = bm.ClientBlockStore(CUBE_SIZE)
    man = {}
    evt = threading.Event()

    def on_man(sample):
        nonlocal man
        try:
            man = bm.unpack_manifest(bytes(sample.payload)); evt.set()
        except Exception:
            pass

    z.declare_subscriber(K["pcd_manifest"], on_man)
    if not evt.wait(timeout):
        print("✗ BLOCKS: no manifest (is block-sync server running?)")
        return None
    need, _ = bm.diff_manifest({}, man)
    print(f"[blocks] manifest {len(man)} cubes; requesting all…")
    for reply in z.get(K["pcd_blocks"], payload=bm.pack_request(need), timeout=timeout):
        if reply.ok:
            store.apply_bundle_bytes(bytes(reply.result.payload))
    return store.merged()


def main():
    ap = argparse.ArgumentParser(description="Fetch + inspect a live PRISM cloud")
    ap.add_argument("--out", default=os.path.expanduser("~/Downloads/pcd"))
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--blocks", action="store_true",
                    help="reconstruct the CLIENT view via block-sync instead of the full snapshot")
    ap.add_argument("--both", action="store_true",
                    help="fetch BOTH (server full + client blocks) to compare in pano_viz")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    print(f"Connected to {ROUTER}")
    try:
        if args.both or not args.blocks:
            r = fetch_full(z, args.timeout)
            if r:
                _stats("full", *r); _save(args.out, "full", *r)
        if args.both or args.blocks:
            r = fetch_blocks(z, args.timeout)
            if r:
                _stats("blocks", *r); _save(args.out, "blocks", *r)
    finally:
        z.close()
    if args.both:
        print("→ open BOTH npz in pano_viz: if 'full' is misaligned the SERVER map is; "
              "if only 'blocks' is, the client/sync is.")


if __name__ == "__main__":
    main()
