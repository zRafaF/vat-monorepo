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

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
import vat_protocol as proto  # noqa: E402
import zenoh  # noqa: E402

ROUTER        = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
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


def main():
    ap = argparse.ArgumentParser(description="Fetch + inspect a PRISM snapshot")
    ap.add_argument("--out", default=os.path.expanduser("~/Downloads/pcd"))
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    print(f"Connected to {ROUTER}; querying '{K['pcd_snapshot']}' …")

    data = None
    for reply in z.get(K["pcd_snapshot"], timeout=args.timeout):
        try:
            if reply.ok:
                buf = bytes(reply.result.payload)
                if data is None or len(buf) > len(data):
                    data = buf
        except Exception:
            pass
    z.close()
    if not data:
        print("✗ no snapshot reply — is the mapping server running & has it mapped?")
        return

    magic, version, n_hdr, is_snap, since_v, enc = struct.unpack_from("!iiiiii", data, 0)
    print(f"wire: {len(data)/1e6:.3f} MB  encoding={_ENC.get(enc, enc)}  "
          f"version={version}  n(header)={n_hdr}  snapshot={bool(is_snap)}")

    v, xyz, rgb, snap, sv = proto.unpack_pcd(data)
    n = xyz.shape[0]
    if n == 0:
        print("decoded 0 points (empty/cleared map).")
        return
    finite = np.isfinite(xyz).all(axis=1)
    far = np.abs(xyz).max(axis=1) > 50.0
    print(f"decoded: {n} pts   {len(data)/max(n,1):.2f} wire-bytes/pt")
    print(f"  non-finite: {int((~finite).sum())}   |coord|>50m: {int(far.sum())}")
    fx = xyz[finite]
    if fx.shape[0]:
        print(f"  bbox min = {np.round(fx.min(0), 3)}   max = {np.round(fx.max(0), 3)}"
              f"   span = {np.round(fx.max(0) - fx.min(0), 3)} m")
    print(f"  rgb range = [{rgb.min():.2f}, {rgb.max():.2f}]   sample xyz = {np.round(xyz[0], 3)}")

    ts = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.out, f"pcd_{ts}")
    np.savez(base + ".npz", points=xyz.astype(np.float32),
             colors=(np.clip(rgb, 0, 1) * 255).astype(np.uint8))   # pano_viz format
    with open(base + ".bin", "wb") as f:
        f.write(data)
    try:
        _write_ply(base + ".ply", xyz, rgb)
        ply_note = base + ".ply"
    except Exception as e:
        ply_note = f"(ply skipped: {e})"
    print(f"saved:\n  {base}.npz   ← open this in pano_viz.py\n  {ply_note}\n  {base}.bin (raw)")


if __name__ == "__main__":
    main()
