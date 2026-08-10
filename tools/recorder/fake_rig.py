#!/usr/bin/env python3
"""
VAT — fake rig: a synthetic robot + cloud on a real Zenoh bus
============================================================
Publishes the **real wire messages on the real Zenoh keys**, at realistic rates, so
``vat_record.py`` (and any other client-side tool) can be exercised end to end without
a robot, a GPU or the mapping server. This is how the recorder's *live* path — Zenoh
subscriptions, the archive query/reply, the block-repair query/reply, the periscope
elementary stream — gets tested at all; the recorder's own ``--selftest`` only drives
its handlers directly and never touches a bus.

It plays both ends of the system:

**Robot side** (peer, like ``theta_camera.py`` / ``pose_fuser.py``)
  * ``{robot}/prism/camera/frame`` — decimated JPEG panorama, ~2.5 Hz, with a monotonic
    ``seq`` and a per-frame ``camera_height``
  * ``{robot}/prism/pose`` — fused pose, 30 Hz, walking a circle
  * ``{robot}/prism/periscope/frame`` — a **genuine H.264 elementary stream** cut into
    per-frame access units by ffprobe, so the recorder's ffmpeg remux is really tested
    (``--codec mjpeg`` for the JPEG fallback instead)
  * ``{robot}/prism/camera/archive/get`` — queryable serving full-res twins by ``seq``,
    replying ``reply_err`` outside the rolling window, exactly like ``FrameArchive.get``

**Cloud side**
  * ``{server}/pcd/push`` then ``{server}/pcd/manifest`` — in that order, as
    ``block_publisher.py`` does, from a real ``vat_blockmap.BlockGrid`` that grows
  * ``{server}/pcd/blocks`` — queryable answering a client's cube-key request with a
    Draco bundle
  * ``{server}/trajectory``, ``{server}/pose_correction`` (gated: every other submap),
    ``{server}/esdf_slice``, ``{server}/status``

Usage
-----
::

    # terminal 1 — the real router
    cd server/router && ZENOH_LISTEN=tcp/127.0.0.1:7447 uv run python router.py

    # terminal 2 — the rig
    cd client && ZENOH_ROUTER=tcp/127.0.0.1:7447 uv run python ../tools/recorder/fake_rig.py

    # terminal 3 — the recorder under test
    cd client && ZENOH_ROUTER=tcp/127.0.0.1:7447 uv run python \\
        ../tools/recorder/vat_record.py --duration 30s --scene fake --camera-height 1.15

``--drop-pushes 0.3`` sheds 30 % of pushes to prove the manifest-diff repair pull
actually heals a lossy link; ``--archive-window`` shrinks the rolling archive so the
full-res puller's miss path is exercised too.

**This is a test fixture, not part of the system.** It never runs on the robot.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_blockmap as bm          # noqa: E402
import vat_protocol as proto       # noqa: E402
import zenoh                       # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] [rig] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("fake-rig")


# ═════════════════════════════════════════════════════════════════════════════
# Media
# ═════════════════════════════════════════════════════════════════════════════


def _jpeg(width: int, height: int, tint: int) -> bytes:
    """A cheap but real JPEG of the requested size."""
    import cv2
    img = np.zeros((height, width, 3), np.uint8)
    img[:, :, 0] = tint % 256
    img[:, :, 1] = (tint * 3) % 256
    step = max(1, width // 16)
    img[:, ::step] = 255
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def have_ffmpeg() -> bool:
    """Both binaries — encoding needs ffmpeg, packet boundaries need ffprobe."""
    return bool(shutil.which("ffmpeg")) and bool(shutil.which("ffprobe"))


def h264_access_units(n_frames: int = 60, w: int = 640, h: int = 480,
                      fps: int = 15, gop: int = 15):
    """Encode a real H.264 Annex-B stream and split it into per-frame access units.

    Returns ``[(payload_bytes, is_keyframe), …]``. Uses ffprobe's per-packet ``pos`` /
    ``size`` / ``flags`` rather than hand-parsing start codes, so the boundaries are
    exactly what a decoder would see — which is the point: the recorder concatenates
    these back and remuxes with ``ffmpeg -c copy``, and that only works on real AUs.
    """
    if not have_ffmpeg():
        raise RuntimeError(
            "the H.264 periscope stream needs ffmpeg + ffprobe on PATH.\n"
            "  install:  sudo apt-get install -y ffmpeg\n"
            "  or run:   make rig ARGS=\"--codec mjpeg\"   (JPEG periscope instead)\n"
            "Note the RECORDER itself does not need ffmpeg — it only uses it for the "
            "optional periscope mp4 remux, and skips that with a log line.")
    tmp = tempfile.mkdtemp(prefix="rig-h264-")
    es = os.path.join(tmp, "s.h264")
    subprocess.run(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-f", "lavfi", "-i", f"testsrc2=size={w}x{h}:rate={fps}",
         "-frames:v", str(n_frames), "-c:v", "libx264", "-preset", "ultrafast",
         # match PERISCOPE_BITRATE in vat.env so payload sizes are realistic
         "-b:v", "1500k", "-maxrate", "1500k", "-bufsize", "3000k",
         "-g", str(gop), "-bf", "0", "-f", "h264", es], check=True)
    # JSON, not csv: `-of csv` emits packet fields in ffprobe's own internal order
    # regardless of the order requested, so positional parsing silently swaps pos/size.
    import json as _json
    out = subprocess.run(
        ["ffprobe", "-hide_banner", "-loglevel", "error", "-select_streams", "v",
         "-show_entries", "packet=pos,size,flags", "-of", "json", es],
        check=True, capture_output=True, text=True).stdout
    blob = open(es, "rb").read()
    units = []
    for pkt in _json.loads(out).get("packets", []):
        pos, size = pkt.get("pos"), pkt.get("size")
        if pos in (None, "N/A") or size in (None, "N/A"):
            continue
        p, n = int(pos), int(size)
        units.append((blob[p:p + n], "K" in str(pkt.get("flags", ""))))
    total = sum(len(u) for u, _ in units)
    if units and not (200 <= total / len(units) <= 200_000):
        raise RuntimeError(
            f"implausible mean access-unit size {total / len(units):.0f} B — the "
            f"ffprobe packet boundaries are wrong, not the encoder")
    if not units:
        raise RuntimeError("ffprobe returned no packets — cannot build the H.264 stream")
    log.info(f"[periscope] {len(units)} real H.264 access units "
             f"({sum(1 for _, k in units if k)} keyframes, "
             f"{sum(len(u) for u, _ in units) / 1024:.0f} kB)")
    return units


# ═════════════════════════════════════════════════════════════════════════════
# The rig
# ═════════════════════════════════════════════════════════════════════════════


class FakeRig:
    def __init__(self, args):
        self.args = args
        self.K = rcfg.KEYS
        self.stop = threading.Event()
        self.t0 = time.time()

        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{args.router}"]')
        conf.insert_json5("mode", f'"{args.mode}"')
        if args.mode == "peer":
            # zenoh's default peer listener is tcp/[::]:0, which fails to bind on a
            # host without IPv6 (CI containers, some Jetson setups). Bind IPv4 only.
            conf.insert_json5("listen/endpoints", f'["{args.listen}"]')
        self.z = zenoh.open(conf)
        log.info(f"connected {args.mode} → {args.router}  "
                 f"robot={rcfg.ROBOT_NAME} server={rcfg.SERVER_PREFIX}")

        cc_drop = zenoh.CongestionControl.DROP
        low = zenoh.Priority.DATA_LOW
        self.pub_frame = self.z.declare_publisher(self.K["camera_frame"])
        self.pub_pose = self.z.declare_publisher(self.K["pose"])
        self.pub_peri = self.z.declare_publisher(self.K["periscope_frame"])
        self.pub_push = self.z.declare_publisher(
            self.K["pcd_push"], congestion_control=cc_drop, priority=low)
        self.pub_man = self.z.declare_publisher(
            self.K["pcd_manifest"], congestion_control=cc_drop, priority=low)
        self.pub_traj = self.z.declare_publisher(
            self.K["trajectory"], congestion_control=cc_drop)
        self.pub_corr = self.z.declare_publisher(
            self.K["pose_correction"], congestion_control=cc_drop)
        self.pub_esdf = self.z.declare_publisher(
            self.K["esdf_slice"], congestion_control=cc_drop, priority=low)
        self.pub_status = self.z.declare_publisher(
            self.K["status"], congestion_control=cc_drop)

        # rolling full-res archive: {seq: pack_frame payload}
        self._archive = {}
        self._arch_lock = threading.Lock()
        self.z.declare_queryable(self.K["camera_archive_get"], self._on_archive_get)

        self._grid = bm.BlockGrid(cube_m=rcfg.CUBE_SIZE,
                                  crc_quant_m=max(0.06, 0.03) * 0.5)
        self._grid_lock = threading.Lock()
        self.z.declare_queryable(self.K["pcd_blocks"], self._on_blocks)

        codec = args.codec
        if codec == "auto":
            codec = "h264" if have_ffmpeg() else "mjpeg"
            if codec == "mjpeg":
                log.warning("ffmpeg/ffprobe not on PATH — periscope falls back to MJPEG. "
                            "That still exercises the recorder's periscope path end to "
                            "end; only the H.264 remux goes untested. "
                            "`sudo apt-get install -y ffmpeg` for the full check.")
        self.codec = codec
        self.units = ([] if codec == "none"
                      else h264_access_units(n_frames=args.periscope_frames)
                      if codec == "h264" else None)
        self.seq = 0
        self.pose_seq = 0
        self.peri_seq = 0
        self.map_version = 100
        self.n_pushes = 0
        self.n_dropped = 0
        self.n_archive_hits = 0
        self.n_archive_misses = 0
        self.n_block_pulls = 0
        self._tx = _jpeg(args.tx_width, args.tx_height, 7)
        self._full = _jpeg(args.full_width, args.full_height, 21)
        log.info(f"transmit {args.tx_width}x{args.tx_height} "
                 f"({len(self._tx) / 1024:.0f} kB/frame)  full-res "
                 f"{args.full_width}x{args.full_height} "
                 f"({len(self._full) / 1024:.0f} kB/frame)")

    # ── queryables ───────────────────────────────────────────────────────────
    def _on_archive_get(self, query):
        """Mirror of ``theta_camera._on_archive_get`` + ``FrameArchive.get``."""
        try:
            params = query.parameters if hasattr(query, "parameters") else \
                query.selector.parameters
            seq = int(params["seq"]) if "seq" in params else -1
            with self._arch_lock:
                payload = self._archive.get(seq)
            if payload is not None:
                query.reply(self.K["camera_archive_get"], payload)
                self.n_archive_hits += 1
            else:
                query.reply_err(f"archive seq {seq} not found".encode())
                self.n_archive_misses += 1
        except Exception as e:                                  # noqa: BLE001
            log.warning(f"archive query failed: {e}")
            try:
                query.reply_err(str(e).encode())
            except Exception:
                pass

    def _on_blocks(self, query):
        """Mirror of ``block_publisher._on_request``."""
        try:
            payload = bytes(query.payload) if query.payload is not None else b""
            keys = bm.unpack_request(payload) if payload else []
            with self._grid_lock:
                blocks = self._grid.collect(keys)
            bundle = bm.pack_bundle(blocks, self._grid.cube_m)
            query.reply(query.key_expr, bundle)
            self.n_block_pulls += 1
            log.info(f"[blocks] served {len(blocks)}/{len(keys)} cubes "
                     f"({len(bundle) / 1024:.0f} kB)")
        except Exception as e:                                  # noqa: BLE001
            log.warning(f"blocks query failed: {e}")

    # ── producers ────────────────────────────────────────────────────────────
    def _pose_loop(self):
        period = 1.0 / self.args.pose_hz
        while not self.stop.is_set():
            t = time.time() - self.t0
            ang = t * 0.3
            pos = np.array([math.cos(ang) * 2.0, math.sin(ang) * 2.0, 0.35], np.float32)
            quat = np.array([0.0, 0.0, math.sin(ang / 2), math.cos(ang / 2)], np.float32)
            vel = np.array([-math.sin(ang) * 0.6, math.cos(ang) * 0.6, 0.0], np.float32)
            self.pub_pose.put(proto.pack_pose(proto.PoseState(
                timestamp_ns=time.time_ns(), seq=self.pose_seq,
                position=pos, quaternion=quat, linear_velocity=vel,
                angular_velocity=np.array([0, 0, 0.3], np.float32),
                fix_quality=(proto.FIX_CORRECTED if self.pose_seq % 30 == 0
                             else proto.FIX_DEADRECKON))))
            self.pose_seq += 1
            self.stop.wait(period)

    def _frame_loop(self):
        period = 1.0 / self.args.frame_hz
        while not self.stop.is_set():
            ts = time.time_ns()
            self.seq += 1
            cam_h = 1.15 + 0.01 * math.sin(self.seq / 10.0)
            self.pub_frame.put(proto.pack_frame(ts, self.seq, cam_h, self._tx))
            # the full-res twin, same seq/ts/camera_height — a 1:1 mapping
            with self._arch_lock:
                self._archive[self.seq] = proto.pack_frame(ts, self.seq, cam_h,
                                                           self._full)
                for old in [s for s in self._archive
                            if s < self.seq - self.args.archive_window]:
                    del self._archive[old]
            self.stop.wait(period)

    def _periscope_loop(self):
        if self.units is None:                       # MJPEG fallback
            period = 1.0 / self.args.periscope_hz
            jpg = _jpeg(480, 360, 90)
            while not self.stop.is_set():
                self.pub_peri.put(proto.pack_periscope_frame(proto.PeriscopeFrame(
                    seq=self.peri_seq, timestamp_ns=time.time_ns(),
                    codec=proto.PSCOPE_CODEC_MJPEG, is_keyframe=True,
                    width=480, height=360, native_w=480, hfov_deg=45.0, vfov_deg=34.0,
                    aspect_w=4, aspect_h=3, payload=jpg)))
                self.peri_seq += 1
                self.stop.wait(period)
            return
        if not self.units:
            return
        period = 1.0 / self.args.periscope_hz
        i = 0
        while not self.stop.is_set():
            payload, is_kf = self.units[i % len(self.units)]
            self.pub_peri.put(proto.pack_periscope_frame(proto.PeriscopeFrame(
                seq=self.peri_seq, timestamp_ns=time.time_ns(),
                codec=proto.PSCOPE_CODEC_H264, is_keyframe=is_kf,
                width=640, height=480, native_w=640, hfov_deg=60.0, vfov_deg=45.0,
                aspect_w=4, aspect_h=3, optical=True, payload=payload)))
            self.peri_seq += 1
            i += 1
            self.stop.wait(period)

    def _submap_loop(self):
        """One submap per period, in the mapping server's exact publish order."""
        period = 1.0 / self.args.submap_hz
        rng = np.random.default_rng(0)
        n_sub = 0
        while not self.stop.is_set():
            self.stop.wait(period)
            if self.stop.is_set():
                break
            n_sub += 1
            self.map_version += 1
            mv = self.map_version
            cap_ns = time.time_ns() - int(self.args.map_lag_s * 1e9)

            # a cloud that grows outward, so the diff has a real frontier
            n = 2500 + 400 * n_sub
            pts = np.column_stack([
                rng.normal(0, 1.0 + 0.15 * n_sub, n),
                rng.normal(0, 1.0 + 0.15 * n_sub, n),
                rng.uniform(0, 2.0, n)]).astype(np.float32)
            pts = np.floor(pts / 0.06) * 0.06 + 0.03          # voxel-snapped
            pts = np.unique(pts, axis=0).astype(np.float32)
            col = (rng.random((pts.shape[0], 3)) * 255).astype(np.uint8)

            with self._grid_lock:
                changed, removed = self._grid.ingest(pts, col, stamp=mv)
                blocks = self._grid.collect(changed)
                man = self._grid.manifest()

            # PUSH FIRST, manifest second — the order block_publisher relies on
            push = bm.pack_block_push(blocks, removed, map_version=mv,
                                      cube_m=self._grid.cube_m)
            if random.random() < self.args.drop_pushes:
                self.n_dropped += 1                # simulate a DROP-shed push
            else:
                self.pub_push.put(push)
                self.n_pushes += 1
            self.pub_man.put(bm.pack_manifest(man))

            traj = np.column_stack([
                np.cos(np.linspace(0, n_sub * 0.3, 40)) * 2.0,
                np.sin(np.linspace(0, n_sub * 0.3, 40)) * 2.0,
                np.full(40, 1.15)]).astype(np.float32)
            self.pub_traj.put(proto.pack_trajectory(traj))

            if n_sub % 2 == 0:                     # the gate suppresses the others
                self.pub_corr.put(proto.pack_pose_correction(proto.PoseCorrection(
                    timestamp_ns=cap_ns, map_version=mv,
                    position=traj[-1].astype(np.float32),
                    quaternion=np.array([0, 0, 0, 1], np.float32))))

            gx, gy = np.meshgrid(np.linspace(-3, 3, 60), np.linspace(-3, 3, 60))
            cells = np.column_stack([gx.ravel(), gy.ravel(),
                                     np.full(gx.size, 0.6)]).astype(np.float32)
            d = np.clip(np.hypot(gx.ravel(), gy.ravel()) * 0.3, 0, 1).astype(np.float32)
            rgb = np.stack([np.clip((1 - d) * 2, 0, 1), np.clip(d * 2, 0, 1),
                            np.zeros_like(d)], axis=1)
            self.pub_esdf.put(proto.pack_pcd(n_sub, cells,
                                             (rgb * 255).astype(np.uint8),
                                             is_snapshot=True))

            self.pub_status.put(_json({
                "state": "processing", "ts": time.time(), "map_version": mv,
                "n_points": int(pts.shape[0]), "cubes": len(man),
                "cubes_changed": len(changed), "cubes_removed": len(removed),
                "submap": n_sub, "submap_s": round(period * 0.8, 2),
                "frames_buffered": 15, "trigger": "frames", "seq_gaps": 0,
                "server_send_ns": time.time_ns(),
                "newest_frame_robot_ns": cap_ns,
                "robot_kbps": round(len(self._tx) * self.args.frame_hz / 1024, 1),
                "robot_fps": self.args.frame_hz,
                "robot_offset_ms": 0.0, "robot_to_server_ms": 40.0,
                "cloud_mbps": round(len(push) / 1e6 * self.args.submap_hz, 3)}))
            log.info(f"submap v{mv}: {pts.shape[0]} pts  {len(man)} cubes "
                     f"(+{len(changed)}/-{len(removed)})  push {len(push) / 1024:.0f} kB")

    # ── lifecycle ────────────────────────────────────────────────────────────
    def run(self) -> int:
        threads = [threading.Thread(target=f, daemon=True) for f in
                   (self._pose_loop, self._frame_loop, self._periscope_loop,
                    self._submap_loop)]
        for t in threads:
            t.start()
        log.info(f"publishing — {self.args.duration or 'forever'}"
                 f"{'s' if self.args.duration else ''}  (Ctrl-C to stop)")
        try:
            if self.args.duration:
                self.stop.wait(self.args.duration)
            else:
                while not self.stop.wait(1.0):
                    pass
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            for t in threads:
                t.join(timeout=2.0)
            log.info(f"done: {self.seq} frames, {self.pose_seq} poses, "
                     f"{self.peri_seq} periscope, {self.n_pushes} pushes "
                     f"({self.n_dropped} shed), archive {self.n_archive_hits} hit / "
                     f"{self.n_archive_misses} miss, {self.n_block_pulls} block pulls")
            self.z.close()
        return 0


def _json(obj) -> bytes:
    import json
    return json.dumps(obj).encode()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Publish synthetic VAT traffic on a real Zenoh bus.")
    p.add_argument("--router", default=rcfg.ZENOH_ROUTER)
    p.add_argument("--listen", default="tcp/127.0.0.1:0",
                   help="peer-mode inbound endpoint (IPv4 to avoid an IPv6 bind)")
    p.add_argument("--mode", choices=("peer", "client"), default="peer",
                   help="robot-side processes use peer (default)")
    p.add_argument("--duration", type=float, default=0.0, help="seconds (0 = forever)")
    p.add_argument("--frame-hz", type=float, default=2.5)
    p.add_argument("--pose-hz", type=float, default=30.0)
    p.add_argument("--periscope-hz", type=float, default=15.0)
    p.add_argument("--submap-hz", type=float, default=1.0)
    p.add_argument("--map-lag-s", type=float, default=1.2,
                   help="how far behind capture a submap is published")
    p.add_argument("--tx-width", type=int, default=1036)
    p.add_argument("--tx-height", type=int, default=518)
    p.add_argument("--full-width", type=int, default=3840)
    p.add_argument("--full-height", type=int, default=1920)
    p.add_argument("--archive-window", type=int, default=200,
                   help="rolling archive depth in frames (small = exercise misses)")
    p.add_argument("--codec", choices=("auto", "h264", "mjpeg", "none"), default="auto",
                   help="periscope codec. 'auto' (default) uses real H.264 when ffmpeg "
                        "is installed and falls back to MJPEG when it is not")
    p.add_argument("--periscope-frames", type=int, default=60)
    p.add_argument("--drop-pushes", type=float, default=0.0,
                   help="fraction of pushes to shed, to exercise manifest repair")
    args = p.parse_args(argv)
    return FakeRig(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
