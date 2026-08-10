#!/usr/bin/env python3
"""
VAT — ``vat-record``: passive live-session recorder
==================================================
Taps the live VAT Zenoh streams **without perturbing the session** and writes each
one independently to disk, every sample stamped on one common clock, so the streams
can be composed afterwards however the story needs — panorama, point cloud, poses,
periscope, ESDF.

Built for the real-capture spec in ``uofa-2026-report/PUBLICATION_ROADMAP.md`` §3.2.

Passive by construction
-----------------------
* Every map / pose / video stream is a **pure subscriber**. The recorder declares no
  publisher on any live key, so it cannot change what the robot sends, what the
  mapping server computes, or what the operator's client sees.
* The **full-resolution panorama** is pulled from the robot's *own* rolling archive
  by ``seq`` (``{robot}/prism/camera/archive/get``), never by asking for a higher
  transmit resolution — so it costs no uplink. Run with ``--where robot`` and the
  recorder opens Zenoh in **peer** mode, like the robot's own processes
  (``theta_camera.py``, ``pose_fuser.py``), so scouting links it directly to the
  camera process and the frames never traverse the field link.
* The only other queries are the ones a normal client already makes: the
  manifest-diff **repair pull** on ``{server}/pcd/blocks`` (which is what lets a
  mid-session start still yield a complete map) and, only if you ask for it,
  ``--pointcloud-snapshot-query-s``. Map *keyframes* are materialised from the
  recorder's own mirror and cost the server nothing.
* The periscope is recorded **opportunistically**. The robot encodes it only while
  an operator client keeps a ``ViewRequest`` alive; the recorder never sends one.

Usage
-----
::

    # from the repo root, in the client env (has zenoh + DracoPy + PyAV)
    make record ARGS="--scene lab --trajectory-family loop --pass 1 \
                      --camera-height 1.152 --operator rafael"

    # or directly
    cd client && uv run python ../tools/recorder/vat_record.py --help

    # full-res too, run ON THE ROBOT (pulls from the local archive)
    uv run python ../tools/recorder/vat_record.py --where robot --all \
        --panorama-fullres --fullres-max-size 8GB --duration 5m

Everything lands in ``recordings/<session_id>/`` — see ``tools/recorder/README.md``
and ``docs/recording.md`` for the layout and the composition workflow.

Offline self-test (no robot, no Zenoh, no GPU)::

    python tools/recorder/vat_record.py --selftest
"""

from __future__ import annotations

import argparse
import datetime as _dt
import logging
import os
import platform
import re
import signal
import socket
import sys
import threading
import time
from typing import Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:                  # so the rec_* siblings import as a script
    sys.path.insert(0, _HERE)

log = logging.getLogger("vat-record")

#: every recordable stream, in the order they appear in the console/manifest
STREAMS = ("panorama_transmit", "panorama_fullres", "periscope",
           "pointcloud", "poses", "esdf", "status", "trajectory")
#: enabled when no stream flag is given at all, from the cloud/client side —
#: everything cheap and passive
DEFAULT_STREAMS = tuple(s for s in STREAMS if s != "panorama_fullres")
#: default from the ROBOT side. The router lives on the server, so every cloud→client
#: stream (map pushes, manifests, ESDF, status, trajectory) would travel INBOUND across
#: the field link to reach a recorder on the robot — the opposite of what --where robot
#: is for. So the robot default is the streams the robot itself produces, plus the pose
#: correction, which the cloud already sends down to the robot anyway (44 bytes).
#: Record the map from the cloud side, in a second recorder, at the same time.
#: `status` is included despite being a cloud stream: it is one small JSON per submap
#: (~1 Hz) and it is the ONLY source of map_version→capture-time pins, without which a
#: robot-side recording cannot be placed against the map at all.
DEFAULT_STREAMS_ROBOT = ("panorama_transmit", "panorama_fullres", "periscope", "poses",
                        "status")
#: bulk streams published by the cloud — these are the ones that would cost real
#: inbound bandwidth on the field link if recorded from the robot side
CLOUD_BULK_STREAMS = ("pointcloud", "esdf", "trajectory")

TRAJECTORY_FAMILIES = ("smooth", "stop-and-go", "loop", "other")

_SESSION_README = """\
VAT session recording — {session_id}
{underline}

Written by tools/recorder/vat_record.py (vat-record) v{version}.
Docs: docs/recording.md  ·  tools/recorder/README.md

THE COMMON CLOCK
  Every index carries `src_ts_ns` on ONE timeline: the robot capture clock in
  nanoseconds. `ts_src` says where it came from:
    source  = verbatim from the wire (a real capture timestamp)
    derived = local arrival mapped onto the session clock (map transport carries
              no timestamp)
    wall    = local arrival only (offset not yet known — early samples)
  Map records additionally carry `capture_ts_ns`: the true capture time of that
  map_version, pinned from pose_correction (exact) or status (approximate).
  Align streams on `src_ts_ns` / `capture_ts_ns`, never on file order.

FILES
  meta.json                      session identity, capture metadata, config hash,
                                 clock epoch, Zenoh keys subscribed/queried
  MANIFEST.json                  per-stream health: counts, bytes, rates, gaps,
                                 caps hit, derived cross-checks
  recorder.log                   the recorder's own log for this session

  panorama_transmit/             the 360° stream as sent to the cloud
    frames/<seq>.jpg             encoded body, byte-exact off the wire
    frame_index.csv              seq, src_ts_ns, wall_ns, wire_bytes, image_bytes,
                                 camera_height_m, width, height  (wire_bytes is the
                                 real uplink cost per frame)
  panorama_fullres/              full-res twins pulled from the robot's archive,
                                 same seq/ts/camera_height as the transmit frame
  periscope/periscope.h264|hevc|mjpeg    elementary stream, byte-exact
  periscope_timestamps.csv       AUTHORITATIVE periscope timing: per frame the
                                 capture ts + (segment, byte_offset, byte_len)
  periscope.mp4                  convenience remux at the mean rate (nominal
                                 timing — use the CSV to align)

  pointcloud/index.jsonl         ordered index of every map artefact
  pointcloud/blocks/*.bin        raw pcd/push, pcd/manifest and repair bundles;
                                 replay with vat_blockmap.unpack_block_push /
                                 unpack_manifest / unpack_bundle
  pointcloud/keyframes/*.npz     complete map states (points, colors, map_version,
                                 capture_ts_ns) materialised from the recorder's
                                 mirror — seek here instead of replaying
  pointcloud/snapshots/*.bin     whole-map pack_pcd snapshots (STREAM_MODE=snapshot
                                 or an explicit snapshot query)

  poses/robot_fused.tum          authoritative fused pose, TUM format (evo-ready)
  poses/robot_fused.jsonl        + velocity, acceleration, fix_quality, seq
  poses/cloud_correction.jsonl   cloud corrections + recomputed gate metrics
  poses/trajectory.jsonl|/*.npy  streamed camera-position trail (positions only)

  esdf/index.jsonl, esdf/slices/ ESDF slices: wire bytes + distances inverted from
                                 the colour ramp (saturates at 0 m and 1 m)
  status/status.jsonl            server status: map_version <-> time pins and the
                                 measured uplink (robot_kbps / robot_fps)

NEXT STEP
  python tools/recorder/compose.py info   {session_id_path}
  python tools/recorder/compose.py export {session_id_path} --fps 10
"""


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI.

    Deliberately free of ``rec_config`` (and therefore of ``vat_protocol``) so the
    transport overrides below can be pushed into ``os.environ`` *before* those
    modules resolve their configuration at import time — the same env-first
    convention ``mapping_config`` uses.
    """
    p = argparse.ArgumentParser(
        prog="vat-record",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Passively record the live VAT streams onto one common clock.",
        epilog="Streams default to everything except --panorama-fullres. Naming any "
               "stream flag selects exactly those. See docs/recording.md.")

    g = p.add_argument_group("session")
    g.add_argument("--session-id", default=None,
                   help="output folder name (default: UTC timestamp + scene + "
                        "trajectory family + pass)")
    g.add_argument("--out", default=None,
                   help="output root (default $RECORDER_OUT_ROOT or ./recordings)")
    g.add_argument("--where", choices=("robot", "cloud"), default="cloud",
                   help="where this recorder is running. 'robot' opens Zenoh in peer "
                        "mode (like the robot's own processes) so the full-res "
                        "archive pull stays local; 'cloud' opens in client mode. "
                        "(default: cloud)")
    g.add_argument("--duration", default=None, metavar="T",
                   help="stop after T ('90', '90s', '5m', '1h'); default unlimited")
    g.add_argument("--max-size", default=None, metavar="SIZE",
                   help="stop when the session exceeds SIZE ('2GB'); excludes the "
                        "full-res stream, which has its own cap")
    g.add_argument("--progress", type=float, default=5.0, metavar="S",
                   help="console progress interval, seconds (0 = quiet)")
    g.add_argument("--dry-run", action="store_true",
                   help="print the resolved plan (streams, keys, caps, paths) and exit")
    g.add_argument("--selftest", action="store_true",
                   help="run the offline self-tests (no robot / Zenoh / GPU) and exit")
    g.add_argument("--selftest-keep", default=None, metavar="DIR",
                   help="with --selftest: build the synthetic session in DIR and keep "
                        "it, so you can try compose.py against a known-good recording")

    s = p.add_argument_group(
        "streams", "each independently toggleable; --no-X forces one off")
    for name in STREAMS:
        s.add_argument(f"--{name.replace('_', '-')}", dest=name,
                       action=argparse.BooleanOptionalAction, default=None,
                       help=_STREAM_HELP[name])
    s.add_argument("--all", action="store_true",
                   help="every stream INCLUDING --panorama-fullres")

    t = p.add_argument_group("transport (override vat.env)")
    t.add_argument("--router", default=None,
                   help="Zenoh endpoint (default $ZENOH_ROUTER / $ZENOH_CONNECT)")
    t.add_argument("--robot", default=None, help="robot name (default $ROBOT_NAME)")
    t.add_argument("--server-prefix", default=None,
                   help="server key prefix (default $SERVER_PREFIX)")
    t.add_argument("--cube-size", default=None, type=float,
                   help="cube grid size, m — MUST match the server (default $CUBE_SIZE)")
    t.add_argument("--zenoh-mode", choices=("client", "peer"), default=None,
                   help="override the mode implied by --where")
    t.add_argument("--zenoh-listen", default=None, metavar="ENDPOINT",
                   help="peer mode only: bind this inbound endpoint. By default the "
                        "recorder binds NOTHING — it is a pure consumer and never needs "
                        "inbound peers, and zenoh's default peer listener fails outright "
                        "on hosts without IPv6 (common in the robot container)")
    t.add_argument("--connect-retries", type=int, default=3,
                   help="Zenoh connect attempts before giving up (default 3)")

    pt = p.add_argument_group("panorama (transmit)")
    pt.add_argument("--transmit-every", type=int, default=1, metavar="N",
                    help="keep every Nth frame (default 1 = all)")
    pt.add_argument("--transmit-index-only", action="store_true",
                    help="write frame_index.csv but no images — characterise the "
                         "uplink with almost no disk")

    pf = p.add_argument_group("panorama (full resolution, from the robot's archive)")
    pf.add_argument("--fullres-every", type=int, default=1, metavar="N",
                    help="pull every Nth frame (default 1 = all)")
    pf.add_argument("--fullres-max-size", default="8GB", metavar="SIZE",
                    help="cap for the full-res stream alone (default 8GB)")
    pf.add_argument("--fullres-duration", default=None, metavar="T",
                    help="stop pulling full-res after T (default: session duration)")
    pf.add_argument("--fullres-ring", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="at the cap, evict oldest frames instead of stopping "
                         "(default: ring on — keeps the newest window)")
    pf.add_argument("--fullres-lag", type=float, default=2.0, metavar="S",
                    help="wait S seconds before pulling a seq; the robot's archive "
                         "writer is asynchronous (default 2.0)")
    pf.add_argument("--fullres-timeout", type=float, default=5.0, metavar="S",
                    help="archive query timeout (default 5.0)")
    pf.add_argument("--fullres-quality", type=int, default=0, metavar="Q",
                    help="ask the robot to re-encode at JPEG quality Q (1-100) before "
                         "replying. 0 = as archived (default). Full-res is rarely needed "
                         "pristine, and this is the only place link bytes can be saved")
    pf.add_argument("--fullres-max-width", type=int, default=0, metavar="W",
                    help="ask the robot to downscale to at most W px wide (e.g. 1920). "
                         "0 = full resolution")
    pf.add_argument("--force", action="store_true",
                    help="allow --panorama-fullres with --where cloud (pulls full-res "
                         "over the field link — do not do this during a real capture)")

    pc = p.add_argument_group("point cloud / map")
    pc.add_argument("--pointcloud-keyframe-s", type=float, default=10.0, metavar="S",
                    help="materialise a complete map keyframe from the recorder's "
                         "own mirror every S seconds; costs the server nothing "
                         "(0 = off, default 10)")
    pc.add_argument("--pointcloud-repair", action=argparse.BooleanOptionalAction,
                    default=None,
                    help="diff each manifest and pull missing cubes from "
                         "{server}/pcd/blocks, exactly as the client does — this is "
                         "what makes a mid-session start complete. Default: on from "
                         "the cloud side, OFF with --where robot (a whole-map bootstrap "
                         "pull is a reliable multi-MB transfer and would cross the "
                         "field link while stalling the server's submap lock)")
    pc.add_argument("--pointcloud-snapshot-query-s", type=float, default=0.0,
                    metavar="S",
                    help="also query the server's canonical snapshot every S seconds "
                         "as a cross-check (0 = off, default off: it costs the server "
                         "a full cloud extract)")
    pc.add_argument("--esdf-decode", action=argparse.BooleanOptionalAction, default=True,
                    help="also store ESDF distances inverted from the colour ramp "
                         "(default on)")

    pe = p.add_argument_group("periscope")
    pe.add_argument("--periscope-mp4", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="remux the elementary stream to mp4 with ffmpeg at close "
                         "(default on; the CSV stays authoritative for timing)")

    m = p.add_argument_group("session metadata (goes into meta.json)")
    m.add_argument("--scene", default=None,
                   help="scene name, e.g. 'lab', 'corridor-3rd-floor'")
    m.add_argument("--trajectory-family", choices=TRAJECTORY_FAMILIES, default=None,
                   help="motion family, mirroring the benchmark families")
    m.add_argument("--pass", dest="pass_index", type=int, default=None,
                   help="pass index for this scene+family (2+ passes give a variance "
                        "estimate)")
    m.add_argument("--seed", type=int, default=None,
                   help="seed index, to line up with the rendered benchmark naming")
    m.add_argument("--camera-height", type=float, default=None, metavar="H",
                   help="MEASURED ground→camera height in metres. This is the "
                        "metric-scale anchor — measure it every session")
    m.add_argument("--camera-height-source", default=None,
                   help="how H was measured, e.g. 'tape, floor→lens centre, standing'")
    m.add_argument("--mount-geometry", default=None,
                   help="camera mount description, e.g. 'rear selfie-stick, rigid'")
    m.add_argument("--clear-flat-floor", action=argparse.BooleanOptionalAction,
                   default=None,
                   help="did the run START over clear, flat, visible floor? "
                        "(the metric-scale plane fit needs it)")
    m.add_argument("--operator", default=None, help="who drove")
    m.add_argument("--note", action="append", default=[], metavar="TEXT",
                   help="free-form operator note (repeatable)")
    m.add_argument("--meta", action="append", default=[], metavar="KEY=VALUE",
                   help="extra metadata field (repeatable)")
    return p


_STREAM_HELP = {
    "panorama_transmit":
        "the 360° stream as actually sent to the cloud (bytes/frame + frames)",
    "panorama_fullres":
        "full-res twins pulled from the robot's LOCAL archive by seq; large, needs "
        "--where robot and ARCHIVE_ENABLE=true on the robot",
    "periscope":
        "the periscope video slice, recorded opportunistically while an operator "
        "client is aiming it (the recorder never requests it)",
    "pointcloud":
        "versioned map transport: pushes + manifests + repair pulls + materialised "
        "keyframes (implies --status, which carries the version↔time index)",
    "poses":
        "the robot's authoritative fused pose (TUM + JSONL) and the cloud pose "
        "corrections",
    "esdf": "ESDF navigation slices over time",
    "status": "server status JSON: map_version↔time pins and the measured uplink",
    "trajectory": "the streamed camera-position trail (positions only)",
}


def resolve_streams(args) -> set:
    """Turn the tri-state stream flags into the set actually enabled."""
    on = {s for s in STREAMS if getattr(args, s) is True}
    off = {s for s in STREAMS if getattr(args, s) is False}
    robot_side = args.where == "robot"
    if args.all:
        enabled = set(STREAMS) - off
    elif on:
        enabled = on
    else:
        enabled = set(DEFAULT_STREAMS_ROBOT if robot_side else DEFAULT_STREAMS) - off
    # The map transport carries no timestamps; `status` is the only stream that
    # pins a map_version to a capture time, so recording the cloud without it
    # produces a map you cannot place on the timeline.
    # Full-res learns which seqs exist from the live transmit stream, and its own
    # arrivals are lagged by design so they cannot establish the clock baseline. The
    # transmit stream supplies both — and it is cheap — so it is implied.
    if ("panorama_fullres" in enabled and "panorama_transmit" not in enabled
            and "panorama_transmit" not in off):
        enabled.add("panorama_transmit")
        log.info("[plan] --panorama-fullres implies --panorama-transmit (it supplies the "
                 "seq numbers to pull and the clock baseline); --no-panorama-transmit "
                 "overrides, but then full-res opens its own subscription and the "
                 "session clock has no baseline")
    if "pointcloud" in enabled and "status" not in enabled and "status" not in off:
        enabled.add("status")
        log.info("[plan] --pointcloud implies --status (it carries the "
                 "map_version↔time index); pass --no-status to override")
    if robot_side:
        bulk = sorted(enabled.intersection(CLOUD_BULK_STREAMS))
        if bulk:
            log.warning(
                f"[plan] --where robot with {', '.join(bulk)}: these are published by "
                f"the CLOUD, and the Zenoh router lives on the server — so recording "
                f"them here pulls them INBOUND across the field link, competing with "
                f"the pose downlink. Prefer a second recorder on the cloud/client side "
                f"for the map, and keep this one to the robot's own streams.")
    return enabled


def apply_transport_overrides(args) -> None:
    """Push transport overrides into the environment before rec_config resolves."""
    if args.router:
        os.environ["ZENOH_ROUTER"] = args.router
        os.environ["ZENOH_CONNECT"] = args.router
    if args.robot:
        os.environ["ROBOT_NAME"] = args.robot
    if args.server_prefix:
        os.environ["SERVER_PREFIX"] = args.server_prefix
    if args.cube_size is not None:
        os.environ["CUBE_SIZE"] = str(args.cube_size)


def default_session_id(args) -> str:
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d-%H%M%S")
    parts = [stamp]
    if args.scene:
        parts.append(args.scene)
    if args.trajectory_family:
        parts.append(args.trajectory_family)
    if args.pass_index is not None:
        parts.append(f"p{args.pass_index}")
    if args.seed is not None:
        parts.append(f"s{args.seed}")
    return _slug("_".join(parts))


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(text)).strip("-") or "session"


def parse_meta_pairs(pairs) -> dict:
    out = {}
    for item in pairs or ():
        if "=" not in item:
            raise SystemExit(f"--meta expects KEY=VALUE, got {item!r}")
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Session
# ═════════════════════════════════════════════════════════════════════════════


class _PlanSession:
    """A Zenoh stand-in for ``--dry-run``: records what would be declared, does nothing.

    Lets the real ``attach()`` implementations run unchanged, so the printed plan is
    the plan — including that no publisher is ever declared.
    """

    def __init__(self):
        self.subscribed, self.queried, self.published = [], [], []

    def declare_subscriber(self, key, handler):
        self.subscribed.append(key)
        return self

    def declare_queryable(self, key, handler):
        self.queried.append(key)
        return self

    def declare_publisher(self, key, **_kw):            # never called — asserted below
        self.published.append(key)
        raise AssertionError(f"the recorder must not publish (attempted on {key!r})")

    def get(self, key, **_kw):
        self.queried.append(key)
        return []

    def close(self):
        pass


def _setup_logging(session_dir: str) -> None:
    root = logging.getLogger()
    root.setLevel(os.environ.get("LOG_LEVEL", "INFO").upper())
    fmt = logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s",
                            datefmt="%H:%M:%S")
    if not any(isinstance(h, logging.StreamHandler) for h in root.handlers):
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(sh)
    fh = logging.FileHandler(os.path.join(session_dir, "recorder.log"),
                             encoding="utf-8")
    fh.setFormatter(fmt)
    root.addHandler(fh)


def build_recorders(enabled, sw, clock, args, rcfg, budgets):
    """Instantiate the enabled stream recorders, wired to their budgets."""
    from rec_cloud import EsdfRecorder, PointCloudRecorder, StatusRecorder
    from rec_frames import PanoramaFullresRecorder, PanoramaTransmitRecorder
    from rec_periscope import PeriscopeRecorder
    from rec_poses import CorrectionRecorder, FusedPoseRecorder, TrajectoryRecorder

    recs = []
    fullres = None
    if "panorama_fullres" in enabled:
        fullres = PanoramaFullresRecorder(
            sw, clock, budgets["fullres"], every=args.fullres_every,
            lag_s=args.fullres_lag, timeout_s=args.fullres_timeout,
            quality=args.fullres_quality, max_width=args.fullres_max_width,
            # When the transmit stream is also recorded it feeds us the seqs, so we
            # skip a second subscription to the same key.
            own_subscription=("panorama_transmit" not in enabled))
    if "panorama_transmit" in enabled:
        recs.append(PanoramaTransmitRecorder(
            sw, clock, budgets["session"], every=args.transmit_every,
            index_only=args.transmit_index_only,
            seq_sink=(fullres.offer if fullres is not None else None)))
    if fullres is not None:
        recs.append(fullres)
    if "periscope" in enabled:
        recs.append(PeriscopeRecorder(sw, clock, budgets["session"],
                                      mux_mp4=args.periscope_mp4))
    if "pointcloud" in enabled:
        repair = args.pointcloud_repair
        if repair is None:                 # not specified → decide from --where
            repair = args.where != "robot"
            if not repair:
                log.warning(
                    "[plan] --where robot: manifest repair pulls are OFF by default "
                    "(a bootstrap pull is a reliable whole-map transfer over the field "
                    "link and stalls the server's submap lock while it collects). "
                    "Dropped pushes will therefore leave holes in this recording — "
                    "record the map from the cloud side, or pass --pointcloud-repair "
                    "if you accept the cost.")
        recs.append(PointCloudRecorder(
            sw, clock, budgets["session"], cube_m=rcfg.CUBE_SIZE,
            keyframe_s=args.pointcloud_keyframe_s, repair=repair,
            snapshot_query_s=args.pointcloud_snapshot_query_s))
    if "poses" in enabled:
        recs.append(FusedPoseRecorder(sw, clock, budgets["session"]))
        recs.append(CorrectionRecorder(sw, clock, budgets["session"]))
    if "trajectory" in enabled:
        recs.append(TrajectoryRecorder(sw, clock, budgets["session"]))
    if "esdf" in enabled:
        recs.append(EsdfRecorder(sw, clock, budgets["session"],
                                 decode=args.esdf_decode))
    if "status" in enabled:
        recs.append(StatusRecorder(sw, clock, budgets["session"]))
    return recs


def build_meta(args, enabled, recs, sw, clock, rcfg, zenoh_mode) -> dict:
    subscribed, queried = [], []
    for r in recs:
        subscribed += r.keys_subscribed
        queried += r.keys_queried
    return {
        "schema": rcfg.SCHEMA,
        "session_id": sw.session_id,
        "status": "recording",
        "recorder": {
            "version": rcfg.RECORDER_VERSION,
            "tool": "tools/recorder/vat_record.py",
            "where": args.where,
            "zenoh_mode": zenoh_mode,
            "argv": sys.argv[1:],
            "host": socket.gethostname(),
            "platform": platform.platform(),
        },
        "started_wall_utc": _dt.datetime.fromtimestamp(
            clock.epoch_wall_ns / 1e9, _dt.timezone.utc).isoformat(),
        "clock": clock.meta(),
        "capture": {
            "scene": args.scene,
            "trajectory_family": args.trajectory_family,
            "pass_index": args.pass_index,
            "seed": args.seed,
            "camera_height_m": args.camera_height,
            "camera_height_declared": args.camera_height is not None,
            "camera_height_source": args.camera_height_source,
            "camera_height_note": ("The metric-scale anchor. The per-frame wire value "
                                   "is also recorded in the panorama frame indexes "
                                   "and summarised in MANIFEST.json."),
            "mount_geometry": args.mount_geometry,
            "start_over_clear_flat_floor": args.clear_flat_floor,
            "operator": args.operator,
            "notes": list(args.note or []),
            "extra": parse_meta_pairs(args.meta),
        },
        "streams_enabled": sorted(enabled),
        "zenoh": rcfg.zenoh_summary(subscribed, queried),
        "caps": {
            "session_max_bytes": rcfg.parse_size(args.max_size) or None,
            "session_duration_s": rcfg.parse_duration(args.duration) or None,
            "fullres_max_bytes": rcfg.parse_size(args.fullres_max_size) or None,
            "fullres_duration_s": (rcfg.parse_duration(args.fullres_duration)
                                   or None),
            "fullres_ring": bool(args.fullres_ring),
            "transmit_every": args.transmit_every,
            "fullres_every": args.fullres_every,
            "pointcloud_keyframe_s": args.pointcloud_keyframe_s or None,
            "pointcloud_snapshot_query_s": args.pointcloud_snapshot_query_s or None,
        },
        "config": rcfg.session_provenance(),
        "passivity": {
            "publishers_declared": [],
            "note": ("The recorder declares no publisher on any live key. Queries are "
                     "limited to the robot's own frame archive and the block-repair / "
                     "snapshot queryables a normal client already uses; see the "
                     "keys_queried list."),
        },
    }


def derived_stats(recs, clock) -> dict:
    """Cross-checks that only make sense across streams."""
    by = {r.name: r for r in recs}
    out = {}

    # § 3.2 wants gating stats. The gate's own counters never reach the wire, but
    # every submap shows up in `status` and only accepted corrections are published,
    # so the difference is the suppressed+rejected+stale count.
    st, cr = by.get("status"), by.get("poses_cloud_correction")
    if st is not None and cr is not None:
        submaps = st.extra_summary()["submaps_seen"]
        published = cr.stats.n
        out["pose_correction_gating"] = {
            "submaps_seen_in_status": submaps,
            "corrections_published": published,
            "suppressed_or_rejected": max(0, submaps - published),
            "note": ("Derived, not measured: PoseCorrectionGate's "
                     "published/suppressed/rejected/stale counters are only in the "
                     "mapping server's log. Per-sample gate quantities are recomputed "
                     "in poses/cloud_correction.jsonl."),
        }

    # The real uplink, two independent ways: what we counted on the wire, and what
    # the server measured. They should agree to within the throughput EMA.
    tx = by.get("panorama_transmit")
    if tx is not None:
        s = tx.stats.summary()
        span = s["src_ts_span_s"] or 0.0
        measured = {
            "frames": s["samples"], "wire_bytes_total": s["bytes"],
            "mean_wire_bytes_per_frame": s["mean_bytes"],
            "mean_hz": s["mean_hz"],
            "mean_kbps": (round(s["bytes"] / span / 1024.0, 1) if span > 0 else None),
            "seq_samples_missing": s["seq_samples_missing"],
        }
        if st is not None:
            m = st.extra_summary()["server_metrics"]
            measured["server_reported_robot_kbps"] = m.get("robot_kbps")
            measured["server_reported_robot_fps"] = m.get("robot_fps")
        out["uplink"] = measured
        out["camera_height_observed"] = tx.extra_summary()["camera_height_wire"]

    # The window in which every DENSE stream has data — the usable span for
    # composition. Sparse streams (the gated pose correction) are reported beside it
    # rather than allowed to shrink it, and empty streams are named so a gap in the
    # capture is obvious rather than silently absorbed.
    firsts, lasts, empty, sparse = [], [], [], {}
    for r in recs:
        s = r.stats.summary()
        if not (s["samples"] and s["first_src_ts_ns"] and s["last_src_ts_ns"]):
            empty.append(r.name)
            continue
        if getattr(r, "dense", True):
            firsts.append((s["first_src_ts_ns"], r.name))
            lasts.append((s["last_src_ts_ns"], r.name))
        else:
            sparse[r.name] = {"first_src_ts_ns": s["first_src_ts_ns"],
                              "last_src_ts_ns": s["last_src_ts_ns"],
                              "samples": s["samples"]}
    if firsts and lasts:
        lo, lo_by = max(firsts)
        hi, hi_by = min(lasts)
        out["aligned_window"] = {
            "start_src_ts_ns": lo, "start_limited_by": lo_by,
            "end_src_ts_ns": hi, "end_limited_by": hi_by,
            "duration_s": round((hi - lo) / 1e9, 3) if hi > lo else 0.0,
            "streams_with_no_samples": empty,
            "sparse_streams": sparse,
            "note": ("The span where every DENSE stream has data; compose.py clamps "
                     "its timeline to this by default. Sparse streams (the gated pose "
                     "correction) are listed separately — they legitimately go quiet "
                     "and must not shrink the window."),
        }
    out["clock"] = clock.summary()
    out["version_pins"] = {
        "count": len(clock.version_pins()),
        "exact_from_pose_correction": sum(
            1 for v in clock.version_pins().values()
            if v["source"] == "pose_correction"),
    }
    return out


def run_session(args) -> int:
    parse_meta_pairs(args.meta)        # fail on a bad --meta before touching anything
    apply_transport_overrides(args)

    # Imported here, AFTER the env overrides above, because rec_config resolves the
    # key schema and tunables from the environment at import time.
    import rec_config as rcfg
    from rec_clock import SessionClock
    from rec_sinks import Budget, RingBudget, SessionWriter

    out_root = args.out or rcfg.DEFAULT_OUT_ROOT
    session_id = _slug(args.session_id) if args.session_id else default_session_id(args)
    zenoh_mode = args.zenoh_mode or ("peer" if args.where == "robot" else "client")
    enabled = resolve_streams(args)

    # Full-res must not be pulled across the field link during a real capture.
    if "panorama_fullres" in enabled and args.where != "robot" and not args.force:
        raise SystemExit(
            "--panorama-fullres pulls full-resolution frames from the robot's own\n"
            "archive. Run the recorder ON THE ROBOT (--where robot) so the frames\n"
            "stay on the local bus instead of consuming the field link.\n"
            "If you really mean to pull them over the link, add --force.")

    # Validate the caps before creating anything, and turn a typo into a usable message
    # instead of a raw ValueError traceback.
    try:
        session_duration = rcfg.parse_duration(args.duration)
        for name, raw, fn in (("--duration", args.duration, rcfg.parse_duration),
                              ("--fullres-duration", args.fullres_duration,
                               rcfg.parse_duration),
                              ("--max-size", args.max_size, rcfg.parse_size),
                              ("--fullres-max-size", args.fullres_max_size,
                               rcfg.parse_size)):
            fn(raw)
    except ValueError as e:
        raise SystemExit(f"cannot parse {name} {raw!r}: {e}\n"
                         f"  durations: 90 | 90s | 5m | 1h | 2m30s\n"
                         f"  sizes:     500MB | 8GB | 10737418240")
    budgets = {
        "session": Budget(max_bytes=rcfg.parse_size(args.max_size),
                          duration_s=session_duration, name="session"),
        "fullres": (RingBudget if args.fullres_ring else Budget)(
            max_bytes=rcfg.parse_size(args.fullres_max_size),
            duration_s=(rcfg.parse_duration(args.fullres_duration)
                        or session_duration),
            name="fullres"),
    }

    sw = SessionWriter(out_root, session_id)
    _setup_logging(sw.root)
    clock = SessionClock(window_s=rcfg.CLOCK_WINDOW_S)
    recs = build_recorders(enabled, sw, clock, args, rcfg, budgets)

    log.info(f"[vat-record] v{rcfg.RECORDER_VERSION}  session={session_id}")
    log.info(f"[vat-record] → {sw.root}")
    log.info(f"[vat-record] streams: {', '.join(sorted(enabled))}")
    log.info(f"[vat-record] zenoh {zenoh_mode} → {rcfg.ZENOH_ROUTER}  "
             f"robot={rcfg.ROBOT_NAME}  server={rcfg.SERVER_PREFIX}  "
             f"stream_mode={rcfg.STREAM_MODE} cube={rcfg.CUBE_SIZE}m")
    # The clock's transport baseline comes from a low-latency source-stamped stream.
    # Without one, every ts_src=derived timestamp is absent or biased and the map cannot
    # be placed on the session clock at all — so this is an error, not a warning.
    baseline = {"poses", "panorama_transmit", "periscope"}.intersection(enabled)
    if not baseline:
        raise SystemExit(
            "no stream can establish the clock baseline.\n"
            "The map transport carries no timestamps, so its session time is derived from\n"
            "arrival minus the robot->local offset — and that offset is only learnable from\n"
            "a low-latency source-stamped stream (--poses, --panorama-transmit or\n"
            "--periscope). pose_correction cannot supply it: its timestamp is a keyframe\n"
            "capture time from seconds earlier.\n"
            "Add --poses (cheap, 30 Hz, and the authoritative trajectory anyway).")
    if args.camera_height is None:
        log.warning("[vat-record] no --camera-height given. That measurement is the "
                    "METRIC-SCALE ANCHOR for this session (roadmap §3.2). The "
                    "per-frame wire value is still recorded, but measure and pass it.")
    if args.clear_flat_floor is False:
        log.warning("[vat-record] --no-clear-flat-floor: the metric-scale floor fit "
                    "may not commit; expect scale warm-up to be longer or to fail.")

    if args.dry_run:
        # Attach a stub session so the recorders declare their keys through exactly
        # the same code path a real run uses — the plan you see is the plan that runs.
        for r in recs:
            r.attach(_PlanSession())
        print_plan(args, enabled, recs, sw, rcfg, zenoh_mode, budgets)
        for r in recs:
            r.close()
        sw.close()
        return 0

    # ── connect ──────────────────────────────────────────────────────────────
    import zenoh
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{rcfg.ZENOH_ROUTER}"]')
    conf.insert_json5("mode", f'"{zenoh_mode}"')
    if zenoh_mode == "peer":
        # A peer normally opens an inbound listener; on a host without IPv6 zenoh's
        # default (tcp/[::]:0) fails to bind and zenoh.open raises outright. The
        # recorder is a pure consumer, so bind nothing unless asked. Scouting and
        # gossip still work, which is what makes the local archive pull local.
        listen = f'["{args.zenoh_listen}"]' if args.zenoh_listen else "[]"
        conf.insert_json5("listen/endpoints", listen)
        log.info(f"[vat-record] peer mode, listen={listen} (pure consumer)")
    z = None
    for attempt in range(1, max(1, args.connect_retries) + 1):
        try:
            z = zenoh.open(conf)
            break
        except Exception as e:                                  # noqa: BLE001
            log.warning(f"[vat-record] zenoh connect failed ({attempt}/"
                        f"{args.connect_retries}): {e}")
            if attempt < args.connect_retries:
                time.sleep(2.0)
    if z is None:
        log.error(f"[vat-record] could not reach {rcfg.ZENOH_ROUTER} — is the router "
                  f"up? (make router)")
        sw.close()
        return 2

    stop = threading.Event()
    stop_reason = "unknown"
    status = "error"
    try:
        for r in recs:
            r.attach(z)
        sw.write_json(build_meta(args, enabled, recs, sw, clock, rcfg, zenoh_mode),
                      "meta.json")
        sw.write_blob(_session_readme(session_id, out_root,
                                      rcfg.RECORDER_VERSION).encode("utf-8"),
                      "README.txt")

        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
            if sig is not None:
                try:
                    signal.signal(sig, lambda *_: stop.set())
                except (ValueError, OSError):        # not the main thread / unsupported
                    pass

        log.info("[vat-record] recording — Ctrl-C to stop cleanly")
        write_live(sw, recs, budgets, clock, "recording")
        last_progress = 0.0
        while not stop.is_set():
            stop.wait(0.5)
            now = time.monotonic()
            for r in recs:
                try:
                    r.tick(now)
                except Exception as e:                          # noqa: BLE001
                    r.stats.error(f"tick: {e}")
                    log.debug(f"[{r.name}] tick failed", exc_info=True)
            b = budgets["session"]
            if b.time_expired():
                stop_reason = f"duration cap reached ({b.duration_s:g}s)"
                break
            if b.bytes_exhausted():
                stop_reason = (f"size cap reached "
                               f"({rcfg.human_size(b.max_bytes)})")
                break
            if args.progress > 0 and now - last_progress >= args.progress:
                last_progress = now
                log.info(_progress_line(recs, b, rcfg))
                write_live(sw, recs, budgets, clock, "recording")
        else:
            stop_reason = "stopped by signal (Ctrl-C)"
        status = "complete"
    except KeyboardInterrupt:
        stop_reason = "stopped by KeyboardInterrupt"
        status = "complete"
    except Exception:
        stop_reason = "recorder error"
        status = "error"
        log.exception("[vat-record] recording failed")
    finally:
        # Finalise no matter what: a partial recording must stay usable.
        log.info(f"[vat-record] stopping ({stop_reason}) — flushing…")
        for r in recs:
            try:
                r.close()
            except Exception:                                   # noqa: BLE001
                log.exception(f"[{r.name}] close failed")
        summaries = {}
        for r in recs:
            try:
                summaries[r.name] = r.summary()
            except Exception as e:                              # noqa: BLE001
                summaries[r.name] = {"stream": r.name, "summary_error": str(e)}
        try:
            manifest = {
                "schema": rcfg.SCHEMA,
                "session_id": session_id,
                "status": status,
                "stop_reason": stop_reason,
                "started_wall_utc": _iso(clock.epoch_wall_ns),
                "ended_wall_utc": _iso(time.time_ns()),
                "duration_s": round(budgets["session"].elapsed_s, 3),
                "bytes_session": budgets["session"].bytes_written,
                "bytes_fullres": budgets["fullres"].bytes_written,
                "budgets": {k: v.summary() for k, v in budgets.items()},
                "streams": summaries,
                "derived": derived_stats(recs, clock),
                "version_pins": clock.version_pins(),
            }
            sw.write_json(manifest, "MANIFEST.json")
            meta = sw.read_json("meta.json")
            meta["status"] = status
            meta["stop_reason"] = stop_reason
            meta["ended_wall_utc"] = manifest["ended_wall_utc"]
            meta["duration_s"] = manifest["duration_s"]
            meta["clock"] = clock.meta()
            sw.write_json(meta, "meta.json")
        except Exception:
            log.exception("[vat-record] could not write the session manifest")
        sw.close()
        try:
            z.close()
        except Exception:
            pass
        _print_report(recs, budgets, sw, rcfg)
    return 0 if status == "complete" else 1


def _rss_bytes():
    """Resident set size of this process, or None. No hard dependency on psutil."""
    try:
        with open("/proc/self/status", "r") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    try:
        import psutil
        return int(psutil.Process().memory_info().rss)
    except Exception:
        return None


def write_live(sw, recs, budgets, clock, state: str, stop_reason: str = "") -> None:
    """Write ``<session>/live.json`` — a machine-readable snapshot of the run.

    Polled by ``ui.py`` for the live dashboard, and useful on its own
    (``watch -n1 jq . recordings/<id>/live.json``) to watch a capture from another
    shell without touching the recorder. Best-effort: a failure here must never
    disturb the recording.
    """
    try:
        b = budgets["session"]
        streams = {}
        for r in recs:
            st = r.stats.summary()
            streams[r.name] = {
                "samples": st["samples"], "bytes": st["bytes"],
                "mean_hz": st["mean_hz"], "errors": st["errors"],
                "skipped": st["skipped"],
                "seq_samples_missing": st["seq_samples_missing"],
                "last_error": st["last_error"],
                "status": r.status_line(),
            }
        sw.write_json({
            "state": state, "stop_reason": stop_reason,
            "session_id": sw.session_id, "pid": os.getpid(),
            "updated_wall_ns": time.time_ns(),
            "elapsed_s": round(b.elapsed_s, 2),
            "duration_cap_s": b.duration_s or None,
            "bytes_session": b.bytes_written,
            "bytes_fullres": budgets["fullres"].bytes_written,
            "max_bytes": b.max_bytes or None,
            "rss_bytes": _rss_bytes(),
            "clock_offset_s": clock.offset_s,
            "clock_baseline_ok": clock.baseline_observed,
            "version_pins": len(clock.version_pins()),
            "streams": streams,
        }, "live.json")
    except Exception:
        log.debug("[vat-record] live.json write failed", exc_info=True)


def _iso(ns: int) -> str:
    return _dt.datetime.fromtimestamp(ns / 1e9, _dt.timezone.utc).isoformat()


def _session_readme(session_id: str, out_root: str, version: str) -> str:
    path = os.path.join(out_root, session_id).replace(os.sep, "/")
    return _SESSION_README.format(session_id=session_id,
                                  underline="=" * (len(session_id) + 28),
                                  version=version, session_id_path=path)


def _progress_line(recs, budget, rcfg) -> str:
    fields = " ".join(r.status_line() for r in recs)
    return (f"[{budget.elapsed_s:7.1f}s] {fields}  "
            f"disk={rcfg.human_size(budget.bytes_written)}")


def print_plan(args, enabled, recs, sw, rcfg, zenoh_mode, budgets) -> None:
    print(f"\nvat-record v{rcfg.RECORDER_VERSION} — DRY RUN (nothing recorded)\n")
    print(f"  session id     {sw.session_id}")
    print(f"  output         {sw.root}")
    print(f"  where          {args.where}  (zenoh mode={zenoh_mode})")
    print(f"  router         {rcfg.ZENOH_ROUTER}")
    print(f"  robot/server   {rcfg.ROBOT_NAME} / {rcfg.SERVER_PREFIX}")
    print(f"  stream mode    {rcfg.STREAM_MODE}  cube={rcfg.CUBE_SIZE}m")
    print(f"  config hash    {rcfg.session_provenance()['mapping_config_hash'] or 'n/a'}")
    print(f"\n  streams enabled ({len(enabled)}):")
    for name in STREAMS:
        mark = "on " if name in enabled else "off"
        print(f"    [{mark}] {name}")
    print("\n  keys (subscribe):")
    for r in recs:
        for k in r.keys_subscribed:
            print(f"    ← {k:<48} {r.name}")
    print("\n  keys (query — the only non-passive calls):")
    any_q = False
    for r in recs:
        for k in r.keys_queried:
            any_q = True
            print(f"    ? {k:<48} {r.name}")
    if not any_q:
        print("    (none)")
    print("\n  publishers declared: none (pure observer)")
    print("\n  caps:")
    for k, b in budgets.items():
        print(f"    {k:<9} max_bytes={rcfg.human_size(b.max_bytes) if b.max_bytes else 'uncapped'}"
              f"  duration={(str(b.duration_s) + 's') if b.duration_s else 'uncapped'}"
              f"{'  (ring)' if hasattr(b, 'track') else ''}")
    print()


def _print_report(recs, budgets, sw, rcfg) -> None:
    print("\n" + "─" * 78)
    print(f"vat-record — {sw.session_id}")
    print("─" * 78)
    for r in recs:
        s = r.summary()
        hz = f"{s['mean_hz']:.2f} Hz" if s.get("mean_hz") else "—"
        gaps = (f"  gaps={s['seq_gap_events']}/-{s['seq_samples_missing']}"
                if s.get("seq_samples_missing") else "")
        errs = f"  errors={s['errors']}" if s.get("errors") else ""
        skipped = f"  skipped={s['skipped']}" if s.get("skipped") else ""
        print(f"  {r.name:<24} {s['samples']:>7} samples  "
              f"{rcfg.human_size(s['bytes']):>9}  {hz:>9}{gaps}{skipped}{errs}")
    print(f"\n  session {rcfg.human_size(budgets['session'].bytes_written)}"
          f"   full-res {rcfg.human_size(budgets['fullres'].bytes_written)}"
          f"   {budgets['session'].elapsed_s:.1f}s")
    print(f"  {sw.root}")
    print(f"\n  next:  python tools/recorder/compose.py info   {sw.root}")
    print(f"         python tools/recorder/compose.py export {sw.root} --fps 10")
    print("─" * 78 + "\n")


# ═════════════════════════════════════════════════════════════════════════════
# Self-test
# ═════════════════════════════════════════════════════════════════════════════


def _selftest(keep_dir: Optional[str] = None) -> int:
    """Offline end-to-end check: synthesise a session, then compose it.

    Exercises the real wire packers, the real writers and the real composer — no
    robot, no Zenoh, no GPU. This is the test to run after touching anything here.

    ``keep_dir`` writes the synthetic session there and leaves it in place, which is
    the quickest way to try ``compose.py`` against a known-good recording.
    """
    import json
    import shutil
    import tempfile

    import numpy as np

    import rec_clock
    import rec_cloud
    import rec_config
    import rec_frames
    import rec_periscope
    import rec_poses
    import rec_sinks

    logging.basicConfig(level=logging.WARNING)
    for mod in (rec_config, rec_clock, rec_sinks, rec_frames, rec_periscope,
                rec_poses, rec_cloud):
        mod._selftest()                                      # noqa: SLF001

    import vat_blockmap as bm
    import vat_protocol as proto
    import compose

    class _S:
        def __init__(self, payload):
            self.payload = payload

    # A virtual local clock so the synthetic session behaves like a real one: the
    # robot clock runs LOCAL_OFFSET behind local wall time, and each stream arrives
    # with its own transport latency. Without this, ten seconds of synthetic capture
    # would collapse into a few milliseconds of real time and the derived-timestamp
    # path (which is exactly what compose.py leans on for the map) would go untested.
    T0 = 1_700_000_000_000_000_000          # session-clock (robot capture) epoch
    LOCAL_OFFSET_NS = 3_000_000_000         # local wall clock is 3 s ahead
    LAT_POSE_NS = 20_000_000                # 30 Hz pose: the low-latency baseline
    LAT_FRAME_NS = 80_000_000               # panorama uplink
    LAT_PERI_NS = 60_000_000                # periscope encode + send
    LAT_MAP_NS = 1_200_000_000              # cloud: capture → submap published
    virt = {"wall": T0 + LOCAL_OFFSET_NS, "mono": 0}

    tmp = keep_dir or tempfile.mkdtemp(prefix="vatrec-e2e-")
    os.makedirs(tmp, exist_ok=True)
    try:
        sw = rec_sinks.SessionWriter(tmp, "e2e")
        clock = rec_clock.SessionClock(window_s=30.0,
                                       wall_ns=lambda: virt["wall"],
                                       mono_ns=lambda: virt["mono"])
        budget = rec_sinks.Budget(name="e2e")

        fullres = rec_frames.PanoramaFullresRecorder(
            sw, clock, rec_sinks.Budget(name="fr"), own_subscription=False)
        tx = rec_frames.PanoramaTransmitRecorder(sw, clock, budget,
                                                 seq_sink=fullres.offer)
        peri = rec_periscope.PeriscopeRecorder(sw, clock, budget, mux_mp4=False)
        pc = rec_cloud.PointCloudRecorder(sw, clock, budget, cube_m=1.0,
                                          keyframe_s=0.0, repair=False)
        esdf = rec_cloud.EsdfRecorder(sw, clock, budget)
        stat = rec_cloud.StatusRecorder(sw, clock, budget)
        fused = rec_poses.FusedPoseRecorder(sw, clock, budget)
        corr = rec_poses.CorrectionRecorder(sw, clock, budget)
        traj = rec_poses.TrajectoryRecorder(sw, clock, budget)
        recs = [tx, fullres, peri, pc, esdf, stat, fused, corr, traj]

        try:
            import cv2
            ok, jb = cv2.imencode(".jpg", np.zeros((518, 1036, 3), np.uint8))
            jpeg = jb.tobytes() if ok else b"\xff\xd8" + b"\x00" * 64
        except ImportError:
            jpeg = (b"\xff\xd8\xff\xc0\x00\x11\x08\x02\x06\x04\x0c" + b"\x00" * 8)

        # ── a 10 s synthetic session: 2.5 Hz panorama, 30 Hz pose, 1 Hz submap ──
        # Built as (delivery_local_ns, callable) events and then replayed in arrival
        # order, so every recorder sees the interleaving a live session produces.
        rng = np.random.default_rng(7)
        grid = bm.BlockGrid(cube_m=1.0, crc_quant_m=0.015)
        events = []

        def at(local_ns, fn):
            events.append((int(local_ns), fn))

        for i in range(25):                                  # panorama, 2.5 Hz
            src = T0 + i * 400_000_000
            wire = proto.pack_frame(src, 500 + i, 1.152, jpeg)
            at(src + LOCAL_OFFSET_NS + LAT_FRAME_NS,
               lambda w=wire: tx._on_frame(_S(w)))
        for i in range(300):                                 # fused pose, 30 Hz
            src = T0 + i * 33_333_333
            wire = proto.pack_pose(proto.PoseState(
                timestamp_ns=src, seq=i,
                position=np.array([i * 0.01, 0.0, 0.35], np.float32),
                quaternion=np.array([0, 0, 0, 1], np.float32),
                linear_velocity=np.array([0.3, 0, 0], np.float32),
                angular_velocity=np.zeros(3, np.float32),
                fix_quality=(proto.FIX_CORRECTED if i % 30 == 0
                             else proto.FIX_DEADRECKON)))
            at(src + LOCAL_OFFSET_NS + LAT_POSE_NS,
               lambda w=wire: fused._on_pose(_S(w)))
        for i in range(60):                                  # periscope, ~6 Hz
            src = T0 + i * 166_666_666
            wire = proto.pack_periscope_frame(proto.PeriscopeFrame(
                seq=i, timestamp_ns=src, codec=proto.PSCOPE_CODEC_H264,
                is_keyframe=(i % 20 == 0), width=640, height=480, native_w=640,
                hfov_deg=60.0, vfov_deg=45.0, aspect_w=4, aspect_h=3,
                payload=bytes([i % 256]) * 200))
            at(src + LOCAL_OFFSET_NS + LAT_PERI_NS,
               lambda w=wire: peri._on_frame(_S(w)))

        cells = np.stack([np.linspace(-1, 1, 9), np.zeros(9),
                          np.full(9, 0.6)], axis=1).astype(np.float32)
        t_ramp = np.linspace(0.0, 1.0, 9).astype(np.float32)
        ramp = np.clip(np.rint(np.stack([np.clip((1 - t_ramp) * 2, 0, 1),
                                         np.clip(t_ramp * 2, 0, 1),
                                         np.zeros_like(t_ramp)], axis=1) * 255),
                       0, 255).astype(np.uint8)
        for sub in range(10):                                # 10 submaps, 1 Hz
            mv = 100 + sub
            cap = T0 + sub * 1_000_000_000
            pts = (np.floor(rng.random((1500, 3)) * (20 + sub)) + 0.5) * 0.06
            pts = np.unique(pts.astype(np.float32), axis=0)
            col = np.full((pts.shape[0], 3), 100 + sub, np.uint8)
            changed, removed = grid.ingest(pts, col)
            # Same publish order the mapping server uses: push, manifest, trajectory,
            # correction, status, ESDF — 1 ms apart, then our own free keyframe.
            base = cap + LOCAL_OFFSET_NS + LAT_MAP_NS
            push = bm.pack_block_push(grid.collect(changed), removed,
                                      map_version=mv, cube_m=1.0)
            man = bm.pack_manifest(grid.manifest())
            tj = proto.pack_trajectory(np.stack(
                [np.linspace(0, sub * 0.3, 5), np.zeros(5),
                 np.full(5, 1.15)], axis=1).astype(np.float32))
            status = json.dumps({
                "state": "processing", "map_version": mv,
                "newest_frame_robot_ns": cap, "robot_kbps": 320.0,
                "robot_fps": 2.5, "n_points": int(pts.shape[0]),
                "cubes": len(grid.manifest()), "submap_s": 0.8}).encode()
            slice_wire = proto.pack_pcd(sub, cells, ramp, is_snapshot=True)
            at(base + 0, lambda w=push: pc._on_push(_S(w)))
            at(base + 1_000_000, lambda w=man: pc._on_manifest(_S(w)))
            at(base + 2_000_000, lambda w=tj: traj._on_traj(_S(w)))
            if sub % 2 == 0:                       # the gate suppresses the others
                cwire = proto.pack_pose_correction(proto.PoseCorrection(
                    timestamp_ns=cap, map_version=mv,
                    position=np.array([sub * 0.3, 0, 1.15], np.float32),
                    quaternion=np.array([0, 0, 0, 1], np.float32)))
                at(base + 3_000_000, lambda w=cwire: corr._on_correction(_S(w)))
            at(base + 4_000_000, lambda w=status: stat._on_status(_S(w)))
            at(base + 5_000_000, lambda w=slice_wire: esdf._on_slice(_S(w)))
            at(base + 6_000_000, lambda: pc.write_keyframe("timer"))

        for delivery, fn in sorted(events, key=lambda e: e[0]):
            virt["wall"] = delivery
            virt["mono"] = delivery - (T0 + LOCAL_OFFSET_NS)
            fn()

        # The offset estimator must have locked onto the LOWEST-latency stream (pose),
        # so a map artefact's derived time = capture + its excess latency, not
        # capture + the full transport delay. This is the contract compose.py relies on.
        assert abs(clock.offset_s - (LOCAL_OFFSET_NS + LAT_POSE_NS) / 1e9) < 1e-6, \
            clock.offset_s
        first_push = next(r for r in
                          [json.loads(l) for l in open(
                              sw.path("pointcloud", "index.jsonl")).read().splitlines()]
                          if r["kind"] == "push")
        assert first_push["ts_src"] == "derived"
        excess_ms = (first_push["src_ts_ns"] - T0) / 1e6
        assert abs(excess_ms - (LAT_MAP_NS - LAT_POSE_NS) / 1e6) < 2.0, excess_ms
        # A submap's first artefacts are published BEFORE its pose_correction/status
        # arrive, so this row has no pin yet — compose.py backfills it from
        # MANIFEST.json's complete table (asserted after the load below).
        assert first_push.get("capture_ts_ns") is None

        for r in recs:
            r.close()

        # ── manifest + derived cross-checks ──
        manifest = {
            "schema": rec_config.SCHEMA, "session_id": "e2e", "status": "complete",
            "stop_reason": "selftest", "started_wall_utc": _iso(clock.epoch_wall_ns),
            "ended_wall_utc": _iso(time.time_ns()), "duration_s": 10.0,
            "streams": {r.name: r.summary() for r in recs},
            "derived": derived_stats(recs, clock),
            "version_pins": clock.version_pins(),
        }
        sw.write_json(manifest, "MANIFEST.json")

        class _A:                                            # minimal args for meta
            pass
        a = _A()
        for k, v in dict(where="cloud", scene="selftest",
                         trajectory_family="loop", pass_index=1, seed=0,
                         camera_height=1.152, camera_height_source="tape",
                         mount_geometry="stick", clear_flat_floor=True,
                         operator="selftest", note=["synthetic"], meta=["k=v"],
                         max_size=None, duration=None, fullres_max_size="1GB",
                         fullres_duration=None, fullres_ring=True,
                         transmit_every=1, fullres_every=1,
                         pointcloud_keyframe_s=0.0,
                         pointcloud_snapshot_query_s=0.0).items():
            setattr(a, k, v)
        sw.write_json(build_meta(a, set(DEFAULT_STREAMS), recs, sw, clock,
                                 rec_config, "client"), "meta.json")

        d = manifest["derived"]
        assert d["pose_correction_gating"]["submaps_seen_in_status"] == 10
        assert d["pose_correction_gating"]["corrections_published"] == 5
        assert d["pose_correction_gating"]["suppressed_or_rejected"] == 5
        assert d["uplink"]["frames"] == 25
        assert abs(d["uplink"]["mean_hz"] - 2.5) < 0.05
        assert d["camera_height_observed"]["mean_m"] == 1.152
        # The window is bounded below by the map (published ~1.18 s after capture)
        # and above by the panorama (last frame at T0+9.6 s) — NOT by the sparse,
        # gated correction stream, which is reported separately.
        aw = d["aligned_window"]
        assert aw["duration_s"] > 7.0, aw
        assert aw["start_limited_by"] in ("pointcloud", "esdf", "status",
                                         "poses_trajectory"), aw
        assert aw["end_limited_by"] == "panorama_transmit", aw
        assert "poses_cloud_correction" in aw["sparse_streams"], aw
        assert aw["streams_with_no_samples"] == ["panorama_fullres"], aw
        assert d["version_pins"]["exact_from_pose_correction"] == 5
        assert d["version_pins"]["count"] == 10          # status pinned the rest
        assert manifest["streams"]["pointcloud"]["counts"]["keyframe"] == 11  # +final
        assert manifest["streams"]["periscope"]["keyframes"] == 3
        # nothing errored anywhere
        for name, s in manifest["streams"].items():
            assert s.get("errors", 0) == 0, (name, s.get("last_error"))
        sw.close()

        # ── the composer must read it back and align everything ──
        rep = compose.load(sw.root)
        # the pin table backfills the capture time the index row could not know yet
        pushes = [r for r in rep.pointcloud if r["kind"] == "push"]
        assert pushes[0]["capture_ts_ns"] == T0, pushes[0]
        assert pushes[0]["capture_ts_src"] == "pose_correction"
        assert all(r.get("capture_ts_ns") for r in rep.pointcloud
                   if r.get("map_version") is not None)
        info = compose.info(rep)
        assert info["streams"]["panorama_transmit"]["rows"] == 25
        assert info["streams"]["pointcloud"]["keyframes"] == 11
        assert info["streams"]["poses_robot_fused"]["rows"] == 300

        out = os.path.join(tmp, "aligned")
        tl = compose.export(rep, out, fps=5.0, map_mode="keyframe", link=False)
        assert len(tl) >= 40, len(tl)                   # ~8.4 s at 5 Hz
        for row in tl:
            assert row["panorama_transmit_file"], row
            assert row["pose_position_x"] is not None
            assert row["map_keyframe_file"], row
            assert row["periscope_byte_offset"] is not None
            # every asset must be within a plausible distance of its tick
            assert abs(row["panorama_transmit_dt_ms"]) <= 250.0
            assert abs(row["pose_dt_ms"]) <= 40.0
        # interpolated pose is monotonic in x (the synthetic robot walks forward)
        xs = [r["pose_position_x"] for r in tl]
        assert all(b >= a - 1e-6 for a, b in zip(xs, xs[1:])), "pose not monotonic"
        assert os.path.exists(os.path.join(out, "timeline.csv"))
        assert os.path.exists(os.path.join(out, "timeline.jsonl"))
        assert os.path.exists(os.path.join(out, "README.md"))

        # replaying raw pushes must reproduce the final keyframe exactly
        xyz_replay, rgb_replay, mv = compose.replay_map(rep, until_ts_ns=None)
        with np.load(os.path.join(sw.root, tl[-1]["map_keyframe_file"])) as z:
            kf_pts = z["points"]
        assert xyz_replay.shape == kf_pts.shape, (xyz_replay.shape, kf_pts.shape)
        assert mv == 109, mv
        a_sorted = np.sort(xyz_replay.view([("x", "f4"), ("y", "f4"), ("z", "f4")]),
                           axis=0)
        b_sorted = np.sort(kf_pts.view([("x", "f4"), ("y", "f4"), ("z", "f4")]), axis=0)
        assert np.array_equal(a_sorted, b_sorted), "replay != materialised keyframe"

        # ── the effective map time must be the per-submap CAPTURE time, not one
        #    arrival time smeared across the first second by the monotonic clamp ──
        push_t = [r["_t"] for r in rep.pointcloud if r["kind"] == "push"]
        assert len(set(push_t)) == 10, push_t
        assert push_t == sorted(push_t)
        assert push_t[0] == T0 and push_t[-1] == T0 + 9_000_000_000
        assert all(r["_t_src"] == "capture" for r in rep.pointcloud
                   if r.get("capture_ts_ns"))
        # …so distinct map versions are actually addressable through the timeline
        assert len({r["map_version"] for r in tl if r["map_version"]}) >= 9

        # periscope extraction slices real frames back out of the elementary stream
        pdir = os.path.join(tmp, "peri")
        n = compose.periscope_extract(rep, pdir, limit=5)
        assert n == 5 and len(os.listdir(pdir)) == 5

        # ── a hard kill can tear the last CSV row: it must be dropped, not crash ──
        pcsv = sw.path("periscope_timestamps.csv")
        text = open(pcsv).read().splitlines()
        with open(pcsv, "w") as f:                       # truncate the final row
            f.write("\n".join(text[:-1]) + "\n" + text[-1][:len(text[-1]) // 2])
        torn = compose.load(sw.root)
        assert len(torn.periscope) == len(rep.periscope) - 1
        assert compose.periscope_extract(torn, os.path.join(tmp, "peri2"),
                                         limit=0) == len(torn.periscope)

        # ── a blob the ring evicted (or any missing file) must be ignored on load,
        #    and frames/ must stay gap-free so the documented ffmpeg call works ──
        victim = rep.panorama_transmit[3]["file"]
        os.remove(sw.path(*victim.split("/")))
        pruned = compose.load(sw.root)
        assert pruned.missing_blobs.get("panorama_transmit") == 1
        assert len(pruned.panorama_transmit) == 24
        out2 = os.path.join(tmp, "aligned2")
        tl2 = compose.export(pruned, out2, fps=5.0, map_mode="none", link="copy")
        linked = sorted(os.listdir(os.path.join(out2, "frames")))
        assert linked == [f"{i:06d}.jpg" for i in range(len(linked))], "gap in frames/"
        assert all(r["linked_frame"] for r in tl2), "a tick has no frame"

        # ── --where robot must not default to the cloud's bulk streams ──
        class _Args:
            pass
        ra = _Args()
        for name in DEFAULT_STREAMS:
            setattr(ra, name, None)
        for name in STREAMS:
            if not hasattr(ra, name):
                setattr(ra, name, None)
        ra.all = False
        ra.where = "robot"
        robot_set = resolve_streams(ra)
        assert robot_set == set(DEFAULT_STREAMS_ROBOT), robot_set
        assert not robot_set.intersection(CLOUD_BULK_STREAMS)
        ra.where = "cloud"
        assert resolve_streams(ra) == set(DEFAULT_STREAMS)

        print("\nvat_record self-test OK — synthetic session recorded, cross-checked "
              "and composed end-to-end")
        if keep_dir:
            print(f"\nsynthetic session kept at {sw.root}\n"
                  f"  python tools/recorder/compose.py info   {sw.root}\n"
                  f"  python tools/recorder/compose.py export {sw.root} --fps 10")
        return 0
    finally:
        if not keep_dir:
            shutil.rmtree(tmp, ignore_errors=True)


# ═════════════════════════════════════════════════════════════════════════════

def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        logging.basicConfig(level=logging.WARNING)
        return _selftest(args.selftest_keep)
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO").upper(),
                        format="[%(asctime)s] [%(levelname)s] %(message)s",
                        datefmt="%H:%M:%S")
    return run_session(args)


if __name__ == "__main__":
    raise SystemExit(main())
