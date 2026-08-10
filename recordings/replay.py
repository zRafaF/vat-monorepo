#!/usr/bin/env python3
"""
VAT — ``replay``: play a recorded session back in Rerun
======================================================
Takes a ``recordings/data/<session_id>/`` written by ``tools/recorder`` and streams every
stream into `Rerun <https://rerun.io>`_ on **one timeline** — the session clock, i.e. the
robot capture clock — so you can scrub through a real walk afterwards: the map growing,
the trajectory being walked, the panorama the robot was looking at, the periscope the
operator was aiming, the ESDF underneath.

This is the "did we actually capture the run?" tool, and the one to reach for when
picking the stretch of a walk to build the paper's figure from. It reads only what
``vat_record`` wrote — no robot, no Zenoh, no mapping server.

    make replay                       # folder picker, then the Rerun app opens
    make replay ARGS="recordings/data/<session_id>"
    make replay ARGS="--list"         # what is in recordings/data/

**Run this on a machine with a screen.** The default mode is Rerun's own recommended one
(`rr.spawn`): it launches the native Rerun Viewer — the ``rerun`` executable that ships
inside ``rerun-sdk``, so ``uv run`` already has it on ``PATH`` — and streams the session
into it. Nothing to open in a browser, nothing to port-forward. The window stays up after
this script exits, because the data lives in the viewer, not here.

The other modes exist for when the recording is on a headless box:

* ``--save run.rrd`` writes a file; copy it to your laptop and ``rerun run.rrd``. With
  ``--open`` it opens the file here instead.
* ``--connect`` streams into a viewer you already have open (``rerun --serve`` /
  ``rerun``), including one across a tailnet.
* ``--serve`` is the old browser path, kept as a last resort. It needs *two* ports open to
  your browser and is much worse to scrub; prefer ``--save``.

What gets logged
----------------
=========================  ====================================================
``world/map``              the point cloud, from materialised keyframes or (with
                           ``--map replay``) rebuilt per submap from the recorded
                           Draco pushes
``world/trajectory``       the camera trail the cloud streamed
``world/robot``            the robot's fused pose — a transform plus a walked path
``world/esdf``             ESDF slices, coloured as published
``panorama``               the 360° frame, logged as an encoded JPEG (Rerun decodes
                           it, so this stays cheap)
``periscope``              the operator's video slice, if PyAV can decode it
``metrics/*``              map version, cube count, uplink kB/s and the observed
                           latency from the ``status`` stream
=========================  ====================================================

Everything is stamped on the ``session`` timeline in nanoseconds. Map artefacts use
``capture_ts_ns`` (the true capture time of that map version) rather than arrival, so the
map lines up with where the robot actually was — see ``docs/recording.md``.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
_RECORDER = os.path.join(_REPO, "tools", "recorder")
for _p in (_RECORDER,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import compose                     # noqa: E402  the recorder's own reader
import rec_config as rcfg          # noqa: E402,F401

import rerun as rr                 # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("vat-replay")

TIMELINE = "session"


def _t(ns: int) -> None:
    """Put the next log call at ``ns`` on the session timeline (robot capture clock)."""
    rr.set_time(TIMELINE, duration=float(ns) / 1e9)


# ═════════════════════════════════════════════════════════════════════════════
# Streams
# ═════════════════════════════════════════════════════════════════════════════


def log_static(rep: compose.Recording) -> None:
    """Frame convention + the session's identity, logged once."""
    # The VAT world frame is Z-up with the floor at Z=0 (see world_anchor / nav_esdf).
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    cap = rep.meta.get("capture", {}) or {}
    lines = [f"session: {rep.session_id}",
             f"scene: {cap.get('scene')}  family: {cap.get('trajectory_family')}  "
             f"pass: {cap.get('pass_index')}",
             f"camera height: {cap.get('camera_height_m')} m "
             f"({cap.get('camera_height_source') or 'source not recorded'})",
             f"operator: {cap.get('operator')}",
             f"config hash: {(rep.meta.get('config') or {}).get('mapping_config_hash')}",
             f"status: {rep.manifest.get('status')} ({rep.manifest.get('stop_reason')})"]
    for note in (cap.get("notes") or []):
        lines.append(f"note: {note}")
    rr.log("session_info", rr.TextDocument("\n".join(lines)), static=True)


def log_poses(rep: compose.Recording, decimate: int = 1) -> int:
    """The robot's fused pose: a transform per sample plus the path walked so far."""
    rows = rep.rows("fused")
    if not rows:
        return 0
    order = rep.order("fused")
    trail = []
    n = 0
    for i, idx in enumerate(order):
        r = rows[int(idx)]
        p = r.get("position")
        if not p:
            continue
        trail.append(p)
        if i % max(1, decimate):
            continue
        _t(int(r["src_ts_ns"]))
        q = r.get("quaternion") or [0, 0, 0, 1]
        rr.log("world/robot", rr.Transform3D(translation=p, quaternion=q))
        # fix_quality distinguishes a sample that just absorbed a cloud correction from
        # one that is dead-reckoning — worth seeing on the timeline.
        rr.log("metrics/fix_corrected",
               rr.Scalars(1.0 if r.get("fix_quality") else 0.0))
        rr.log("world/robot/path", rr.LineStrips3D([np.asarray(trail, np.float32)],
                                                   colors=[[80, 170, 255]]))
        n += 1
    return n


def log_corrections(rep: compose.Recording) -> int:
    """Cloud pose corrections — sparse, gated, and each one re-anchors the robot."""
    n = 0
    for r in rep.rows("corrections"):
        _t(int(r["src_ts_ns"]))
        rr.log("world/correction",
               rr.Points3D([r["position"]], colors=[[255, 90, 90]], radii=[0.06]))
        rr.log("metrics/map_version_corrected",
               rr.Scalars(float(r.get("map_version") or 0)))
        n += 1
    return n


def log_trajectory(rep: compose.Recording) -> int:
    n = 0
    for r in rep.rows("trajectory"):
        path = rep.fp(r)
        if not path or not os.path.exists(path):
            continue
        try:
            pts = np.load(path)
        except Exception:
            continue
        _t(int(r.get("capture_ts_ns") or r["src_ts_ns"]))
        rr.log("world/trajectory", rr.LineStrips3D([pts.astype(np.float32)],
                                                   colors=[[255, 200, 40]]))
        n += 1
    return n


def log_map(rep: compose.Recording, mode: str = "keyframe", max_points: int = 0) -> int:
    """The point cloud over time.

    ``keyframe`` uses the ``.npz`` states the recorder materialised from its mirror —
    cheap and exact. ``replay`` rebuilds the map per submap from the recorded Draco
    pushes, which shows every incremental change but costs a Draco decode per push.
    """
    n = 0
    if mode == "replay":
        import vat_blockmap as bm
        if not getattr(bm, "_HAVE_DRACO", False):
            log.warning("[map] DracoPy missing — falling back to --map keyframe")
            mode = "keyframe"
    if mode == "replay":
        import vat_blockmap as bm
        cube_m = float((rep.meta.get("zenoh") or {}).get("cube_size_m")
                       or rcfg.CUBE_SIZE)
        store = bm.ClientBlockStore(cube_m)
        for rec in rep.pointcloud:
            if rec.get("kind") not in ("push", "repair", "manifest", "snapshot"):
                continue
            if not compose._map_apply(store, rep, rec):          # noqa: SLF001
                continue
            merged = store.merged()
            if merged is None:
                continue
            xyz, rgb = merged
            if xyz.shape[0] == 0:
                continue
            _t(int(rec["_t"]))
            _log_cloud("world/map", xyz, rgb, max_points)
            rr.log("metrics/map_version",
                   rr.Scalars(float(rec.get("map_version") or 0)))
            n += 1
        return n

    for rec in rep.keyframes or rep.snapshots:
        try:
            if rec.get("kind") == "snapshot":
                import vat_protocol as proto
                with open(rep.fp(rec), "rb") as f:
                    _v, xyz, rgb, _s, _sv = proto.unpack_pcd(f.read())
                rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
            else:
                xyz, rgb, _mv = compose.load_map_npz(rep, rec)
        except Exception as e:                                   # noqa: BLE001
            log.warning(f"[map] {rec.get('file')}: {e}")
            continue
        _t(int(rec["_t"]))
        _log_cloud("world/map", xyz, rgb, max_points)
        rr.log("metrics/map_version", rr.Scalars(float(rec.get("map_version") or 0)))
        n += 1
    return n


def _log_cloud(path: str, xyz, rgb, max_points: int) -> None:
    xyz = np.asarray(xyz, np.float32)
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8:
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    if max_points and xyz.shape[0] > max_points:
        # Deterministic subsample so consecutive frames keep the same points and the
        # cloud does not shimmer as you scrub.
        step = int(np.ceil(xyz.shape[0] / max_points))
        xyz, rgb = xyz[::step], rgb[::step]
    rr.log(path, rr.Points3D(xyz, colors=rgb, radii=0.02))


def log_esdf(rep: compose.Recording, max_points: int = 0) -> int:
    n = 0
    for r in rep.rows("esdf"):
        npz = r.get("npz")
        if not npz:
            continue
        p = os.path.join(r.get("_root") or rep.root, *str(npz).split("/"))
        if not os.path.exists(p):
            continue
        try:
            with np.load(p) as z:
                pts, col = z["points"], z["colors"]
        except Exception:
            continue
        _t(int(r["src_ts_ns"]))
        _log_cloud("world/esdf", pts, col, max_points)
        n += 1
    return n


def log_panorama(rep: compose.Recording, which: str = "auto",
                 decimate: int = 1) -> int:
    """Log the panorama as an *encoded* JPEG — Rerun decodes it, so this stays cheap."""
    # --panorama takes auto|transmit|fullres; the Recording attributes are the full
    # stream names. Mapping them directly meant "fullres" silently matched nothing.
    stream = {"transmit": "panorama_transmit",
              "fullres": "panorama_fullres"}.get(which, "")
    if which == "auto":
        stream = ("panorama_fullres" if rep.rows("panorama_fullres")
                  else "panorama_transmit")
    if not rep.rows(stream):
        alt = ("panorama_transmit" if stream == "panorama_fullres"
               else "panorama_fullres")
        if rep.rows(alt):
            log.warning(f"[panorama] no {stream} frames in this recording — using "
                        f"{alt} instead (run backfill.py for full-res)")
            stream = alt
    rows = rep.rows(stream)
    if not rows:
        return 0
    ent = "panorama" if stream == "panorama_transmit" else "panorama_fullres"
    n = 0
    for i, idx in enumerate(rep.order(stream)):
        if i % max(1, decimate):
            continue
        r = rows[int(idx)]
        path = rep.fp(r)
        if not path or not os.path.exists(path):
            continue
        _t(int(r["src_ts_ns"]))
        rr.log(ent, rr.EncodedImage(path=path))
        if r.get("camera_height_m"):
            rr.log("metrics/camera_height_m",
                   rr.Scalars(float(r["camera_height_m"])))
        if r.get("wire_bytes"):
            rr.log("metrics/uplink_bytes_per_frame",
                   rr.Scalars(float(r["wire_bytes"])))
        n += 1
    return n


def log_periscope(rep: compose.Recording, decimate: int = 1) -> int:
    """Decode the periscope with the *viewer's own* decoder, if it is available."""
    rows = rep.rows("periscope")
    if not rows:
        return 0
    try:
        decoder = compose._load_periscope_decoder()               # noqa: SLF001
    except SystemExit as e:
        log.info(f"[periscope] not logged: {str(e).splitlines()[0]}")
        return 0
    handles, n = {}, 0
    try:
        for i, r in enumerate(rows):
            if i % max(1, decimate):
                continue
            seg = r.get("segment")
            if not seg:
                continue
            if seg not in handles:
                handles[seg] = open(rep.fp(r, "segment"), "rb")
            f = handles[seg]
            f.seek(int(r["byte_offset"]))
            payload = f.read(int(r["byte_len"]))
            frame = compose._decode_periscope(decoder, r.get("codec"),  # noqa: SLF001
                                              payload)
            if frame is None:
                continue                     # normal before the first keyframe
            _t(int(r["src_ts_ns"]))
            rr.log("periscope", rr.Image(np.asarray(frame)))
            n += 1
    finally:
        for f in handles.values():
            f.close()
    return n


def log_status(rep: compose.Recording) -> int:
    """Server telemetry: the measured uplink and the observed latency, over time."""
    n = 0
    for r in rep.rows("status"):
        st = r.get("status") or {}
        _t(int(r["src_ts_ns"]))
        for field, ent in (("robot_kbps", "metrics/robot_kbps"),
                           ("robot_fps", "metrics/robot_fps"),
                           ("robot_to_server_ms", "metrics/robot_to_server_ms"),
                           ("n_points", "metrics/map_points"),
                           ("cubes", "metrics/map_cubes"),
                           ("submap_s", "metrics/submap_seconds")):
            v = st.get(field)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                rr.log(ent, rr.Scalars(float(v)))
        n += 1
    return n


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════


def replay(session: str, *, partners=(), map_mode: str = "keyframe",
           panorama: str = "auto", max_points: int = 300_000,
           pose_decimate: int = 3, pano_decimate: int = 1,
           peri_decimate: int = 2, skip=()) -> compose.Recording:
    rep = compose.load(session, extra_roots=partners)
    log.info(f"[replay] {rep.session_id}")
    log_static(rep)
    counts = {}
    if "map" not in skip:
        counts["map"] = log_map(rep, map_mode, max_points)
    if "poses" not in skip:
        counts["poses"] = log_poses(rep, pose_decimate)
        counts["corrections"] = log_corrections(rep)
    if "trajectory" not in skip:
        counts["trajectory"] = log_trajectory(rep)
    if "panorama" not in skip:
        counts["panorama"] = log_panorama(rep, panorama, pano_decimate)
    if "periscope" not in skip:
        counts["periscope"] = log_periscope(rep, peri_decimate)
    if "esdf" not in skip:
        counts["esdf"] = log_esdf(rep, max_points)
    if "status" not in skip:
        counts["status"] = log_status(rep)
    log.info("[replay] logged " + "  ".join(f"{k}={v}" for k, v in counts.items()
                                            if v))
    return rep


def viewer_host() -> str:
    """An address the *browser* can reach this machine on (``--serve`` only).

    Lives in ``rec_config`` so the recorder console (``tools/recorder/ui.py``) and this
    tool cannot disagree about the URL they hand the operator.
    """
    return rcfg.viewer_host()


# ═════════════════════════════════════════════════════════════════════════════
# Where the data goes: the native viewer, a file, or (last resort) a browser
# ═════════════════════════════════════════════════════════════════════════════


def viewer_executable(name: str = "rerun") -> str:
    """Path to the native Rerun Viewer, or ``''``.

    ``rr.spawn`` searches ``PATH`` for this; ``rerun-sdk`` installs it into this project's
    ``bin``/``Scripts``, so ``uv run`` finds it without anything extra. Resolving it
    ourselves first only exists so we can say *why* the window did not open. Shared with
    the recorder console via ``rec_config``.
    """
    return rcfg.viewer_executable(name)


def has_display() -> bool:
    """Can a GUI window actually appear on this machine? (see ``rec_config``)"""
    return rcfg.has_display()


def port_open(port: int, host: str = "127.0.0.1", timeout: float = 0.5) -> bool:
    """Is something accepting connections there? Used to tell "the viewer opened" from
    "the viewer died on startup", which otherwise looks identical from this side."""
    import socket
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def pick_session_dialog(root_dir: str) -> str:
    """A native folder picker for choosing ``recordings/data/<session_id>``.

    Deliberately a *system* dialog rather than anything web: this tool is meant to be run
    on the machine with the screen, and a real file browser is what you want when you are
    hunting for "the one from yesterday afternoon". Returns ``''`` when there is no GUI or
    no Tk (both perfectly normal), and the caller falls back to a numbered prompt.
    """
    if not has_display():
        return ""
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:                                    # noqa: BLE001
        return ""
    try:
        root = tk.Tk()
        root.withdraw()
        root.update()
        chosen = filedialog.askdirectory(
            initialdir=root_dir if os.path.isdir(root_dir) else os.getcwd(),
            title="Choose a recorded session  (recordings/data/<session_id>)",
            mustexist=True)
        root.destroy()
    except Exception:                                    # noqa: BLE001
        return ""
    return chosen or ""


def prompt_session(sessions) -> str:
    """Numbered fallback picker, for a terminal with no GUI."""
    if not sys.stdin or not sys.stdin.isatty():
        return ""
    print(f"\nrecordings under {rcfg.DEFAULT_OUT_ROOT}:\n")
    for i, (name, when, size) in enumerate(sessions):
        print(f"  [{i}] {name:<48} {when}  {rcfg.human_size(size)}")
    try:
        raw = input("\nwhich one? [0] ").strip() or "0"
        return os.path.join(rcfg.DEFAULT_OUT_ROOT, sessions[int(raw)][0])
    except (ValueError, IndexError, EOFError, KeyboardInterrupt):
        return ""


def list_sessions():
    """``[(name, mtime_iso, bytes)]`` for every recording, newest first."""
    root = rcfg.DEFAULT_OUT_ROOT
    if not os.path.isdir(root):
        return []
    import datetime
    out = []
    for name in os.listdir(root):
        d = os.path.join(root, name)
        if not os.path.isdir(d) or name.startswith("_"):
            continue
        total = 0
        for r, _dirs, files in os.walk(d):
            for fn in files:
                try:
                    total += os.path.getsize(os.path.join(r, fn))
                except OSError:
                    pass
        when = datetime.datetime.fromtimestamp(
            os.path.getmtime(d)).strftime("%Y-%m-%d %H:%M")
        out.append((name, when, total))
    out.sort(key=lambda t: t[1], reverse=True)
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="replay",
        description="Play a recorded VAT session back in Rerun.")
    p.add_argument("session", nargs="?", default=None,
                   help="recordings/data/<session_id>; omit for a folder picker "
                        "(--newest to skip it, --list to just see what is there)")
    p.add_argument("--list", action="store_true",
                   help="list the recordings under the output root and exit")
    p.add_argument("--newest", action="store_true",
                   help="use the most recent recording without asking")
    p.add_argument("--with", dest="partners", action="append", default=[],
                   metavar="SESSION",
                   help="merge a partner session from the same capture (repeatable)")

    g = p.add_argument_group(
        "where to send it",
        "Default: open the native Rerun Viewer on THIS machine and stream into it "
        "(rr.spawn — Rerun's recommended mode). The rest are for a headless box.")
    g.add_argument("--save", default=None, metavar="FILE.rrd",
                   help="write an .rrd instead of opening a viewer — copy it to a "
                        "machine with a screen and run `rerun FILE.rrd`")
    g.add_argument("--open", dest="open_saved", action="store_true",
                   help="with --save, also open the file in the viewer here afterwards")
    g.add_argument("--connect", nargs="?", const="", default=None, metavar="URL",
                   help="stream into a viewer that is ALREADY running instead of "
                        "spawning one (default rerun+http://127.0.0.1:9876/proxy)")
    g.add_argument("--serve", action="store_true",
                   help="last resort: serve a browser viewer from this machine. Needs "
                        "BOTH ports reachable from the browser and scrubs poorly — "
                        "prefer --save")
    g.add_argument("--memory-limit", default="75%",
                   help="cap the viewer's memory; it drops the oldest data past this "
                        "(default 75%% of system RAM)")
    g.add_argument("--viewer-exe", default="rerun", metavar="NAME_OR_PATH",
                   help="the viewer executable rr.spawn should launch")
    p.add_argument("--viewer-host", default=None, metavar="HOST",
                   help="--serve only: host the BROWSER should use to reach the gRPC "
                        "stream. Defaults to the router host from vat.env, because "
                        "'localhost' resolves on the browser's machine, not the server")
    p.add_argument("--port", type=int, default=9090, help="--serve web viewer port")
    p.add_argument("--grpc-port", type=int, default=9876)
    p.add_argument("--map", dest="map_mode", choices=("keyframe", "replay", "none"),
                   default="keyframe",
                   help="'keyframe' = the materialised map states (default); 'replay' = "
                        "rebuild per submap from the recorded Draco pushes")
    p.add_argument("--panorama", choices=("auto", "transmit", "fullres"),
                   default="auto")
    p.add_argument("--max-points", type=int, default=300_000,
                   help="cap points per cloud frame (0 = all)")
    p.add_argument("--pose-every", type=int, default=3,
                   help="log every Nth fused pose (30 Hz is more than a viewer needs)")
    p.add_argument("--panorama-every", type=int, default=1)
    p.add_argument("--periscope-every", type=int, default=2)
    p.add_argument("--skip", action="append", default=[],
                   choices=("map", "poses", "trajectory", "panorama", "periscope",
                            "esdf", "status"),
                   help="omit a stream (repeatable) — handy when one is huge")
    a = p.parse_args(argv)

    # ── which recording ──────────────────────────────────────────────────────
    if a.list or not a.session:
        sessions = list_sessions()
        if not sessions:
            raise SystemExit(f"no recordings under {rcfg.DEFAULT_OUT_ROOT}")
        if a.list:
            print(f"\nrecordings under {rcfg.DEFAULT_OUT_ROOT}:\n")
            for i, (name, when, size) in enumerate(sessions):
                print(f"  [{i}] {name:<48} {when}  {rcfg.human_size(size)}")
            print()
            return 0
        if a.newest:
            a.session = os.path.join(rcfg.DEFAULT_OUT_ROOT, sessions[0][0])
            log.info(f"[replay] newest: {sessions[0][0]}")
        else:
            a.session = (pick_session_dialog(rcfg.DEFAULT_OUT_ROOT)
                         or prompt_session(sessions))
            if not a.session:
                a.session = os.path.join(rcfg.DEFAULT_OUT_ROOT, sessions[0][0])
                log.info(f"[replay] nothing picked — using the newest: "
                         f"{sessions[0][0]}")
    a.session = os.path.normpath(a.session)
    if not os.path.isdir(a.session):
        raise SystemExit(f"not a recording directory: {a.session}")
    if not any(os.path.exists(os.path.join(a.session, f))
               for f in ("MANIFEST.json", "meta.json")):
        log.warning(f"[replay] {a.session} has no MANIFEST.json/meta.json — is this a "
                    f"session directory, or its parent?")

    # ── which sink ───────────────────────────────────────────────────────────
    mode = ("save" if a.save else "serve" if a.serve
            else "connect" if a.connect is not None else "viewer")
    exe = viewer_executable(a.viewer_exe)
    if mode == "viewer":
        # Say exactly what is missing instead of letting rr.spawn fail obscurely — the
        # usual cause is running this on the headless server rather than on a laptop.
        if not exe:
            raise SystemExit(
                f"cannot find the Rerun Viewer executable {a.viewer_exe!r} on PATH.\n"
                f"  `make replay` runs inside recordings/ where rerun-sdk provides it — "
                f"try `cd recordings && uv sync` first,\n"
                f"  or pass --viewer-exe /path/to/rerun, or use --save run.rrd.")
        if not has_display():
            raise SystemExit(
                "no display on this machine, so no viewer window can open.\n"
                "  Replay is meant to run where the screen is: copy the session to your "
                "laptop and `make replay` there,\n"
                "  or from here:  --save run.rrd   (then `rerun run.rrd` on the laptop)"
                "   ·   --serve   (browser, needs two ports open)")

    # No '/' in the application id: Rerun 0.36 migrates such names and warns.
    name = f"vat_replay_{os.path.basename(a.session)}"
    rr.init(name, spawn=False)
    if mode == "save":
        rr.save(a.save)
    elif mode == "viewer":
        # rr.spawn is Rerun's own recommended mode: it launches the native viewer and
        # streams to it over gRPC. detach_process (the default) is what lets the window
        # outlive this script, and an already-open viewer on the port simply receives the
        # data instead of a second one starting.
        log.info(f"[replay] opening the Rerun Viewer ({exe}) — the window stays open "
                 f"after this finishes")
        rr.spawn(port=a.grpc_port, memory_limit=a.memory_limit,
                 hide_welcome_screen=True, executable_path=exe)
    elif mode == "connect":
        url = a.connect or f"rerun+http://127.0.0.1:{a.grpc_port}/proxy"
        if not a.connect and not port_open(a.grpc_port):
            raise SystemExit(
                f"nothing is listening on 127.0.0.1:{a.grpc_port}, so there is no viewer "
                f"to connect to.\n  Start one first (`rerun`), or drop --connect and let "
                f"this open its own.")
        log.info(f"[replay] streaming into the viewer at {url}")
        rr.connect_grpc(url)
    else:
        rr.serve_grpc(grpc_port=a.grpc_port)

    replay(a.session, partners=a.partners, map_mode=a.map_mode,
           panorama=a.panorama, max_points=a.max_points,
           pose_decimate=a.pose_every, pano_decimate=a.panorama_every,
           peri_decimate=a.periscope_every, skip=set(a.skip))

    if mode in ("viewer", "connect", "save"):
        # Flush and close the sink before exiting: with a file this closes it cleanly, and
        # with the viewer it guarantees the last batch left this process.
        rr.disconnect()
        if mode == "save":
            size = rcfg.human_size(os.path.getsize(a.save)) \
                if os.path.exists(a.save) else "?"
            log.info(f"[replay] wrote {a.save} ({size}) — open it with:  rerun {a.save}")
            if a.open_saved:
                if not exe:
                    log.warning("[replay] --open: no viewer executable found")
                else:
                    import subprocess
                    log.info(f"[replay] opening {a.save}")
                    subprocess.Popen([exe, a.save])
        elif not port_open(a.grpc_port):
            # The viewer is a separate process: if it fell over on startup (a graphics
            # driver it cannot use is the classic one) this side happily "logs" into a
            # socket that nobody is holding. Say so instead of claiming success.
            log.error(
                f"nothing is listening on 127.0.0.1:{a.grpc_port} — the viewer process "
                f"exited instead of opening a window, so NOTHING was displayed.\n"
                f"  Its own error is above (a graphics/driver complaint is the usual "
                f"one; try `{exe}` on its own to see it).\n"
                f"  Meanwhile:  --save run.rrd  captures the same replay to a file.")
            return 2
        else:
            log.info("[replay] streamed into the viewer — scrub it there. "
                     "(This process is done; the window is not.)")
        return 0

    # --serve: `connect_to` is resolved BY THE BROWSER, so it must be an address the
    # browser can reach. "localhost" points at whatever machine you are browsing from —
    # which is why the page used to load and then show nothing.
    host = a.viewer_host or viewer_host()
    rr.serve_web_viewer(web_port=a.port, open_browser=False,
                        connect_to=f"rerun+http://{host}:{a.grpc_port}/proxy")
    log.info(f"[replay] open  http://{host}:{a.port}   (streaming from "
             f"{host}:{a.grpc_port})")
    log.info("[replay] BOTH ports must be reachable from your browser; --viewer-host "
             "overrides. Ctrl-C to stop.")
    try:
        import time
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
