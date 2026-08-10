#!/usr/bin/env python3
"""
VAT — ``backfill``: pull full-resolution panoramas into a finished recording
==========================================================================
The robot keeps a full-resolution twin of **every** transmitted frame in a local
rolling archive (``ARCHIVE_ENABLE=true``, ~10 GB ≈ 6 hours at 2.5 Hz), tagged with the
same ``seq`` / ``ts_ns`` / ``camera_height`` as the thin frame the cloud received. So
the full-res stream does **not** have to be captured live: walk the robot, stop the
recorder, and *then* fetch the twins for the frames you actually recorded.

That is strictly better than pulling them during the run:

* **Zero realtime pressure.** Nothing is competing with the live pose downlink or the
  map stream, so the capture itself is untouched — which is the whole point of a passive
  recorder.
* **You choose the cost afterwards.** ``--every 2`` halves it; ``--max-size`` caps it;
  ``--seq-range`` grabs only the interesting stretch of the walk.
* **Resumable.** Frames already present are skipped, so an interrupted or rate-limited
  backfill just continues.

The only deadline is the archive's rolling window: fetch before the robot evicts those
seqs. For a five-minute walk that is hours of slack.

The result lands in ``panorama_fullres/`` of the *same* session, in exactly the layout
the live puller would have produced, so ``compose.py`` treats it identically
(``--panorama fullres``). Together with ``poses/robot_fused.tum`` that gives the
full-resolution frames + timestamped trajectory an offline reconstruction (Gaussian
splat, NeRF, photogrammetry) needs.

Usage
-----
::

    make backfill ARGS="recordings/<session_id>"
    make backfill ARGS="recordings/<session_id> --every 2 --max-size 20GB"

    python tools/recorder/backfill.py recordings/<id> --dry-run   # what it would cost

Run it while nothing else is capturing.
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import sys
import time
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_protocol as proto       # noqa: E402

from rec_frames import _FRAME_COLUMNS, image_dims  # noqa: E402
from rec_sinks import Budget, RingBudget, SessionWriter  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("vat-backfill")


def read_transmit_index(session: str) -> List[dict]:
    """The recorded transmit frames — the seq list to backfill, in capture order."""
    path = os.path.join(session, "panorama_transmit", "frame_index.csv")
    if not os.path.exists(path):
        raise SystemExit(
            f"{path} not found.\n"
            f"Backfill needs the transmit index: it is the record of which seqs existed "
            f"and when they were captured. Record --panorama-transmit (it is in the "
            f"default set) and the twins can always be fetched afterwards.")
    with open(path, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r.get("seq")]
    for r in rows:
        r["seq"] = int(r["seq"])
        r["src_ts_ns"] = int(r["src_ts_ns"])
    rows.sort(key=lambda r: r["seq"])
    return rows


def existing_seqs(session: str) -> set:
    """Seqs already backfilled, so a re-run resumes instead of refetching."""
    d = os.path.join(session, "panorama_fullres", "frames")
    if not os.path.isdir(d):
        return set()
    out = set()
    for name in os.listdir(d):
        stem, ext = os.path.splitext(name)
        if ext.lower() in (".jpg", ".jpeg", ".webp", ".png") and stem.isdigit():
            out.add(int(stem))
    return out


def backfill(session: str, *, every: int = 1, max_bytes: int = 0, ring: bool = False,
             timeout_s: float = 5.0, seq_lo: Optional[int] = None,
             seq_hi: Optional[int] = None, dry_run: bool = False,
             progress_s: float = 5.0, sleep_ms: float = 0.0,
             status_cb=None) -> dict:
    """Fetch full-res twins for a finished recording. Returns a summary dict."""
    session = os.path.abspath(session)
    rows = read_transmit_index(session)
    if seq_lo is not None:
        rows = [r for r in rows if r["seq"] >= seq_lo]
    if seq_hi is not None:
        rows = [r for r in rows if r["seq"] <= seq_hi]
    every = max(1, int(every))
    wanted = rows[::every]
    have = existing_seqs(session)
    todo = [r for r in wanted if r["seq"] not in have]

    mean_tx = (sum(int(r.get("image_bytes") or 0) for r in rows) / len(rows)) if rows else 0
    # Full-res is ~4x the linear resolution of the transmit frame, so ~an order of
    # magnitude more bytes. This is only an estimate for the operator; the real cost is
    # measured as we go.
    est = len(todo) * mean_tx * 8
    log.info(f"[backfill] {session}")
    log.info(f"[backfill] {len(rows)} recorded frames → {len(wanted)} wanted "
             f"(--every {every}) → {len(todo)} to fetch "
             f"({len(wanted) - len(todo)} already present)")
    log.info(f"[backfill] rough estimate {rcfg.human_size(int(est))} "
             f"(transmit mean {rcfg.human_size(int(mean_tx))}/frame)")
    if dry_run:
        return {"planned": len(todo), "already_present": len(wanted) - len(todo),
                "estimate_bytes": int(est), "fetched": 0, "dry_run": True}
    if not todo:
        log.info("[backfill] nothing to do")
        return {"planned": 0, "already_present": len(wanted), "fetched": 0,
                "missing": 0, "bytes": 0}

    import zenoh
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{rcfg.ZENOH_ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    key = rcfg.KEYS["camera_archive_get"]
    log.info(f"[backfill] ? '{key}' via {rcfg.ZENOH_ROUTER}")

    sw = SessionWriter(os.path.dirname(session), os.path.basename(session))
    sw.subdir("panorama_fullres", "frames")
    idx_path = os.path.join(session, "panorama_fullres", "frame_index.csv")
    append = os.path.exists(idx_path)
    # Appending keeps a resumed backfill's earlier rows; CsvIndex always writes a
    # header, so open the file directly when continuing.
    if append:
        fh = open(idx_path, "a", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        idx = None
    else:
        idx = sw.csv_index("panorama_fullres", "frame_index.csv",
                           columns=_FRAME_COLUMNS)
        fh, writer = None, None

    budget = (RingBudget if ring else Budget)(max_bytes=max_bytes, name="backfill")
    n_ok = n_miss = n_timeout = 0
    n_bytes = 0
    t0 = time.monotonic()
    last = 0.0
    try:
        for i, row in enumerate(todo):
            seq = row["seq"]
            if budget.bytes_exhausted() and not ring:
                log.warning(f"[backfill] size cap {rcfg.human_size(max_bytes)} reached "
                            f"after {n_ok} frames — stopping")
                break
            payload = None
            err = None
            try:
                for reply in z.get(f"{key}?seq={seq}", timeout=timeout_s):
                    if reply.ok:
                        payload = bytes(reply.result.payload)
                        break
                    try:
                        err = bytes(reply.err.payload).decode(errors="replace")
                    except Exception:
                        err = "error reply"
            except Exception as e:                              # noqa: BLE001
                err = f"{type(e).__name__}: {e}"
            if payload is None:
                if err:
                    n_miss += 1
                    if n_miss <= 3 or n_miss % 50 == 0:
                        log.warning(f"[backfill] seq={seq}: {err}")
                else:
                    n_timeout += 1
                    if n_timeout <= 3:
                        log.warning(f"[backfill] seq={seq}: no reply in {timeout_s}s")
            else:
                # The archive replies with a pack_frame payload carrying the ORIGINAL
                # ts/seq/camera_height, so the twin keeps the transmit frame's identity.
                a_ts, a_seq, a_h, body = proto.unpack_frame(payload)
                budget.claim(len(body))
                path, _ = sw.write_blob(body, "panorama_fullres", "frames",
                                       f"{a_seq:09d}.jpg")
                track = getattr(budget, "track", None)
                if callable(track):
                    track(path, len(body))
                w, h = image_dims(body)
                # ts_src=source: this is the true capture timestamp, so a backfilled
                # frame aligns exactly like a live-recorded one. latency_ms is blank —
                # this was fetched long after capture and a transport latency would be
                # meaningless here.
                rec = [a_seq, a_ts, "source", time.time_ns(), time.monotonic_ns(),
                       len(payload), len(body), f"{a_h:.4f}", w, h, "",
                       f"panorama_fullres/frames/{a_seq:09d}.jpg"]
                if writer is not None:
                    writer.writerow(rec)
                    fh.flush()
                else:
                    idx.append(rec)
                n_ok += 1
                n_bytes += len(body)
            if sleep_ms:
                time.sleep(sleep_ms / 1000.0)
            now = time.monotonic()
            if progress_s > 0 and now - last >= progress_s:
                last = now
                done = i + 1
                rate = done / max(now - t0, 1e-6)
                eta = (len(todo) - done) / rate if rate > 0 else 0
                msg = (f"[backfill] {done}/{len(todo)}  ok={n_ok} miss={n_miss} "
                       f"{rcfg.human_size(n_bytes)}  {rate:.1f} frame/s  "
                       f"ETA {eta / 60:.1f} min")
                log.info(msg)
                if status_cb:
                    status_cb({"done": done, "total": len(todo), "ok": n_ok,
                               "missing": n_miss, "bytes": n_bytes,
                               "rate_fps": rate, "eta_s": eta, "message": msg})
    finally:
        if fh is not None:
            fh.close()
        if idx is not None:
            idx.close()
        sw.close()
        try:
            z.close()
        except Exception:
            pass

    dt = time.monotonic() - t0
    summary = {
        "session": session, "planned": len(todo), "fetched": n_ok,
        "missing": n_miss, "timeouts": n_timeout, "bytes": n_bytes,
        "already_present": len(wanted) - len(todo), "every": every,
        "seconds": round(dt, 1),
        "mean_bytes_per_frame": int(n_bytes / n_ok) if n_ok else 0,
    }
    log.info(f"[backfill] done: {n_ok} fetched, {n_miss} missing, "
             f"{rcfg.human_size(n_bytes)} in {dt:.0f}s")
    if n_miss:
        log.warning(f"[backfill] {n_miss} seq(s) were not in the robot's archive — "
                    f"either evicted from the rolling window (fetch sooner next time) "
                    f"or dropped by the archive writer under back-pressure "
                    f"(ARCHIVE_ENABLE / disk on the robot).")
    # Record what happened, next to the frames, so the recording stays self-describing.
    try:
        sw2 = SessionWriter(os.path.dirname(session), os.path.basename(session))
        prev = []
        p = os.path.join(session, "panorama_fullres", "backfill.json")
        if os.path.exists(p):
            import json
            with open(p, encoding="utf-8") as f:
                old = json.load(f)
            prev = old.get("runs", []) if isinstance(old, dict) else []
        summary["finished_wall_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                     time.gmtime())
        sw2.write_json({"tool": "tools/recorder/backfill.py",
                        "note": ("Full-res twins fetched from the robot's rolling "
                                 "archive AFTER the capture; src_ts_ns is the original "
                                 "capture time, so these align exactly like "
                                 "live-recorded frames."),
                        "runs": prev + [summary]},
                       "panorama_fullres", "backfill.json")
        sw2.close()
    except Exception:
        log.debug("[backfill] could not write backfill.json", exc_info=True)
    return summary


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="backfill",
        description="Pull full-resolution panoramas into a finished recording.")
    p.add_argument("session", help="recordings/<session_id>")
    p.add_argument("--every", type=int, default=1, metavar="N",
                   help="fetch every Nth recorded frame (default 1 = all)")
    p.add_argument("--max-size", default="0", metavar="SIZE",
                   help="stop after SIZE ('20GB'); 0 = uncapped")
    p.add_argument("--ring", action="store_true",
                   help="at the cap, evict oldest instead of stopping")
    p.add_argument("--seq-from", type=int, default=None, help="first seq to fetch")
    p.add_argument("--seq-to", type=int, default=None, help="last seq to fetch")
    p.add_argument("--timeout", type=float, default=5.0, help="per-frame query timeout")
    p.add_argument("--sleep-ms", type=float, default=0.0,
                   help="pause between fetches, to be gentle on the link")
    p.add_argument("--progress", type=float, default=5.0)
    p.add_argument("--dry-run", action="store_true",
                   help="report what would be fetched, and roughly how big, then exit")
    a = p.parse_args(argv)

    # Accept the same relative paths compose.py does.
    import compose
    session = compose._resolve_session(a.session)          # noqa: SLF001
    backfill(session, every=a.every, max_bytes=rcfg.parse_size(a.max_size),
             ring=a.ring, timeout_s=a.timeout, seq_lo=a.seq_from, seq_hi=a.seq_to,
             dry_run=a.dry_run, progress_s=a.progress, sleep_ms=a.sleep_ms)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
