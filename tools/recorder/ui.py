#!/usr/bin/env python3
"""
VAT — recorder console: a browser UI for capturing a session
===========================================================
A small Gradio app around ``vat_record.py`` / ``backfill.py`` / ``compose.py`` so a
capture can be driven from a phone or a laptop browser while you are holding the robot's
remote, instead of from a terminal.

    make record-ui                      # → http://<server>:7860
    make record-ui ARGS="--port 8080 --host 0.0.0.0"

What it does
------------
* **Start / Stop** a recording. The recorder runs as a child process, exactly as the CLI
  does, and Stop sends **SIGINT** so it takes the normal clean-flush path — the UI has no
  privileged shortcut that the command line doesn't.
* **Live progress**, polled from ``<session>/live.json`` that the recorder writes each
  tick: per-stream sample counts, rates, errors and dropped samples, bytes on disk,
  resident memory, the clock offset, and whether the clock baseline is established.
* **Fetch full-res** for a finished recording, from the robot's rolling archive
  (``backfill.py``). This is the recommended way to get 4K panoramas: nothing is fetched
  during the walk, so the capture stays untouched, and afterwards you choose how much to
  pull. Tick *fetch on stop* to have it start automatically.
* **Reset the PRISM map** — see the warning below.
* **Archive**: every recording under the output root, with its size, duration, status and
  stream counts; download any of them as a zip.

.. warning::
   The reset button is the **one** control here that publishes on the bus. It puts an
   empty payload on ``{server}/cmd/reset``, which is what the viewer's reset does and what
   the mapping server listens for. It is an explicit operator action, deliberately kept
   out of the recorder: ``vat_record.py`` itself still declares no publisher and cannot
   perturb a session. Resetting mid-recording is legitimate (the recording will show the
   map going empty and rebuilding, and the manifest's version pins will follow) — just
   know that you did it.

Everything the UI does is available from the CLI; nothing here is a separate code path.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import zipfile
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import gradio as gr                # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("vat-ui")

PY = sys.executable
RECORD = os.path.join(_HERE, "vat_record.py")
TRAJECTORY_FAMILIES = ("smooth", "stop-and-go", "loop", "other")
STREAM_CHOICES = ("panorama_transmit", "periscope", "pointcloud", "poses",
                  "esdf", "status", "trajectory")


# ═════════════════════════════════════════════════════════════════════════════
# State
# ═════════════════════════════════════════════════════════════════════════════


class Console:
    """Owns the recorder child process and the background backfill thread."""

    def __init__(self, out_root: str):
        self.out_root = os.path.abspath(out_root)
        os.makedirs(self.out_root, exist_ok=True)
        self.proc: Optional[subprocess.Popen] = None
        self.session_dir: Optional[str] = None
        self.log_lines: List[str] = []
        self._lock = threading.Lock()
        self.backfill_state = {"running": False, "message": "idle", "done": 0,
                               "total": 0, "bytes": 0}
        self.fetch_on_stop = False
        self.fetch_every = 1

    # ── recording ────────────────────────────────────────────────────────────
    @property
    def recording(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, meta: dict, streams, duration: str, max_size: str,
              keyframe_s: float) -> str:
        if self.recording:
            return "⚠️ already recording — Stop first."
        session_id = meta.get("session_id") or ""
        cmd = [PY, RECORD, "--out", self.out_root, "--progress", "2",
               "--pointcloud-keyframe-s", str(keyframe_s)]
        if session_id:
            cmd += ["--session-id", session_id]
        for flag, key in (("--scene", "scene"), ("--trajectory-family", "family"),
                          ("--camera-height-source", "height_source"),
                          ("--mount-geometry", "mount"), ("--operator", "operator")):
            v = (meta.get(key) or "").strip()
            if v:
                cmd += [flag, v]
        if meta.get("pass_index") not in (None, ""):
            cmd += ["--pass", str(int(meta["pass_index"]))]
        if meta.get("camera_height"):
            cmd += ["--camera-height", str(float(meta["camera_height"]))]
        if meta.get("flat_floor") is True:
            cmd += ["--clear-flat-floor"]
        elif meta.get("flat_floor") is False:
            cmd += ["--no-clear-flat-floor"]
        for line in (meta.get("notes") or "").splitlines():
            if line.strip():
                cmd += ["--note", line.strip()]
        if duration.strip():
            cmd += ["--duration", duration.strip()]
        if max_size.strip():
            cmd += ["--max-size", max_size.strip()]
        # Explicitly naming streams selects exactly those, so only pass them when the
        # operator narrowed the set — otherwise let the recorder apply its own default.
        chosen = list(streams or [])
        if chosen and set(chosen) != set(STREAM_CHOICES):
            for s in chosen:
                cmd += [f"--{s.replace('_', '-')}"]

        before = set(os.listdir(self.out_root))
        with self._lock:
            self.log_lines = [f"$ {' '.join(cmd)}", ""]
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, cwd=rcfg.REPO_ROOT)
        except Exception as e:                                  # noqa: BLE001
            return f"❌ could not start: {e}"
        threading.Thread(target=self._drain, daemon=True).start()

        # The recorder invents the session id from the timestamp + metadata, so find the
        # directory it created rather than trying to predict it.
        self.session_dir = None
        for _ in range(60):
            new = set(os.listdir(self.out_root)) - before
            if new:
                self.session_dir = os.path.join(self.out_root, sorted(new)[-1])
                break
            if self.proc.poll() is not None:
                return ("❌ the recorder exited immediately — see the log below "
                        "(a missing clock-baseline stream and an unreachable router "
                        "are the usual causes).")
            time.sleep(0.1)
        return (f"● recording → {os.path.basename(self.session_dir)}"
                if self.session_dir else
                "● started, but no session directory appeared yet")

    def _drain(self) -> None:
        p = self.proc
        if p is None or p.stdout is None:
            return
        for line in p.stdout:
            with self._lock:
                self.log_lines.append(line.rstrip())
                del self.log_lines[:-400]
        code = p.wait()
        with self._lock:
            self.log_lines.append(f"— recorder exited with code {code} —")
        if self.fetch_on_stop and self.session_dir:
            self.run_backfill(self.session_dir, every=self.fetch_every)

    def stop(self) -> str:
        if not self.recording:
            return "not recording."
        try:
            # SIGINT, not kill: this is the recorder's documented clean-stop path —
            # flush every index, write a final map keyframe, remux, write MANIFEST.json.
            self.proc.send_signal(signal.SIGINT)
        except Exception as e:                                  # noqa: BLE001
            return f"could not signal the recorder: {e}"
        for _ in range(200):                                     # up to ~20 s to flush
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        if self.proc.poll() is None:
            self.proc.terminate()
            return "⚠️ recorder did not stop within 20 s — terminated (may be partial)."
        return f"■ stopped — {os.path.basename(self.session_dir or '?')}"

    def log_text(self) -> str:
        with self._lock:
            return "\n".join(self.log_lines[-200:])

    # ── full-res backfill ────────────────────────────────────────────────────
    def run_backfill(self, session: str, every: int = 1, max_size: str = "0",
                     sleep_ms: float = 0.0) -> str:
        if self.backfill_state["running"]:
            return "⚠️ a fetch is already running."
        if self.recording and os.path.abspath(session) != (self.session_dir or ""):
            return ("⚠️ a recording is in progress — fetching full-res now would compete "
                    "with it on the link. Stop first.")

        def work():
            import backfill as bf
            self.backfill_state.update(running=True, message="starting…", done=0,
                                       total=0, bytes=0)
            try:
                summary = bf.backfill(
                    session, every=int(every), max_bytes=rcfg.parse_size(max_size),
                    sleep_ms=float(sleep_ms), progress_s=2.0,
                    status_cb=lambda d: self.backfill_state.update(
                        running=True, **{k: d[k] for k in
                                         ("done", "total", "bytes", "message")}))
                self.backfill_state.update(
                    running=False,
                    message=(f"✔ fetched {summary['fetched']} frame(s), "
                             f"{rcfg.human_size(summary['bytes'])}"
                             + (f", {summary['missing']} missing from the robot's archive"
                                if summary.get("missing") else "")))
            except SystemExit as e:
                self.backfill_state.update(running=False, message=f"❌ {e}")
            except Exception as e:                              # noqa: BLE001
                log.exception("backfill failed")
                self.backfill_state.update(running=False,
                                           message=f"❌ {type(e).__name__}: {e}")

        threading.Thread(target=work, daemon=True).start()
        return f"fetching full-res for {os.path.basename(session)}…"

    # ── PRISM map reset (the one publishing control) ──────────────────────────
    def reset_map(self) -> str:
        try:
            import zenoh
            conf = zenoh.Config()
            conf.insert_json5("connect/endpoints", f'["{rcfg.ZENOH_ROUTER}"]')
            conf.insert_json5("mode", '"client"')
            z = zenoh.open(conf)
            try:
                z.put(rcfg.RESET_KEY, b"")
            finally:
                z.close()
        except Exception as e:                                  # noqa: BLE001
            return f"❌ reset failed: {e}"
        msg = f"↺ reset published on '{rcfg.RESET_KEY}'"
        if self.recording:
            msg += " — mid-recording: the map will go empty and rebuild in the capture."
        return msg


# ═════════════════════════════════════════════════════════════════════════════
# Views
# ═════════════════════════════════════════════════════════════════════════════


def _read_live(session: Optional[str]) -> Optional[dict]:
    if not session:
        return None
    try:
        with open(os.path.join(session, "live.json"), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def live_view(con: Console):
    """(headline markdown, per-stream rows) for the live panel."""
    live = _read_live(con.session_dir)
    if live is None:
        state = "recording (waiting for the first tick…)" if con.recording else "idle"
        return f"### {state}", []
    dot = "🔴" if live["state"] == "recording" else "⚪"
    cap = f" / {live['duration_cap_s']:g}s" if live.get("duration_cap_s") else ""
    head = [
        f"### {dot} {live['state']} — `{live['session_id']}`",
        f"**{live['elapsed_s']:.0f}s{cap}** elapsed · "
        f"**{rcfg.human_size(live['bytes_session'])}** on disk"
        + (f" (+{rcfg.human_size(live['bytes_fullres'])} full-res)"
           if live.get("bytes_fullres") else "")
        + f" · RSS **{rcfg.human_size(live['rss_bytes'] or 0)}**",
    ]
    off = live.get("clock_offset_s")
    head.append(
        ("✅ clock baseline established" if live.get("clock_baseline_ok")
         else "⚠️ **no clock baseline yet** — map timestamps cannot be derived")
        + (f" · offset {off * 1e3:.1f} ms" if off is not None else "")
        + f" · {live.get('version_pins', 0)} map-version pin(s)")
    if live.get("stop_reason"):
        head.append(f"_{live['stop_reason']}_")
    bad = [f"`{k}` {v['errors']} error(s): {v['last_error']}"
           for k, v in live["streams"].items() if v.get("errors")]
    if bad:
        head.append("⚠️ " + " · ".join(bad))
    rows = []
    for name, v in live["streams"].items():
        rows.append([name, v["samples"],
                     f"{v['mean_hz']:.2f}" if v.get("mean_hz") else "—",
                     rcfg.human_size(v["bytes"]),
                     v.get("seq_samples_missing") or 0,
                     v.get("skipped") or 0, v.get("errors") or 0])
    return "\n\n".join(head), rows


def _session_summary(path: str) -> Optional[list]:
    man = None
    for name in ("MANIFEST.json", "live.json"):
        try:
            with open(os.path.join(path, name), encoding="utf-8") as f:
                man = json.load(f)
            break
        except (OSError, ValueError):
            continue
    if man is None:
        return None
    total = 0
    for root, _dirs, files in os.walk(path):
        for fn in files:
            try:
                total += os.path.getsize(os.path.join(root, fn))
            except OSError:
                pass
    streams = man.get("streams") or {}
    n = sum((s.get("samples") or 0) for s in streams.values())
    fullres = (streams.get("panorama_fullres") or {}).get("samples") or 0
    if not fullres:                       # a backfill adds frames after the manifest
        d = os.path.join(path, "panorama_fullres", "frames")
        fullres = len(os.listdir(d)) if os.path.isdir(d) else 0
    return [os.path.basename(path), man.get("status", "?"),
            f"{man.get('duration_s') or man.get('elapsed_s') or 0:.0f}",
            rcfg.human_size(total), n, fullres]


def archive_rows(out_root: str) -> list:
    if not os.path.isdir(out_root):
        return []
    rows = []
    for name in sorted(os.listdir(out_root), reverse=True):
        p = os.path.join(out_root, name)
        if os.path.isdir(p):
            r = _session_summary(p)
            if r:
                rows.append(r)
    return rows


def make_zip(out_root: str, session_id: str, include_fullres: bool = True):
    """Zip a session for download. Returns the archive path, or None."""
    if not session_id:
        return None, "pick a recording first."
    src = os.path.join(out_root, session_id)
    if not os.path.isdir(src):
        return None, f"no such recording: {session_id}"
    zips = os.path.join(out_root, "_zips")
    os.makedirs(zips, exist_ok=True)
    tag = "" if include_fullres else "-nofullres"
    dst = os.path.join(zips, f"{session_id}{tag}.zip")
    n = 0
    # ZIP_STORED, not DEFLATE: the payloads are already-compressed JPEG/H.264/Draco/npz,
    # so deflating them again costs minutes of CPU for ~1 % — and this runs on the box
    # that may still be mapping.
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_STORED, allowZip64=True) as zf:
        for root, _dirs, files in os.walk(src):
            if not include_fullres and "panorama_fullres" in os.path.relpath(root, src).split(os.sep):
                continue
            for fn in files:
                full = os.path.join(root, fn)
                zf.write(full, os.path.join(session_id, os.path.relpath(full, src)))
                n += 1
    return dst, (f"✔ {n} file(s), {rcfg.human_size(os.path.getsize(dst))}"
                 + ("" if include_fullres else " (full-res excluded)"))


# ═════════════════════════════════════════════════════════════════════════════
# App
# ═════════════════════════════════════════════════════════════════════════════


def build_app(con: Console) -> gr.Blocks:
    with gr.Blocks(title="VAT recorder") as app:
        gr.Markdown(
            f"# VAT recorder console\n"
            f"`{rcfg.ZENOH_ROUTER}` · robot `{rcfg.ROBOT_NAME}` · server "
            f"`{rcfg.SERVER_PREFIX}` · output `{con.out_root}`\n\n"
            f"The recorder is a **pure subscriber** — it cannot disturb the live "
            f"session. The only control here that publishes is *Reset PRISM map*.")

        with gr.Tab("Capture"):
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("#### Session")
                    scene = gr.Textbox(label="Scene", placeholder="lab")
                    with gr.Row():
                        family = gr.Dropdown(TRAJECTORY_FAMILIES, label="Trajectory",
                                             value="smooth")
                        pass_ix = gr.Number(label="Pass", value=1, precision=0)
                    cam_h = gr.Number(
                        label="Camera height (m) — the metric-scale anchor",
                        value=1.15)
                    h_src = gr.Textbox(label="How it was measured",
                                       placeholder="tape, floor→lens centre, standing")
                    mount = gr.Textbox(label="Mount geometry",
                                       placeholder="rear selfie-stick, rigid")
                    operator = gr.Textbox(label="Operator")
                    flat = gr.Checkbox(label="Started over clear, flat, visible floor",
                                       value=True)
                    notes = gr.Textbox(label="Notes (one per line)", lines=2)
                    with gr.Accordion("Advanced", open=False):
                        session_id = gr.Textbox(
                            label="Session id (blank = auto from timestamp + metadata)")
                        streams = gr.CheckboxGroup(
                            STREAM_CHOICES, value=list(STREAM_CHOICES),
                            label="Streams (full-res is fetched afterwards, not live)")
                        duration = gr.Textbox(label="Auto-stop after", placeholder="5m")
                        max_size = gr.Textbox(label="Size cap", placeholder="4GB")
                        keyframe_s = gr.Number(label="Map keyframe every (s)", value=10)
                    with gr.Row():
                        btn_start = gr.Button("● Start recording", variant="primary")
                        btn_stop = gr.Button("■ Stop", variant="stop")
                    status = gr.Markdown("idle")
                    with gr.Row():
                        fetch_on_stop = gr.Checkbox(
                            label="Fetch full-res from the robot when I stop",
                            value=False)
                        fetch_every = gr.Number(label="every Nth frame", value=1,
                                                precision=0)
                    btn_reset = gr.Button("↺ Reset PRISM map (publishes!)")

                with gr.Column(scale=2):
                    headline = gr.Markdown("### idle")
                    table = gr.Dataframe(
                        headers=["stream", "samples", "Hz", "bytes", "missing",
                                 "skipped", "errors"],
                        datatype=["str", "number", "str", "str", "number", "number",
                                  "number"],
                        interactive=False, label="Live streams", wrap=True)
                    logbox = gr.Textbox(label="Recorder log", lines=14,
                                        max_lines=14, autoscroll=True)

        with gr.Tab("Full-res & archive"):
            gr.Markdown(
                "#### Fetch full-resolution panoramas\n"
                "The robot archives a full-res twin of every transmitted frame locally "
                "(~10 GB rolling ≈ 6 h). Nothing is fetched during the walk, so the "
                "capture stays untouched — pull the twins afterwards, as much as you "
                "want. The only deadline is the archive's rolling window.")
            with gr.Row():
                pick = gr.Dropdown(label="Recording", choices=[], interactive=True)
                bf_every = gr.Number(label="every Nth frame", value=1, precision=0)
                bf_cap = gr.Textbox(label="Size cap", value="0", placeholder="20GB")
                bf_sleep = gr.Number(label="pause between fetches (ms)", value=0)
            with gr.Row():
                btn_dry = gr.Button("Estimate only")
                btn_fetch = gr.Button("⬇ Fetch full-res", variant="primary")
            bf_status = gr.Markdown("idle")

            gr.Markdown("#### Archive")
            arch = gr.Dataframe(
                headers=["session", "status", "duration s", "size", "samples",
                         "full-res frames"],
                datatype=["str", "str", "str", "str", "number", "number"],
                interactive=False, wrap=True)
            with gr.Row():
                btn_refresh = gr.Button("↻ Refresh")
                inc_full = gr.Checkbox(label="include full-res in the zip", value=True)
                btn_zip = gr.Button("🗜 Build zip")
            zip_status = gr.Markdown("")
            zip_file = gr.File(label="Download", interactive=False)

        # ── wiring ───────────────────────────────────────────────────────────
        def on_start(scene, family, pass_ix, cam_h, h_src, mount, operator, flat,
                     notes, session_id, streams, duration, max_size, keyframe_s,
                     fos, fev):
            con.fetch_on_stop = bool(fos)
            con.fetch_every = max(1, int(fev or 1))
            msg = con.start(
                {"scene": scene, "family": family, "pass_index": pass_ix,
                 "camera_height": cam_h, "height_source": h_src, "mount": mount,
                 "operator": operator, "flat_floor": bool(flat), "notes": notes,
                 "session_id": session_id},
                streams, duration or "", max_size or "", keyframe_s or 10)
            return msg, gr.update(choices=[r[0] for r in archive_rows(con.out_root)])

        btn_start.click(
            on_start,
            [scene, family, pass_ix, cam_h, h_src, mount, operator, flat, notes,
             session_id, streams, duration, max_size, keyframe_s, fetch_on_stop,
             fetch_every],
            [status, pick])
        btn_stop.click(lambda: con.stop(), None, status).then(
            lambda: (gr.update(choices=[r[0] for r in archive_rows(con.out_root)]),
                     archive_rows(con.out_root)), None, [pick, arch])
        btn_reset.click(lambda: con.reset_map(), None, status)

        def on_dry(sid, every):
            if not sid:
                return "pick a recording first."
            import backfill as bf
            try:
                s = bf.backfill(os.path.join(con.out_root, sid),
                                every=max(1, int(every or 1)), dry_run=True)
            except SystemExit as e:
                return f"❌ {e}"
            return (f"would fetch **{s['planned']}** frame(s) "
                    f"({s['already_present']} already present), roughly "
                    f"**{rcfg.human_size(s['estimate_bytes'])}**")

        btn_dry.click(on_dry, [pick, bf_every], bf_status)
        btn_fetch.click(
            lambda sid, e, c, sl: (con.run_backfill(os.path.join(con.out_root, sid),
                                                    int(e or 1), c or "0", sl or 0)
                                   if sid else "pick a recording first."),
            [pick, bf_every, bf_cap, bf_sleep], bf_status)

        btn_refresh.click(
            lambda: (archive_rows(con.out_root),
                     gr.update(choices=[r[0] for r in archive_rows(con.out_root)])),
            None, [arch, pick])

        def on_zip(sid, include):
            path, msg = make_zip(con.out_root, sid, bool(include))
            return (path, msg) if path else (None, msg)

        btn_zip.click(on_zip, [pick, inc_full], [zip_file, zip_status])

        def tick():
            head, rows = live_view(con)
            bs = con.backfill_state
            bmsg = bs["message"]
            if bs["running"] and bs.get("total"):
                bmsg = (f"⏳ {bs['done']}/{bs['total']} · "
                        f"{rcfg.human_size(bs['bytes'])}")
            return head, rows, con.log_text(), bmsg

        gr.Timer(2.0).tick(tick, None, [headline, table, logbox, bf_status])
        app.load(lambda: (archive_rows(con.out_root),
                          gr.update(choices=[r[0] for r in archive_rows(con.out_root)])),
                 None, [arch, pick])
    return app


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Browser console for vat-record.")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=7860)
    p.add_argument("--out", default=None, help="output root (default <repo>/recordings)")
    p.add_argument("--share", action="store_true", help="public gradio tunnel")
    a = p.parse_args(argv)
    con = Console(a.out or rcfg.DEFAULT_OUT_ROOT)
    log.info(f"[ui] output root {con.out_root}")
    log.info(f"[ui] zenoh {rcfg.ZENOH_ROUTER}  robot={rcfg.ROBOT_NAME}")
    if shutil.which("ffmpeg") is None:
        log.info("[ui] ffmpeg not on PATH — periscope mp4 remux will be skipped "
                 "(the elementary stream + timestamps CSV are still written)")
    build_app(con).launch(server_name=a.host, server_port=a.port, share=a.share,
                          theme=gr.themes.Soft(), show_api=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
