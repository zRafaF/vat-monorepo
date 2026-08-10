# `recordings/` — captured sessions, and the tool to play them back

```
recordings/
  replay.py         play a recorded session back in Rerun (this project)
  pyproject.toml    isolated uv env: `cd recordings && uv sync`
  data/             the sessions themselves — NOT in git (large, per-run)
    <session_id>/   written by tools/recorder; see docs/recording.md for the layout
```

`data/` is the recorder's default output root, so `make record` and `make record-ui` put
sessions there without any flag. Everything in `data/` is gitignored; this tool is not.

## Play a run back

**Run this where the screen is** — your laptop, not the mapping server. Rerun is a desktop
app, and the default mode is the one Rerun itself recommends
([operating modes](https://rerun.io/docs/reference/sdk/operating-modes)): `rr.spawn()`
launches the native viewer — the `rerun` executable that ships inside `rerun-sdk`, so
`uv run` already has it on `PATH` — and streams the session into it. No browser, no ports.

```bash
make replay                                       # folder picker, then the app opens
make replay ARGS="--list"                         # what is in data/
make replay ARGS="--newest"                       # skip the picker
make replay ARGS="recordings/data/<session_id>"
```

With no argument you get a native folder picker (Tk) rooted at `data/`; with no GUI it
falls back to a numbered prompt, and failing that the newest recording. The window
outlives the command — the data lives in the viewer, not in the script — and if a viewer
is already open on the port, the session simply appears in it.

The recording is on the headless server instead? Two options, both better than a browser:

```bash
# on the server
make replay ARGS="<session> --save /tmp/run.rrd"      # then copy it over…
# on your laptop
rerun /tmp/run.rrd                                    # …and scrub it locally
```

or point the server at a viewer you already have open (works across the tailnet, and
`rerun` must be listening — start it, then):

```bash
make replay ARGS="<session> --connect rerun+http://<your-laptop>:9876/proxy"
```

`--serve` still exists (browser viewer, `--viewer-host` for the address the browser should
use) but needs **two** ports reachable and scrubs poorly; prefer `--save`.

The recorder console's **Replay** tab wraps the same two paths: *Open in Rerun* when the
console is running on a machine with a screen, *Build .rrd* + download when it is not.

> First run of the viewer prints Rerun's analytics notice; `rerun analytics disable` turns
> it off for good.

Everything lands on **one timeline** (`session`, the robot capture clock in nanoseconds),
so the map, the trajectory, the panorama, the periscope and the ESDF all line up. Map
artefacts are placed at `capture_ts_ns` — the true capture time of that map version — not
at arrival, so the map sits where the robot actually was. See
[`docs/recording.md`](../docs/recording.md#the-common-clock).

| entity | what it is |
|---|---|
| `world/map` | the point cloud (materialised keyframes, or `--map replay` to rebuild per submap from the recorded Draco pushes) |
| `world/robot` | the fused pose, plus the path walked so far |
| `world/correction` | cloud pose corrections — sparse, gated, each one re-anchors the robot |
| `world/trajectory` | the camera trail the cloud streamed |
| `world/esdf` | ESDF slices, coloured as published |
| `panorama` | the 360° frame, logged as an encoded JPEG so Rerun does the decoding |
| `periscope` | the operator's video slice, if PyAV can decode it |
| `metrics/*` | map version, cube count, uplink kB/s, observed latency, camera height |

Useful knobs (all exposed under *Advanced* in the console's Replay tab too):
`--map replay` to watch the map build incrementally, `--panorama fullres`
after a backfill, `--skip periscope` when one stream is huge, `--pose-every N` (30 Hz is
more than a viewer needs), `--max-points` to cap cloud size, and `--with <other-session>`
to merge a paired robot + cloud capture.

## A recording is self-describing

`meta.json` (identity, capture metadata, config hash, clock epoch, Zenoh keys) and
`MANIFEST.json` (per-stream health, derived cross-checks, version pins) sit next to the
data, and `README.txt` inside each session explains its own layout. `tools/recorder/`
holds the capture and composition tools.
