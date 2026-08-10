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

The easy way — no paths to type: open the recorder console and use the **Replay** tab,
which lists every recording under `data/` with its duration and size, and prints a
clickable viewer link once the run is loaded.

```bash
make record-ui                    # → Replay tab → pick a recording → ▶ Open in Rerun
```

From a terminal:

```bash
make replay ARGS="--list"                                    # what is in data/
make replay                                                  # the newest recording
make replay ARGS="recordings/data/<session_id>"              # web viewer, headless-friendly
make replay ARGS="recordings/data/<session_id> --save run.rrd"
```

Then either open the printed URL, or copy the `.rrd` to your own machine and
`rerun run.rrd` — scrubbing is much nicer locally than over a remote web viewer.

> **The viewer URL must be reachable *from your browser*.** Rerun's web page fetches the
> data over gRPC from a second port, and that address is resolved by the browser — so on
> a headless server `localhost` would point at your own laptop and the page would load
> and then sit empty. Both tools therefore default `--viewer-host` to the router host
> from `vat.env` (a Tailscale address here), and **both** the viewer port (9090) and the
> gRPC port (9876) have to be reachable. `--viewer-host` overrides it.

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
