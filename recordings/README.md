# `recordings/` — where a capture lands

This is the recorder's default output root. `make record` and `make record-ui` write
`recordings/data/<session_id>/` here, and `data/` is gitignored (a session is large, and
`make backfill` makes it larger).

That is all this directory is. **Replay, composition, video and figures are not here.**

## Where they went

They live in the report repo, `uofa-2026-report/realworld/`:

| you want to | run |
|---|---|
| capture a session | `make record` / `make record-ui` (this repo) |
| pull the full-res panoramas afterwards | `make backfill` (this repo) |
| look at a capture in Rerun | `make replay` in `uofa-2026-report/realworld/` |
| align the streams, export frames, build video | `make info` / `make export` there |
| build a paper figure or a metric from a capture | that project, next to the report that quotes it |

The split is deliberate: this repo is the *system* — robot, mapping server, transport,
recorder — and it keeps moving. A finished capture is *evidence*, and evidence belongs with
the document that cites it, together with the code that turns it into a figure. That
project vendors frozen copies of the readers (`vat_protocol`, `vat_blockmap`, the periscope
decoder) so a figure stays reproducible without this checkout at a matching commit; see
`uofa-2026-report/realworld/vendor/README.md`.

The recording **format** is still documented here, in
[`docs/recording.md`](../docs/recording.md) — including the clock contract, the on-disk
layout and the map-timestamp derivation — because this is the code that writes it.

## Moving a capture to where it gets used

A session directory is self-contained; copy or move it into
`uofa-2026-report/realworld/data/`. From the recorder console (*Full-res & archive* tab)
*Build zip* packages one for transfer — note the zip contains a top-level folder, so
unzipping it into `data/` leaves the session one level deeper than expected. The tools
there look through that extra level, but flattening it is tidier.
