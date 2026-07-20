# Streaming POC (dev log)

!!! warning "Archived"
    This is a historical development log. It was the original end-to-end write-up of the live
    point-cloud proof of concept, written while the pipeline was still churning. The
    authoritative, current documentation now lives in the main pages — use those:

    - [Architecture](../architecture.md) — data path, pose model, client prediction
    - [Reconstruction Engine](../reconstruction_engine.md) — PRISM-VGGT internals
    - [Wire Protocol](../reference/wire_protocol.md) — Zenoh keys + byte layouts
    - [Robot](../setup/robot.md) / [Server](../setup/server.md) / [Client](../setup/client.md) setup
    - [Bring-up Runbook](../bringup.md) — the staged `make steps` sequence

    It is kept for provenance — to see how the design arrived where it is.

## What the POC proved

The POC established the whole VAT data path end to end: the robot captures the RICOH Theta X
panorama, decimates and streams frames to a GPU mapping server running PRISM-VGGT, which fuses
them into a live metric point cloud and streams changed blocks to a desktop viewer; in parallel
the robot publishes an authoritative fused pose that the client predicts between samples. That
contract — `robot camera → server mapping`, `robot pose → client`, `server pose correction →
robot` — is exactly what the system still runs on. It locked in the Zenoh key schema and wire
formats (now in the [Wire Protocol](../reference/wire_protocol.md)) so that individual
components could be swapped without breaking the others.

## What changed since this log was written

The original page contradicted itself in places as the code moved. For the record, the current
state (verified against the code) is:

- **Point-cloud streaming** uses block-diff sync by default (`STREAM_MODE=blocks`) — the client
  fetches a snapshot once, then only the blocks whose hash changed.
- **Mapping** runs in reset/hybrid mode (`PRISM_RESET_EACH_BATCH=1`); the old
  online-accumulate path produced thick, duplicated walls and is retired.
- **A navigation ESDF slice is published** (`COMPUTE_ESDF=1`) — groundwork for the planned
  autonomous navigation (see the [Roadmap](../roadmap.md)).
- **The viewer is VisPy** (`client/prism_viewer.py`, `make viewer`) — the earlier Rerun viewer
  froze on the live stream and Open3D was finicky.
- **The pose fuser is an ESKF** (`POSE_BACKEND=eskf`), not a placeholder; an earlier GTSAM
  backend was removed. Odometry is wheel-based (from `/lowstate`) with leg-FK height, because
  the Go2-W's `SportModeState` velocity/height fields are dead (see
  [Robot Data Sources](../robot_data_sources.md)).
- **Submap alignment defaults to Sim(3)** (`PRISM_ALIGN=sim3`) — more loop-robust than SL(4)
  (see the [Reconstruction Engine](../reconstruction_engine.md)).
- The `vat_salt.py` revisit-logging module and the earlier block/Draco helpers referenced in
  the original log were removed or superseded.

For anything operational, follow the current pages linked above rather than this log.
