# Client setup

The client is your **laptop**. It runs the live 3D viewer and a set of
diagnostic tools. It does no heavy computation — it receives the point cloud and
pose over the VPN and draws them. Any modern laptop works; a GPU helps the
viewer but isn't required.

The viewer is [**VisPy**](https://vispy.org/) (native OpenGL point scatter) —
**not** Rerun and **not** Open3D. It renders the PRISM point cloud, the robot
avatar at its predicted pose, the legs, and the camera trajectory, with a
latency HUD.

!!! tip "Quick start (after the one-time install below)"
    From the repo root on the laptop:
    ```bash
    make viewer         # the full VisPy 3D viewer (Stage 4)
    ```
    The router and mapping server must already be running on the SERVER, and the
    robot container on the ROBOT. See the [Bring-up Runbook](../bringup.md) for
    the staged order.

---

## 1. Prerequisites

Install the base tools first if you haven't — see the
[setup overview](index.md#linux-from-zero-skip-if-you-know-this): `git`, `make`,
and `uv`. Then clone the repo
([with submodules](index.md#get-the-code-clone-with-submodules)) and `cd` into
it.

The viewer opens an OpenGL window. On Linux you need a graphical desktop (not a
headless server). VisPy uses **glfw** as its windowing backend, which `uv`
installs for you; no extra system packages are normally required.

!!! note "Windows / macOS laptops"
    The client is the one component that also runs on Windows and macOS, since
    it's pure Python + OpenGL. Install `uv`
    ([astral.sh/uv](https://docs.astral.sh/uv/getting-started/installation/)) and
    `make` for your OS, then use the same targets below. Everything runs inside
    the isolated `client/.venv` that `uv` creates.

---

## 2. Point the client at the router

In `vat.env` at the repo root, set `ROUTER_IP` to the VPN address of the machine
running the Zenoh router (the SERVER):

```bash
ROUTER_IP=100.76.214.80     # example — use YOUR router's Tailscale/VPN IP
```

This must match what the robot uses. See [VPN setup](development/vpn.md) for how
to find the address.

---

## 3. Create the client environment

```bash
make sync-client
```

This creates `client/.venv` (an isolated Python environment) with the viewer and
tool dependencies: VisPy + glfw (rendering), eclipse-zenoh (transport), DracoPy
(point-cloud decompression), OpenCV + PyAV (image/video decode), and the avatar
mesh loaders. You only need to run this once; the `make viewer` and test targets
run it automatically on first use.

!!! note "First run is slow"
    `uv` downloads and installs everything the first time (and may fetch a
    matching Python interpreter). Later runs are instant.

---

## 4. Run the viewer

```bash
make viewer
```

This runs `client/prism_viewer.py --snapshot`. A window opens showing the point
cloud, the robot block at its predicted pose (green when freshly VGGT-corrected,
amber when dead-reckoning between corrections), the four legs, the camera
trajectory, and a latency HUD in the top-left corner.

!!! note "Nothing appears?"
    The viewer only draws what it receives. If the window is empty, the router
    or mapping server probably isn't up, or `ROUTER_IP` is wrong. Run
    `make test_link` first (below) to confirm the transport is alive.

### Viewer controls

The controls are printed to the log on start-up. The main ones:

| Key(s) | Action |
|---|---|
| `←` / `→` | Orbit the camera |
| `↑` / `↓` | Tilt the camera |
| `W`/`A`/`S`/`D`/`Q`/`E` | Pan the view |
| scroll / `F` | Zoom / zoom-to-fit |
| `T` | Follow the robot (third-person) |
| `1` | Re-fetch the full cloud from the server |
| `R` | Reset the map |
| `N` / `M` | Point size − / + |
| `C` | Toggle the ceiling clip on/off |
| `[` / `]` | Lower / raise the ceiling-clip height |
| `,` / `.` / `/` | Rotate the cloud↔robot yaw ∓5° / reset to 0° |
| `U` | Toggle the robot mesh vs. wireframe skeleton |

Mouse: drag to orbit, `Shift`+drag to pan, scroll to zoom. A separate Telemetry
window also opens alongside the 3D view.

!!! tip "Remote periscope & depth panel"
    If the robot's periscope and RealSense (D435i) features are enabled, the
    viewer also has aim keys for the HD video slice (`j`/`l` yaw, `i`/`k` pitch,
    `o`/`p` zoom, `g` recenter, `v` toggle panel) and RGBD keys (`x` cycles
    depth/color/off). See [Remote Periscope](../periscope.md).

---

## 5. Diagnostic & bring-up tools

These run in the same `client/.venv` and let you verify each stage of the
pipeline independently, so you can tell *which* machine is at fault when
something doesn't show up. Run them from the repo root. Each prints a reminder of
what must already be running.

| Command | Stage | What it checks |
|---|---|---|
| `make test_link` | 0 | Transport is alive: the router is reachable and the robot bridge is publishing (shows the bridge state + message rates in Hz). Run this first. |
| `make test_frames_server` | 1 | The decimated 360° frames the mapping server actually ingests, in an OpenCV window (plus the stamped camera height). |
| `make test_robot_state` | 2 | The robot's body frame and the four feet, moving live. |
| `make test_poses` | 3 | The camera trajectory, the VGGT pose corrections, and the fused robot pose. Needs the mapping server running. |
| `make teleop` | — | Keyboard drive (WASD) with a deadman + e-stop. Keep the physical Unitree remote in hand. |
| `make fetch_pcd` | — | Fetch one live PRISM cloud, print its stats, and save it (`.npz`/`.ply`) — proves whether a bad cloud is a streaming/codec issue or a render issue. `ARGS="--both"` compares the server vs. client copy. |
| `make record-frames` | — | Save incoming 360° frames to a folder you pick (offline analysis). |
| `make fetch_frame SEQ=1234` | — | Pull one full-resolution archived frame from the robot by its sequence number. |

!!! note "`make teleop` drives a real robot"
    Teleop sends velocity commands to the physical Go2-W. Start with the
    conservative default speed clamps in `vat.env`, keep the physical remote in
    your hand as the hard e-stop, and make sure you have clear space. The bridge
    stops the robot automatically if the command stream pauses (deadman).

---

## 6. Where to go next

Follow the **[Bring-up Runbook](../bringup.md)** for the exact order to start
everything and the check to run at each stage (transport → frames → body →
poses → full viewer). For what the pose numbers mean and how the client
dead-reckons between samples, see the
[Architecture](../architecture.md) page.
