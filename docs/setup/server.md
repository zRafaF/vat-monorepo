# Server setup

The server is a **GPU box** (an NVIDIA GPU with **CUDA 12.8** is required for the
mapping server). It runs two microservices, each with its **own** isolated
Python environment:

1. **`vat-router`** — a pure-Python Zenoh router (`server/router/`). Everything
   (robot, mapping server, client) connects to it. It is the hub.
2. **`vat-mapping`** — the PRISM-VGGT mapping server (`server/mapping/`), with
   heavy CUDA/torch dependencies. It turns the robot's camera frames into the 3D
   point cloud and pose corrections.

They are kept separate on purpose so the router's single light dependency never
clashes with the mapper's CUDA stack.

There are two ways to run the server: a **containerized deployment**
(recommended — everything in one Docker container) or a **manual / bare-metal**
install (run the two services directly on the host). Pick one.

!!! note "Prerequisites"
    - **Base tools + repo** — `git`, `make`, `uv`, and the repo cloned
      [with submodules](index.md#get-the-code-clone-with-submodules). See the
      [setup overview](index.md#linux-from-zero-skip-if-you-know-this).
    - **NVIDIA GPU + CUDA 12.8** — the mapping server needs it. The router does
      not.
    - **For the containerized deploy** — Docker plus the **NVIDIA Container
      Toolkit** (lets Docker containers use the GPU) and `/dev/net/tun` (standard
      on Linux; used by the in-container VPN).

---

## Containerized deployment (GPU workstation — recommended)

On the lab GPU workstation both microservices run inside **one self-contained
Docker container** defined in [`server/deploy/`](https://github.com/zRafaF/vat-monorepo/tree/main/server/deploy)
(see its `README.md`). You copy **only** `docker-compose.yml` (+ a `.env` file)
onto the workstation — the container then clones the repo, installs everything,
joins Tailscale (the VPN) as its **own node**, and starts the router. Nothing
else lands on the host — not even the repo or Python.

("Docker Compose" runs a container from a description file; the `.env` file holds
the secret bits, like your Tailscale auth key.)

```bash
# on the workstation, in an empty dir holding docker-compose.yml + .env
docker compose up -d
docker compose logs -f          # first run is slow: clone + uv sync + weights
```

`docker compose up -d` starts the container in the background;
`docker compose logs -f` follows its output (`Ctrl`+`C` stops watching, container
keeps running). The first boot is slow because it downloads everything and the
PanoVGGT model weights.

What it does:

- **Tailscale runs *inside* the container** (kernel mode, own state, own node),
  so it never touches a Tailscale instance already on the host. The container
  shows up in the admin console as `vat-server` (set via `TS_HOSTNAME`).
- **The Zenoh router auto-starts** on `:7447` over the container's Tailscale
  interface. Robot and client dial this node — set `ROUTER_IP=vat-server` (or its
  `100.x` Tailscale IP) in the **robot's** and **client's** `vat.env`.
- **The GPU mapping server is manual.** SSH in and run it from the repo; inside
  the container the router is local, so point the mapper at localhost:

```bash
tailscale ssh root@vat-server            # or: ssh root@<host> -p 2222
cd /root/vat-monorepo
make mapping ROUTER_IP=127.0.0.1
```

("SSH" is a way to open a remote terminal on another machine. `tailscale ssh`
does it over the VPN.)

!!! note "The `.env` file (Tailscale auth key)"
    `docker compose` reads a `.env` file next to `docker-compose.yml`. You must
    set `TS_AUTHKEY` there to a **reusable, non-ephemeral** Tailscale auth key so
    the container can join your tailnet as a stable node. See the
    [`server/deploy/README.md`](https://github.com/zRafaF/vat-monorepo/tree/main/server/deploy)
    for the full list of knobs (`GIT_REF`, `TS_HOSTNAME`, `ROOT_PASSWORD`, …).

!!! tip "Applying updates / clearing the container"
    The bootstrap **hard-refreshes the clone to the latest `GIT_REF` on every
    boot**, so after you push a fix, just recreate the container:

    ```bash
    docker compose up -d --force-recreate
    ```

    This re-runs the bootstrap (`git fetch origin && git reset --hard FETCH_HEAD`)
    and picks up pushed changes — no host-side file deletion needed. The cloned
    repo and venvs live in the root-owned `./container_workspace/` volume, so if
    you ever must wipe it, do it from a throwaway root container:

    ```bash
    docker compose down
    docker run --rm -v "$PWD/container_workspace:/w" \
      nvidia/cuda:12.8.0-cudnn-devel-ubuntu24.04 rm -rf /w/vat-monorepo
    docker compose up -d
    ```

> Requires the NVIDIA Container Toolkit on the host (GPU passthrough) and
> `/dev/net/tun` + `NET_ADMIN` (for in-container Tailscale). `nvblox` installs
> from the **prebuilt** wheel (the default), validated on this workstation. The
> base image is `nvidia/cuda:12.8.0-cudnn-devel-ubuntu24.04`.

The sections below describe the **manual / bare-metal** alternative (router and
mapper run directly on the host), useful for a non-containerized server.

---

## 1. Zenoh router microservice

We run the router from **pure Python** (a `router`-mode Zenoh session) rather
than the `zenohd` binary or a Docker container — no extra system packages, and
the deps stay isolated in the router's own venv.

The easiest way is the `make` target from the repo root, which creates the
isolated environment and runs the router in one step:

```bash
make router             # syncs server/router/.venv, then runs router.py
```

Or the equivalent by hand:

```bash
cd server/router
uv sync                 # creates server/router/.venv with just eclipse-zenoh
uv run python router.py # listens on tcp/0.0.0.0:7447
```

Leave it running. A healthy start prints the listen endpoint and then sits
quietly. Configuration (environment variables, normally set in `vat.env`):

| Variable | Default | Description |
|---|---|---|
| `ZENOH_LISTEN` | `tcp/0.0.0.0:7447` | listen endpoint(s), comma-separated |
| `ZENOH_CONNECT` | _(none)_ | other routers to mesh with, comma-separated |
| `ZENOH_CONFIG` | _(none)_ | path to a full JSON5 Zenoh config (overrides the above) |

!!! note
    The eclipse-zenoh Python package does **not** provide a `zenoh.router`
    module (`python -m zenoh.router` only exists for the `zenohd` Rust binary).
    Running a `router`-mode session as `router.py` does is the supported
    pure-Python equivalent. If you ever need the router's REST/admin plugins or
    storages, install the `zenohd` binary instead — but for VAT this is enough.

### Auto-start on boot (systemd)

To keep the router running across reboots, install it as a **systemd** service (a
background program the system starts on boot). Create the unit file:

```bash
sudo nano /etc/systemd/system/vat-router.service
```

Paste this, editing `User` and the paths to match your machine:

```ini
[Unit]
Description=VAT Zenoh Router (pure-Python microservice)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=lab                      # replace with your username
WorkingDirectory=/home/lab/vat-monorepo/server/router
# uv created the isolated venv here during `uv sync`
ExecStart=/home/lab/vat-monorepo/server/router/.venv/bin/python router.py
Environment=ZENOH_LISTEN=tcp/0.0.0.0:7447
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Then enable and start it, and watch its logs:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now vat-router
sudo journalctl -fu vat-router
```

`enable --now` both starts it now and sets it to start on every boot;
`journalctl -fu` follows its logs.

---

## 2. Mapping server (PRISM-VGGT)

Requires an NVIDIA GPU + CUDA 12.8. The PRISM-VGGT code is a **git submodule** (a
second repo nested inside this one) at `server/mapping/PRISM-VGGT`. If you cloned
with `--recurse-submodules` it's already present; otherwise pull it in from the
repo root:

```bash
git submodule update --init --recursive server/mapping/PRISM-VGGT
```

Create its own isolated environment and run it (all from the repo root):

```bash
make mapping            # syncs server/mapping/.venv (heavy CUDA/torch), then runs
```

`make mapping` reads `ROUTER_IP` and the tuning knobs from `vat.env`. The
equivalent by hand:

```bash
cd server/mapping && uv sync
cd ../.. && ZENOH_ROUTER=tcp/<router-ip>:7447 make mapping
# or directly: cd server/mapping && ZENOH_ROUTER=tcp/<router-ip>:7447 uv run python mapping_server.py
```

!!! warning "The router must be running first"
    The mapping server connects to the router at `ROUTER_IP`. Start the router
    (`make router`) before the mapper. The router and mapper can even live in
    **different datacenters** — `ROUTER_IP` just has to be the router's reachable
    VPN address, never `localhost` (unless they genuinely share a host, as in the
    container above).

!!! note "First sync is slow"
    `uv sync` for the mapping env downloads the CUDA torch stack, the prebuilt
    nvblox wheel, and (on first run) the PanoVGGT model weights. This can take a
    while and a few GB of disk.

### Submap alignment defaults to Sim(3)

The mapping server now defaults to **Sim(3) alignment** (`PRISM_ALIGN=sim3`) — a
7-DoF similarity transform (rotation + translation + one global scale) for
registering each new submap into the persistent world frame.

This default is set two ways so it holds however you launch the server:
`PRISM_ALIGN=sim3` in `vat.env` (exported by the `make` target), **and** an
`os.environ.setdefault("PRISM_ALIGN", "sim3")` in `mapping_server.py`. Note that
the PRISM-VGGT submodule's own built-in default is still `sl4` (a 15-DoF
projective group); the monorepo overrides it to `sim3` because Sim(3) is
materially more robust when the robot's path loops. Set `PRISM_ALIGN=se3` or
`sl4` in `vat.env` to run the other variants.

!!! note "Why Sim(3)?"
    The reasoning (and the alignment-group ablation it's based on) is on the
    **[Reconstruction Engine](../reconstruction_engine.md)** page.

See [the bring-up runbook](../bringup.md) for the staged test sequence and
the annotated `vat.env` at the repo root for the full env-var reference.
