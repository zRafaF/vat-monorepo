# Server setup

The server host runs two microservices, each with its **own** environment:

1. **`vat-router`** — a pure-Python Zenoh router (`server/router/`). Everything
   (robot, mapping server, client) connects to it.
2. **`vat-mapping`** — the PRISM-VGGT mapping server (`server/mapping/`), heavy
   CUDA/torch deps.

They are kept separate on purpose so the router's single light dependency never
clashes with the mapper's CUDA stack.

---

## Containerized deployment (GPU workstation — recommended)

On the lab GPU workstation both microservices run inside **one self-contained
Docker container** defined in [`server/deploy/`](https://github.com/zRafaF/vat-monorepo/tree/main/server/deploy)
(see its `README.md`). You copy **only** `docker-compose.yml` (+ a `.env`) onto
the workstation — the container then clones the repo, installs everything, joins
Tailscale as its **own node**, and starts the router. Nothing lands on the host.

```bash
# on the workstation, in an empty dir holding docker-compose.yml + .env
docker compose up -d
docker compose logs -f          # first run is slow: clone + uv sync + weights
```

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
> from the **prebuilt** wheel (the default), validated on this workstation.

The sections below describe the **manual / bare-metal** alternative (router and
mapper run directly on the host), useful for a non-containerized server.

---

## 1. Zenoh router microservice

We run the router from **pure Python** (a `router`-mode Zenoh session) rather
than the `zenohd` binary or a Docker container — no extra system packages, and
the deps stay isolated in the router's own venv.

```bash
# from the repo root
cd server/router
uv sync                 # creates server/router/.venv with just eclipse-zenoh
uv run python router.py # listens on tcp/0.0.0.0:7447
```

Configuration (environment variables):

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

```bash
sudo nano /etc/systemd/system/vat-router.service
```

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

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now vat-router
sudo journalctl -fu vat-router
```

---

## 2. Mapping server (PRISM-VGGT)

Requires an NVIDIA GPU + CUDA. The PRISM-VGGT submodule lives at
`server/mapping/PRISM-VGGT`.

```bash
# from the repo root
git submodule update --init server/mapping/PRISM-VGGT

# its own isolated env (heavy CUDA/torch deps — does not touch router/client)
cd server/mapping && uv sync

# run it (reads ROUTER_IP etc. from ../../vat.env via the Makefile)
cd ../.. && make mapping
# or directly:  cd server/mapping && ZENOH_ROUTER=tcp/<router-ip>:7447 uv run python mapping_server.py
```

See [the bring-up runbook](../bringup.md) for the staged test sequence and
[the streaming POC](../streaming_poc.md) for the full env-var reference.
