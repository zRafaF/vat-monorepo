# Server setup

The server host runs two microservices, each with its **own** environment:

1. **`vat-router`** — a pure-Python Zenoh router (`server/router/`). Everything
   (robot, mapping server, client) connects to it.
2. **`vat-mapping`** — the PRISM-VGGT mapping server (`server/mapping/`), heavy
   CUDA/torch deps.

They are kept separate on purpose so the router's single light dependency never
clashes with the mapper's CUDA stack.

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
uv sync --package vat-mapping      # heavy CUDA/torch deps
source .venv/bin/activate

ZENOH_ROUTER=tcp/127.0.0.1:7447 ROBOT_NAME=go2 \
  python server/mapping/mapping_server.py
```

See [the bring-up runbook](../bringup.md) for the staged test sequence and
[the streaming POC](../streaming_poc.md) for the full env-var reference.
