# VAT server deployment (GPU workstation)

Brings the **Zenoh router** + **PRISM-VGGT mapping server** up on the lab GPU workstation, fully self-contained in one Docker container. Tailscale runs *inside* the container as its own node, so the host stays untouched (and any Tailscale already running on the host is unaffected).

## What it does

The container, on first `up`:

* clones `vat-monorepo` (recursively, incl. the PRISM-VGGT submodule) into its own `/root` — nothing is cloned onto the host


* installs `uv`, syncs the `server/mapping` (CUDA/torch/**prebuilt nvblox**) and `server/router` environments, and downloads the PanoVGGT weights


* joins your tailnet as a dedicated node (default name `vat-server`)


* **auto-starts the Zenoh router** on `:7447`, supervised (restarts if it dies)



The **GPU mapping server is manual** — you start it over SSH when you want it.

## One-time host setup

On the workstation, in an empty directory, you only need **two files**:

* `docker-compose.yml` (copy from this folder)


* `.env` (copy .env.example -> .env and fill in TS_AUTHKEY)



Then:

```bash
docker compose up -d
docker compose logs -f        # watch clone + install (first run is slow: weights + uv sync)

```

That's it — no repo checkout, no Python, no Tailscale on the host.

> Requires NVIDIA Container Toolkit on the host (for GPU passthrough) and `/dev/net/tun` available (standard on Linux, used for Tailscale).
> 
> 

## Wiring the rest of the fleet

The router now lives on `vat-server`, not the laptop. On the **robot** and the **client**, set the router address in their `vat.env`:

```bash
ROUTER_IP=vat-server      # Tailscale MagicDNS name (or its 100.x Tailscale IP)

```

Everything else (keys, ports) is unchanged — robot and client already dial `ROUTER_IP`.

## Running the mapping server (manual)

SSH into the container, then run it from the repo. Inside the container the router is local, so point the mapper at localhost:

```bash
# via Tailscale SSH (from any tailnet machine):
tailscale ssh root@vat-server
# or host-port fallback:
ssh -o StrictHostKeyChecking=no root@<workstation-host> -p 2222   # pw: ROOT_PASSWORD

cd /root/vat-monorepo
make mapping ROUTER_IP=127.0.0.1

```

---

## Troubleshooting: Remote Container Restarts (GPU Drops)

If the NVIDIA Container Toolkit loses the GPU mount (causing `nvidia-smi` errors), you must restart the container using `docker compose down` and `docker compose up -d`. Because Tailscale runs *inside* the container as its own node, restarting the container directly kills your VPN connection mid-command. To safely perform this restart remotely, use the container as a jump host:

1. SSH into the container via Tailscale: `tailscale ssh root@vat-server`.


2. From inside the container, SSH out to the bare-metal host using its local LAN IP or the Docker bridge IP (e.g., `172.17.0.1`).
3. Navigate to the directory containing your `docker-compose.yml`.


4. Run a shielded restart command so it completes in the background after the container drops the VPN connection: `nohup sh -c 'docker compose down && docker compose up -d' > docker_restart.log 2>&1 &`
5. Expect your SSH connection to drop immediately. Wait roughly 60 seconds for the container and Tailscale to boot back up, then reconnect.

---

## Notes

* **Tailscale isolation.** `tailscaled` runs in the container's own network namespace with its own state dir (`./tailscale_state`), kernel mode via `NET_ADMIN` + `/dev/net/tun`, and `--netfilter-mode=off` so it never edits the host's iptables. It is a separate tailnet node from anything on the host.


* **Auth key.** Use a **reusable, non-ephemeral** key so the node is stable across restarts; the persisted `tailscale_state/` volume means it won't re-register each boot.


* **nvblox = prebuilt** (the default, already validated on this workstation). A source build is *not* performed here; if you ever need it, follow PRISM-VGGT's `docs/ADVANCED_NVBLOX.md` inside the container.


* **PRISM-VGGT is untouched** — its own `docker-compose.yml` remains its dev sandbox; this deployment is a vat-monorepo concern only.