#!/usr/bin/env bash
# VAT — server container bootstrap (idempotent).
# Runs INSIDE the container as PID 1. Safe to re-run on every restart: it skips
# work that's already done and ends in a supervision loop for the Zenoh router.
#
# Flow: system packages -> sshd -> uv -> repo/submodules -> mapping+router envs
#       -> PanoVGGT weights -> Tailscale (own node) -> supervise router.
set -euo pipefail

log() { echo "[vat-deploy $(date +%H:%M:%S)] $*"; }

REPO=/root/vat-monorepo
ROUTER_PORT="${ROUTER_PORT:-7447}"
TS_HOSTNAME="${TS_HOSTNAME:-vat-server}"
ROOT_PASSWORD="${ROOT_PASSWORD:-lab_password}"
TS_SOCK=/var/run/tailscale/tailscaled.sock

# ── 1. System packages ──────────────────────────────────────────────────────
log "Installing system packages..."
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  openssh-server git curl ca-certificates \
  libgl1 libglib2.0-0 python3.12 python3.12-venv python3-pip ninja-build iproute2

# ── 2. SSH (host-port 2222 fallback; Tailscale SSH is enabled below too) ─────
mkdir -p /var/run/sshd
echo "root:${ROOT_PASSWORD}" | chpasswd
sed -i 's/#\?PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config
sed -i 's/#\?PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config
pgrep -x sshd >/dev/null 2>&1 || /usr/sbin/sshd
log "sshd running (host port 2222)."

# ── 3. uv ────────────────────────────────────────────────────────────────────
if ! command -v uv >/dev/null 2>&1; then
  log "Installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="/root/.local/bin:${PATH}"

# ── 4. Repo + submodules (PRISM-VGGT) ───────────────────────────────────────
cd "$REPO"
git config --global --add safe.directory "$REPO" || true
log "Updating submodules (PRISM-VGGT)..."
git submodule update --init --recursive

# ── 5. Python envs (prebuilt nvblox = default; already validated on this box) ─
export UV_SKIP_WHEEL_FILENAME_CHECK=1
export UV_PYTHON_PREFERENCE=system
log "Syncing mapping env (CUDA/torch/nvblox)... first run is slow."
( cd "$REPO/server/mapping" && uv sync )
log "Syncing router env..."
( cd "$REPO/server/router" && uv sync )

# ── 6. PanoVGGT weights (one-time, into the submodule's checkpoints/) ────────
WEIGHTS="$REPO/server/mapping/PRISM-VGGT/checkpoints/model.pt"
if [ ! -f "$WEIGHTS" ]; then
  log "Downloading PanoVGGT weights..."
  ( cd "$REPO/server/mapping" && \
    uv run python -c "from prism_vggt import download_weights; download_weights('PRISM-VGGT/checkpoints/model.pt')" )
else
  log "Weights already present."
fi

# ── 7. Tailscale — in-container, OWN node, kernel mode ───────────────────────
# Separate netns + own state dir => does not interfere with any Tailscale
# instance on the host. --netfilter-mode=off avoids touching iptables.
if ! command -v tailscale >/dev/null 2>&1; then
  log "Installing Tailscale..."
  curl -fsSL https://tailscale.com/install.sh | sh
fi
mkdir -p /var/run/tailscale /var/lib/tailscale
if ! pgrep -x tailscaled >/dev/null 2>&1; then
  log "Starting tailscaled..."
  tailscaled --state=/var/lib/tailscale/tailscaled.state --socket="$TS_SOCK" \
    >/var/log/tailscaled.log 2>&1 &
  sleep 2
fi
log "Bringing up Tailscale as '${TS_HOSTNAME}'..."
tailscale --socket="$TS_SOCK" up \
  --authkey="${TS_AUTHKEY:?TS_AUTHKEY not set}" \
  --hostname="${TS_HOSTNAME}" \
  --ssh \
  --netfilter-mode=off \
  --accept-dns=false
TS_IP="$(tailscale --socket="$TS_SOCK" ip -4 2>/dev/null | head -n1 || true)"
log "Tailscale up. node='${TS_HOSTNAME}' ip='${TS_IP}'"

# ── 8. Supervise the Zenoh router (this loop keeps the container alive) ──────
ROUTER_PY="$REPO/server/router/.venv/bin/python"
export ZENOH_LISTEN="tcp/0.0.0.0:${ROUTER_PORT}"
log "Router listening on ${ZENOH_LISTEN}.  Robot/client dial: ${TS_HOSTNAME}:${ROUTER_PORT}"
log "Mapping server is MANUAL. SSH in, then:"
log "    cd /root/vat-monorepo && make mapping ROUTER_IP=127.0.0.1"
while true; do
  log "Starting Zenoh router..."
  "$ROUTER_PY" "$REPO/server/router/router.py" || true
  log "Router exited; restarting in 5s."
  sleep 5
done
