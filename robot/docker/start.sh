#!/bin/bash
# VAT robot container entrypoint — process supervisor.
# ===================================================
# Runs the three robot-side processes and restarts ANY that exit, independently.
# A crash in one (e.g. a transient Zenoh drop) no longer takes down the others
# or the container — that is the "fail-over, no crashes" requirement.
#
# The container itself only stops on SIGTERM/SIGINT (docker stop), at which
# point we tear the children down cleanly.

# NOTE: no `set -u` — ROS's setup.bash references unset vars (e.g.
# AMENT_TRACE_SETUP_FILES) and would abort under `set -u`.
set -o pipefail

# The Go2 host speaks CycloneDDS on a specific interface. This (Humble) container
# must match the RMW *and* the interface to discover the host's Foxy topics —
# otherwise the bridge sees an empty ROS graph. (--network host is set by run.sh.)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ── Resolve the CycloneDDS network interface ─────────────────────────────────
# CycloneDDS only enumerates interfaces that are UP *and* have an IPv4 address;
# binding any other name fails with "<iface>: does not match an available
# interface", which crash-loops the bridge. So we must pick a *usable* NIC.
# Note: an interface can EXIST in /sys (e.g. eth0) yet be down / address-less —
# that is exactly this robot's case — so existence alone is not enough.
#
# Order: 1) NET_IFACE if usable; 2) the NIC on the Unitree subnet 192.168.123.x
# (the Go2 MCU's network); 3) the default-route NIC; 4) the first UP NIC w/ IPv4.
iface_usable() {
    local n="$1" st
    [ -d "/sys/class/net/$n" ] || return 1
    st="$(cat "/sys/class/net/$n/operstate" 2>/dev/null)"
    # "up" = link present; "unknown" = lo/tun/veth that still carry traffic
    [ "$st" = "up" ] || [ "$st" = "unknown" ] || return 1
    ip -o -4 addr show dev "$n" 2>/dev/null | grep -q 'inet ' || return 1
    return 0
}
iface_addr() { ip -o -4 addr show dev "$1" 2>/dev/null | awk '{print $4}' | head -1; }

_want="${NET_IFACE:-}"
NET_IFACE=""
if [ -n "$_want" ] && iface_usable "$_want"; then
    NET_IFACE="$_want"
fi
if [ -z "$NET_IFACE" ]; then
    NET_IFACE="$(ip -o -4 addr show 2>/dev/null | awk '/inet 192\.168\.123\./{print $2; exit}')"
fi
if [ -z "$NET_IFACE" ]; then
    _c="$(ip route show default 2>/dev/null | awk '/^default/{print $5; exit}')"
    [ -n "$_c" ] && iface_usable "$_c" && NET_IFACE="$_c"
fi
if [ -z "$NET_IFACE" ]; then
    for _d in /sys/class/net/*; do
        _n="$(basename "$_d")"; [ "$_n" = "lo" ] && continue
        iface_usable "$_n" && { NET_IFACE="$_n"; break; }
    done
fi

echo "[start] interfaces (UP + IPv4 are the candidates):"
ip -br addr 2>/dev/null | sed 's/^/[start]   /' || true
if [ -n "$NET_IFACE" ]; then
    echo "[start] DDS interface: ${NET_IFACE} ($(iface_addr "$NET_IFACE"))"
    if [ -n "$_want" ] && [ "$NET_IFACE" != "$_want" ]; then
        echo "[start] (NET_IFACE='${_want}' was not usable — auto-selected '${NET_IFACE}')."
        echo "[start]  If the bridge shows [no data], pin NET_IFACE in vat.env."
    fi
else
    echo "[start] ERROR: no UP interface with an IPv4 address — CycloneDDS can't bind."
    echo "[start]        Connect the Go2 network, or set NET_IFACE in vat.env."
    echo "[start]        Falling back to 'lo' so the container doesn't crash-loop."
    NET_IFACE="lo"
fi
export NET_IFACE
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>}"

source /opt/ros/humble/setup.bash
# Overlay with the unitree_go messages (built in the image) so the bridge can
# subscribe to /sportmodestate. Best-effort: absent if the build was skipped.
if [ -f /opt/unitree_ws/install/setup.bash ]; then
    source /opt/unitree_ws/install/setup.bash
    echo "[start] unitree_go overlay sourced (custom msgs available)"
else
    echo "[start] NOTE: unitree_go overlay not found — /sportmodestate won't forward"
    echo "[start]       (Stage 2). Standard-typed topics are unaffected."
fi
echo "[start] RMW=$RMW_IMPLEMENTATION  iface=$NET_IFACE  domain=${ROS_DOMAIN_ID:-0}"

PIDS=()

# run_forever <name> <cmd...> : keep a process alive, restart on exit
run_forever() {
    local name="$1"; shift
    (
        while true; do
            echo "[start] launching ${name}"
            "$@"
            code=$?
            echo "[start] ${name} exited (code=${code}) — restarting in 3s"
            sleep 3
        done
    ) &
    PIDS+=("$!")
}

shutdown() {
    echo "[start] shutdown requested — stopping children"
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
    # kill the python grandchildren too
    pkill -P $$ 2>/dev/null || true
    exit 0
}
trap shutdown SIGTERM SIGINT

echo "[start] ROBOT_NAME=${ROBOT_NAME}  ZENOH_CONNECT=${ZENOH_CONNECT}"

run_forever bridge python3 /app/dynamic_bridge.py
run_forever camera python3 /app/theta_camera.py
run_forever fuser  python3 /app/pose_fuser.py

echo "[start] supervising ${#PIDS[@]} processes (bridge, theta_camera, fuser)"
wait
