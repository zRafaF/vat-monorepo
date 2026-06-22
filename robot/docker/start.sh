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
# Binding a NIC that doesn't exist makes node creation fail outright
# ("<iface>: does not match an available interface"), which crash-loops the
# bridge. Resolve a REAL interface, in order:
#   1. NET_IFACE, if it exists;
#   2. the NIC on the Unitree internal subnet 192.168.123.x (where the Go2 MCU
#      publishes /sportmodestate etc.);
#   3. the default-route NIC;
#   4. the first non-loopback UP NIC.
iface_exists() { [ -d "/sys/class/net/$1" ]; }

_want="${NET_IFACE:-eth0}"
if iface_exists "$_want"; then
    NET_IFACE="$_want"
else
    echo "[start] NET_IFACE='${_want}' not present. Interfaces available:"
    ip -br link 2>/dev/null | sed 's/^/[start]   /' || ls /sys/class/net | sed 's/^/[start]   /'
    _pick="$(ip -o -4 addr show 2>/dev/null | awk '/inet 192\.168\.123\./{print $2; exit}')"
    [ -z "$_pick" ] && _pick="$(ip route show default 2>/dev/null | awk '/^default/{print $5; exit}')"
    if [ -z "$_pick" ] || ! iface_exists "$_pick"; then
        for _d in /sys/class/net/*; do
            _n="$(basename "$_d")"; [ "$_n" = "lo" ] && continue
            [ "$(cat "$_d/operstate" 2>/dev/null)" = "up" ] && { _pick="$_n"; break; }
        done
    fi
    NET_IFACE="${_pick:-lo}"
    echo "[start] auto-selected NET_IFACE='${NET_IFACE}'."
    echo "[start] If the bridge reports no topics / [no data], pin the correct one:"
    echo "[start]   set NET_IFACE=<iface> in vat.env (the NIC on the Go2's subnet)."
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
