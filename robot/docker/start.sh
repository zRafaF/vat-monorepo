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
NET_IFACE="${NET_IFACE:-eth0}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>}"

source /opt/ros/humble/setup.bash
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
