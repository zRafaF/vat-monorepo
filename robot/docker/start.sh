#!/bin/bash
# VAT robot container entrypoint — process supervisor.
# ===================================================
# Runs the three robot-side processes and restarts ANY that exit, independently.
# A crash in one (e.g. a transient Zenoh drop) no longer takes down the others
# or the container — that is the "fail-over, no crashes" requirement.
#
# The container itself only stops on SIGTERM/SIGINT (docker stop), at which
# point we tear the children down cleanly.

set -uo pipefail
source /opt/ros/humble/setup.bash

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

run_forever bridge    python3 /app/dynamic_bridge.py
run_forever decimator python3 /app/frame_decimator.py
run_forever fuser     python3 /app/pose_fuser.py

echo "[start] supervising ${#PIDS[@]} processes (bridge, decimator, fuser)"
wait
