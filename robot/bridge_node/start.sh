#!/bin/bash
# VAT bridge container entrypoint
# Starts both the DynamicZenohBridge and the FrameDecimator.
# Both processes are supervised: if either exits, the container exits
# (triggering a Docker restart policy retry).

set -euo pipefail

source /opt/ros/humble/setup.bash

echo "[start.sh] Starting DynamicZenohBridge..."
python3 /app/dynamic_bridge.py &
BRIDGE_PID=$!

echo "[start.sh] Starting FrameDecimator..."
python3 /app/frame_decimator.py &
DECIMATOR_PID=$!

echo "[start.sh] Both processes running. Bridge PID=$BRIDGE_PID  Decimator PID=$DECIMATOR_PID"

# Wait for either process to exit, then kill the other and exit
wait -n $BRIDGE_PID $DECIMATOR_PID
EXIT_CODE=$?

echo "[start.sh] A child process exited (code=$EXIT_CODE) — stopping container."
kill $BRIDGE_PID $DECIMATOR_PID 2>/dev/null || true
exit $EXIT_CODE
