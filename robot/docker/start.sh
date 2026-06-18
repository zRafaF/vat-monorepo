#!/bin/bash
# Starts both DynamicZenohBridge and FrameDecimator.
# If either exits the container exits (triggering Docker restart policy).
set -euo pipefail

source /opt/ros/humble/setup.bash

echo "[start] Starting DynamicZenohBridge..."
python3 /app/dynamic_bridge.py &
BRIDGE_PID=$!

echo "[start] Starting FrameDecimator..."
python3 /app/frame_decimator.py &
DECIMATOR_PID=$!

echo "[start] Bridge PID=$BRIDGE_PID  Decimator PID=$DECIMATOR_PID"

wait -n $BRIDGE_PID $DECIMATOR_PID
EXIT_CODE=$?
echo "[start] A process exited (code=$EXIT_CODE) — stopping container."
kill $BRIDGE_PID $DECIMATOR_PID 2>/dev/null || true
exit $EXIT_CODE
