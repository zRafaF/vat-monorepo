#!/bin/bash
# VAT monorepo cleanup — removes dead files from previous iterations.
# Run once from the repo root, then delete this script too.
#
#   bash cleanup.sh
#   git commit -m "chore: remove dead files"

set -e
cd "$(git rev-parse --show-toplevel)"

echo "Removing dead files..."

# Old bridge_node dir (replaced by robot/docker/)
git rm -r --cached --ignore-unmatch robot/bridge_node/

# Old frame_publisher ROS2 node (never worked with ROS Foxy/Zenoh)
git rm -r --cached --ignore-unmatch robot/frame_publisher/

# frame_decimator.py at wrong location (now in robot/docker/)
git rm --cached --ignore-unmatch robot/frame_decimator.py

# Old vat_bringup at wrong location (now in robot/ros/vat_bringup/)
git rm -r --cached --ignore-unmatch robot/vat_bringup/

# Scratch / test files
git rm --cached --ignore-unmatch robot/front_camera.py
git rm --cached --ignore-unmatch robot/requirements.txt
git rm --cached --ignore-unmatch server/ros_test.py
git rm --cached --ignore-unmatch server/requirements.txt
git rm --cached --ignore-unmatch client/requirements.txt
git rm --cached --ignore-unmatch client/slam_poc.py
git rm --cached --ignore-unmatch client/list_topics.py
git rm --cached --ignore-unmatch client/odom_sniffer.py

# Generated protobuf file (no longer used)
git rm --cached --ignore-unmatch proto/go2_stream_pb2.py

# Root-level leftovers
git rm --cached --ignore-unmatch requirements.txt

# Committed Windows venv (should be gitignored)
git rm -r --cached --ignore-unmatch venv-windows/

# Add venv-windows to .gitignore if not already there
grep -qxF 'venv-windows/' .gitignore || echo 'venv-windows/' >> .gitignore

echo ""
echo "Done. The files are untracked from git but still on disk."
echo "Run: git status   to review, then   git commit -m 'chore: remove dead files'"
echo "You can physically delete them with: git clean -fd (careful — removes ALL untracked files)"
