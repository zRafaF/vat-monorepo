# VAT monorepo cleanup - removes dead files from previous iterations.
# Run once from the repo root, then delete this script too.
#
#   powershell -ExecutionPolicy Bypass -File cleanup.ps1
#   git commit -m "chore: remove dead files"

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$root = git rev-parse --show-toplevel
Set-Location $root

Write-Host "Removing dead files from git index..."

$paths = @(
    "cleanup.sh",               # bash version, not needed on Windows
    "robot/bridge_node",        # replaced by robot/docker/
    "robot/frame_publisher",    # old ROS2 node, never worked with Foxy+Zenoh
    "robot/frame_decimator.py", # moved to robot/docker/frame_decimator.py
    "robot/vat_bringup",        # moved to robot/ros/vat_bringup/
    "robot/front_camera.py",
    "robot/requirements.txt",
    "server/ros_test.py",
    "server/requirements.txt",
    "client/requirements.txt",
    "client/slam_poc.py",
    "client/list_topics.py",
    "client/odom_sniffer.py",
    "proto/go2_stream_pb2.py",  # generated protobuf, no longer used
    "requirements.txt",         # root-level leftover
    "venv-windows"              # committed venv
)

foreach ($p in $paths) {
    git rm -r --cached --ignore-unmatch $p
}

# Add venv-windows to .gitignore if missing
$gitignore = ".gitignore"
$entry = "venv-windows/"
if (-not (Select-String -Path $gitignore -Pattern ([regex]::Escape($entry)) -Quiet)) {
    Add-Content -Path $gitignore -Value "`n$entry"
    Write-Host "Added '$entry' to .gitignore"
}

Write-Host ""
Write-Host "Done. Files are untracked from git but still on disk."
Write-Host "Review with:  git status"
Write-Host "Commit with:  git commit -m 'chore: remove dead files'"
Write-Host "To also delete from disk: git clean -fd  (removes ALL untracked files - be careful)"