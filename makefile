# ============================================================================
# VAT — central control file
# ============================================================================
# Public config lives in vat.env (safe to commit). This Makefile drives setup,
# the services (router / mapping / robot), and the staged pre-POC tests.
#
#   make help     → list targets + the staged test order
#   make steps    → print the full pre-POC → POC runbook
# ============================================================================

include vat.env
export                          # export every vat.env var to recipe shells

# Client-side tools + viewer run in the client's OWN isolated env (client/.venv).
# (The bring-up tools live in ../tools but run in this same env.)
CLIENT_RUN ?= cd client && uv run python

.DEFAULT_GOAL := help

.PHONY: help steps \
        sync-mapping sync-client sync-router sync-robot sync-docs \
        router mapping theta-uvc theta-uvc-kill theta-stream robot-docker viewer record-frames \
        record compose record-selftest rig backfill record-ui sync-recorder \
        replay sync-replay \
        test_link test_frames_robot test_frames_server test_robot_state test_poses \
        teleop fetch_frame fetch_pcd periscope-probe rgbd-probe rgbd-camera rgbd-relay \
        docs docs-serve clean

# ── Help / runbook ───────────────────────────────────────────────────────────
help:
	@echo "VAT control file   router=$(ROUTER_IP):$(ROUTER_PORT)   robot=$(ROBOT_NAME)"
	@echo ""
	@echo "Setup:"
	@echo "  make sync-router     isolated env for the Zenoh router"
	@echo "  make sync-mapping    mapping server env (GPU machine, CUDA)"
	@echo "  make sync-client     client + bring-up tools env"
	@echo "  make sync-robot      [ROBOT] host-side tools env (theta_pub)"
	@echo "  make sync-docs        docs env (mkdocs + plugins)"
	@echo ""
	@echo "Documentation:"
	@echo "  make docs            build the documentation"
	@echo "  make docs-serve      serve the documentation locally"
	@echo "  make docs-deploy     deploy the documentation to GitHub Pages"
	@echo ""
	@echo "Services:"
	@echo "  make router          [SERVER] run the Zenoh router (hub)"
	@echo "  make mapping         [SERVER] run the PRISM mapping server"
	@echo "  make theta-uvc       [ROBOT]  expose Theta X UVC → /dev/video10 (host)"
	@echo "  make theta-uvc-kill  [ROBOT]  stop the Theta UVC feed (after a camera reboot)"
	@echo "  make theta-stream    [ROBOT]  headless: Theta → Zenoh (view on host)"
	@echo "  make robot-docker    [ROBOT]  bridge + theta_camera + pose fuser container"
	@echo "  make viewer          [CLIENT] full POC viewer — VisPy 3D + Telemetry window"
	@echo "                                (cloud + robot + legs; keys: arrows orbit, WASD pan,"
	@echo "                                 N/M point size, C/[/] ceiling clip, 1 re-fetch)"
	@echo "  make record-frames   [CLIENT] save 360° frames to disk (offline analysis)"
	@echo "  make record          [ANY]    passively record a LIVE session (all streams,"
	@echo "                                one common clock) → recordings/<session_id>/"
	@echo "                                ARGS=\"--scene lab --trajectory-family loop --pass 1 --camera-height 1.152\""
	@echo "  make compose         [ANY]    align/compose a recording (info | export | periscope | render)"
	@echo "                                ARGS=\"export recordings/<id> --fps 10\""
	@echo "  make record-selftest [ANY]    offline check of the recorder + composer (no robot)"
	@echo "  make rig             [ANY]    fake robot+cloud on the real Zenoh keys (test the live path)"
	@echo "  make record-ui       [ANY]    browser console: start/stop, live progress, fetch full-res, zip"
	@echo "  make backfill        [ANY]    fetch full-res panoramas into a FINISHED recording"
	@echo "  make replay          [CLIENT] play a recorded session back in the Rerun app (run it where the screen is)"
	@echo "                                ARGS=\"recordings/data/<id> --save run.rrd\""
	@echo "                                ARGS=\"recordings/<id> --every 2\""
	@echo "  make periscope-probe [CLIENT] headless periscope stream check (ARGS=\"--decode --save f.png\")"
	@echo "  make rgbd-probe      [CLIENT] headless D435i panel stream check (ARGS=\"--kind depth --decode\")"
	@echo "  make rgbd-camera     [ROBOT]  launch RealSense depth node (pinned to the Go2 NIC for the container)"
	@echo "  make rgbd-relay      [ROBOT]  (fallback) run the RGBD relay on the host instead of the container"
	@echo ""
	@echo "Staged pre-POC tests (run in this order — see 'make steps'):"
	@echo "  make test_link           0  transport alive (router + bridge + rates)"
	@echo "  make test_frames_robot   1  [ROBOT] preview the Theta UVC directly"
	@echo "  make test_frames_server  1  decimated frames the server ingests"
	@echo "  make test_robot_state    2  body + limb/foot positions, live"
	@echo "  make test_poses          3  camera trajectory + fused robot pose"
	@echo ""
	@echo "Teleop:"
	@echo "  make teleop          [CLIENT] keyboard drive (WASD), deadman + e-stop"
	@echo ""
	@echo "Then: make viewer  (Stage 4, the real POC)"

steps:
	@echo "VAT pre-POC → POC runbook  (router=$(ROUTER_IP))"
	@echo "Set up once: SERVER 'make sync-router && make sync-mapping',"
	@echo "             CLIENT 'make sync-client', ROBOT builds the docker image."
	@echo ""
	@echo "Stage 0  Transport"
	@echo "  [SERVER] make router          # the hub; leave it running"
	@echo "  [ROBOT]  make theta-uvc        # Theta X → /dev/video10 (leave running)"
	@echo "  [ROBOT]  make theta-stream     # headless: Theta → Zenoh; view via test_frames_server"
	@echo "  [ROBOT]  make robot-docker     # bridge + theta_camera + fuser"
	@echo "  [CLIENT] make test_link        # expect bridge ALIVE + non-zero Hz"
	@echo ""
	@echo "Stage 1  Frames"
	@echo "  [ROBOT]  make test_frames_robot   # preview the Theta UVC directly (camera alone)"
	@echo "  [CLIENT] make test_frames_server  # decimated frames + camera_height"
	@echo ""
	@echo "Stage 2  Body & limbs"
	@echo "  [CLIENT] make test_robot_state    # body frame + 4 feet move live"
	@echo ""
	@echo "Stage 3  Poses"
	@echo "  [SERVER] make mapping             # PRISM-VGGT (needs GPU)"
	@echo "  [CLIENT] make test_poses          # trajectory + fused pose track"
	@echo ""
	@echo "Stage 4  Full POC"
	@echo "  [CLIENT] make viewer              # point cloud + predicted robot block"

# ── Setup (uv) — each service syncs into its OWN folder (.venv + uv.lock) ─────
sync-mapping:
	cd server/mapping && uv sync

sync-client:
	cd client && uv sync

# [ROBOT] Host-side tools env (theta_pub.py): eclipse-zenoh + headless OpenCV.
sync-robot:
	cd robot && uv sync

sync-router:
	cd server/router && uv sync

# Browser console env (Gradio) — its own isolated project, so the client/viewer
# env does not grow a web stack.
sync-recorder:
	cd tools/recorder && uv sync

# Replay env (Rerun) — a pure reader, so it needs no Zenoh and no Gradio.
sync-replay:
	cd recordings && uv sync

sync-docs:
	uv sync --group docs

# ── Services ─────────────────────────────────────────────────────────────────
# [SERVER] The Zenoh router (hub). Binds ZENOH_LISTEN; leave it running.
router: sync-router
	@echo ">> Router binding $(ZENOH_LISTEN)  (clients dial $(ROUTER_IP):$(ROUTER_PORT))"
	cd server/router && uv run python router.py

# [SERVER] PRISM mapping server. Needs a GPU + the PRISM-VGGT submodule.
# Runs in its own isolated env (server/mapping/.venv).
mapping: sync-mapping
	@echo ">> Reminder: the router must be running on $(ROUTER_IP)."
	@echo ">> Connecting to $(ZENOH_ROUTER)  (may be a different datacenter — OK)"
	cd server/mapping && uv run python mapping_server.py

# [ROBOT] Expose the RICOH Theta X UVC stream as /dev/video10 on the host
# (libuvc-theta via the gstthetauvc plugin). Leave running; the container reads it.
theta-uvc:
	@echo ">> [ROBOT] Theta X UVC → /dev/video10 (leave running in its own shell)"
	bash robot/theta/theta_uvc.sh

# [ROBOT] Stop the Theta UVC feed (gstthetauvc / gst_loopback). Use this when the
# camera dropped and you've rebooted it: kill the old feed, then re-run
# `make theta-uvc`. The v4l2 loopback device is left loaded (theta_uvc.sh reuses
# it); the container's frozen-stream watchdog reconnects automatically once the
# fresh feed is up — no robot-docker restart needed.
theta-uvc-kill:
	@echo ">> [ROBOT] stopping Theta UVC feed (gstthetauvc / gst_loopback) ..."
	-@pkill -f 'gst-launch-1.0.*thetauvcsrc' 2>/dev/null || true
	-@pkill -f 'gst_loopback' 2>/dev/null || true
	-@pkill -f 'robot/theta/theta_uvc.sh' 2>/dev/null || true
	@echo ">> stopped. Loopback /dev/video10 kept loaded; re-run 'make theta-uvc' to restart the feed."

# [ROBOT] Headless: publish the Theta loopback to Zenoh so you can view it on
# the host (make test_frames_server). No display on the robot; no container.
theta-stream: sync-robot
	@echo ">> [ROBOT] Theta /dev/video10 → Zenoh (view on host: make test_frames_server)"
	cd robot && uv run python ../tools/theta_pub.py

# [ROBOT] Container: ROS↔Zenoh bridge + theta_camera + pose fuser.
# Invoked via `bash` so it doesn't depend on the executable bit (git on Windows
# doesn't preserve it). If docker needs root on your robot: `sudo make robot-docker`.
robot-docker:
	@echo ">> [ROBOT] build + run container → router $(ROUTER_IP)"
	bash robot/docker/run.sh $(ROUTER_IP)

# [CLIENT] Full POC viewer — native Open3D (low-latency, no gRPC/spawn stream).
viewer: sync-client
	@echo ">> Reminder: router + mapping server must be running."
	$(CLIENT_RUN) prism_viewer.py --snapshot

# [CLIENT] Record 360° frames to disk for offline analysis.
# Opens a folder picker, then saves every incoming JPEG named by timestamp_ns.
record-frames: sync-client
	@echo ">> Recording 360° frames — a folder picker will open."
	$(CLIENT_RUN) ../tools/record_frames.py

# ── Session recorder (tools/recorder) ────────────────────────────
# [ANY] Passively record a LIVE session: every stream, independently, all stamped on
# one common clock (the robot capture clock) so they re-align afterwards. Pure
# subscriber — it cannot disturb the robot, the mapping server or the client. Writes
# recordings/<session_id>/ at the repo root; Ctrl-C stops cleanly and finalises.
# Run `make record ARGS=--dry-run` first to see exactly which keys it will tap.
#   make record ARGS="--scene lab --trajectory-family loop --pass 1 --camera-height 1.152"
# For the full-res 360° archive pull, run this ON THE ROBOT with --where robot.
# See docs/recording.md.
record: sync-client
	@echo ">> Reminder: router + robot container + 'make mapping' running."
	@echo ">> MEASURE the camera height this session and pass --camera-height (metric anchor)."
	$(CLIENT_RUN) ../tools/recorder/vat_record.py $(ARGS)

# [ANY] Align / compose a recording. Subcommands: info | export | periscope | render.
#   make compose ARGS="info recordings/<id>"
#   make compose ARGS="export recordings/<id> --fps 10 --link hard"
compose: sync-client
	$(CLIENT_RUN) ../tools/recorder/compose.py $(ARGS)

# [ANY] Offline end-to-end check of the recorder + composer: synthesises a session
# through the real wire packers, records it, composes it. No robot / Zenoh / GPU.
record-selftest: sync-client
	$(CLIENT_RUN) ../tools/recorder/vat_record.py --selftest
	@echo ">> console smoke test (builds, launches, serves) ..."
	-cd tools/recorder && uv run python ui.py --selftest

# [ANY] Fake rig: publish synthetic robot+cloud traffic on the REAL Zenoh keys so the
# recorder's live path (subscriptions, the archive query/reply, the block-repair pull,
# a real H.264 periscope stream) can be exercised without the robot. Needs 'make router'.
#   make rig ARGS="--drop-pushes 0.35"      # shed pushes to exercise manifest repair
rig: sync-client
	@echo ">> Reminder: 'make router' must be running. This is a TEST FIXTURE, not the robot."
	$(CLIENT_RUN) ../tools/recorder/fake_rig.py $(ARGS)

# [ANY] Fetch full-resolution panoramas into a FINISHED recording, from the robot's
# rolling archive (~10GB ~ 6h). Nothing is fetched during the walk, so the capture is
# untouched; afterwards you choose how much to pull. Resumable. Run it while nothing
# else is capturing.  See docs/recording.md.
#   make backfill ARGS="recordings/<id> --dry-run"      # what it would cost
#   make backfill ARGS="recordings/<id> --every 2"
backfill: sync-client
	$(CLIENT_RUN) ../tools/recorder/backfill.py $(ARGS)

# [ANY] Browser console for capturing: start/stop, live per-stream progress, memory and
# size, reset the PRISM map, fetch full-res after the run, browse past recordings and
# download any of them as a zip.  http://<host>:7860
#   make record-ui ARGS="--port 8080"
record-ui: sync-recorder
	@echo ">> Recorder console on :7860 (share URL printed): start/stop, live progress, fetch full-res, zips."
	cd tools/recorder && uv run python ui.py $(ARGS)

# [CLIENT] Play a recorded session back in the Rerun app: the map growing, the trajectory
# walked, the panorama, the periscope and the ESDF, all on the ONE session timeline. Reads
# only what the recorder wrote -- no robot, no Zenoh, no mapping server. RUN THIS WHERE THE
# SCREEN IS (your laptop): the default mode launches the native Rerun viewer. On a headless
# box use --save and open the .rrd locally. See recordings/README.md.
#   make replay                                              # folder picker, then Rerun opens
#   make replay ARGS="--list"                                # what is in recordings/data/
#   make replay ARGS="recordings/data/<id>"
#   make replay ARGS="recordings/data/<id> --save run.rrd"   # headless: then `rerun run.rrd`
replay: sync-replay
	cd recordings && uv run python replay.py $(ARGS)

# ── Staged pre-POC tests ─────────────────────────────────────────────────────
# All run in the client's own env; tools live in ../tools relative to client/.
# Stage 0 — transport alive.
test_link: sync-client
	@echo ">> Reminder: 'make router' (SERVER) + robot bridge must be running."
	$(CLIENT_RUN) ../tools/check_link.py

# Stage 1a — [ROBOT] preview the Theta straight off UVC (camera alone, no Zenoh).
# Runs on the robot host with its OpenCV; uses THETA_DEVICE / THETA_GST_PIPELINE.
test_frames_robot:
	@echo ">> [ROBOT] previewing the Theta UVC device directly (THETA_DEVICE=$(THETA_DEVICE))"
	@echo ">> NOTE: opens an OpenCV window (needs a display). Headless robot? Use 'make theta-stream' + host 'make test_frames_server'."
	python3 tools/view_theta.py

# Stage 1b — [CLIENT] the decimated frames the mapping server actually ingests.
test_frames_server: sync-client
	@echo ">> Reminder: router + 'make theta-uvc' + 'make robot-docker' running."
	$(CLIENT_RUN) ../tools/view_frames.py

# [CLIENT] Keyboard teleop — drive the robot (continuous stream + deadman).
teleop: sync-client
	@echo ">> Reminder: router + robot container (teleop_bridge) running. Keep the physical remote in hand."
	$(CLIENT_RUN) ../tools/teleop_keyboard.py

# [ANY] Fetch one FULL-RES archived frame by seq:  make fetch_frame SEQ=1234
fetch_frame: sync-client
	@echo ">> fetching full-res archive frame seq=$(SEQ) from $(ROBOT_NAME)"
	$(CLIENT_RUN) ../tools/fetch_archive.py --seq $(SEQ)

# [CLIENT] Diagnostic: fetch one PRISM cloud, print stats, save .npz/.ply to open
# in pano_viz.py — proves whether a bad cloud is a streaming/codec or render issue.
fetch_pcd: sync-client
	@echo ">> fetch a live PRISM cloud for pano_viz. ARGS=--both compares server vs client."
	$(CLIENT_RUN) ../tools/fetch_pcd.py $(if $(OUT),--out $(OUT),) $(ARGS)

# Stage 2 — body frame + limb/foot positions, live.
test_robot_state: sync-client
	@echo ">> Reminder: router + robot bridge (publishing sportmodestate) running."
	$(CLIENT_RUN) ../tools/view_robot_state.py

# Stage 3 — camera trajectory + VGGT correction + fused robot pose.
test_poses: sync-client
	@echo ">> Reminder: router + robot container + 'make mapping' running."
	$(CLIENT_RUN) ../tools/view_poses.py

# Data-source probe -- which robot topics/fields actually carry usable data?
# Text-only: discovers every bridged topic, decodes it, reports nonzero/changing
# fields + a pose-critical verdict (wheel dq, IMU, velocity, odom). DRIVE the dog
# during the capture window.  PROBE_S=25 / PROBE_ALL=1 to widen it.
probe_robot: sync-client
	@echo ">> Reminder: router + robot container (dynamic_bridge.py) running."
	@echo ">> DRIVE / MOVE the dog during the $${PROBE_S:-15}s capture window."
	$(CLIENT_RUN) ../tools/probe_robot_data.py

# [CLIENT] Remote periscope stream probe -- isolates the video pipeline from the
# VisPy viewer: publishes a ViewRequest (+keepalive), subscribes to the encoded
# frames, and reports codec/size/rate. Add --decode to decode with the viewer's
# exact decoder and --save to dump a frame to disk -- this cleanly separates
# "robot not publishing" from "client can't decode" from a viewer-only display bug.
# Deps are uv-controlled: this runs in client/.venv, and 'uv sync' installs PyAV
# (av, in client/pyproject.toml) so H.265/H.264 decode works (cv2 covers MJPEG).
#   make periscope-probe ARGS="--decode --save /tmp/peri.png --yaw 30 --fov 60"
periscope-probe: sync-client
	@echo ">> Reminder: router + robot container (periscope service, PERISCOPE_ENABLE=1) running."
	$(CLIENT_RUN) ../tools/periscope_probe.py $(ARGS)

# [CLIENT] RGBD (D435i) single-frame stream probe -- isolates the depth/color panel
# stream from the viewer. Runs in client/.venv (opencv decodes depth PNG / color JPEG).
#   make rgbd-probe ARGS="--kind depth --decode --save /tmp/rgbd.png"
rgbd-probe: sync-client
	@echo ">> Reminder: robot container (rgbd_relay) + realsense2_camera running."
	$(CLIENT_RUN) ../tools/rgbd_probe.py $(ARGS)

# [ROBOT/HOST] Launch the RealSense D435i driver (ROS2) so it publishes depth+color
# to DDS for the relay. Run this where the camera is plugged in (host, or a container
# with realsense2_camera). Needs ros-humble-realsense2-camera installed.
# [ROBOT/HOST] Run the RGBD relay ON THE HOST, in the SAME ROS2 env as the camera
# (source ROS2 first, and `pip install eclipse-zenoh opencv-python-headless`). Use this
# when the in-container relay cannot discover the host realsense node over DDS (RMW /
# interface mismatch) -- running side-by-side makes discovery automatic. Set
# RGBD_ENABLE=0 for the container (rebuild) so only one relay publishes.
rgbd-relay:
	@echo ">> host RGBD relay -> router $(ZENOH_CONNECT) (needs ROS2 sourced + eclipse-zenoh + opencv)"
	PYTHONPATH="$(CURDIR)/common:$$PYTHONPATH" ZENOH_CONNECT="$(ZENOH_CONNECT)" ROBOT_NAME="$(ROBOT_NAME)" \
	  python3 robot/docker/rgbd_relay.py

rgbd-camera:
	@echo ">> launching realsense2_camera (depth+color, no pointcloud) ..."
	@echo ">> RMW=$${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} DOMAIN=$${ROS_DOMAIN_ID:-0} (MUST match the robot container so the relay can discover it)"
	@echo ">> pinning CycloneDDS to $${RGBD_CAM_IFACE:-eth1} (MUST match the container NIC)"
	CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='$${RGBD_CAM_IFACE:-eth1}'/></Interfaces></General></Domain></CycloneDDS>" \
	RMW_IMPLEMENTATION=$${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} ROS_DOMAIN_ID=$${ROS_DOMAIN_ID:-0} \
	  ros2 launch realsense2_camera rs_launch.py \
	    enable_depth:=true enable_color:=false pointcloud.enable:=false \
	    depth_module.profile:=$${RGBD_CAM_PROFILE:-424x240x15}

# ── Docs ─────────────────────────────────────────────────────────────────────
docs: sync-docs
	uv run mkdocs build

docs-serve: sync-docs
	uv run mkdocs serve

docs-deploy: sync-docs
	uv run mkdocs gh-deploy --force

# ── Housekeeping ─────────────────────────────────────────────────────────────
clean:
	@find . -name __pycache__ -type d -not -path './.venv/*' -exec rm -rf {} + 2>/dev/null || true
	@echo "cleaned __pycache__"
