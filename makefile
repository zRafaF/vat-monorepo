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
        sync-mapping sync-client sync-router sync-docs \
        router mapping theta-uvc robot-docker viewer \
        test_link test_frames_robot test_frames_server test_robot_state test_poses \
        docs docs-serve clean

# ── Help / runbook ───────────────────────────────────────────────────────────
help:
	@echo "VAT control file   router=$(ROUTER_IP):$(ROUTER_PORT)   robot=$(ROBOT_NAME)"
	@echo ""
	@echo "Setup:"
	@echo "  make sync-router     isolated env for the Zenoh router"
	@echo "  make sync-mapping    mapping server env (GPU machine, CUDA)"
	@echo "  make sync-client     client + bring-up tools env"
	@echo ""
	@echo "Services:"
	@echo "  make router          [SERVER] run the Zenoh router (hub)"
	@echo "  make mapping         [SERVER] run the PRISM mapping server"
	@echo "  make theta-uvc       [ROBOT]  expose Theta X UVC → /dev/video0 (host)"
	@echo "  make robot-docker    [ROBOT]  bridge + theta_camera + pose fuser container"
	@echo "  make viewer          [CLIENT] full POC viewer (cloud + robot block)"
	@echo ""
	@echo "Staged pre-POC tests (run in this order — see 'make steps'):"
	@echo "  make test_link           0  transport alive (router + bridge + rates)"
	@echo "  make test_frames_robot   1  [ROBOT] preview the Theta UVC directly"
	@echo "  make test_frames_server  1  decimated frames the server ingests"
	@echo "  make test_robot_state    2  body + limb/foot positions, live"
	@echo "  make test_poses          3  camera trajectory + fused robot pose"
	@echo ""
	@echo "Then: make viewer  (Stage 4, the real POC)"

steps:
	@echo "VAT pre-POC → POC runbook  (router=$(ROUTER_IP))"
	@echo "Set up once: SERVER 'make sync-router && make sync-mapping',"
	@echo "             CLIENT 'make sync-client', ROBOT builds the docker image."
	@echo ""
	@echo "Stage 0  Transport"
	@echo "  [SERVER] make router          # the hub; leave it running"
	@echo "  [ROBOT]  make theta-uvc        # Theta X → /dev/video0 (leave running)"
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

sync-router:
	cd server/router && uv sync

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

# [ROBOT] Expose the RICOH Theta X UVC stream as /dev/video0 on the host
# (libuvc-theta gst_loopback). Leave running; the container reads it.
theta-uvc:
	@echo ">> [ROBOT] Theta X UVC → /dev/video0 (leave running in its own shell)"
	bash robot/theta/theta_uvc.sh

# [ROBOT] Container: ROS↔Zenoh bridge + theta_camera + pose fuser.
# Invoked via `bash` so it doesn't depend on the executable bit (git on Windows
# doesn't preserve it). If docker needs root on your robot: `sudo make robot-docker`.
robot-docker:
	@echo ">> [ROBOT] build + run container → router $(ROUTER_IP)"
	bash robot/docker/run.sh $(ROUTER_IP)

# [CLIENT] Full POC viewer.
viewer: sync-client
	@echo ">> Reminder: router + mapping server must be running."
	$(CLIENT_RUN) prism_rerun_viewer.py --snapshot

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
	python3 tools/view_theta.py

# Stage 1b — [CLIENT] the decimated frames the mapping server actually ingests.
test_frames_server: sync-client
	@echo ">> Reminder: router + 'make theta-uvc' + 'make robot-docker' running."
	$(CLIENT_RUN) ../tools/view_frames.py

# Stage 2 — body frame + limb/foot positions, live.
test_robot_state: sync-client
	@echo ">> Reminder: router + robot bridge (publishing sportmodestate) running."
	$(CLIENT_RUN) ../tools/view_robot_state.py

# Stage 3 — camera trajectory + VGGT correction + fused robot pose.
test_poses: sync-client
	@echo ">> Reminder: router + robot container + 'make mapping' running."
	$(CLIENT_RUN) ../tools/view_poses.py

# ── Docs ─────────────────────────────────────────────────────────────────────
docs: sync-docs
	uv run mkdocs build

docs-serve: sync-docs
	uv run mkdocs serve

# ── Housekeeping ─────────────────────────────────────────────────────────────
clean:
	@find . -name __pycache__ -type d -not -path './.venv/*' -exec rm -rf {} + 2>/dev/null || true
	@echo "cleaned __pycache__"
