# Configuration
PROTO_DIR := proto
OUT_DIR := proto
PROTO_FILES := $(wildcard $(PROTO_DIR)/*.proto)

# OS Detection for Clean Command
ifeq ($(OS),Windows_NT)
    # Windows
    CLEAN_CMD := del /Q *_pb2.py
else
    # Linux / macOS
    CLEAN_CMD := rm -f *_pb2.py
endif

.PHONY: all build clean \
        sync sync-server sync-client sync-docs \
        docs docs-serve

all: build

# ── uv workspace ────────────────────────────────────────────────────────────
# Install all workspace members (server + client + doc deps)
sync:
	uv sync --all-groups

# Install only the server package (heavy CUDA deps — run on the GPU machine)
sync-server:
	uv sync --package vat-server

# Install only the client package (lightweight — Rerun viewer)
sync-client:
	uv sync --package vat-client

# Install docs dev-deps and serve locally
sync-docs:
	uv sync --group docs

# ── Documentation ───────────────────────────────────────────────────────────
docs: sync-docs
	uv run mkdocs build

docs-serve: sync-docs
	uv run mkdocs serve

build:
	@echo "Compiling Protobuf files from $(PROTO_DIR)..."
	protoc -I=$(PROTO_DIR) --python_out=$(OUT_DIR) $(PROTO_FILES)
	@echo "Done! Generated Python files in $(OUT_DIR)"

clean:
	@echo "Cleaning generated files..."
	-$(CLEAN_CMD)
	@echo "Clean complete."