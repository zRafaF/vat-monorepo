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

.PHONY: all build clean

all: build

build:
	@echo "Compiling Protobuf files from $(PROTO_DIR)..."
	protoc -I=$(PROTO_DIR) --python_out=$(OUT_DIR) $(PROTO_FILES)
	@echo "Done! Generated Python files in $(OUT_DIR)"

clean:
	@echo "Cleaning generated files..."
	-$(CLEAN_CMD)
	@echo "Clean complete."