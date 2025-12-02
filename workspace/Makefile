# Master Makefile for TurtleBot4 ROS Bridge
# Builds all modules in correct order

.PHONY: all clean common rosbridge_client parsers app tools help

# Build order matters: common -> parsers -> app
all: common parsers app tools
	@echo ""
	@echo "=========================================="
	@echo "  Build complete!"
	@echo "=========================================="
	@echo "  Main app:    bin/turtlebot4_bridge"
	@echo "  Tools:       bin/shm_monitor"
	@echo "               bin/cmd_vel_writer"
	@echo "=========================================="

common:
	@echo "[BUILD] common"
	$(MAKE) -C common

parsers: common
	@echo "[BUILD] parsers"
	$(MAKE) -C parsers

rosbridge_client: common
	@echo "[BUILD] rosbridge_client"
	$(MAKE) -C rosbridge_client

app: common parsers
	@echo "[BUILD] app"
	$(MAKE) -C app

tools: common
	@echo "[BUILD] tools"
	$(MAKE) -C tools

clean:
	@echo "[CLEAN] all modules"
	$(MAKE) -C common clean
	$(MAKE) -C parsers clean
	$(MAKE) -C rosbridge_client clean
	$(MAKE) -C app clean
	$(MAKE) -C tools clean
	rm -rf bin/*.a bin/turtlebot4_bridge bin/rosbridge_client

help:
	@echo "TurtleBot4 ROS Bridge Build System"
	@echo ""
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all              - Build all modules (default)"
	@echo "  common           - Build common library (datatypes, shared memory)"
	@echo "  parsers          - Build parser library"
	@echo "  rosbridge_client - Build standalone rosbridge client"
	@echo "  app              - Build main application"
	@echo "  tools            - Build utility tools (shm_monitor, cmd_vel_writer)"
	@echo "  clean            - Clean all build artifacts"
	@echo "  help             - Show this help message"
	@echo ""
	@echo "Dependencies:"
	@echo "  - libboost-dev (for websocketpp)"
	@echo "  - nlohmann-json3-dev"
	@echo ""
	@echo "Install on Ubuntu/Debian:"
	@echo "  sudo apt install libboost-all-dev nlohmann-json3-dev"
