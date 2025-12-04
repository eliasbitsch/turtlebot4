# TurtleBot4 ROS Bridge - Simplified Build

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I./include
LDFLAGS = -lboost_system -lpthread -lrt

BUILD = build
TARGET = $(BUILD)/turtlebot4_bridge

.PHONY: all clean tools help

all: $(TARGET)
	@echo "Built: $(TARGET)"

$(TARGET): src/main.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS)

$(BUILD):
	@mkdir -p $(BUILD)

# Tools
tools: $(BUILD)/shm_monitor $(BUILD)/cmd_vel_writer $(BUILD)/test_parsers
	@echo "Built tools"

$(BUILD)/shm_monitor: tools/shm_monitor.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ -lrt -lpthread

$(BUILD)/cmd_vel_writer: tools/cmd_vel_writer.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ -lrt -lpthread

$(BUILD)/test_parsers: tools/test_parsers.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ -lrt -lpthread

clean:
	rm -rf $(BUILD)

help:
	@echo "Targets:"
	@echo "  all    - Build main app (default)"
	@echo "  tools  - Build shm_monitor + cmd_vel_writer"
	@echo "  clean  - Remove build artifacts"
	@echo ""
	@echo "Dependencies: libboost-all-dev nlohmann-json3-dev"
