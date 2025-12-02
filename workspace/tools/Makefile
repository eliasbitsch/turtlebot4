CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I../common/include
LDFLAGS = -lrt -lpthread

BIN_DIR = ../bin

TARGETS = $(BIN_DIR)/shm_monitor $(BIN_DIR)/cmd_vel_writer

.PHONY: all clean

all: $(TARGETS)

$(BIN_DIR)/shm_monitor: shm_monitor.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS)
	@echo "Built: $@"

$(BIN_DIR)/cmd_vel_writer: cmd_vel_writer.cpp
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS)
	@echo "Built: $@"

clean:
	rm -f $(TARGETS)
