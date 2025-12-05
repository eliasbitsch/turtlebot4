.PHONY: all clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -I.
SRCS := waypoints.cpp
HDRS := waypoints.h
BIN_DIR := bin
OBJ_DIR := build
OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(notdir $(SRCS)))
TARGET := $(BIN_DIR)/waypoints

# Try to pick up ROS compile/link flags via pkg-config (optional)
ROS_CFLAGS := $(shell pkg-config --cflags roscpp 2>/dev/null || echo)
ROS_LIBS   := $(shell pkg-config --libs roscpp 2>/dev/null || echo)

all: $(TARGET)

$(TARGET): $(OBJS) | $(BIN_DIR)
    @echo "[LINK] $@"
    $(CXX) $(CXXFLAGS) $(ROS_CFLAGS) $(OBJS) -o $@ $(ROS_LIBS)
    @echo "[OK] $@"

$(OBJ_DIR)/%.o: %.cpp $(HDRS) | $(OBJ_DIR)
    @echo "[COMPILE] $<"
    $(CXX) $(CXXFLAGS) $(ROS_CFLAGS) -c $< -o $@

$(BIN_DIR):
    mkdir -p $(BIN_DIR)

$(OBJ_DIR):
    mkdir -p $(OBJ_DIR)

run: all
    @echo "[RUN] $(TARGET)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] waypoints"
    rm -rf $(OBJ_DIR) $(BIN_DIR)

help:
    @echo "Makefile for building waypoints (waypoints.cpp / waypoints.h)"
    @echo ""
    @echo "Targets:"
    @echo "  all (default)  - build binary"
    @echo "  run            - build and run"
    @echo "  clean          - remove build artifacts"
    @echo "  help           - this message"
```# filepath: /home/philip/turtlebot4/workspace/waypoints/Makefile
.PHONY: all clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -I.
SRCS := waypoints.cpp
HDRS := waypoints.h
BIN_DIR := bin
OBJ_DIR := build
OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(notdir $(SRCS)))
TARGET := $(BIN_DIR)/waypoints

# Try to pick up ROS compile/link flags via pkg-config (optional)
ROS_CFLAGS := $(shell pkg-config --cflags roscpp 2>/dev/null || echo)
ROS_LIBS   := $(shell pkg-config --libs roscpp 2>/dev/null || echo)

all: $(TARGET)

$(TARGET): $(OBJS) | $(BIN_DIR)
    @echo "[LINK] $@"
    $(CXX) $(CXXFLAGS) $(ROS_CFLAGS) $(OBJS) -o $@ $(ROS_LIBS)
    @echo "[OK] $@"

$(OBJ_DIR)/%.o: %.cpp $(HDRS) | $(OBJ_DIR)
    @echo "[COMPILE] $<"
    $(CXX) $(CXXFLAGS) $(ROS_CFLAGS) -c $< -o $@

$(BIN_DIR):
    mkdir -p $(BIN_DIR)

$(OBJ_DIR):
    mkdir -p $(OBJ_DIR)

run: all
    @echo "[RUN] $(TARGET)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] waypoints"
    rm -rf $(OBJ_DIR) $(BIN_DIR)

help:
    @echo "Makefile for building waypoints (waypoints.cpp / waypoints.h)"
    @echo ""
    @echo "Targets:"
    @echo "  all (default)  - build binary"
    @echo "  run            - build and run"
    @echo "  clean          - remove build artifacts"
    @echo "  help           - this message"