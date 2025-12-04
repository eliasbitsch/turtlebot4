.PHONY: all build clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -pipe
SRCS := waypoints.cpp
BIN_DIR := bin
TARGET := $(BIN_DIR)/waypoints
RM := rm -f

all: build

build:
    @echo "[BUILD] waypoints"
    @mkdir -p $(BIN_DIR)
    $(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
    @echo "[OK] $(TARGET)"

run: build
    @echo "[RUN] $(TARGET)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] waypoints"
    $(RM) $(TARGET)

help:
    @echo "waypoints Makefile"
    @echo "Targets: all, build, run, clean, help"
```# filepath: /home/philip/turtlebot4/workspace/waypoints/Makefile
.PHONY: all build clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -pipe
SRCS := waypoints.cpp
BIN_DIR := bin
TARGET := $(BIN_DIR)/waypoints
RM := rm -f

all: build

build:
    @echo "[BUILD] waypoints"
    @mkdir -p $(BIN_DIR)
    $(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
    @echo "[OK] $(TARGET)"

run: build
    @echo "[RUN] $(TARGET)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] waypoints"
    $(RM) $(TARGET)

help:
    @echo "waypoints Makefile"
    @echo "Targets: all, build, run, clean, help"