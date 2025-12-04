.PHONY: all build clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -pipe
SRCS := read_control.cpp
BIN_DIR := bin
TARGET := $(BIN_DIR)/read_control
RM := rm -f

all: build

build:
    @echo "[BUILD] xbox_control"
    @mkdir -p $(BIN_DIR)
    $(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
    @echo "[OK] $(TARGET)"

run: build
    @echo "Note: ensure read permission for /dev/input/js0 (sudo or udev rule)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] xbox_control"
    $(RM) $(TARGET)

help:
    @echo "xbox_control Makefile"
    @echo "Targets: all, build, run, clean, help"
```# filepath: /home/philip/turtlebot4/workspace/xbox_control/Makefile
.PHONY: all build clean run help

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -pipe
SRCS := read_control.cpp
BIN_DIR := bin
TARGET := $(BIN_DIR)/read_control
RM := rm -f

all: build

build:
    @echo "[BUILD] xbox_control"
    @mkdir -p $(BIN_DIR)
    $(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
    @echo "[OK] $(TARGET)"

run: build
    @echo "Note: ensure read permission for /dev/input/js0 (sudo or udev rule)"
    ./$(TARGET)

clean:
    @echo "[CLEAN] xbox_control"
    $(RM) $(TARGET)

help:
    @echo "xbox_control Makefile"
    @echo "Targets: all, build, run, clean, help"