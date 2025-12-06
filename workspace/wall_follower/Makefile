# Wall Follower with RANSAC - Build Configuration

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I./include -I../include
LDFLAGS = -lrt -lpthread

BUILD = build
OBJ = $(BUILD)/obj
TARGET = $(BUILD)/wall_follower

# Source files
SRCS = src/main.cpp src/wall_follower.cpp src/ransac_processor.cpp
OBJS = $(OBJ)/main.o $(OBJ)/wall_follower.o $(OBJ)/ransac_processor.o

.PHONY: all clean help test_visual

all: $(TARGET)
	@echo "Built: $(TARGET)"

$(TARGET): $(OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJS) -o $@ $(LDFLAGS)

test_visual: test_ransac_visual.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $(BUILD)/test_visual $(LDFLAGS)
	@echo "Built: $(BUILD)/test_visual"

$(OBJ)/%.o: src/%.cpp | $(OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD):
	@mkdir -p $(BUILD)

$(OBJ):
	@mkdir -p $(OBJ)

clean:
	rm -rf $(BUILD)

help:
	@echo "Targets:"
	@echo "  all    - Build wall follower (default)"
	@echo "  clean  - Remove build artifacts"
	@echo ""
	@echo "Dependencies: shared_memory.hpp, datatypes.hpp from parent include/"
