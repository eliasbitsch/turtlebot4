CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
# FIX: Added -I. for main.cpp
# FIX: Added -I./include to find ransac_processor.hpp and wall_follower.hpp
# FIX: Added system includes for robustness (Boost, WebSocketpp)
INCLUDES = -I../common/include -I../rosbridge_client/include -I../parsers/include -I. -I./include -I/usr/include -I/usr/local/include
LDFLAGS = -L../bin -lparsers -lboost_system -lpthread -lrt

OBJ_DIR = obj
BIN_DIR = ../bin

TARGET = $(BIN_DIR)/turtlebot4_bridge

# UPDATED: Explicitly lists sources, using 'src/' for the new components based on your structure.
SOURCES = main.cpp \
          src/ransac_processor.cpp \
          src/wall_follower.cpp \
          ../rosbridge_client/src/rosbridge_client.cpp \
          ../rosbridge_client/src/message_router.cpp

# Define OBJECTS corresponding to SOURCES
OBJECTS = $(OBJ_DIR)/main.o \
          $(OBJ_DIR)/ransac_processor.o \
          $(OBJ_DIR)/wall_follower.o \
          $(OBJ_DIR)/rosbridge_client.o \
          $(OBJ_DIR)/message_router.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJECTS) $(BIN_DIR)/libparsers.a
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Built: $(TARGET)"

# Rule for main.cpp (in the root app directory)
$(OBJ_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# NEW FIX: Rule to compile ransac_processor.cpp from the 'src' subdirectory
$(OBJ_DIR)/ransac_processor.o: src/ransac_processor.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# NEW FIX: Rule to compile wall_follower.cpp from the 'src' subdirectory
$(OBJ_DIR)/wall_follower.o: src/wall_follower.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@
	
# Rule for external source files (rosbridge client)
$(OBJ_DIR)/rosbridge_client.o: ../rosbridge_client/src/rosbridge_client.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/message_router.o: ../rosbridge_client/src/message_router.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(TARGET)