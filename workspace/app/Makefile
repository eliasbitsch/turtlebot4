CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I../common/include -I../rosbridge_client/include -I../parsers/include
LDFLAGS = -L../bin -lparsers -lboost_system -lpthread -lrt

SRC_DIR = .
OBJ_DIR = obj
BIN_DIR = ../bin

TARGET = $(BIN_DIR)/turtlebot4_bridge

SOURCES = main.cpp \
          ../rosbridge_client/src/rosbridge_client.cpp \
          ../rosbridge_client/src/message_router.cpp

OBJECTS = $(OBJ_DIR)/main.o \
          $(OBJ_DIR)/rosbridge_client.o \
          $(OBJ_DIR)/message_router.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJECTS) $(BIN_DIR)/libparsers.a
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Built: $(TARGET)"

$(OBJ_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/rosbridge_client.o: ../rosbridge_client/src/rosbridge_client.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/message_router.o: ../rosbridge_client/src/message_router.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(TARGET)
