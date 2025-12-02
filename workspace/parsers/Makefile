CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -fPIC
INCLUDES = -I./include -I../common/include
LDFLAGS = -lpthread -lrt

SRC_DIR = src
OBJ_DIR = obj
LIB_DIR = ../bin

SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
TARGET = $(LIB_DIR)/libparsers.a

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(LIB_DIR)

$(TARGET): $(OBJECTS)
	ar rcs $(TARGET) $(OBJECTS)
	@echo "Built: $(TARGET)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(TARGET)
