CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -fPIC
INCLUDES = -I./include

SRC_DIR = src
OBJ_DIR = obj
LIB_DIR = ../bin

TARGET = $(LIB_DIR)/libcommon.a

# Header-only library, but we create a placeholder for consistency
HEADERS = $(wildcard include/*.hpp)

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(LIB_DIR)

# Create empty static library (headers-only, but keeps build system consistent)
$(TARGET): $(HEADERS)
	@echo "Common module is header-only"
	@touch $(TARGET)

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(TARGET)
