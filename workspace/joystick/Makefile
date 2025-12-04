# Standalone Joystick -> Shared Memory Controller

CXX = g++
CXXFLAGS = -Wall -Wextra -O2

# Target
TARGET = joy_shm

.PHONY: all clean

all: $(TARGET)

$(TARGET): joy_shm.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ -lrt

clean:
	rm -f $(TARGET)
