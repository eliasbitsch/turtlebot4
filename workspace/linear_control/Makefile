# Linear Controller for TurtleBot4

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -lrt -lpthread

TARGET = linear_controller

.PHONY: all clean

all: $(TARGET)

$(TARGET): linear_controller.cpp ../include/shared_memory.hpp ../include/datatypes.hpp
	$(CXX) $(CXXFLAGS) linear_controller.cpp -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET)
