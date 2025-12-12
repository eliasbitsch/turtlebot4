# Linear Controller and Waypoint Manager for TurtleBot4

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -lrt -lpthread

TARGETS = linear_controller waypoint_manager
DEPS = ../include/shared_memory.hpp ../include/datatypes.hpp

.PHONY: all clean

all: $(TARGETS)

linear_controller: linear_controller.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) linear_controller.cpp -o $@ $(LDFLAGS)

waypoint_manager: waypoint_manager.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) waypoint_manager.cpp -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGETS)
