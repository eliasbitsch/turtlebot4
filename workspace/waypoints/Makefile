CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.
TARGET = waypoints

.PHONY: all clean

all: $(TARGET)

$(TARGET): waypoints.cpp waypoints.h
    $(CXX) $(CXXFLAGS) waypoints.cpp -o $@

clean:
    rm -f $(TARGET)
```// filepath: /home/philip/turtlebot4/workspace/waypoints/Makefile
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.
TARGET = waypoints

.PHONY: all clean

all: $(TARGET)

$(TARGET): waypoints.cpp waypoints.h
    $(CXX) $(CXXFLAGS) waypoints.cpp -o $@

clean:
    rm -f $(TARGET)