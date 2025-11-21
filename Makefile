# Makefile to build linear_control.cpp

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pthread

SRC := linear_control.cpp
OBJ := $(SRC:.cpp=.o)
TARGET := linear_control

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJ) $(TARGET)
