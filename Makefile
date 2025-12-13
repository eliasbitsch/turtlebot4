CXX = g++
CXXFLAGS = -O2 -Wall

SRC = src
BIN = bin

all: core mapping

core:
	$(CXX) $(CXXFLAGS) $(SRC)/core.cpp -o $(BIN)/core

mapping:
	$(CXX) $(CXXFLAGS) $(SRC)/mapping.cpp -o $(BIN)/mapping

clean:
	rm -f $(BIN)/*
