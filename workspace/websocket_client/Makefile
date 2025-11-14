CXX = g++
CXXFLAGS = -std=c++17 -Wall
LDFLAGS = -lboost_system -lpthread

all: websocket_client

websocket_client: src/main.cpp
	$(CXX) $(CXXFLAGS) src/main.cpp -o websocket_client $(LDFLAGS)

clean:
	rm -f websocket_client
