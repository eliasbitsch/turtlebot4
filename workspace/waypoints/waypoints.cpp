#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <sstream>
#include <algorithm>

using websocketpp::connection_hdl;
typedef websocketpp::client<websocketpp::config::asio_client> WebSocketClient;
std::mutex gridMutex;

// -------------------- Roboterposition --------------------
struct Pose {
    double x, y, theta; // Position + Richtung
};

// -------------------- Grid --------------------
#define GRID_WIDTH 50
#define GRID_HEIGHT 50

struct Grid {
    int width = GRID_WIDTH;
    int height = GRID_HEIGHT;
    int cells[GRID_WIDTH][GRID_HEIGHT]{0}; // 0 = frei, 1 = Hindernis
};

Grid grid;

// -------------------- Start und Ziel --------------------
Pose start;
Pose goal;
std::vector<Pose> path;

void initStartGoal() {
    start = {0, 0, 0};
    goal  = {45, 45, 0};
}

// -------------------- Hilfsfunktionen --------------------
double heuristic(int x1, int y1, int x2, int y2){
    return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool isFree(int x, int y){
    return x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT && grid.cells[x][y] == 0;
}

// -------------------- A* Pfadplanung --------------------
struct Node {
    int x, y;
    double g, h;
    Node* parent;
};

int computeAStarPath(Pose start, Pose goal, std::vector<Pose>& path){
    std::lock_guard<std::mutex> lock(gridMutex);
    path.clear();

    bool closed[GRID_WIDTH][GRID_HEIGHT]{false};
    std::vector<Node*> openList;

    Node* startNode = new Node{(int)start.x, (int)start.y, 0.0,
                               heuristic((int)start.x,(int)start.y,(int)goal.x,(int)goal.y),
                               nullptr};
    openList.push_back(startNode);
    Node* goalNode = nullptr;

    int dx[8] = {1,1,0,-1,-1,-1,0,1};
    int dy[8] = {0,1,1,1,0,-1,-1,-1};

    while(!openList.empty()){
        // Knoten mit kleinstem f = g + h wählen
        size_t bestIdx = 0;
        double bestF = openList[0]->g + openList[0]->h;
        for(size_t i=1; i<openList.size(); i++){
            double f = openList[i]->g + openList[i]->h;
            if(f < bestF){ bestF=f; bestIdx=i; }
        }
        Node* current = openList[bestIdx];
        openList.erase(openList.begin()+bestIdx);

        if(closed[current->x][current->y]) { delete current; continue; }
        closed[current->x][current->y] = true;

        if(current->x == (int)goal.x && current->y == (int)goal.y){
            goalNode = current;
            break;
        }

        for(int i=0; i<8; i++){
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];
            if(!isFree(nx, ny) || closed[nx][ny]) continue;

            double cost = (i % 2 == 0) ? 1.0 : 1.414; // diagonal teuerer
            Node* neighbor = new Node{nx, ny, current->g + cost,
                                      heuristic(nx, ny, (int)goal.x, (int)goal.y),
                                      current};
            openList.push_back(neighbor);
        }
    }

    if(!goalNode){
        std::cerr << "Pfad nicht gefunden!\n";
        for(auto n : openList) delete n;
        return 0;
    }

    // Pfad zurückverfolgen
    Node* n = goalNode;
    std::vector<Node*> nodes;
    while(n){ nodes.push_back(n); n=n->parent; }
    std::reverse(nodes.begin(), nodes.end());

    for(size_t i=0; i<nodes.size(); i++){
        double theta=0;
        if(i < nodes.size()-1){
            double dx = nodes[i+1]->x - nodes[i]->x;
            double dy = nodes[i+1]->y - nodes[i]->y;
            theta = std::atan2(dy, dx);
        } else if(i>0){
            double dx = nodes[i]->x - nodes[i-1]->x;
            double dy = nodes[i]->y - nodes[i-1]->y;
            theta = std::atan2(dy, dx);
        }
        path.push_back({(double)nodes[i]->x,(double)nodes[i]->y,theta});
    }

    for(auto n : openList) delete n;
    return path.size();
}

// -------------------- WebSocket --------------------
std::string createCmdVelMsg(double vx, double vy, double vz, double ax, double ay, double az){
    std::ostringstream oss;
    oss << R"({"op":"publish","topic":"/cmd_vel","msg":{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":""},"twist":{"linear":{"x":)"
        << vx << R"(,"y":)" << vy << R"(,"z":)" << vz << R"(},"angular":{"x":)"
        << ax << R"(,"y":)" << ay << R"(,"z":)" << az << R"(}}}})";
    return oss.str();
}

// -------------------- Nachricht empfangen --------------------
void onMessage(WebSocketClient* c, connection_hdl hdl, WebSocketClient::message_ptr msg){
    std::string payload = msg->get_payload();
    // Hier könnte man Sensoren auswerten
}

// -------------------- Main --------------------
int main() {
    initStartGoal();

    WebSocketClient c;
    c.init_asio();
    c.clear_access_channels(websocketpp::log::alevel::all);
    c.clear_error_channels(websocketpp::log::elevel::all);
    c.set_message_handler(std::bind(&onMessage,&c,std::placeholders::_1,std::placeholders::_2));

    std::string uri = "ws://localhost:9090";
    websocketpp::lib::error_code ec;
    auto con = c.get_connection(uri, ec);
    if(ec){ std::cerr<<"Verbindung fehlgeschlagen: "<<ec.message()<<std::endl; return 1; }
    c.connect(con);

    std::thread wsThread([&]{ c.run(); });

    while(true){
        int len = computeAStarPath(start, goal, path);
        if(len > 0){
            for(auto &p : path){
                std::string cmd = createCmdVelMsg(0.2,0,0,0,0,p.theta);
                c.send(con->get_handle(), cmd, websocketpp::frame::opcode::text);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    wsThread.join();
    return 0;
}
