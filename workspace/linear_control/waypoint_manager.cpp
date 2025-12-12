// Waypoint Manager for TurtleBot4
// Sends a sequence of waypoints to /shm_goal for the linear controller
//
// Usage:
//   ./waypoint_manager                    # use built-in demo waypoints
//   ./waypoint_manager waypoints.txt      # load waypoints from file
//
// File format (one waypoint per line):
//   x y theta
//   1.0 0.0 0.0
//   1.0 1.0 1.57
//   0.0 0.0 0.0

#include "../include/shared_memory.hpp"
#include "../include/datatypes.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running(true);

void signal_handler(int) {
    std::cout << "\nStopping...\n";
    g_running = false;
}

struct Waypoint {
    double x;
    double y;
    double theta;
};

// Load waypoints from file
std::vector<Waypoint> loadWaypointsFromFile(const std::string& filename) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << "\n";
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        Waypoint wp;
        if (iss >> wp.x >> wp.y >> wp.theta) {
            waypoints.push_back(wp);
        }
    }

    return waypoints;
}

// Demo waypoints (square pattern)
std::vector<Waypoint> getDemoWaypoints() {
    return {
        {1.0, 0.0, 0.0},        // Forward 1m
        {1.0, 1.0, M_PI/2},     // Turn left, go to (1,1)
        {0.0, 1.0, M_PI},       // Turn left, go to (0,1)
        {0.0, 0.0, 0.0}         // Return to origin
    };
}

void printWaypoints(const std::vector<Waypoint>& waypoints) {
    std::cout << "\nWaypoints:\n";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << (i+1) << ": (" << waypoints[i].x << ", "
                  << waypoints[i].y << ", " << waypoints[i].theta << ")\n";
    }
    std::cout << "\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Waypoint Manager ===\n";

    // Load waypoints
    std::vector<Waypoint> waypoints;
    if (argc > 1) {
        std::cout << "Loading waypoints from: " << argv[1] << "\n";
        waypoints = loadWaypointsFromFile(argv[1]);
        if (waypoints.empty()) {
            std::cerr << "No waypoints loaded!\n";
            return 1;
        }
    } else {
        std::cout << "Using demo waypoints (square pattern)\n";
        waypoints = getDemoWaypoints();
    }

    printWaypoints(waypoints);

    // Open shared memory for goal
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedGoal>> goal_shm;

    try {
        goal_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedGoal>>(
            turtlebot4::shm_names::GOAL, true);  // create=true
        std::cout << "Created/Opened: " << turtlebot4::shm_names::GOAL << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Cannot create/open goal shm: " << e.what() << "\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nPress Ctrl+C to stop\n";
    std::cout << "Make sure linear_controller is running!\n\n";

    // Process waypoints
    uint32_t goal_id = 0;

    for (size_t i = 0; i < waypoints.size() && g_running; ++i) {
        const Waypoint& wp = waypoints[i];
        goal_id++;

        std::cout << "[" << (i+1) << "/" << waypoints.size() << "] Sending goal: ("
                  << wp.x << ", " << wp.y << ", " << wp.theta << ")\n";

        // Write goal to shared memory
        goal_shm->update([&](turtlebot4::SharedGoal& goal) {
            goal.x = wp.x;
            goal.y = wp.y;
            goal.theta = wp.theta;
            goal.active = true;
            goal.reached = false;
            goal.goal_id = goal_id;
            goal.sequence++;
            auto now = std::chrono::system_clock::now().time_since_epoch();
            goal.timestamp_sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
            goal.timestamp_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
        });

        // Wait for goal to be reached
        std::cout << "  Waiting for goal to be reached...\n";
        while (g_running) {
            bool reached = false;
            uint32_t current_id = 0;

            goal_shm->read([&](const turtlebot4::SharedGoal& goal) {
                reached = goal.reached;
                current_id = goal.goal_id;
            });

            if (reached && current_id == goal_id) {
                std::cout << "  Goal " << goal_id << " reached!\n\n";
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (g_running) {
        std::cout << "All waypoints completed!\n";
    }

    // Deactivate goal
    goal_shm->update([](turtlebot4::SharedGoal& goal) {
        goal.active = false;
    });

    std::cout << "Waypoint manager stopped.\n";
    return 0;
}
