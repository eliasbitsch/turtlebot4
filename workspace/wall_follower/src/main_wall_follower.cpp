// Standalone Wall Follower - Reads scan from shared memory
// NOTE: Requires scan data to be written to shared memory by the main bridge
//
// This is a placeholder/demo. The elias_dev branch currently doesn't write
// scan data to shared memory - only odom and cmd_vel are shared.
// 
// To use this, you need to:
// 1. Add ParsedScan shared memory support to the main bridge parsers
// 2. OR integrate this directly into the main bridge application

#include "ransac_processor.hpp" 
#include "wall_follower.hpp"
#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <ctime>

using namespace turtlebot4;

std::atomic<bool> g_shutdown(false);

void signal_handler(int signal) {
    (void)signal;
    std::cout << "\nShutdown requested..." << std::endl;
    g_shutdown = true;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Wall Follower Demo (STANDALONE)      " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "NOTE: This demo requires scan data in shared memory." << std::endl;
    std::cout << "      The current bridge doesn't provide this yet." << std::endl;
    std::cout << std::endl;
    std::cout << "To use wall follower, integrate it into the main" << std::endl;
    std::cout << "bridge application's scan callback instead." << std::endl;
    std::cout << std::endl;
    
    // Initialize random seed for RANSAC
    srand(time(0)); 
    
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Open shared memory for commands only
        SharedMemory<SharedCmdVel> cmd_shm(shm_names::CMD_VEL, false);
        
        std::cout << "[WallFollower] Connected to cmd_vel shared memory" << std::endl;
        std::cout << "  - " << shm_names::CMD_VEL << std::endl;
        std::cout << std::endl;

        // Create wall follower controller
        WallFollower wall_follower(cmd_shm, 3.0, 5.0, 0.5);
        
        std::cout << "[WallFollower] Demo mode - sending test commands" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;

        // Demo: send a simple command
        RansacResult test_result;
        test_result.perpendicular_distance = 0.5;
        test_result.angle_error_rad = 0.0;
        test_result.front_clearance = 2.0;
        
        int count = 0;
        while (!g_shutdown && count < 5) {
            wall_follower.computeAndCommand(test_result);
            std::cout << "Sent test command " << (count + 1) << "/5" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            count++;
        }
        
        std::cout << std::endl;
        std::cout << "[WallFollower] Stopping..." << std::endl;
        wall_follower.emergencyStop();

    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::cerr << "[Error] Make sure the main bridge is running first!" << std::endl;
        return 1;
    }

    std::cout << "[WallFollower] Demo complete" << std::endl;
    return 0;
}
