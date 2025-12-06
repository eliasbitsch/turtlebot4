// Standalone Wall Follower - Reads scan from shared memory
// Runs as a separate process, reading LIDAR data and writing velocity commands

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
    // Initialize random seed for RANSAC
    srand(time(0)); 
    
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================" << std::endl;
    std::cout << "  TurtleBot4 Wall Follower (RANSAC)    " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    try {
        // Open shared memory segments (created by main bridge)
        SharedMemory<SharedScan> scan_shm(shm_names::SCAN, false);
        SharedMemory<SharedCmdVel> cmd_shm(shm_names::CMD_VEL, false);

        std::cout << "[WallFollower] Connected to shared memory segments:" << std::endl;
        std::cout << "  - " << shm_names::SCAN << std::endl;
        std::cout << "  - " << shm_names::CMD_VEL << std::endl;
        std::cout << std::endl;

        // Create wall follower controller with proportional gains
        // Using simple proportional control like linear_controller for stability
        WallFollower wall_follower(cmd_shm, 
            2.0,   // kp_dist: proportional gain for distance error
            0.0,   // ki_dist: no integral (set to 0)
            0.0,   // kd_dist: no derivative (set to 0)
            3.0,   // kp_angle: proportional gain for angle error  
            0.0,   // ki_angle: no integral (set to 0)
            0.0);  // kd_angle: no derivative (set to 0)
        // Wall following distance is set by DESIRED_WALL_DISTANCE constant in wall_follower.hpp
        
        std::cout << "[WallFollower] Running... Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;

        uint64_t loop_count = 0;
        uint64_t last_seq = 0;
        
        // Main control loop
        while (!g_shutdown) {
            // Read latest scan data from shared memory
            SharedScan scan = scan_shm.read_copy();
            
            // Skip if no new data
            if (scan.sequence == last_seq || scan.num_ranges == 0 || scan.angle_increment <= 0.0f) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            last_seq = scan.sequence;
            
            // Process scan with RANSAC and control
            process_scan_with_ransac(scan, &wall_follower);
            loop_count++;

            // Sleep to match typical scan rate (~20-50 Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cout << std::endl;
        std::cout << "[WallFollower] Stopping..." << std::endl;
        
        // Send stop command
        wall_follower.emergencyStop();

    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::cerr << "[Error] Make sure the main bridge is running first!" << std::endl;
        return 1;
    }

    std::cout << "[WallFollower] Shutdown complete" << std::endl;
    return 0;
}
