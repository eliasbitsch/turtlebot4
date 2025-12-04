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
        SharedMemory<ParsedScan> scan_shm(shm_names::SCAN, false);
        SharedMemory<SharedCmdVel> cmd_shm(shm_names::CMD_VEL, false);
        SharedMemory<SharedOdometry> odom_shm(shm_names::ODOM, false);

        std::cout << "[WallFollower] Connected to shared memory segments:" << std::endl;
        std::cout << "  - " << shm_names::SCAN << std::endl;
        std::cout << "  - " << shm_names::ODOM << std::endl;
        std::cout << "  - " << shm_names::CMD_VEL << std::endl;
        std::cout << std::endl;

        // Create wall follower controller
        WallFollower wall_follower(cmd_shm, 3.0, 5.0, 0.5);
        
        std::cout << "[WallFollower] Running... Press Ctrl+C to stop" << std::endl;
        std::cout << std::endl;

        uint64_t loop_count = 0;
        
        // Main control loop
        while (!g_shutdown) {
            // Read latest scan data from shared memory
            scan_shm.read([&wall_follower, &loop_count](const ParsedScan& scan) {
                if (scan.valid && scan.num_ranges > 0) {
                    // Process scan with RANSAC and control
                    process_scan_with_ransac(scan, &wall_follower);
                    
                    // Print status every 20 iterations (~1 second at 20Hz)
                    if (++loop_count % 20 == 0) {
                        std::cout << "\r[Loop " << loop_count << "] Processing scans..." << std::flush;
                    }
                }
            });

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
