#include "message_router.hpp"
#include "parser_manager.hpp"
#include "ransac_processor.hpp" 
#include "wall_follower.hpp"    
#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <cstdlib>
#include <ctime>

using namespace turtlebot4;

std::atomic<bool> g_shutdown(false);

void signal_handler(int signal) {
    (void)signal;
    std::cout << "\nShutdown requested..." << std::endl;
    g_shutdown = true;
}

// Function signature simplified due to 'using namespace turtlebot4;'
void print_stats(MessageRouter& router, ParserManager& parsers) {
    std::cout << "\r[Stats] Scan: " << std::setw(6) << parsers.scan_count()
              << " | Odom: " << std::setw(6) << parsers.odom_count()
              << " | Bumper: " << std::setw(4) << parsers.bumper_count()
              << " | Connected: " << (router.is_running() ? "Yes" : "No ")
              << std::flush;
}

int main() {
    // 1. INITIAL SETUP
    // Initialize random seed once at startup for RANSAC's random sampling algorithm.
    srand(time(0)); 
    
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================" << std::endl;
    std::cout << "  TurtleBot4 ROS Bridge Application    " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    MessageRouter router;
    ParserManager parsers;

    // 2. STARTUP SEQUENCE
    std::cout << "[Main] Starting message router..." << std::endl;
    if (!router.start()) {
        std::cerr << "[Main] Failed to start message router" << std::endl;
        return 1;
    }

    std::cout << "[Main] Initializing parsers..." << std::endl;
    try {
        parsers.init(router.scan_queue(), router.odom_queue(), router.bumper_queue());
    } catch (const std::exception& e) {
        std::cerr << "[Main] Failed to initialize parsers: " << e.what() << std::endl;
        router.stop();
        return 1;
    }
    
    // --- 3. CONTROLLER INSTANTIATION AND RANSAC INTEGRATION ---
    
    // Instantiate the Wall Follower controller, passing it the command shared memory
    WallFollower wall_follower(router.cmd_vel_shm(), 3.0, 5.0, 0.5);
    
    // Create a lambda to capture the wall_follower instance and pass it to the RANSAC processor.
    std::cout << "[Main] Registering RANSAC processor callback..." << std::endl;
    parsers.set_scan_callback([&wall_follower](const ParsedScan& scan) {
        process_scan_with_ransac(scan, &wall_follower);
    });
    
    // -----------------------------------------------------------

    // Set other optional callbacks
    parsers.set_bumper_callback([](const ParsedBumper& b) {
        if (b.is_pressed) {
            std::cout << "\n[BUMPER] Collision detected!" << std::endl;
        }
    });

    // Start parser threads
    std::cout << "[Main] Starting parsers..." << std::endl;
    parsers.start();

    std::cout << std::endl;
    std::cout << "Running... Press Ctrl+C to stop" << std::endl;

    // 4. MAIN APPLICATION LOOP
    int stats_counter = 0;
    while (!g_shutdown && router.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (++stats_counter >= 10) {
            stats_counter = 0;
            print_stats(router, parsers);
        }
    }

    // 5. CLEANUP
    std::cout << std::endl << std::endl;
    std::cout << "[Main] Stopping parsers..." << std::endl;
    parsers.stop();

    std::cout << "[Main] Stopping router..." << std::endl;
    router.stop();

    std::cout << "[Main] Shutdown complete" << std::endl;
    return 0;
}