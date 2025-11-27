#include "message_router.hpp"
#include "parser_manager.hpp"
#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>

std::atomic<bool> g_shutdown(false);

void signal_handler(int signal) {
    (void)signal;
    std::cout << "\nShutdown requested..." << std::endl;
    g_shutdown = true;
}

void print_stats(turtlebot4::MessageRouter& router, turtlebot4::ParserManager& parsers) {
    std::cout << "\r[Stats] Scan: " << std::setw(6) << parsers.scan_count()
              << " | Odom: " << std::setw(6) << parsers.odom_count()
              << " | Bumper: " << std::setw(4) << parsers.bumper_count()
              << " | Connected: " << (router.is_running() ? "Yes" : "No ")
              << std::flush;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================" << std::endl;
    std::cout << "  TurtleBot4 ROS Bridge Application    " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Create message router (handles websocket connection)
    turtlebot4::MessageRouter router;

    // Create parser manager
    turtlebot4::ParserManager parsers;

    // Start router first
    std::cout << "[Main] Starting message router..." << std::endl;
    if (!router.start()) {
        std::cerr << "[Main] Failed to start message router" << std::endl;
        return 1;
    }

    // Initialize parsers with router's queues
    std::cout << "[Main] Initializing parsers..." << std::endl;
    try {
        parsers.init(router.scan_queue(), router.odom_queue(), router.bumper_queue());
    } catch (const std::exception& e) {
        std::cerr << "[Main] Failed to initialize parsers: " << e.what() << std::endl;
        router.stop();
        return 1;
    }

    // Set debug callbacks (optional)
    parsers.set_bumper_callback([](const turtlebot4::ParsedBumper& b) {
        if (b.is_pressed) {
            std::cout << "\n[BUMPER] Collision detected!" << std::endl;
        }
    });

    // Start parsers
    std::cout << "[Main] Starting parsers..." << std::endl;
    parsers.start();

    std::cout << std::endl;
    std::cout << "Shared Memory Regions:" << std::endl;
    std::cout << "  - " << turtlebot4::shm_names::ODOM << " (Odometry data)" << std::endl;
    std::cout << "  - " << turtlebot4::shm_names::CMD_VEL << " (Command velocity)" << std::endl;
    std::cout << std::endl;
    std::cout << "Running... Press Ctrl+C to stop" << std::endl;
    std::cout << std::endl;

    // Main loop with stats
    int stats_counter = 0;
    while (!g_shutdown && router.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Print stats every second
        if (++stats_counter >= 10) {
            stats_counter = 0;
            print_stats(router, parsers);
        }
    }

    std::cout << std::endl << std::endl;

    // Cleanup
    std::cout << "[Main] Stopping parsers..." << std::endl;
    parsers.stop();

    std::cout << "[Main] Stopping router..." << std::endl;
    router.stop();

    std::cout << "[Main] Shutdown complete" << std::endl;
    return 0;
}
