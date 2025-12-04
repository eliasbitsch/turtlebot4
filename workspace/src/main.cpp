#include "rosbridge.hpp"
#include "parsers.hpp"
#include <csignal>
#include <iomanip>

std::atomic<bool> g_shutdown(false);

void signal_handler(int) {
    std::cout << "\nShutdown...\n";
    g_shutdown = true;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Default: TurtleBot4 robot IP, or override via command line argument
    std::string uri = "ws://192.168.100.100:9090";
    if (argc > 1) {
        uri = "ws://" + std::string(argv[1]) + ":9090";
    }

    std::cout << "=== TurtleBot4 Bridge ===\n";
    std::cout << "Connecting to: " << uri << "\n\n";

    // Start bridge (WebSocket + Queues)
    turtlebot4::RobotBridge bridge(uri);
    if (!bridge.start()) {
        std::cerr << "Failed to start bridge\n";
        return 1;
    }

    // Start parsers (3 threads)
    // Note: Using joint_states instead of odom due to TRANSIENT_LOCAL QoS issues
    turtlebot4::Parsers parsers(
        bridge.scan_queue(),
        bridge.joints_queue(),
        bridge.bumper_queue()
    );

    parsers.on_bumper([](const turtlebot4::ParsedBumper& b) {
        if (b.is_pressed) std::cout << "\n[BUMPER] Hit!\n";
    });

    parsers.start();

    std::cout << "Shared Memory:\n";
    std::cout << "  " << turtlebot4::shm_names::ODOM << "\n";
    std::cout << "  " << turtlebot4::shm_names::CMD_VEL << "\n";
    std::cout << "\nRunning... Ctrl+C to stop\n\n";

    // Main loop
    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "\r[Stats] scan:" << std::setw(5) << parsers.scan_count()
                  << " odom:" << std::setw(5) << parsers.odom_count()
                  << " bumper:" << std::setw(3) << parsers.bumper_count()
                  << "   " << std::flush;
    }

    std::cout << "\n\nStopping...\n";
    parsers.stop();
    bridge.stop();

    return 0;
}
