#include "message_router.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

std::atomic<bool> g_shutdown(false);

void signal_handler(int signal) {
    (void)signal;
    std::cout << "\nShutdown requested..." << std::endl;
    g_shutdown = true;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== TurtleBot4 Rosbridge Client ===" << std::endl;

    turtlebot4::MessageRouter router;

    if (!router.start()) {
        std::cerr << "Failed to start message router" << std::endl;
        return 1;
    }

    std::cout << "Running... Press Ctrl+C to stop" << std::endl;

    while (!g_shutdown && router.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    router.stop();

    std::cout << "Shutdown complete" << std::endl;
    return 0;
}
