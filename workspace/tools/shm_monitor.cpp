// Standalone tool to monitor shared memory contents
// Compile: g++ -std=c++17 -o shm_monitor shm_monitor.cpp -lrt -lpthread

#include "../common/include/shared_memory.hpp"
#include "../common/include/datatypes.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>
#include <atomic>

std::atomic<bool> g_running(true);

void signal_handler(int) {
    g_running = false;
}

double quaternion_to_yaw(double x, double y, double z, double w) {
    // Convert quaternion to yaw (rotation around Z axis)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::cout << "=== Shared Memory Monitor ===" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl << std::endl;

    bool monitor_odom = true;
    bool monitor_cmd = true;

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "odom") monitor_cmd = false;
        else if (arg == "cmd") monitor_odom = false;
    }

    // Open shared memories (not as owner)
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedOdometry>> odom_shm;
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>> cmd_shm;

    if (monitor_odom) {
        try {
            odom_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedOdometry>>(
                turtlebot4::shm_names::ODOM, false);
            std::cout << "Opened: " << turtlebot4::shm_names::ODOM << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Cannot open odom shm: " << e.what() << std::endl;
            monitor_odom = false;
        }
    }

    if (monitor_cmd) {
        try {
            cmd_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>>(
                turtlebot4::shm_names::CMD_VEL, false);
            std::cout << "Opened: " << turtlebot4::shm_names::CMD_VEL << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Cannot open cmd_vel shm: " << e.what() << std::endl;
            monitor_cmd = false;
        }
    }

    if (!monitor_odom && !monitor_cmd) {
        std::cerr << "No shared memory available to monitor" << std::endl;
        return 1;
    }

    std::cout << std::endl;

    uint64_t last_odom_seq = 0;
    uint64_t last_cmd_seq = 0;

    while (g_running) {
        std::cout << "\033[2J\033[H"; // Clear screen

        std::cout << "=== Shared Memory Monitor ===" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl << std::endl;

        if (monitor_odom && odom_shm) {
            auto odom = odom_shm->read();
            double yaw = quaternion_to_yaw(odom.orientation_x, odom.orientation_y,
                                           odom.orientation_z, odom.orientation_w);

            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[ODOM] seq=" << odom.sequence;
            if (odom.sequence != last_odom_seq) {
                std::cout << " (updated)";
                last_odom_seq = odom.sequence;
            }
            std::cout << std::endl;
            std::cout << "  Position:  x=" << std::setw(8) << odom.position_x
                      << "  y=" << std::setw(8) << odom.position_y
                      << "  z=" << std::setw(8) << odom.position_z << std::endl;
            std::cout << "  Yaw:       " << std::setw(8) << yaw << " deg" << std::endl;
            std::cout << "  Lin Vel:   x=" << std::setw(8) << odom.linear_velocity_x
                      << "  y=" << std::setw(8) << odom.linear_velocity_y << std::endl;
            std::cout << "  Ang Vel:   z=" << std::setw(8) << odom.angular_velocity_z << std::endl;
            std::cout << std::endl;
        }

        if (monitor_cmd && cmd_shm) {
            auto cmd = cmd_shm->read();

            std::cout << "[CMD_VEL] seq=" << cmd.sequence;
            if (cmd.sequence != last_cmd_seq) {
                std::cout << " (updated)";
                last_cmd_seq = cmd.sequence;
            }
            std::cout << " new_cmd=" << (cmd.new_command ? "yes" : "no") << std::endl;
            std::cout << "  Linear:    x=" << std::setw(8) << cmd.linear_x
                      << "  y=" << std::setw(8) << cmd.linear_y
                      << "  z=" << std::setw(8) << cmd.linear_z << std::endl;
            std::cout << "  Angular:   x=" << std::setw(8) << cmd.angular_x
                      << "  y=" << std::setw(8) << cmd.angular_y
                      << "  z=" << std::setw(8) << cmd.angular_z << std::endl;
            std::cout << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Exiting..." << std::endl;
    return 0;
}
