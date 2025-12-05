// Standalone tool to monitor shared memory contents
// Compile: g++ -std=c++17 -I../include -o shm_monitor shm_monitor.cpp -lrt -lpthread

#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>

std::atomic<bool> g_running(true);

void signal_handler(int) { g_running = false; }

double quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::cout << "=== Shared Memory Monitor (Lock-Free) ===" << std::endl;
    std::cout << "Press Ctrl+C to exit\n" << std::endl;

    bool monitor_odom = true, monitor_cmd = true, monitor_scan = true;

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "odom") { monitor_cmd = false; monitor_scan = false; }
        else if (arg == "cmd") { monitor_odom = false; monitor_scan = false; }
        else if (arg == "scan") { monitor_odom = false; monitor_cmd = false; }
    }

    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedOdometry>> odom_shm;
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>> cmd_shm;
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedScan>> scan_shm;

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

    if (monitor_scan) {
        try {
            scan_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedScan>>(
                turtlebot4::shm_names::SCAN, false);
            std::cout << "Opened: " << turtlebot4::shm_names::SCAN << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Cannot open scan shm: " << e.what() << std::endl;
            monitor_scan = false;
        }
    }

    if (!monitor_odom && !monitor_cmd && !monitor_scan) {
        std::cerr << "No shared memory available" << std::endl;
        return 1;
    }

    uint64_t last_odom_seq = 0, last_cmd_seq = 0, last_scan_seq = 0;

    while (g_running) {
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "=== Shared Memory Monitor (Lock-Free) ===\n\n";

        if (monitor_odom && odom_shm) {
            // Zero-Copy read with callback
            odom_shm->read([&](const turtlebot4::SharedOdometry& odom) {
                double yaw = quaternion_to_yaw(odom.orientation_x, odom.orientation_y,
                                               odom.orientation_z, odom.orientation_w);
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "[ODOM] seq=" << odom.sequence;
                if (odom.sequence != last_odom_seq) {
                    std::cout << " (updated)";
                    last_odom_seq = odom.sequence;
                }
                std::cout << "\n  Position:  x=" << std::setw(8) << odom.position_x
                          << "  y=" << std::setw(8) << odom.position_y << "\n"
                          << "  Yaw:       " << std::setw(8) << yaw << " deg\n"
                          << "  Lin Vel:   x=" << std::setw(8) << odom.linear_velocity_x << "\n"
                          << "  Ang Vel:   z=" << std::setw(8) << odom.angular_velocity_z << "\n\n";
            });
        }

        if (monitor_cmd && cmd_shm) {
            // Zero-Copy read with callback
            cmd_shm->read([&](const turtlebot4::SharedCmdVel& cmd) {
                std::cout << "[CMD_VEL] seq=" << cmd.sequence;
                if (cmd.sequence != last_cmd_seq) {
                    std::cout << " (updated)";
                    last_cmd_seq = cmd.sequence;
                }
                std::cout << " new=" << (cmd.new_command ? "yes" : "no") << "\n"
                          << "  Linear:  x=" << std::setw(8) << cmd.linear_x << "\n"
                          << "  Angular: z=" << std::setw(8) << cmd.angular_z << "\n\n";
            });
        }

        if (monitor_scan && scan_shm) {
            scan_shm->read([&](const turtlebot4::SharedScan& scan) {
                std::cout << "[SCAN] seq=" << scan.sequence;
                if (scan.sequence != last_scan_seq) {
                    std::cout << " (updated)";
                    last_scan_seq = scan.sequence;
                }
                std::cout << "\n  Ranges:    " << scan.num_ranges << " points\n"
                          << "  Angle:     " << std::setw(6) << scan.angle_min << " to " << scan.angle_max << " rad\n"
                          << "  Range:     " << std::setw(6) << scan.range_min << " to " << scan.range_max << " m\n";

                // Show some sample ranges
                if (scan.num_ranges > 0) {
                    std::cout << "  Samples:   ";
                    size_t step = scan.num_ranges / 5;
                    for (size_t i = 0; i < scan.num_ranges && i < 5 * step; i += step) {
                        float r = scan.ranges[i];
                        if (r > scan.range_max) std::cout << "inf ";
                        else std::cout << std::setw(5) << r << " ";
                    }
                    std::cout << "\n";

                    // Find min distance
                    float min_dist = scan.range_max + 1;
                    for (size_t i = 0; i < scan.num_ranges; ++i) {
                        if (scan.ranges[i] >= scan.range_min && scan.ranges[i] < min_dist) {
                            min_dist = scan.ranges[i];
                        }
                    }
                    std::cout << "  Min dist:  " << std::setw(6) << min_dist << " m\n";
                }
                std::cout << "\n";
            });
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Exiting..." << std::endl;
    return 0;
}
