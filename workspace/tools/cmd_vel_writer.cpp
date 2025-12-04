// Standalone tool to write cmd_vel to shared memory (simulates controller)
// Compile: g++ -std=c++17 -I../include -o cmd_vel_writer cmd_vel_writer.cpp -lrt -lpthread

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

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    if (argc < 2) {
        std::cout << "Usage: cmd_vel_writer <linear_x> <angular_z>\n"
                  << "       cmd_vel_writer circle   (figure-8)\n"
                  << "       cmd_vel_writer stop     (zero velocity)\n";
        return 1;
    }

    std::string mode = argv[1];
    double linear_x = 0.0, angular_z = 0.0;
    bool circle_mode = false;

    if (mode == "circle") {
        circle_mode = true;
        linear_x = 0.15;
        angular_z = 0.5;
    } else if (mode == "stop") {
        // Keep zeros
    } else if (argc >= 3) {
        linear_x = std::stod(argv[1]);
        angular_z = std::stod(argv[2]);
    } else {
        std::cerr << "Invalid arguments\n";
        return 1;
    }

    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>> cmd_shm;

    try {
        cmd_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>>(
            turtlebot4::shm_names::CMD_VEL, false);
        std::cout << "Opened: " << turtlebot4::shm_names::CMD_VEL << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Cannot open cmd_vel shm: " << e.what() << "\n"
                  << "Make sure turtlebot4_bridge is running!\n";
        return 1;
    }

    std::cout << "Sending commands... Ctrl+C to stop\n";

    uint64_t seq = 0;
    double time = 0.0;

    while (g_running) {
        double vx = linear_x;
        double wz = circle_mode ? angular_z * std::sin(time * 0.5) : angular_z;

        // Lock-Free write
        cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
            cmd.linear_x = vx;
            cmd.linear_y = 0.0;
            cmd.linear_z = 0.0;
            cmd.angular_x = 0.0;
            cmd.angular_y = 0.0;
            cmd.angular_z = wz;
            cmd.sequence = ++seq;
            cmd.new_command = true;

            auto now = std::chrono::system_clock::now().time_since_epoch();
            cmd.timestamp_sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
            cmd.timestamp_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
        });

        std::cout << "\rSent: vx=" << std::fixed << std::setprecision(3) << vx
                  << " wz=" << wz << " seq=" << seq << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        time += 0.05;
    }

    // Stop command
    cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
        cmd.linear_x = 0.0;
        cmd.angular_z = 0.0;
        cmd.sequence = ++seq;
        cmd.new_command = true;
    });

    std::cout << "\nSent stop command\n";
    return 0;
}
