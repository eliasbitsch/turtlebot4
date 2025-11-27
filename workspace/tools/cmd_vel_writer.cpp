// Standalone tool to write cmd_vel to shared memory (simulates controller)
// Compile: g++ -std=c++17 -o cmd_vel_writer cmd_vel_writer.cpp -lrt -lpthread

#include "../common/include/shared_memory.hpp"
#include "../common/include/datatypes.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>

std::atomic<bool> g_running(true);

void signal_handler(int) {
    g_running = false;
}

void print_usage() {
    std::cout << "Usage: cmd_vel_writer <linear_x> <angular_z>" << std::endl;
    std::cout << "       cmd_vel_writer circle   (drive in circle)" << std::endl;
    std::cout << "       cmd_vel_writer stop     (send zero velocity)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example: cmd_vel_writer 0.2 0.0  (forward at 0.2 m/s)" << std::endl;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    if (argc < 2) {
        print_usage();
        return 1;
    }

    std::string mode = argv[1];
    double linear_x = 0.0;
    double angular_z = 0.0;
    bool circle_mode = false;

    if (mode == "circle") {
        circle_mode = true;
        linear_x = 0.15;
        angular_z = 0.5;
    } else if (mode == "stop") {
        linear_x = 0.0;
        angular_z = 0.0;
    } else if (argc >= 3) {
        linear_x = std::stod(argv[1]);
        angular_z = std::stod(argv[2]);
    } else {
        print_usage();
        return 1;
    }

    // Open shared memory (not as owner)
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>> cmd_shm;

    try {
        cmd_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>>(
            turtlebot4::shm_names::CMD_VEL, false);
        std::cout << "Opened: " << turtlebot4::shm_names::CMD_VEL << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Cannot open cmd_vel shm: " << e.what() << std::endl;
        std::cerr << "Make sure turtlebot4_bridge is running first!" << std::endl;
        return 1;
    }

    std::cout << "Sending commands... Press Ctrl+C to stop" << std::endl;

    uint64_t seq = 0;
    double time = 0.0;

    while (g_running) {
        double vx = linear_x;
        double wz = angular_z;

        // In circle mode, vary the angular velocity for a figure-8
        if (circle_mode) {
            wz = angular_z * std::sin(time * 0.5);
        }

        cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
            cmd.linear_x = vx;
            cmd.linear_y = 0.0;
            cmd.linear_z = 0.0;
            cmd.angular_x = 0.0;
            cmd.angular_y = 0.0;
            cmd.angular_z = wz;
            cmd.sequence = ++seq;
            cmd.new_command = true;

            auto now = std::chrono::system_clock::now();
            auto epoch = now.time_since_epoch();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
            cmd.timestamp_sec = static_cast<int32_t>(secs.count());
            cmd.timestamp_nanosec = static_cast<uint32_t>(nsecs.count());
        });

        std::cout << "\rSent: linear_x=" << std::fixed << std::setprecision(3) << vx
                  << " angular_z=" << wz << " seq=" << seq << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
        time += 0.05;
    }

    // Send stop command
    cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
        cmd.linear_x = 0.0;
        cmd.angular_z = 0.0;
        cmd.sequence = ++seq;
        cmd.new_command = true;
    });

    std::cout << std::endl << "Sent stop command" << std::endl;
    return 0;
}
