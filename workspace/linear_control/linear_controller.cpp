// Linear Controller for TurtleBot4
// Reads odometry from shared memory, computes control, writes cmd_vel to shared memory
// The turtlebot4_bridge forwards cmd_vel to the robot via rosbridge
//
// Usage:
//   ./linear_controller <goal_x> <goal_y> <goal_theta>
//   ./linear_controller 1.0 0.0 0.0    # drive 1m forward
//   ./linear_controller 0.0 0.0 1.57   # rotate 90 degrees

#include "../include/shared_memory.hpp"
#include "../include/datatypes.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <algorithm>

std::atomic<bool> g_running(true);

void signal_handler(int) {
    std::cout << "\nStopping...\n";
    g_running = false;
}

// Normalize angle to [-pi, pi]
double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Convert quaternion to yaw angle
double quaternionToYaw(double x, double y, double z, double w) {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Compute control output
void computeControl(double current_x, double current_y, double current_theta,
                    double goal_x, double goal_y, double goal_theta,
                    double kp_linear, double k_alpha, double k_beta,
                    double pos_threshold, double ang_threshold,
                    double &out_linear, double &out_angular, bool &reached) {
    double dx = goal_x - current_x;
    double dy = goal_y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double angle_to_goal = std::atan2(dy, dx);
    double alpha = normalizeAngle(angle_to_goal - current_theta);
    double beta = normalizeAngle(goal_theta - current_theta - alpha);

    out_linear = 0.0;
    out_angular = 0.0;
    reached = false;

    // Check if goal reached
    double angle_error = normalizeAngle(goal_theta - current_theta);
    if (distance < pos_threshold && std::fabs(angle_error) < ang_threshold) {
        reached = true;
        return;
    }

    // Move toward goal position if not close enough
    if (distance >= pos_threshold) {
        out_linear = kp_linear * distance * std::cos(alpha);
        out_angular = (k_alpha * alpha) + (k_beta * beta);
    } else {
        // At position, rotate to final orientation
        // Use simple proportional control on angle error
        double angle_error = normalizeAngle(goal_theta - current_theta);
        out_linear = 0.0;
        out_angular = 2.0 * angle_error;  // Simple P-control for final rotation
    }

    // Limit speeds (TurtleBot4 limits)
    const double MAX_LINEAR = 0.22;
    const double MAX_ANGULAR = 2.84;
    out_linear = std::clamp(out_linear, -MAX_LINEAR, MAX_LINEAR);
    out_angular = std::clamp(out_angular, -MAX_ANGULAR, MAX_ANGULAR);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse arguments
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <goal_x> <goal_y> <goal_theta>\n";
        std::cout << "Examples:\n";
        std::cout << "  " << argv[0] << " 1.0 0.0 0.0    # drive 1m forward\n";
        std::cout << "  " << argv[0] << " 0.0 0.0 1.57   # rotate 90 degrees left\n";
        std::cout << "  " << argv[0] << " 1.0 1.0 0.0    # drive to (1,1)\n";
        return 1;
    }

    double goal_x = std::stod(argv[1]);
    double goal_y = std::stod(argv[2]);
    double goal_theta = std::stod(argv[3]);

    // Control gains (tuned for smoother driving)
    double kp_linear = 1.5;   // was 3.0 - reduced for less aggressive acceleration
    double k_alpha = 3.0;     // was 8.0 - main cause of oscillation, reduced significantly
    double k_beta = -0.8;     // was -1.5 - reduced for smoother final orientation
    double pos_threshold = 0.05;
    double ang_threshold = 0.1;

    // Open shared memory
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedOdometry>> odom_shm;
    std::unique_ptr<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>> cmd_shm;

    try {
        odom_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedOdometry>>(
            turtlebot4::shm_names::ODOM, false);
        std::cout << "Opened: " << turtlebot4::shm_names::ODOM << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Cannot open odom shm: " << e.what() << "\n";
        std::cerr << "Make sure turtlebot4_bridge is running!\n";
        return 1;
    }

    try {
        cmd_shm = std::make_unique<turtlebot4::SharedMemory<turtlebot4::SharedCmdVel>>(
            turtlebot4::shm_names::CMD_VEL, false);
        std::cout << "Opened: " << turtlebot4::shm_names::CMD_VEL << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Cannot open cmd_vel shm: " << e.what() << "\n";
        std::cerr << "Make sure turtlebot4_bridge is running!\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nGoal: (" << goal_x << ", " << goal_y << ", " << goal_theta << ")\n";
    std::cout << "Press Ctrl+C to stop\n\n";

    // Wait for first odom reading
    std::cout << "Waiting for odometry data...\n";
    uint64_t last_seq = 0;
    int wait_count = 0;
    while (g_running && wait_count < 50) {
        uint64_t seq = odom_shm->sequence();
        if (seq > 0 && seq != last_seq) {
            std::cout << "Got odometry!\n\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }
    if (wait_count >= 50) {
        std::cerr << "Timeout waiting for odometry. Is the bridge receiving data?\n";
        return 1;
    }

    // Control loop
    uint64_t cmd_seq = 0;
    const auto dt = std::chrono::milliseconds(50);  // 20 Hz control rate

    while (g_running) {
        // Read current pose from shared memory
        double x = 0, y = 0, theta = 0;
        odom_shm->read([&](const turtlebot4::SharedOdometry& odom) {
            x = odom.position_x;
            y = odom.position_y;
            theta = quaternionToYaw(odom.orientation_x, odom.orientation_y,
                                    odom.orientation_z, odom.orientation_w);
        });

        // Compute control
        double v = 0, w = 0;
        bool reached = false;
        computeControl(x, y, theta, goal_x, goal_y, goal_theta,
                       kp_linear, k_alpha, k_beta, pos_threshold, ang_threshold,
                       v, w, reached);

        // Write cmd_vel to shared memory
        cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
            cmd.linear_x = v;
            cmd.linear_y = 0.0;
            cmd.linear_z = 0.0;
            cmd.angular_x = 0.0;
            cmd.angular_y = 0.0;
            cmd.angular_z = w;
            cmd.sequence = ++cmd_seq;
            cmd.new_command = true;

            auto now = std::chrono::system_clock::now().time_since_epoch();
            cmd.timestamp_sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
            cmd.timestamp_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
        });

        // Print status
        double dx = goal_x - x;
        double dy = goal_y - y;
        double dist = std::sqrt(dx * dx + dy * dy);

        // Also read sequence to see if odom is updating
        uint64_t odom_seq = odom_shm->sequence();

        std::cout << "\rpose=(" << std::setw(6) << x << ", " << std::setw(6) << y
                  << ", " << std::setw(5) << theta << ") "
                  << "v=" << std::setw(5) << v << " w=" << std::setw(5) << w
                  << " dist=" << std::setw(5) << dist
                  << " seq=" << odom_seq << "   " << std::flush;

        if (reached) {
            std::cout << "\n\nGoal reached!\n";
            break;
        }

        // Debug: print more info every 20 iterations
        static int dbg_cnt = 0;
        if (++dbg_cnt % 20 == 0) {
            double angle_to_goal = std::atan2(goal_y - y, goal_x - x);
            double alpha = normalizeAngle(angle_to_goal - theta);
            double beta = normalizeAngle(goal_theta - theta - alpha);
            std::cout << "\n  [DBG] alpha=" << alpha << " beta=" << beta
                      << " pos_thr=0.05 ang_thr=0.1\n";
        }

        std::this_thread::sleep_for(dt);
    }

    // Send stop command
    cmd_shm->update([&](turtlebot4::SharedCmdVel& cmd) {
        cmd.linear_x = 0.0;
        cmd.linear_y = 0.0;
        cmd.linear_z = 0.0;
        cmd.angular_x = 0.0;
        cmd.angular_y = 0.0;
        cmd.angular_z = 0.0;
        cmd.sequence = ++cmd_seq;
        cmd.new_command = true;
    });

    std::cout << "Sent stop command\n";
    return 0;
}
