// Test: Parsers + SharedMemory ohne WebSocket
// Simuliert eingehende JSON-Nachrichten (joint_states für odom)

#include "parsers.hpp"
#include "thread_safe_queue.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main() {
    std::cout << "=== Parser + SharedMemory Test ===\n\n";

    // Queues (normalerweise von RobotBridge befüllt)
    turtlebot4::ThreadSafeQueue<std::string> scan_q, joints_q, bumper_q;

    // Parsers starten (now using joint_states for odom)
    turtlebot4::Parsers parsers(scan_q, joints_q, bumper_q);

    parsers.on_scan([](const turtlebot4::ParsedScan& s) {
        std::cout << "[SCAN] " << s.num_ranges << " ranges, min=" << s.range_min << "\n";
    });

    parsers.on_odom([](const turtlebot4::ParsedOdom& o) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[ODOM] x=" << o.position_x << " y=" << o.position_y
                  << " vx=" << o.linear_vel_x << " wz=" << o.angular_vel_z << "\n";
    });

    parsers.on_bumper([](const turtlebot4::ParsedBumper& b) {
        std::cout << "[BUMPER] pressed=" << (b.is_pressed ? "YES" : "no") << "\n";
    });

    parsers.start();

    // Simuliere eingehende JSON-Nachrichten
    std::cout << "Sending test messages...\n\n";

    // Test JointStates - simulate wheel movement
    // Format: TurtleBot4's /joint_states message
    joints_q.push(R"({
        "header": {"stamp": {"sec": 100, "nanosec": 0}},
        "name": ["wheel_drop_left_joint", "left_wheel_joint", "wheel_drop_right_joint", "right_wheel_joint"],
        "position": [0.0, 0.0, 0.0, 0.0]
    })");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Simulate forward motion: both wheels move 1 radian (about 0.036m)
    joints_q.push(R"({
        "header": {"stamp": {"sec": 100, "nanosec": 100000000}},
        "name": ["wheel_drop_left_joint", "left_wheel_joint", "wheel_drop_right_joint", "right_wheel_joint"],
        "position": [0.0, 1.0, 0.0, 1.0]
    })");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Simulate turn: right wheel moves more than left
    joints_q.push(R"({
        "header": {"stamp": {"sec": 100, "nanosec": 200000000}},
        "name": ["wheel_drop_left_joint", "left_wheel_joint", "wheel_drop_right_joint", "right_wheel_joint"],
        "position": [0.0, 1.5, 0.0, 2.0]
    })");

    // Test Scan
    scan_q.push(R"({
        "header": {"stamp": {"sec": 123, "nanosec": 789}},
        "angle_min": -1.57,
        "angle_max": 1.57,
        "range_min": 0.1,
        "range_max": 10.0,
        "ranges": [1.0, 1.1, 1.2, 0.5, 0.6, 2.0, 3.0, 4.0]
    })");

    // Test Bumper (HazardDetection format)
    bumper_q.push(R"({"detections": [{"type": 1}], "header": {"stamp": {"sec": 125}}})");
    bumper_q.push(R"({"detections": []})");

    // Warten bis Parser fertig
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "\n--- Stats ---\n";
    std::cout << "Scan:   " << parsers.scan_count() << " messages\n";
    std::cout << "Odom:   " << parsers.odom_count() << " messages (from joint_states)\n";
    std::cout << "Bumper: " << parsers.bumper_count() << " messages\n";

    // Test SharedMemory lesen (Zero-Copy!)
    std::cout << "\n--- SharedMemory (Zero-Copy Read) ---\n";
    parsers.odom_shm().read([](const turtlebot4::SharedOdometry& o) {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Odom from SHM:\n";
        std::cout << "  Position: x=" << o.position_x << " y=" << o.position_y << "\n";
        std::cout << "  Velocity: vx=" << o.linear_velocity_x << " wz=" << o.angular_velocity_z << "\n";
        std::cout << "  Sequence: " << o.sequence << "\n";
    });

    parsers.stop();

    std::cout << "\nTest complete!\n";
    return 0;
}
