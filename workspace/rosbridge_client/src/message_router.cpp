#include "message_router.hpp"
#include <iostream>
#include <chrono>

namespace turtlebot4 {

MessageRouter::MessageRouter()
    : running_(false), last_cmd_vel_seq_(0) {
}

MessageRouter::~MessageRouter() {
    stop();
}

bool MessageRouter::start() {
    if (running_) return true;

    // Create shared memory for cmd_vel
    try {
        cmd_vel_shm_ = std::make_unique<SharedMemory<SharedCmdVel>>(shm_names::CMD_VEL, true);
        std::cout << "[MessageRouter] Created shared memory: " << shm_names::CMD_VEL << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[MessageRouter] Failed to create cmd_vel shared memory: " << e.what() << std::endl;
        return false;
    }

    // Create rosbridge client
    client_ = std::make_unique<RosbridgeClient>("ws://localhost:9090");

    // Register subscription callbacks (these just forward JSON to queues)
    client_->subscribe(topics::SCAN, [this](const std::string& topic, const json& msg) {
        on_scan_message(topic, msg);
    });

    client_->subscribe(topics::ODOM, [this](const std::string& topic, const json& msg) {
        on_odom_message(topic, msg);
    });

    client_->subscribe(topics::BUMPER, [this](const std::string& topic, const json& msg) {
        on_bumper_message(topic, msg);
    });

    running_ = true;

    // Start rosbridge thread
    rosbridge_thread_ = std::thread(&MessageRouter::rosbridge_thread_func, this);

    // Start cmd_vel publisher thread
    cmd_vel_thread_ = std::thread(&MessageRouter::cmd_vel_publisher_thread_func, this);

    std::cout << "[MessageRouter] Started" << std::endl;
    return true;
}

void MessageRouter::stop() {
    if (!running_) return;

    running_ = false;

    // Shutdown queues to unblock waiting threads
    scan_queue_.shutdown();
    odom_queue_.shutdown();
    bumper_queue_.shutdown();

    // Stop rosbridge client
    if (client_) {
        client_->stop();
    }

    // Join threads
    if (rosbridge_thread_.joinable()) {
        rosbridge_thread_.join();
    }
    if (cmd_vel_thread_.joinable()) {
        cmd_vel_thread_.join();
    }

    client_.reset();
    cmd_vel_shm_.reset();

    std::cout << "[MessageRouter] Stopped" << std::endl;
}

void MessageRouter::rosbridge_thread_func() {
    std::cout << "[MessageRouter] Rosbridge thread started" << std::endl;

    while (running_) {
        if (!client_->connect()) {
            std::cerr << "[MessageRouter] Failed to connect, retrying in 2 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        // Advertise cmd_vel topic
        client_->advertise(topics::CMD_VEL, "geometry_msgs/msg/TwistStamped");

        // Run blocks until connection closes
        client_->run();

        if (running_) {
            std::cout << "[MessageRouter] Connection lost, reconnecting..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::cout << "[MessageRouter] Rosbridge thread stopped" << std::endl;
}

void MessageRouter::cmd_vel_publisher_thread_func() {
    std::cout << "[MessageRouter] CmdVel publisher thread started" << std::endl;

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz

        if (!client_ || !client_->is_connected()) continue;

        SharedCmdVel cmd = cmd_vel_shm_->read();

        // Only publish if there's a new command
        if (cmd.new_command && cmd.sequence != last_cmd_vel_seq_) {
            last_cmd_vel_seq_ = cmd.sequence;

            json twist_msg = {
                {"header", {
                    {"stamp", {
                        {"sec", cmd.timestamp_sec},
                        {"nanosec", cmd.timestamp_nanosec}
                    }},
                    {"frame_id", ""}
                }},
                {"twist", {
                    {"linear", {
                        {"x", cmd.linear_x},
                        {"y", cmd.linear_y},
                        {"z", cmd.linear_z}
                    }},
                    {"angular", {
                        {"x", cmd.angular_x},
                        {"y", cmd.angular_y},
                        {"z", cmd.angular_z}
                    }}
                }}
            };

            client_->publish(topics::CMD_VEL, "geometry_msgs/msg/TwistStamped", twist_msg);

            // Clear new_command flag
            cmd_vel_shm_->update([](SharedCmdVel& c) {
                c.new_command = false;
            });
        }
    }

    std::cout << "[MessageRouter] CmdVel publisher thread stopped" << std::endl;
}

void MessageRouter::on_scan_message(const std::string& topic, const json& msg) {
    (void)topic;
    scan_queue_.push(msg.dump());
}

void MessageRouter::on_odom_message(const std::string& topic, const json& msg) {
    (void)topic;
    odom_queue_.push(msg.dump());
}

void MessageRouter::on_bumper_message(const std::string& topic, const json& msg) {
    (void)topic;
    bumper_queue_.push(msg.dump());
}

} // namespace turtlebot4
