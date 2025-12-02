#include "odom_parser.hpp"
#include <iostream>

namespace turtlebot4 {

OdomParser::OdomParser(ThreadSafeQueue<std::string>& input_queue)
    : ParserBase("OdomParser", input_queue), odom_shm_(nullptr), sequence_(0) {
}

OdomParser::~OdomParser() {
    stop();
}

void OdomParser::attach_shared_memory(SharedMemory<SharedOdometry>* shm) {
    odom_shm_ = shm;
}

ParsedOdom OdomParser::parse(const std::string& json_str) {
    ParsedOdom result;

    json j = json::parse(json_str);

    // Parse header timestamp
    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    // Parse pose
    if (j.contains("pose") && j["pose"].contains("pose")) {
        const auto& pose = j["pose"]["pose"];

        if (pose.contains("position")) {
            result.position_x = pose["position"].value("x", 0.0);
            result.position_y = pose["position"].value("y", 0.0);
            result.position_z = pose["position"].value("z", 0.0);
        }

        if (pose.contains("orientation")) {
            result.orientation_x = pose["orientation"].value("x", 0.0);
            result.orientation_y = pose["orientation"].value("y", 0.0);
            result.orientation_z = pose["orientation"].value("z", 0.0);
            result.orientation_w = pose["orientation"].value("w", 1.0);
        }
    }

    // Parse twist
    if (j.contains("twist") && j["twist"].contains("twist")) {
        const auto& twist = j["twist"]["twist"];

        if (twist.contains("linear")) {
            result.linear_vel_x = twist["linear"].value("x", 0.0);
            result.linear_vel_y = twist["linear"].value("y", 0.0);
            result.linear_vel_z = twist["linear"].value("z", 0.0);
        }

        if (twist.contains("angular")) {
            result.angular_vel_x = twist["angular"].value("x", 0.0);
            result.angular_vel_y = twist["angular"].value("y", 0.0);
            result.angular_vel_z = twist["angular"].value("z", 0.0);
        }
    }

    result.valid = true;
    return result;
}

void OdomParser::on_parsed(const ParsedOdom& data) {
    // Write to shared memory if attached
    if (odom_shm_) {
        odom_shm_->update([&data, this](SharedOdometry& shm) {
            shm.position_x = data.position_x;
            shm.position_y = data.position_y;
            shm.position_z = data.position_z;

            shm.orientation_x = data.orientation_x;
            shm.orientation_y = data.orientation_y;
            shm.orientation_z = data.orientation_z;
            shm.orientation_w = data.orientation_w;

            shm.linear_velocity_x = data.linear_vel_x;
            shm.linear_velocity_y = data.linear_vel_y;
            shm.linear_velocity_z = data.linear_vel_z;

            shm.angular_velocity_x = data.angular_vel_x;
            shm.angular_velocity_y = data.angular_vel_y;
            shm.angular_velocity_z = data.angular_vel_z;

            shm.timestamp_sec = data.stamp_sec;
            shm.timestamp_nanosec = data.stamp_nanosec;

            shm.sequence = ++sequence_;
        });
    }

    // Call parent callback if set
    ParserBase::on_parsed(data);
}

} // namespace turtlebot4
