#include "bumper_parser.hpp"
#include <iostream>

namespace turtlebot4 {

BumperParser::BumperParser(ThreadSafeQueue<std::string>& input_queue)
    : ParserBase("BumperParser", input_queue) {
}

ParsedBumper BumperParser::parse(const std::string& json_str) {
    ParsedBumper result;

    json j = json::parse(json_str);

    // Parse header timestamp
    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    // TurtleBot4 bumper message structure may vary
    // Common formats:
    // 1. kobuki_ros_interfaces/msg/BumperEvent: bumper (uint8), state (uint8)
    // 2. irobot_create_msgs/msg/HazardDetection: type (uint8)

    // Try kobuki format first
    if (j.contains("state")) {
        // state: 0 = RELEASED, 1 = PRESSED
        result.is_pressed = (j["state"].get<int>() == 1);
        result.valid = true;
    }
    // Try irobot_create format
    else if (j.contains("type")) {
        // Check if type indicates bump
        int type = j["type"].get<int>();
        result.is_pressed = (type == 1); // BUMP type
        result.valid = true;
    }
    // Try simple bool format
    else if (j.contains("is_pressed")) {
        result.is_pressed = j["is_pressed"].get<bool>();
        result.valid = true;
    }
    // Try detections array (HazardDetectionVector)
    else if (j.contains("detections") && j["detections"].is_array()) {
        for (const auto& detection : j["detections"]) {
            if (detection.contains("type") && detection["type"].get<int>() == 1) {
                result.is_pressed = true;
                break;
            }
        }
        result.valid = true;
    }

    return result;
}

} // namespace turtlebot4
