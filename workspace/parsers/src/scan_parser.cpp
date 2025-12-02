#include "scan_parser.hpp"
#include <iostream>
#include <algorithm>

namespace turtlebot4 {

ScanParser::ScanParser(ThreadSafeQueue<std::string>& input_queue)
    : ParserBase("ScanParser", input_queue) {
}

ParsedScan ScanParser::parse(const std::string& json_str) {
    ParsedScan result;

    json j = json::parse(json_str);

    // Parse header timestamp
    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    // Parse scan parameters
    result.angle_min = j.value("angle_min", 0.0f);
    result.angle_max = j.value("angle_max", 0.0f);
    result.angle_increment = j.value("angle_increment", 0.0f);
    result.time_increment = j.value("time_increment", 0.0f);
    result.scan_time = j.value("scan_time", 0.0f);
    result.range_min = j.value("range_min", 0.0f);
    result.range_max = j.value("range_max", 0.0f);

    // Parse ranges array
    if (j.contains("ranges") && j["ranges"].is_array()) {
        const auto& ranges = j["ranges"];
        result.num_ranges = std::min(ranges.size(), ParsedScan::MAX_RANGES);

        for (size_t i = 0; i < result.num_ranges; ++i) {
            if (ranges[i].is_number()) {
                result.ranges[i] = ranges[i].get<float>();
            } else {
                // Handle "inf" or null values
                result.ranges[i] = result.range_max + 1.0f; // Mark as invalid
            }
        }
    }

    result.valid = (result.num_ranges > 0);
    return result;
}

} // namespace turtlebot4
