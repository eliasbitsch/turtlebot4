#ifndef RANSAC_PROCESSOR_HPP
#define RANSAC_PROCESSOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "datatypes.hpp" 
#include "wall_follower.hpp" 

namespace turtlebot4 {

// --- RANSAC Model Structures (Internal) ---
struct Point {
    double x; // Cartesian x-coordinate (meters)
    double y; // Cartesian y-coordinate (meters)
};

struct Line {
    double m; // slope (tan(angle))
    double c; // y-intercept
};

/**
 * @brief Processes a laser scan message, runs RANSAC for line fitting, and uses the result 
 * to command the WallFollower.
 * @param raw_scan_data The ParsedScan object received from the ScanParser.
 * @param follower A pointer to the active WallFollower instance used to execute commands.
 */
void process_scan_with_ransac(const ParsedScan& raw_scan_data, WallFollower* follower);

} // namespace turtlebot4

#endif // RANSAC_PROCESSOR_HPP