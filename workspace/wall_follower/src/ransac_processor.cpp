#include "ransac_processor.hpp" // Includes the function declaration and RANSAC structs
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <numeric>

// Ensure M_PI is defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace turtlebot4 {

// --- RANSAC Configuration (Optimization parameters) ---
static const int MAX_RANSAC_ITERATIONS = 500; 
static const double DISTANCE_THRESHOLD = 0.05; // 5 cm tolerance for a point to be an inlier

// --- Utility Functions ---

int random_int(int min, int max) {
    return min + (rand() % (max - min + 1));
}

double distance_squared(const Point& p, const Line& line) {
    double numerator = line.m * p.x - p.y + line.c;
    return (numerator * numerator) / (line.m * line.m + 1.0); //distance = |mx₀ - y₀ + c| / √(m² + 1)
}

// --- Data Conversion Function ---

std::vector<Point> convert_scan_to_points(const SharedScan& scan) {
    std::vector<Point> data;
    
    if (scan.num_ranges == 0 || scan.angle_increment <= 0.0f) {
        return data;
    }

    for (size_t i = 0; i < scan.num_ranges; ++i) {
        float range = scan.ranges[i];
        
        if (range < scan.range_min || range > scan.range_max || std::isnan(range) || range == 0.0f) {
            continue; 
        }
        
        double angle = scan.angle_min + i * scan.angle_increment;
        
        Point p;
        p.x = range * std::cos(angle);
        p.y = range * std::sin(angle);
        
        data.push_back(p);
    }
    return data;
}

// --- RANSAC Core Function ---

Line ransac_fit_line(const std::vector<Point>& data, int max_iterations, double distance_threshold, size_t& out_inliers) {
    if (data.size() < 2) {
        out_inliers = 0;
        return {0.0, 0.0};
    }

    double threshold_sq = distance_threshold * distance_threshold;
    Line best_line = {0.0, 0.0};
    size_t max_inliers = 0;

    for (int i = 0; i < max_iterations; ++i) {
        int idx1 = random_int(0, data.size() - 1);
        int idx2 = random_int(0, data.size() - 1);
        while (idx1 == idx2) { idx2 = random_int(0, data.size() - 1); }

        const Point& p1 = data[idx1];
        const Point& p2 = data[idx2];

        Line current_line;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        if (std::abs(dx) < 1e-6) { continue; } 

        current_line.m = dy / dx;
        current_line.c = p1.y - current_line.m * p1.x;

        size_t current_inliers = 0;
        for (const auto& p : data) {
            if (distance_squared(p, current_line) < threshold_sq) {
                current_inliers++;
            }
        }

        if (current_inliers > max_inliers) {
            max_inliers = current_inliers;
            
            // --- Refit (Least Squares) ---
            std::vector<Point> inliers;
            for (const auto& p : data) {
                 if (distance_squared(p, current_line) < threshold_sq) {
                    inliers.push_back(p);
                }
            }

            if (inliers.size() >= 2) {
                double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
                size_t N = inliers.size();

                for (const auto& p : inliers) {
                    sum_x += p.x;
                    sum_y += p.y;
                    sum_xy += p.x * p.y;
                    sum_xx += p.x * p.x;
                }

                double mean_x = sum_x / N;
                double mean_y = sum_y / N;
                double numerator = sum_xy - N * mean_x * mean_y;
                double denominator = sum_xx - N * mean_x * mean_x;

                if (std::abs(denominator) > 1e-6) {
                    best_line.m = numerator / denominator;
                    best_line.c = mean_y - best_line.m * mean_x;
                }
            } else {
                best_line = current_line;
            }
        }
    }
    
    out_inliers = max_inliers;
    return best_line;
}

// --- Public Interface for Callback (The function called by ParserManager) ---

void process_scan_with_ransac(const SharedScan& raw_scan_data, WallFollower* follower) {
    if (!follower) {
        std::cerr << "[RANSAC] WallFollower instance is null. Cannot command robot.\n";
        return;
    }

    // Debug: Show raw scan info
    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {
        std::cout << "\n[DEBUG] Raw scan: num_ranges=" << raw_scan_data.num_ranges
                  << " angle_min=" << raw_scan_data.angle_min
                  << " angle_max=" << (raw_scan_data.angle_min + raw_scan_data.num_ranges * raw_scan_data.angle_increment)
                  << "\n";
        
        // Show some sample ranges
        if (raw_scan_data.num_ranges > 0) {
            std::cout << "[DEBUG] Sample ranges: ";
            for (size_t i = 0; i < std::min(size_t(5), raw_scan_data.num_ranges); i++) {
                std::cout << raw_scan_data.ranges[i] << " ";
            }
            std::cout << "... ";
            for (size_t i = raw_scan_data.num_ranges - 5; i < raw_scan_data.num_ranges; i++) {
                std::cout << raw_scan_data.ranges[i] << " ";
            }
            std::cout << "\n";
        }
    }

    // 1. Convert raw polar data to Cartesian points
    std::vector<Point> scan_data = convert_scan_to_points(raw_scan_data);

    if (scan_data.empty()) {
        std::cout << "[RANSAC] No valid scan points!\n";
        follower->emergencyStop(); // Safety if no data is available
        return;
    }
    
    // 2. Run RANSAC Algorithm
    size_t inlier_count = 0;
    Line best_line = ransac_fit_line(scan_data, MAX_RANSAC_ITERATIONS, DISTANCE_THRESHOLD, inlier_count);

    // Debug: Show RANSAC results periodically
    if (debug_counter % 20 == 0) {
        std::cout << "[RANSAC] Points: " << scan_data.size() 
                  << " | Inliers: " << inlier_count 
                  << " (" << (scan_data.size() > 0 ? (100.0 * inlier_count / scan_data.size()) : 0) << "%)";
        std::cout << " | Line: y = " << best_line.m << "x + " << best_line.c << "\n";
    }

    // If RANSAC didn't find a strong consensus, we might want to stop or just spin.
    if (inlier_count < scan_data.size() * 0.2) { // Example threshold: require > 20% consensus
         if (debug_counter % 20 == 0) {
             std::cout << "[RANSAC] Low consensus - wall detection uncertain\n";
         }
         // follower->emergencyStop();
         // return;
    }

    // 3. Calculate Wall Follower Inputs from RANSAC Result

    // a) Angle Error (Angle of the line relative to the robot's forward axis)
    // angle_error = atan(m). A slope of 0 is parallel (angle=0).
    double angle_error_rad = std::atan(best_line.m);
    
    // b) Perpendicular Distance (Distance from the robot's origin (0,0) to the line)
    // For y = mx + c (or mx - y + c = 0), distance from (0,0) is |c| / sqrt(m^2 + 1)
    double perpendicular_distance = std::abs(best_line.c) / std::sqrt(best_line.m * best_line.m + 1.0);
    
    // c) Front Clearance (Minimum distance in the robot's forward viewing arc)
    // We assume the forward arc is from -PI/4 to PI/4 (45 degrees left/right) relative to robot X-axis.
    // Since the raw scan is in polar coordinates relative to the robot, we check the ranges array directly.
    
    double front_clearance = raw_scan_data.range_max;
    // We use the raw ranges array here to be fast and avoid iterating Cartesian points.
    
    // Assuming scan starts at 0 rad (forward) and spans to 2*PI. 
    // We need to map the angle range [-PI/4, PI/4] to the raw index.
    
    // Note: The mapping logic relies heavily on how the sensor coordinate system is aligned (often X=Forward).
    // For simplicity, we'll find the minimum distance in the middle 90 degrees of the scan.
    
    const double HALF_ARC_RAD = M_PI / 4.0; // 45 degrees
    double center_angle = raw_scan_data.angle_min + raw_scan_data.angle_increment * (raw_scan_data.num_ranges / 2.0);
    
    for (size_t i = 0; i < raw_scan_data.num_ranges; ++i) {
        double current_angle = raw_scan_data.angle_min + i * raw_scan_data.angle_increment;
        
        // Normalize angle difference to check if it's in the forward cone
        double angle_diff = std::abs(current_angle - center_angle);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        if (angle_diff < HALF_ARC_RAD) {
             front_clearance = std::min(front_clearance, (double)raw_scan_data.ranges[i]);
        }
    }
    // Clamp clearance to max range if it's higher
    front_clearance = std::min(front_clearance, (double)raw_scan_data.range_max);

    // 4. Command Wall Follower
    RansacResult control_result = {
        perpendicular_distance,
        angle_error_rad,
        front_clearance
    };
    
    // Determine wall direction
    const char* wall_direction = (perpendicular_distance > 0) ? "Left" : "Right";
    
    // Debug output: Show control inputs
    std::cout << "[Control] Wall Direction: " << wall_direction
              << " | Distance: " << perpendicular_distance << "m"
              << " | Angle error: " << (angle_error_rad * 180.0 / M_PI) << "°"
              << " | Front clearance: " << front_clearance << "m\n";
    
    follower->computeAndCommand(control_result);
}

} // namespace turtlebot4