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
        
        // Filter out invalid or too-close points (cables, scanner housing, etc.)
        if (range < scan.range_min || range > scan.range_max || std::isnan(range) || range == 0.0f) {
            continue; 
        }
        
        // Skip points closer than 60cm - likely cables or scanner parts
        // Adjust this threshold based on your scanner setup
        if (range < 0.60f) {
            continue;
        }
        
        double angle = scan.angle_min + i * scan.angle_increment;
        
        // ROTATE LASER FRAME: Previously rotated -90°. Add 180° flip so effective
        // rotation becomes +90° (adjusts mapping when front/back were inverted).
        angle = angle + M_PI / 2.0;
        
        // Optional: Skip back angles where cables are (adjust range as needed)
        // Uncomment to exclude rear 90 degrees (±45° from back)
        // double norm_angle = angle;
        // while (norm_angle > M_PI) norm_angle -= 2 * M_PI;
        // while (norm_angle < -M_PI) norm_angle += 2 * M_PI;
        // if (std::abs(norm_angle - M_PI) < M_PI/4 || std::abs(norm_angle + M_PI) < M_PI/4) {
        //     continue; // Skip rear quadrant
        // }
        
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
        // std::cerr << "[RANSAC] WallFollower instance is null. Cannot command robot.\n";
        return;
    }

    static int debug_counter = 0;
    debug_counter++;
    
    // Print scan range info on first run
    if (debug_counter == 1) {
        double angle_max = raw_scan_data.angle_min + raw_scan_data.num_ranges * raw_scan_data.angle_increment;
        std::cout << "\n=== LASER SCAN CONFIGURATION ===\n";
        std::cout << "Total points: " << raw_scan_data.num_ranges << "\n";
        std::cout << "Angle range: " << (raw_scan_data.angle_min * 180 / M_PI) << "° to " 
                  << (angle_max * 180 / M_PI) << "°\n";
        std::cout << "Angle increment: " << (raw_scan_data.angle_increment * 180 / M_PI) << "°\n";
        std::cout << "Range: " << raw_scan_data.range_min << "m to " << raw_scan_data.range_max << "m\n";
        
        // Find and show the actual 0° point
        std::cout << "\n=== FINDING 0° (FORWARD) REFERENCE ===\n";
        for (size_t i = 0; i < raw_scan_data.num_ranges; ++i) {
            double angle = raw_scan_data.angle_min + i * raw_scan_data.angle_increment;
            if (std::abs(angle) < 0.01) { // Within ~0.6 degrees of 0
                std::cout << "Index " << i << " is at " << (angle * 180 / M_PI) 
                         << "° (≈0° FORWARD) with range: " << raw_scan_data.ranges[i] << "m\n";
            }
        }
        
        // Show FULL 360° scan with values every 10 degrees
        std::cout << "\n=== FULL 360° LASER SCAN (every 10°) ===\n";
        std::cout << "Angle    | Range  | Index\n";
        std::cout << "---------|--------|---------\n";
        for (int target_angle = -180; target_angle <= 180; target_angle += 10) {
            // Find closest scan point to this angle
            double target_rad = target_angle * M_PI / 180.0;
            size_t closest_idx = 0;
            double min_diff = 999.0;
            
            for (size_t i = 0; i < raw_scan_data.num_ranges; ++i) {
                double angle = raw_scan_data.angle_min + i * raw_scan_data.angle_increment;
                double diff = std::abs(angle - target_rad);
                if (diff < min_diff) {
                    min_diff = diff;
                    closest_idx = i;
                }
            }
            
            double actual_angle = raw_scan_data.angle_min + closest_idx * raw_scan_data.angle_increment;
            float range = raw_scan_data.ranges[closest_idx];
            
            printf("%6.1f°  | %6.3fm | %4zu\n", 
                   actual_angle * 180.0 / M_PI, range, closest_idx);
        }
        std::cout << "================================\n\n";
    }

    // 1. Convert raw polar data to Cartesian points
    std::vector<Point> scan_data = convert_scan_to_points(raw_scan_data);

    if (scan_data.empty()) {
        // std::cout << "[RANSAC] No valid scan points!\n";
        follower->emergencyStop(); // Safety if no data is available
        return;
    }
    
    // 2. Run RANSAC Algorithm
    size_t inlier_count = 0;
    Line best_line = ransac_fit_line(scan_data, MAX_RANSAC_ITERATIONS, DISTANCE_THRESHOLD, inlier_count);

    // Check RANSAC quality
    double consensus_ratio = scan_data.size() > 0 ? (double)inlier_count / scan_data.size() : 0.0;
    bool good_fit = (inlier_count >= 10) && (consensus_ratio > 0.05); //Ignore readings less tan 5cm

    // 3. Calculate Wall Follower Inputs from RANSAC Result

    // a) Angle Error (Angle of the line relative to the robot's forward axis)
    // angle_error = atan(m). A slope of 0 is parallel (angle=0).
    double angle_error_rad = std::atan(best_line.m);
    
    // b) Perpendicular Distance (Distance from the robot's origin (0,0) to the line)
    // For y = mx + c (or mx - y + c = 0), signed distance from (0,0) is c / sqrt(m^2 + 1)
    // Negative the result: positive distance means wall is to the RIGHT, negative means LEFT
    double perpendicular_distance = -best_line.c / std::sqrt(best_line.m * best_line.m + 1.0);
    
    // c) Regional Clearances (Front, Left, Right)
    // Divide scan into regions to detect walls on different sides
    
    double front_clearance = raw_scan_data.range_max;
    double left_clearance = raw_scan_data.range_max;
    double right_clearance = raw_scan_data.range_max;
    double front_angle = 0, left_angle = 0, right_angle = 0;
    // Use explicit half-widths: 22.5° (π/8) for front and sides so all regions
    // have the same angular width as requested by the user.
    const double FRONT_HALF_RAD = M_PI / 8.0; // 22.5 degrees
    const double SIDE_HALF_RAD  = M_PI / 8.0; // 22.5 degrees
    const double LEFT_CENTER =  M_PI / 2.0;   // +90 degrees
    const double RIGHT_CENTER = -M_PI / 2.0;  // -90 degrees
    int forward_count = 0;
    
    for (size_t i = 0; i < raw_scan_data.num_ranges; ++i) {
        double current_angle = raw_scan_data.angle_min + i * raw_scan_data.angle_increment;
        
        // ROTATE LASER FRAME: Previously rotated -90°. Add 180° flip so effective
        // rotation becomes +90° (adjusts mapping when front/back were inverted).
        current_angle = current_angle + M_PI / 2.0;
        
        float range = raw_scan_data.ranges[i];
        
        // Skip invalid ranges
        if (range < raw_scan_data.range_min || range > raw_scan_data.range_max || std::isnan(range)) {
            continue;
        }
        
        // Normalize angle to [-PI, PI]
        double norm_angle = current_angle;
        while (norm_angle > M_PI) norm_angle -= 2 * M_PI;
        while (norm_angle < -M_PI) norm_angle += 2 * M_PI;
        
        // Front region: -22.5° to +22.5° (centered at 0)
        if (std::abs(norm_angle) <= FRONT_HALF_RAD) {
            if (range < front_clearance) {
                front_clearance = (double)range;
                front_angle = norm_angle * 180.0 / M_PI;
            }
            forward_count++;
        }
        // Left region: centered at +90° with ±22.5° half-width (67.5° .. 112.5°)
        else if (std::abs(norm_angle - LEFT_CENTER) <= SIDE_HALF_RAD) {
            if (range < left_clearance) {
                left_clearance = (double)range;
                left_angle = norm_angle * 180.0 / M_PI;
            }
        }
        // Right region: centered at -90° with ±22.5° half-width (-112.5° .. -67.5°)
        else if (std::abs(norm_angle - RIGHT_CENTER) <= SIDE_HALF_RAD) {
            if (range < right_clearance) {
                right_clearance = (double)range;
                right_angle = norm_angle * 180.0 / M_PI;
            }
        }
    }
    
    // Debug: Print regional clearances with angle information for calibration
    // Suppress calibration prints when not needed
    // std::cout << "[CALIBRATION] Regional clearances:\n"
    //           << "  Front: " << front_clearance << "m at " << front_angle << "° (range: -22.5° to +22.5°)\n"
    //           << "  Left:  " << left_clearance << "m at " << left_angle << "° (range: 67.5° to 112.5°)\n"
    //           << "  Right: " << right_clearance << "m at " << right_angle << "° (range: -112.5° to -67.5°)\n";

    // 4. Command Wall Follower
    const double WALL_DETECT_THRESHOLD = 1.5; // Walls closer than 1.5m are considered present
    
    RansacResult control_result = {
        perpendicular_distance,
        angle_error_rad,
        front_clearance,
        left_clearance,
        right_clearance,
        front_clearance < WALL_DETECT_THRESHOLD,
        left_clearance < WALL_DETECT_THRESHOLD,
        right_clearance < WALL_DETECT_THRESHOLD
    };
    
    // Determine wall position relative to robot
    const char* wall_side = "UNKNOWN";
    if (perpendicular_distance > 0.05) {
        wall_side = "RIGHT";
    } else if (perpendicular_distance < -0.05) {
        wall_side = "LEFT";
    } else {
        wall_side = "CENTER";
    }
    
    // Determine if wall is ahead/behind based on angle
    const char* wall_orientation = "";
    double angle_deg = angle_error_rad * 180.0 / M_PI;
    if (std::abs(angle_deg) < 15.0) {
        wall_orientation = "PARALLEL";
    } else if (std::abs(angle_deg) > 75.0) {
        wall_orientation = "PERPENDICULAR";
    } else {
        wall_orientation = "ANGLED";
    }
    
    // Print clear RANSAC wall detection results (suppressed '[RANSAC]' tagged output)
    // std::cout << "[RANSAC] " 
    //           << " Wall: " << wall_side 
    //           << " " << wall_orientation
    //           << " | Dist: " << std::abs(perpendicular_distance) << "m"
    //           << " Angle: " << angle_deg << "°"
    //           << " | Points: " << scan_data.size() 
    //           << " Inliers: " << inlier_count << "(" << (int)(consensus_ratio * 100) << "% )"
    //           << " " << (good_fit ? "GOOD" : "WEAK")
    //           << " | Front: " << front_clearance << "m\n";
    
    follower->computeAndCommand(control_result);
}

} // namespace turtlebot4