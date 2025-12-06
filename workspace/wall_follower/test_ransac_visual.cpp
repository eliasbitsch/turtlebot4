// Simple RANSAC visualization tool
// Reads scan data and shows what RANSAC detects

#include "ransac_processor.hpp"
#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace turtlebot4;

void print_scan_visual(const SharedScan& scan) {
    if (scan.num_ranges == 0) return;
    
    // Simple ASCII visualization
    const int width = 80;
    const int height = 20;
    char grid[height][width];
    
    // Initialize grid
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            grid[y][x] = ' ';
        }
    }
    
    // Robot at center bottom
    int robot_x = width / 2;
    int robot_y = height - 1;
    grid[robot_y][robot_x] = 'R';
    
    // Plot scan points
    double max_range = 3.0; // meters to show
    for (size_t i = 0; i < scan.num_ranges; i++) {
        float range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max || std::isnan(range)) continue;
        if (range > max_range) continue;
        
        double angle = scan.angle_min + i * scan.angle_increment;
        double x_m = range * std::cos(angle);
        double y_m = range * std::sin(angle);
        
        // Convert to grid coordinates
        int x = robot_x + (int)(x_m * width / (2.0 * max_range));
        int y = robot_y - (int)(y_m * height / max_range);
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[y][x] = '*';
        }
    }
    
    // Print grid
    std::cout << "\n=== Scan Visualization (R=robot, *=obstacle) ===\n";
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::cout << grid[y][x];
        }
        std::cout << "\n";
    }
    std::cout << "===\n";
}

int main() {
    try {
        SharedMemory<SharedScan> scan_shm(shm_names::SCAN, false);
        
        std::cout << "RANSAC Visual Test - Reading scan data...\n";
        std::cout << "Press Ctrl+C to exit\n\n";
        
        uint64_t last_seq = 0;
        
        while (true) {
            SharedScan scan = scan_shm.read_copy();
            
            if (scan.sequence != last_seq && scan.num_ranges > 0) {
                last_seq = scan.sequence;
                
                std::cout << "\n========================================\n";
                std::cout << "Scan #" << scan.sequence << "\n";
                std::cout << "Ranges: " << scan.num_ranges << "\n";
                std::cout << "Angle: " << (scan.angle_min * 180 / M_PI) << "° to " 
                         << ((scan.angle_min + scan.num_ranges * scan.angle_increment) * 180 / M_PI) << "°\n";
                
                // Count valid points
                int valid_count = 0;
                float min_range = 999.0f, max_range = 0.0f;
                for (size_t i = 0; i < scan.num_ranges; i++) {
                    float r = scan.ranges[i];
                    if (r >= scan.range_min && r <= scan.range_max && !std::isnan(r)) {
                        valid_count++;
                        if (r < min_range) min_range = r;
                        if (r > max_range) max_range = r;
                    }
                }
                std::cout << "Valid points: " << valid_count << "/" << scan.num_ranges << "\n";
                std::cout << "Range: " << min_range << "m to " << max_range << "m\n";
                
                print_scan_visual(scan);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Make sure the bridge is running!\n";
        return 1;
    }
    
    return 0;
}
