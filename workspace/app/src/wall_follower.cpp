#include "wall_follower.hpp"
#include <iostream>
#include <chrono> 

namespace turtlebot4 {

WallFollower::WallFollower(SharedMemory<SharedCmdVel>& cmd_shm, 
                           double kp_dist, double kp_angle, double desired_dist)
    : cmd_shm_(cmd_shm),
      KP_DISTANCE_(kp_dist), 
      KP_ANGLE_(kp_angle), 
      DESIRED_DISTANCE_(desired_dist) 
{
    std::cout << "[WallFollower] Initialized. Target distance: " << DESIRED_DISTANCE_ << "m.\n";
}

void WallFollower::emergencyStop() {
    cmd_shm_.update([](SharedCmdVel& cmd) {
        cmd.linear_x = 0.0;
        cmd.angular_z = 0.0;
        cmd.new_command = true;
    });
    // std::cout << "[WallFollower] !!! EMERGENCY STOP COMMANDED !!!\n";
}

void WallFollower::computeAndCommand(const RansacResult& result) {

    double linear_x = SAFE_LINEAR_VEL;
    double angular_z = 0.0;

    // --- 1. Check for Front Obstacle (Safety Override) ---
    if (result.front_clearance < EMERGENCY_STOP_DIST) {
        emergencyStop();
        // Turn aggressively in place to clear the obstacle
        cmd_shm_.update([](SharedCmdVel& cmd) {
            cmd.angular_z = WallFollower::MAX_ANGULAR; 
            cmd.new_command = true;
        });
        return;
    }
    
    // --- 2. Calculate Control Errors ---
    
    // Error 1: Distance Error (e_d). Positive = too far, Negative = too close.
    double distance_error = result.perpendicular_distance - DESIRED_DISTANCE_;
    
    // Error 2: Angle Error (e_a). Angle of the wall relative to robot's heading.
    double angle_error = result.angle_error_rad; 

    // --- 3. Compute Angular Velocity (w) ---
    
    // Combined P-Controller: w = Kp_d * e_d + Kp_a * e_a
    // The negative sign ensures corrective action (e.g., too far -> negative angular velocity -> turn inward)
    angular_z = -(KP_DISTANCE_ * distance_error) - (KP_ANGLE_ * angle_error);
    
    // Decelerate if turning sharply
    if (std::abs(angular_z) > 1.0) {
        linear_x = std::max(0.05, SAFE_LINEAR_VEL - (std::abs(angular_z) * 0.05));
    }

    // --- 4. Apply Limits ---
    linear_x = std::min(MAX_LINEAR, std::max(-MAX_LINEAR, linear_x));
    angular_z = std::min(MAX_ANGULAR, std::max(-MAX_ANGULAR, angular_z));

    // --- 5. Write Command to Shared Memory ---
    
    cmd_shm_.update([&](SharedCmdVel& cmd) {
        cmd.linear_x = linear_x;
        cmd.angular_z = angular_z;
        cmd.new_command = true;

        // Update timestamp
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
        auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
        cmd.timestamp_sec = static_cast<int32_t>(secs.count());
        cmd.timestamp_nanosec = static_cast<uint32_t>(nsecs.count());
    });
}

} // namespace turtlebot4