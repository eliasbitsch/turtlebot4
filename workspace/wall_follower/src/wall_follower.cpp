#include "wall_follower.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace turtlebot4 {

WallFollower::WallFollower(SharedMemory<SharedCmdVel>& cmd_shm, 
                           double kp_dist, double ki_dist, double kd_dist,
                           double kp_angle, double ki_angle, double kd_angle)
    : cmd_shm_(cmd_shm),
      KP_DISTANCE_(kp_dist),
      KI_DISTANCE_(ki_dist),
      KD_DISTANCE_(kd_dist),
      KP_ANGLE_(kp_angle),
      KI_ANGLE_(ki_angle),
      KD_ANGLE_(kd_angle),
      DESIRED_DISTANCE_(DESIRED_WALL_DISTANCE),
      cmd_sequence_(0),
      integral_distance_(0.0),
      integral_angle_(0.0),
      prev_distance_error_(0.0),
      prev_angle_error_(0.0),
      prev_time_(0.0),
      state_(FollowerState::SEARCH),
      state_timer_(0.0),
      target_wall_distance_(0.0),
      target_wall_side_(0),
      closest_wall_distance_(999.0),
      closest_wall_side_(0)
{
}

void WallFollower::emergencyStop() {
    cmd_shm_.update([&](SharedCmdVel& cmd) {
        cmd.linear_x = 0.0;
        cmd.linear_y = 0.0;
        cmd.linear_z = 0.0;
        cmd.angular_x = 0.0;
        cmd.angular_y = 0.0;
        cmd.angular_z = 0.0;
        cmd.sequence = ++cmd_sequence_;
        cmd.new_command = true;

        auto now = std::chrono::system_clock::now().time_since_epoch();
        cmd.timestamp_sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        cmd.timestamp_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
    });
}

void WallFollower::computeAndCommand(const RansacResult& result) {
    // --- 1. Check for Front Obstacle (Safety Override) ---
    // Commented out during calibration - will re-enable when move commands are active
    /*
    if (result.front_clearance < EMERGENCY_STOP_DIST) {
        std::cout << "[OBSTACLE] Front clearance " << result.front_clearance 
                  << "m - Returning to SEARCH\n";
        state_ = FollowerState::SEARCH;
        state_timer_ = 0.0;
        // Don't return - let it continue to SEARCH state below
    }
    */

    // --- 2. Calculate Time Delta ---
    auto now = std::chrono::high_resolution_clock::now();
    double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();
    double dt = (prev_time_ > 0.0) ? (current_time - prev_time_) : 0.05; // default 50ms
    prev_time_ = current_time;
    dt = std::clamp(dt, 0.01, 0.2); // dt too small (near 0): derivative = (Δerror)/dt becomes huge → spikes/noise amplified → unstable angular/linear outputs.
                                    // dt too small can be caused by clock resolution, repeated near-zero reads, or measurement jitter.
                                    // dt too large: integral accumulates a big chunk at once → big overshoot (integrator windup) and slow/unstable response.

    state_timer_ += dt;

    const double signed_distance = result.perpendicular_distance;
    const double abs_distance = std::abs(signed_distance);

    double linear_x = 0.0;
    double angular_z = 0.0;

    // --- STATE MACHINE ---
    switch (state_) {
        case FollowerState::SEARCH: {
            // Stay still and scan - just output distances for calibration
            angular_z = 0.0;
            linear_x = 0.0;

            // Simple distance output for calibration
            std::cout << "[DISTANCES] Front: " << result.front_clearance << "m"
                     << " | Left: " << result.left_clearance << "m"
                     << " | Right: " << result.right_clearance << "m\n";

            // Stay in SEARCH state - no state transitions during calibration
            break;
        }

        case FollowerState::APPROACH: {
            // Drive toward the wall until we reach desired distance
            double distance_to_target = abs_distance - DESIRED_DISTANCE_;
            
            if (distance_to_target < 0.15) {
                // Reached target distance - now align parallel
                state_ = FollowerState::ALIGN;
                state_timer_ = 0.0;
                std::cout << "[STATE] Reached wall at " << abs_distance << "m - Aligning...\n";
            } else {
                // Drive toward wall, adjust heading slightly
                linear_x = std::clamp(distance_to_target * 0.4, 0.1, 0.2);
                // Gentle steering to keep wall on correct side
                angular_z = -result.angle_error_rad * 1.5;
                angular_z = std::clamp(angular_z, -0.8, 0.8);
            }
            break;
        }

        case FollowerState::ALIGN: {
            // Turn to become parallel with the wall
            double angle_deg = std::abs(result.angle_error_rad * 180.0 / M_PI);
            
            // Print alignment status
            std::cout << "[STATE] Aligning - angle=" << angle_deg << "° (target <20°)\n";
            
            if (angle_deg < 20.0) {
                // Wall is close enough to parallel - start following
                state_ = FollowerState::FOLLOW;
                state_timer_ = 0.0;
                integral_distance_ = 0.0;
                integral_angle_ = 0.0;
                std::cout << "[STATE] Aligned (angle=" << angle_deg << "°) - Following...\n";
            } else {
                // Move slowly forward while rotating to change perspective
                linear_x = 0.08;
                // Rotate to align
                angular_z = -result.angle_error_rad * 1.5;
                angular_z = std::clamp(angular_z, -1.0, 1.0);
            }
            break;
        }

        case FollowerState::FOLLOW: {
            // --- Proportional Wall Following (like linear_controller) ---
            const double distance_error = DESIRED_DISTANCE_ - abs_distance;
            const double angle_error = result.angle_error_rad;
            
            // Print status continuously
            double angle_deg = std::abs(angle_error * 180.0 / M_PI);
            std::cout << "[STATE] Following - Wall at " << abs_distance << "m on " 
                     << (target_wall_side_ > 0 ? "LEFT" : "RIGHT") 
                     << " (angle=" << angle_deg << "°)\n";

            // Simple proportional control for distance correction
            double ang_from_distance = KP_DISTANCE_ * distance_error;
            // Apply correction based on which side wall is on
            ang_from_distance *= (signed_distance >= 0.0 ? 1.0 : -1.0);

            // Proportional control for angle (keep parallel)
            double ang_from_angle = KP_ANGLE_ * angle_error;

            // Combine angle corrections
            angular_z = ang_from_angle + ang_from_distance;
            angular_z = std::clamp(angular_z, -MAX_ANGULAR, MAX_ANGULAR);

            // Drive forward at steady speed, slow down when turning sharply
            linear_x = SAFE_LINEAR_VEL;
            double clearance_scale = std::clamp((result.front_clearance - EMERGENCY_STOP_DIST) / 0.8, 0.0, 1.0);
            double turning_scale = std::clamp(1.0 - (std::abs(angular_z) / MAX_ANGULAR) * 0.5, 0.3, 1.0);
            linear_x *= clearance_scale * turning_scale;
            linear_x = std::clamp(linear_x, 0.0, MAX_LINEAR);

            // If we lose the wall, go back to search
            if (abs_distance > 2.5 || abs_distance < 0.2) {
                state_ = FollowerState::SEARCH;
                state_timer_ = 0.0;
                std::cout << "[STATE] Lost wall (dist=" << abs_distance << "m) - Searching...\n";
            }
            break;
        }
    }

    // --- Write Command to Shared Memory ---
    // DISABLED FOR LASER CALIBRATION - Just observing wall detection, not moving robot
    /*
    cmd_shm_.update([&](SharedCmdVel& cmd) {
        cmd.linear_x = linear_x;
        cmd.linear_y = 0.0;
        cmd.linear_z = 0.0;
        cmd.angular_x = 0.0;
        cmd.angular_y = 0.0;
        cmd.angular_z = angular_z;
        cmd.sequence = ++cmd_sequence_;
        cmd.new_command = true;

        auto now = std::chrono::system_clock::now().time_since_epoch();
        cmd.timestamp_sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        cmd.timestamp_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
    });
    */
}

} // namespace turtlebot4