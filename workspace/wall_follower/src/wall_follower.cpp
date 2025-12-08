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
      prev_linear_x_(0.0),
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
    // Re-enable safety check to stop when obstacles are too close in front
    if (result.front_clearance < EMERGENCY_STOP_DIST) {
        std::cout << "[OBSTACLE] Front clearance " << result.front_clearance 
                  << "m - Returning to SEARCH\n";
        state_ = FollowerState::SEARCH;
        state_timer_ = 0.0;
        // Don't return - let it continue to SEARCH state below
    }

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
            // Rotate slowly to find a wall, and transition to APPROACH when detected
            linear_x = 0.0;
            // gentle rotation to scan environment
            angular_z = 0.45; // rad/s (below MAX_ANGULAR)

            // If a wall is detected on either side or front, choose it and approach
            if (result.has_left_wall || result.has_right_wall || result.has_front_wall) {
                // prefer side walls over front wall for wall-following
                if (result.has_left_wall) {
                    target_wall_side_ = +1; // left
                } else if (result.has_right_wall) {
                    target_wall_side_ = -1; // right
                } else {
                    // front wall detected - decide to pick the closest side by ranges
                    target_wall_side_ = (result.left_clearance < result.right_clearance) ? +1 : -1;
                }

                // record target distance and switch to approach state
                target_wall_distance_ = std::abs(result.perpendicular_distance);
                state_ = FollowerState::APPROACH;
                state_timer_ = 0.0;
                std::cout << "[STATE] Found wall - switching to APPROACH (side=" << (target_wall_side_>0?"LEFT":"RIGHT") << ")\n";
            }
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
            const double ALIGN_TARGET_DEG = 30.0; // relaxed to 30° for faster transitions
            const double ALIGN_TIMEOUT = 10.0;     // give up after 10 seconds

            // Print alignment status with timer
            std::cout << "[STATE] Aligning - angle=" << angle_deg << "° (target <" << ALIGN_TARGET_DEG 
                      << "°) timer=" << state_timer_ << "s\n";

            if (angle_deg < ALIGN_TARGET_DEG || state_timer_ > ALIGN_TIMEOUT) {
                if (state_timer_ > ALIGN_TIMEOUT) {
                    std::cout << "[WARN] Alignment timeout - proceeding to FOLLOW anyway\n";
                }
                // Wall is close enough to parallel - start following
                state_ = FollowerState::FOLLOW;
                state_timer_ = 0.0;
                integral_distance_ = 0.0;
                integral_angle_ = 0.0;
                std::cout << "[STATE] Aligned (angle=" << angle_deg << "°) - Following...\n";
            } else {
                // Slower forward motion so rotation has more effect before position changes
                // If nearly perpendicular (>60°), still need some forward motion to escape
                linear_x = (angle_deg > 60.0) ? 0.15 : 0.08;
                
                // Increase angular gain slightly and raise max to turn more aggressively
                const double ALIGN_GAIN = 1.5;            // increased from 1.2 for faster rotation
                const double ALIGN_MAX_ANGULAR = 0.6;     // raised from 0.5 (rad/s)
                angular_z = -result.angle_error_rad * ALIGN_GAIN;
                angular_z = std::clamp(angular_z, -ALIGN_MAX_ANGULAR, ALIGN_MAX_ANGULAR);
                // Debug: show commanded velocities and raw angle error during align
                std::cout << "[DEBUG] ALIGN -> angle_error_rad=" << result.angle_error_rad 
                          << " linear_x=" << linear_x << " angular_z=" << angular_z << "\n";
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

            // Diagnostic print to help understand why FOLLOW speed changes
            std::cout << "[DEBUG] FOLLOW -> front_clearance=" << result.front_clearance
                      << " clearance_scale=" << clearance_scale
                      << " angular_z=" << angular_z
                      << " turning_scale=" << turning_scale
                      << " SAFE_LINEAR_VEL=" << SAFE_LINEAR_VEL
                      << " linear_x=" << linear_x << "\n";

            // If we lose the wall, go back to search
            if (abs_distance > 2.5 || abs_distance < 0.2) {
                state_ = FollowerState::SEARCH;
                state_timer_ = 0.0;
                std::cout << "[STATE] Lost wall (dist=" << abs_distance << "m) - Searching...\n";
            }
            break;
        }
    }

    // --- Apply Velocity Ramping (Slew Rate Limiter) ---
    // Prevents large velocity jumps between states/cycles for smoother motion
    const double MAX_ACCEL = 0.3;  // max change: 0.3 m/s per second
    double max_delta = MAX_ACCEL * dt;
    linear_x = std::clamp(linear_x, prev_linear_x_ - max_delta, prev_linear_x_ + max_delta);
    prev_linear_x_ = linear_x;

    // --- Write Command to Shared Memory ---
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
}

} // namespace turtlebot4