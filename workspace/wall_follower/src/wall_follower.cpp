#include "wall_follower.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace turtlebot4 {

WallFollower::WallFollower(SharedMemory<SharedCmdVel>& cmd_shm, 
                           double kp_dist, double ki_dist, double kd_dist,
                           double kp_angle, double ki_angle, double kd_angle,
                           double desired_dist)
    : cmd_shm_(cmd_shm),
      KP_DISTANCE_(kp_dist),
      KI_DISTANCE_(ki_dist),
      KD_DISTANCE_(kd_dist),
      KP_ANGLE_(kp_angle),
      KI_ANGLE_(ki_angle),
      KD_ANGLE_(kd_angle),
      DESIRED_DISTANCE_(desired_dist),
      cmd_sequence_(0),
      integral_distance_(0.0),
      integral_angle_(0.0),
      prev_distance_error_(0.0),
      prev_angle_error_(0.0),
      prev_time_(0.0),
      state_(FollowerState::SEARCH),
      state_timer_(0.0)
{
    std::cout << "[WallFollower] Initialized PID controller. Target distance: " << DESIRED_DISTANCE_ << "m.\n";
    std::cout << "[WallFollower] Starting in SEARCH mode...\n";
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
    if (result.front_clearance < EMERGENCY_STOP_DIST) {
        std::cout << "[WallFollower] Emergency stop: front clearance " << result.front_clearance << "m\n";
        emergencyStop();
        return;
    }

    // --- 2. Calculate Time Delta ---
    auto now = std::chrono::high_resolution_clock::now();
    double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();
    double dt = (prev_time_ > 0.0) ? (current_time - prev_time_) : 0.05; // default 50ms
    prev_time_ = current_time;
    dt = std::clamp(dt, 0.01, 0.2); // Clamp to avoid instability

    state_timer_ += dt;

    const double signed_distance = result.perpendicular_distance;
    const double abs_distance = std::abs(signed_distance);

    double linear_x = 0.0;
    double angular_z = 0.0;

    // --- STATE MACHINE ---
    switch (state_) {
        case FollowerState::SEARCH: {
            // Rotate in place to find a wall (look for distance < 2m)
            angular_z = 0.5; // Rotate slowly
            linear_x = 0.0;

            if (abs_distance > 0.1 && abs_distance < 2.0) {
                std::cout << "[WallFollower] Wall detected at " << abs_distance << "m! Switching to APPROACH.\n";
                state_ = FollowerState::APPROACH;
                state_timer_ = 0.0;
            }
            break;
        }

        case FollowerState::APPROACH: {
            // Drive toward wall until we're at desired distance
            double approach_error = abs_distance - DESIRED_DISTANCE_;
            
            if (approach_error < 0.1) {
                std::cout << "[WallFollower] Reached wall! Switching to FOLLOW mode.\n";
                state_ = FollowerState::FOLLOW;
                state_timer_ = 0.0;
                integral_distance_ = 0.0;
                integral_angle_ = 0.0;
            } else {
                // Drive forward, slightly turn to align
                linear_x = std::min(0.2, approach_error * 0.5);
                angular_z = -result.angle_error_rad * 2.0; // Align while approaching
                angular_z = std::clamp(angular_z, -1.0, 1.0);
            }
            break;
        }

        case FollowerState::FOLLOW: {
            // --- Full PID Wall Following ---
            const double distance_error = DESIRED_DISTANCE_ - abs_distance;
            const double angle_error = result.angle_error_rad;

            // PID for distance
            integral_distance_ += distance_error * dt;
            integral_distance_ = std::clamp(integral_distance_, -1.0, 1.0);
            double derivative_distance = (distance_error - prev_distance_error_) / dt;
            prev_distance_error_ = distance_error;

            double ang_from_distance = KP_DISTANCE_ * distance_error +
                                       KI_DISTANCE_ * integral_distance_ +
                                       KD_DISTANCE_ * derivative_distance;
            ang_from_distance *= (signed_distance >= 0.0 ? 1.0 : -1.0);

            // PID for angle
            integral_angle_ += angle_error * dt;
            integral_angle_ = std::clamp(integral_angle_, -1.0, 1.0);
            double derivative_angle = (angle_error - prev_angle_error_) / dt;
            prev_angle_error_ = angle_error;

            double ang_from_angle = KP_ANGLE_ * angle_error +
                                   KI_ANGLE_ * integral_angle_ +
                                   KD_ANGLE_ * derivative_angle;

            angular_z = ang_from_angle + ang_from_distance;
            angular_z = std::clamp(angular_z, -MAX_ANGULAR, MAX_ANGULAR);

            // Linear velocity (slow down when turning)
            linear_x = SAFE_LINEAR_VEL;
            double clearance_scale = std::clamp((result.front_clearance - EMERGENCY_STOP_DIST) / (DESIRED_DISTANCE_ + 0.3), 0.0, 1.0);
            double turning_scale = std::clamp(1.0 - (std::abs(angular_z) / MAX_ANGULAR), 0.2, 1.0);
            linear_x *= clearance_scale * turning_scale;
            linear_x = std::clamp(linear_x, -MAX_LINEAR, MAX_LINEAR);

            // If we lose the wall, go back to search
            if (abs_distance > 3.0 || abs_distance < 0.05) {
                std::cout << "[WallFollower] Lost wall! Returning to SEARCH mode.\n";
                state_ = FollowerState::SEARCH;
                state_timer_ = 0.0;
            }
            break;
        }
    }

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