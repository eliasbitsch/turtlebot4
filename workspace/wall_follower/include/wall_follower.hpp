#ifndef WALL_FOLLOWER_HPP
#define WALL_FOLLOWER_HPP

#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace turtlebot4 {

// Wall follower behavior states
enum class FollowerState {
    SEARCH,    // Rotating to find the closest wall
    APPROACH,  // Driving toward the detected wall
    ALIGN,     // Turning to become parallel with the wall
    FOLLOW     // Following along the wall
};

// --- RANSAC Output Struct ---
/**
 * @brief Data structure used to pass the geometric result of RANSAC to the controller.
 */
struct RansacResult {
    double perpendicular_distance; // Perpendicular distance from robot origin (0,0) to the fitted line (meters)
    double angle_error_rad;        // Angle of the fitted line relative to the robot's forward axis (radians)
    double front_clearance;        // Minimum distance in a forward-facing arc, for safety/avoidance (meters)
    double left_clearance;         // Minimum distance to the left side (meters)
    double right_clearance;        // Minimum distance to the right side (meters)
    bool has_front_wall;           // True if there's a wall in front (front_clearance < threshold)
    bool has_left_wall;            // True if there's a wall on the left
    bool has_right_wall;           // True if there's a wall on the right
};

/**
 * @brief Implements the control logic (PID/P) for wall following.
 * Commands are written directly to the SharedCmdVel shared memory region.
 */
class WallFollower { // PID Controller
public:
    // PID Gains and desired distance (can be configured externally)
    WallFollower(SharedMemory<SharedCmdVel>& cmd_shm, 
                 double kp_dist = 3.0, double ki_dist = 0.1, double kd_dist = 0.5,
                 double kp_angle = 5.0, double ki_angle = 0.05, double kd_angle = 0.8);

    /**
     * @brief Computes control commands based on the RANSAC results and writes to Shared Memory.
     * @param result The structured RANSAC output containing distance, angle, and clearance.
     */
    void computeAndCommand(const RansacResult& result);

    /**
     * @brief Get current behavior state
     */
    FollowerState getState() const { return state_; }

    /**
     * @brief Sends a zero velocity command (emergency stop).
     */
    void emergencyStop();

private:
    SharedMemory<SharedCmdVel>& cmd_shm_;
    
    // PID Control Parameters
    const double KP_DISTANCE_;
    const double KI_DISTANCE_;
    const double KD_DISTANCE_;
    const double KP_ANGLE_;
    const double KI_ANGLE_;
    const double KD_ANGLE_;
    const double DESIRED_DISTANCE_;

    // PID State
    double integral_distance_;
    double integral_angle_;
    double prev_distance_error_;
    double prev_angle_error_;
    double prev_time_;
    uint64_t cmd_sequence_;
    double prev_linear_x_;  // For velocity ramping/smoothing

    // State machine
    FollowerState state_;
    double state_timer_;
    double target_wall_distance_;  // Distance to wall when first detected
    int target_wall_side_;         // +1 for left, -1 for right
    double closest_wall_distance_; // Closest wall found during search
    int closest_wall_side_;        // Side of closest wall

    // Limit Constants (Adjust these based on TurtleBot4 specs)
    static constexpr double MAX_ANGULAR = 0.84;        // Reduce 2.84 -> 1.84
    static constexpr double MAX_LINEAR = 0.80;
    static constexpr double EMERGENCY_STOP_DIST = 0.6; // 60 cm stop distance
    static constexpr double SAFE_LINEAR_VEL = 0.50;    // Increase speed: 0.15 → 0.20 m/s
    static constexpr double TURN_DISTANCE = 1;       // Distance at which to trigger turn when front wall is closest
    static constexpr double DESIRED_WALL_DISTANCE = 1; // Target distance to maintain from wall while following (meters)
};

} // namespace turtlebot4

#endif // WALL_FOLLOWER_HPP