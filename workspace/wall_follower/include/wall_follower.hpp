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
    SEARCH,    // Rotating to find a wall
    APPROACH,  // Driving toward detected wall
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
                 double kp_angle = 5.0, double ki_angle = 0.05, double kd_angle = 0.8,
                 double desired_dist = 0.5);

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

    // State machine
    FollowerState state_;
    double state_timer_;

    // Limit Constants (Adjust these based on TurtleBot4 specs)
    static constexpr double MAX_ANGULAR = 2.84;
    static constexpr double MAX_LINEAR = 0.22;
    static constexpr double EMERGENCY_STOP_DIST = 0.3; // 30 cm stop distance
    static constexpr double SAFE_LINEAR_VEL = 0.15;
};

} // namespace turtlebot4

#endif // WALL_FOLLOWER_HPP