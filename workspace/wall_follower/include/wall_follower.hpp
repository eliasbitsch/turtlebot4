#ifndef WALL_FOLLOWER_HPP
#define WALL_FOLLOWER_HPP

#include "shared_memory.hpp"
#include "datatypes.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace turtlebot4 {

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
class WallFollower { // Linear Controller
public:
    // PID Gains and desired distance (can be configured externally)
    WallFollower(SharedMemory<SharedCmdVel>& cmd_shm, 
                 double kp_dist = 3.0, double kp_angle = 5.0, double desired_dist = 0.5);

    /**
     * @brief Computes control commands based on the RANSAC results and writes to Shared Memory.
     * @param result The structured RANSAC output containing distance, angle, and clearance.
     */
    void computeAndCommand(const RansacResult& result);

    /**
     * @brief Sends a zero velocity command (emergency stop).
     */
    void emergencyStop();

private:
    SharedMemory<SharedCmdVel>& cmd_shm_;
    
    // Control Parameters
    const double KP_DISTANCE_;
    const double KP_ANGLE_;
    const double DESIRED_DISTANCE_;

    // Limit Constants (Adjust these based on TurtleBot4 specs)
    static constexpr double MAX_ANGULAR = 2.84;
    static constexpr double MAX_LINEAR = 0.22;
    static constexpr double EMERGENCY_STOP_DIST = 0.3; // 30 cm stop distance
    static constexpr double SAFE_LINEAR_VEL = 0.15;
};

} // namespace turtlebot4

#endif // WALL_FOLLOWER_HPP