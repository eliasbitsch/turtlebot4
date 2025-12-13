#ifndef DATATYPES_HPP
#define DATATYPES_HPP

#include <cstdint>
#include <cstring>

namespace turtlebot4 {

// ============================================================================
// SHARED MEMORY STRUCTURES (Fixed-size, no pointers/STL for shared memory)
// ============================================================================

// Shared Memory: Odometry Data (OdomParser -> Controller/Navigation)
// Name: /shm_odom
struct SharedOdometry {
    // Position
    double position_x;
    double position_y;
    double position_z;

    // Orientation (Quaternion)
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;

    // Linear velocity
    double linear_velocity_x;
    double linear_velocity_y;
    double linear_velocity_z;

    // Angular velocity
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;

    // Timestamp
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    // Sequence number for detecting updates
    uint64_t sequence;

    SharedOdometry() : position_x(0), position_y(0), position_z(0),
        orientation_x(0), orientation_y(0), orientation_z(0), orientation_w(1.0),
        linear_velocity_x(0), linear_velocity_y(0), linear_velocity_z(0),
        angular_velocity_x(0), angular_velocity_y(0), angular_velocity_z(0),
        timestamp_sec(0), timestamp_nanosec(0), sequence(0) {}
};

// Shared Memory: Command Velocity (Controller -> RosbridgeClient)
// Name: /shm_cmd_vel
struct SharedCmdVel {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    // Timestamp
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    // Sequence number for detecting updates
    uint64_t sequence;

    // Flag to indicate new command ready
    bool new_command;

    SharedCmdVel() : linear_x(0), linear_y(0), linear_z(0),
        angular_x(0), angular_y(0), angular_z(0),
        timestamp_sec(0), timestamp_nanosec(0), sequence(0), new_command(false) {}
};

// Shared Memory: Laser Scan (ScanParser -> Mapper/Navigation)
// Name: /shm_scan
struct SharedScan {
    // Scan metadata
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;

    // Timestamp
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    // Sequence number for detecting updates
    uint64_t sequence;

    // Range data (720 points typical for TurtleBot4 LIDAR)
    static constexpr size_t MAX_RANGES = 2048;
    float ranges[MAX_RANGES];
    size_t num_ranges;

    SharedScan() : angle_min(0), angle_max(0), angle_increment(0),
        range_min(0), range_max(0), timestamp_sec(0), timestamp_nanosec(0),
        sequence(0), ranges{}, num_ranges(0) {}
};

// Shared Memory: Goal Pose (WaypointManager -> Controller)
// Name: /shm_goal
struct SharedGoal {
    // Target position
    double x;
    double y;
    double theta;  // Target orientation (yaw)

    // Goal status
    bool active;           // true if goal is active
    bool reached;          // true if controller has reached the goal
    uint32_t goal_id;      // ID to track which goal this is

    // Timestamp when goal was set
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    // Sequence number for detecting updates
    uint64_t sequence;

    SharedGoal() : x(0), y(0), theta(0), active(false), reached(false),
        goal_id(0), timestamp_sec(0), timestamp_nanosec(0), sequence(0) {}
};

// Shared Memory: Occupancy Grid Map (Mapper -> Navigation)
// Name: /shm_map
constexpr uint32_t MAP_MAX_WIDTH = 1024;
constexpr uint32_t MAP_MAX_HEIGHT = 1024;

struct SharedMap {
    // Map metadata
    uint32_t width;
    uint32_t height;
    float resolution;  // meters per cell

    // Origin in world coordinates
    double origin_x;
    double origin_y;
    double origin_z;

    // Orientation of map origin
    double origin_orientation_x;
    double origin_orientation_y;
    double origin_orientation_z;
    double origin_orientation_w;

    // Timestamp
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    // Sequence number
    uint64_t sequence;

    // Occupancy data: -1 = unknown, 0 = free, 100 = occupied
    // Stored as row-major: data[y * width + x]
    int8_t data[MAP_MAX_WIDTH * MAP_MAX_HEIGHT];

    SharedMap() {
        std::memset(this, 0, sizeof(*this));
        origin_orientation_w = 1.0;
        resolution = 0.05f;
        // Initialize all cells to unknown
        std::memset(data, -1, sizeof(data));
    }
};

// ============================================================================
// PARSER OUTPUT STRUCTURES (used internally, can use STL)
// ============================================================================

struct ParsedOdom {
    double position_x, position_y, position_z;
    double orientation_x, orientation_y, orientation_z, orientation_w;
    double linear_vel_x, linear_vel_y, linear_vel_z;
    double angular_vel_x, angular_vel_y, angular_vel_z;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    bool valid;

    ParsedOdom() : position_x(0), position_y(0), position_z(0),
        orientation_x(0), orientation_y(0), orientation_z(0), orientation_w(1.0),
        linear_vel_x(0), linear_vel_y(0), linear_vel_z(0),
        angular_vel_x(0), angular_vel_y(0), angular_vel_z(0),
        stamp_sec(0), stamp_nanosec(0), valid(false) {}
};

struct ParsedScan {
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;

    // Fixed-size array for ranges (typical LIDAR has ~360-720 points)
    static constexpr size_t MAX_RANGES = 2048;
    float ranges[MAX_RANGES];
    size_t num_ranges;

    bool valid;

    ParsedScan() : angle_min(0), angle_max(0), angle_increment(0),
        time_increment(0), scan_time(0), range_min(0), range_max(0),
        stamp_sec(0), stamp_nanosec(0), ranges{}, num_ranges(0), valid(false) {}
};

struct ParsedBumper {
    bool is_pressed;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    bool valid;

    ParsedBumper() : is_pressed(false), stamp_sec(0), stamp_nanosec(0), valid(false) {}
};

struct ParsedCmdVel {
    double linear_x, linear_y, linear_z;
    double angular_x, angular_y, angular_z;
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    bool valid;

    ParsedCmdVel() : linear_x(0), linear_y(0), linear_z(0),
        angular_x(0), angular_y(0), angular_z(0),
        stamp_sec(0), stamp_nanosec(0), valid(false) {}
};

// JointStates from /joint_states topic (wheel encoder data)
struct ParsedJointStates {
    double left_wheel_position;   // radians
    double right_wheel_position;  // radians
    bool wheel_drop_left;         // cliff sensor
    bool wheel_drop_right;        // cliff sensor
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    bool valid;

    ParsedJointStates() : left_wheel_position(0), right_wheel_position(0),
        wheel_drop_left(false), wheel_drop_right(false),
        stamp_sec(0), stamp_nanosec(0), valid(false) {}
};

// ============================================================================
// TOPIC NAMES
// ============================================================================

namespace topics {
    constexpr const char* SCAN = "/scan";
    constexpr const char* ODOM = "/odom";
    constexpr const char* JOINT_STATES = "/joint_states";  // Wheel encoder data (VOLATILE QoS)
    constexpr const char* CMD_VEL = "/cmd_vel";  // motion_control subscribes with BEST_EFFORT/VOLATILE QoS
    constexpr const char* HAZARD_DETECTION = "/hazard_detection";
    constexpr const char* MAP = "/map";
}

// ============================================================================
// SHARED MEMORY NAMES
// ============================================================================

namespace shm_names {
    constexpr const char* ODOM = "/shm_odom";
    constexpr const char* SCAN = "/shm_scan";
    constexpr const char* CMD_VEL = "/shm_cmd_vel";
    constexpr const char* GOAL = "/shm_goal";
    constexpr const char* MAP = "/shm_map";
}

} // namespace turtlebot4

#endif // DATATYPES_HPP
