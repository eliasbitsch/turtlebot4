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

    SharedOdometry() { std::memset(this, 0, sizeof(*this)); orientation_w = 1.0; }
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

    SharedCmdVel() { std::memset(this, 0, sizeof(*this)); }
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

    ParsedOdom() : valid(false) { std::memset(this, 0, sizeof(*this)); orientation_w = 1.0; }
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

    ParsedScan() : valid(false), num_ranges(0) { std::memset(this, 0, sizeof(*this)); }
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

    ParsedCmdVel() : valid(false) { std::memset(this, 0, sizeof(*this)); }
};

// ============================================================================
// TOPIC NAMES
// ============================================================================

namespace topics {
    constexpr const char* SCAN = "/scan";
    constexpr const char* ODOM = "/odom";
    constexpr const char* CMD_VEL = "/cmd_vel";
    constexpr const char* BUMPER = "/bumper";
    constexpr const char* MAP = "/map";
}

// ============================================================================
// SHARED MEMORY NAMES
// ============================================================================

namespace shm_names {
    constexpr const char* ODOM = "/shm_odom";
    constexpr const char* CMD_VEL = "/shm_cmd_vel";
    constexpr const char* MAP = "/shm_map";
}

} // namespace turtlebot4

#endif // DATATYPES_HPP
