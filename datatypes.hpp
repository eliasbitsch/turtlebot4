#ifndef DATATYPES_HPP
#define DATATYPES_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

namespace turtlebot4 {

// ============================================================================
// TIMESTAMP STRUCTURES
// ============================================================================

struct Timestamp {
    int32_t sec;
    uint32_t nanosec;

    Timestamp() : sec(0), nanosec(0) {}
    Timestamp(int32_t s, uint32_t ns) : sec(s), nanosec(ns) {}
};

struct Header {
    Timestamp stamp;
    std::string frame_id;

    Header() : frame_id("") {}
};

// ============================================================================
// GEOMETRY STRUCTURES (for Odometry and cmd_vel)
// ============================================================================

struct Vector3 {
    double x;
    double y;
    double z;

    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;

    Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}
    Quaternion(double x_, double y_, double z_, double w_)
        : x(x_), y(y_), z(z_), w(w_) {}
};

struct Point {
    double x;
    double y;
    double z;

    Point() : x(0.0), y(0.0), z(0.0) {}
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Pose {
    Point position;
    Quaternion orientation;

    Pose() = default;
};

struct PoseWithCovariance {
    Pose pose;
    double covariance[36];  // 6x6 covariance matrix

    PoseWithCovariance() {
        for (int i = 0; i < 36; ++i) covariance[i] = 0.0;
    }
};

struct Twist {
    Vector3 linear;
    Vector3 angular;

    Twist() = default;
    Twist(const Vector3& lin, const Vector3& ang) : linear(lin), angular(ang) {}
};

struct TwistWithCovariance {
    Twist twist;
    double covariance[36];  // 6x6 covariance matrix

    TwistWithCovariance() {
        for (int i = 0; i < 36; ++i) covariance[i] = 0.0;
    }
};

struct TwistStamped {
    Header header;
    Twist twist;

    TwistStamped() = default;
};

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

struct LaserScan {
    Header header;
    float angle_min;           // Start angle of the scan [rad]
    float angle_max;           // End angle of the scan [rad]
    float angle_increment;     // Angular distance between measurements [rad]
    float time_increment;      // Time between measurements [seconds]
    float scan_time;           // Time between scans [seconds]
    float range_min;           // Minimum range value [m]
    float range_max;           // Maximum range value [m]
    std::vector<float> ranges; // Range data [m]
    std::vector<float> intensities; // Intensity data (optional)

    LaserScan()
        : angle_min(0.0f), angle_max(0.0f), angle_increment(0.0f),
          time_increment(0.0f), scan_time(0.0f),
          range_min(0.0f), range_max(0.0f) {}
};

struct Odometry {
    Header header;
    std::string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;

    Odometry() : child_frame_id("base_link") {}
};

// ============================================================================
// SHARED MEMORY STRUCTURES
// ============================================================================

// Shared Memory Region 1: Odometry Data (OdomParser -> Linear Controller)
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

    // Synchronization flag
    volatile bool data_ready;

    SharedOdometry()
        : position_x(0.0), position_y(0.0), position_z(0.0),
          orientation_x(0.0), orientation_y(0.0), orientation_z(0.0), orientation_w(1.0),
          linear_velocity_x(0.0), linear_velocity_y(0.0), linear_velocity_z(0.0),
          angular_velocity_x(0.0), angular_velocity_y(0.0), angular_velocity_z(0.0),
          timestamp_sec(0), timestamp_nanosec(0), data_ready(false) {}
};

// Shared Memory Region 2: Controller State (Xbox Controller <-> Linear Controller)
struct SharedControllerState {
    // Input from Xbox Controller
    double v_input_linear_x;    // Linear velocity X from joystick
    double v_input_linear_y;    // Linear velocity Y from joystick
    double v_input_angular_z;   // Angular velocity Z from joystick

    // Output from Linear Controller (feedback)
    double v_output_linear_x;   // Actual commanded linear velocity X
    double v_output_linear_y;   // Actual commanded linear velocity Y
    double v_output_angular_z;  // Actual commanded angular velocity Z

    // Control mode flags
    int32_t control_mode;       // 0 = manual, 1 = autonomous
    int32_t flag;               // General purpose flag

    // Controller state
    double state_index;         // Current state in state machine
    int32_t integral_term;      // Integral term for PID control

    // Safety flags
    volatile bool emergency_stop;
    volatile bool controller_active;
    volatile bool data_ready;

    SharedControllerState()
        : v_input_linear_x(0.0), v_input_linear_y(0.0), v_input_angular_z(0.0),
          v_output_linear_x(0.0), v_output_linear_y(0.0), v_output_angular_z(0.0),
          control_mode(0), flag(0), state_index(0.0), integral_term(0),
          emergency_stop(false), controller_active(false), data_ready(false) {}
};

// ============================================================================
// PARSED DATA STRUCTURES (Thread Communication)
// ============================================================================

// Scan data parsed from JSON string
struct ParsedScanData {
    Header header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    bool valid;

    ParsedScanData()
        : angle_min(0.0f), angle_max(0.0f), angle_increment(0.0f),
          range_min(0.0f), range_max(0.0f), valid(false) {}
};

// Odometry data parsed from JSON string
struct ParsedOdomData {
    Header header;
    std::string child_frame_id;
    Pose pose;
    Twist twist;
    bool valid;

    ParsedOdomData() : child_frame_id("base_link"), valid(false) {}
};

// Bumper data parsed from JSON string
struct ParsedBumperData {
    Header header;
    bool left_bumper;
    bool center_bumper;
    bool right_bumper;
    bool any_collision;
    bool valid;

    ParsedBumperData()
        : left_bumper(false), center_bumper(false),
          right_bumper(false), any_collision(false), valid(false) {}
};

// Command velocity to be sent to robot
struct CmdVelCommand {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
    int32_t timestamp_sec;
    uint32_t timestamp_nanosec;

    CmdVelCommand()
        : linear_x(0.0), linear_y(0.0), linear_z(0.0),
          angular_x(0.0), angular_y(0.0), angular_z(0.0),
          timestamp_sec(0), timestamp_nanosec(0) {}
};

// ============================================================================
// MAPPER & NAVIGATION STRUCTURES
// ============================================================================

// 2D Map cell
struct MapCell {
    int8_t occupancy;  // -1 = unknown, 0 = free, 100 = occupied
    float probability;

    MapCell() : occupancy(-1), probability(0.5f) {}
};

// Occupancy Grid Map
struct OccupancyGrid {
    Header header;
    uint32_t width;
    uint32_t height;
    float resolution;      // meters per cell
    Point origin;          // map origin in world coordinates
    std::vector<MapCell> data;  // row-major order

    OccupancyGrid() : width(0), height(0), resolution(0.05f) {}
};

// Map relationship data (feedback from Navigation to Mapper)
struct MapRelationData {
    Point robot_position;
    double robot_heading;       // radians
    bool localization_valid;
    double confidence;

    MapRelationData() : robot_heading(0.0), localization_valid(false), confidence(0.0) {}
};

// Navigation waypoint
struct Waypoint {
    Point position;
    double heading;            // desired orientation at waypoint [rad]
    double speed;              // desired speed approaching waypoint [m/s]

    Waypoint() : heading(0.0), speed(0.0) {}
};

// Path from navigation (array of waypoints)
struct Path {
    Header header;
    std::vector<Waypoint> waypoints;
    bool valid;

    Path() : valid(false) {}
};

// ============================================================================
// LINEAR CONTROLLER STRUCTURES
// ============================================================================

// Control parameters
struct ControlParams {
    // PID gains
    double kp_linear;
    double ki_linear;
    double kd_linear;
    double kp_angular;
    double ki_angular;
    double kd_angular;

    // Limits
    double max_linear_velocity;
    double max_angular_velocity;
    double max_linear_acceleration;
    double max_angular_acceleration;

    // Thresholds
    double position_tolerance;
    double heading_tolerance;
    double velocity_tolerance;

    ControlParams()
        : kp_linear(1.0), ki_linear(0.1), kd_linear(0.01),
          kp_angular(2.0), ki_angular(0.2), kd_angular(0.02),
          max_linear_velocity(0.5), max_angular_velocity(1.0),
          max_linear_acceleration(0.5), max_angular_acceleration(1.0),
          position_tolerance(0.05), heading_tolerance(0.1), velocity_tolerance(0.01) {}
};

// Controller internal state
struct ControllerState {
    // Error terms
    double error_linear_x;
    double error_linear_y;
    double error_angular_z;

    // Integral terms
    double integral_linear_x;
    double integral_linear_y;
    double integral_angular_z;

    // Previous errors (for derivative)
    double prev_error_linear_x;
    double prev_error_linear_y;
    double prev_error_angular_z;

    // Current target
    Waypoint current_target;
    size_t current_waypoint_index;

    // Status
    bool at_goal;
    bool path_following_active;

    ControllerState()
        : error_linear_x(0.0), error_linear_y(0.0), error_angular_z(0.0),
          integral_linear_x(0.0), integral_linear_y(0.0), integral_angular_z(0.0),
          prev_error_linear_x(0.0), prev_error_linear_y(0.0), prev_error_angular_z(0.0),
          current_waypoint_index(0), at_goal(false), path_following_active(false) {}
};

// ============================================================================
// XBOX CONTROLLER INPUT
// ============================================================================

struct XboxControllerInput {
    // Joystick axes [-1.0 to 1.0]
    float left_stick_x;
    float left_stick_y;
    float right_stick_x;
    float right_stick_y;
    float left_trigger;   // 0.0 to 1.0
    float right_trigger;  // 0.0 to 1.0

    // Buttons (true = pressed)
    bool button_a;
    bool button_b;
    bool button_x;
    bool button_y;
    bool button_lb;
    bool button_rb;
    bool button_back;
    bool button_start;
    bool button_guide;
    bool button_left_stick;
    bool button_right_stick;

    // D-pad
    bool dpad_up;
    bool dpad_down;
    bool dpad_left;
    bool dpad_right;

    // Timestamp
    Timestamp timestamp;
    bool valid;

    XboxControllerInput()
        : left_stick_x(0.0f), left_stick_y(0.0f),
          right_stick_x(0.0f), right_stick_y(0.0f),
          left_trigger(0.0f), right_trigger(0.0f),
          button_a(false), button_b(false), button_x(false), button_y(false),
          button_lb(false), button_rb(false), button_back(false), button_start(false),
          button_guide(false), button_left_stick(false), button_right_stick(false),
          dpad_up(false), dpad_down(false), dpad_left(false), dpad_right(false),
          valid(false) {}
};

// ============================================================================
// TCP/WEBSOCKET MESSAGE STRUCTURES
// ============================================================================

struct WebSocketMessage {
    std::string operation;  // "publish", "subscribe", "advertise", etc.
    std::string topic;
    std::string msg_type;
    std::string payload;    // JSON formatted message data
    Timestamp timestamp;

    WebSocketMessage() = default;
};

// ============================================================================
// THREAD-SAFE QUEUE ELEMENT (for inter-thread communication)
// ============================================================================

template<typename T>
struct QueueElement {
    T data;
    Timestamp timestamp;
    uint64_t sequence_number;

    QueueElement() : sequence_number(0) {}
    QueueElement(const T& d, const Timestamp& ts, uint64_t seq)
        : data(d), timestamp(ts), sequence_number(seq) {}
};

} // namespace turtlebot4

#endif // DATATYPES_HPP
