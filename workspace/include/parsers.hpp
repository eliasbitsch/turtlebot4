#ifndef PARSERS_HPP
#define PARSERS_HPP

#include "datatypes.hpp"
#include "thread_safe_queue.hpp"
#include "shared_memory.hpp"
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <functional>
#include <iostream>
#include <cmath>

namespace turtlebot4 {

using json = nlohmann::json;

// ============================================================================
// PARSE FUNCTIONS (stateless, can be used standalone)
// ============================================================================

inline ParsedScan parse_scan(const json& j) {
    ParsedScan result;

    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    result.angle_min = j.value("angle_min", 0.0f);
    result.angle_max = j.value("angle_max", 0.0f);
    result.angle_increment = j.value("angle_increment", 0.0f);
    result.time_increment = j.value("time_increment", 0.0f);
    result.scan_time = j.value("scan_time", 0.0f);
    result.range_min = j.value("range_min", 0.0f);
    result.range_max = j.value("range_max", 0.0f);

    if (j.contains("ranges") && j["ranges"].is_array()) {
        const auto& ranges = j["ranges"];
        result.num_ranges = std::min(ranges.size(), ParsedScan::MAX_RANGES);
        for (size_t i = 0; i < result.num_ranges; ++i) {
            result.ranges[i] = ranges[i].is_number()
                ? ranges[i].get<float>()
                : result.range_max + 1.0f;
        }
    }

    result.valid = (result.num_ranges > 0);
    return result;
}

inline ParsedOdom parse_odom(const json& j) {
    ParsedOdom result;

    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    if (j.contains("pose") && j["pose"].contains("pose")) {
        const auto& pose = j["pose"]["pose"];
        if (pose.contains("position")) {
            result.position_x = pose["position"].value("x", 0.0);
            result.position_y = pose["position"].value("y", 0.0);
            result.position_z = pose["position"].value("z", 0.0);
        }
        if (pose.contains("orientation")) {
            result.orientation_x = pose["orientation"].value("x", 0.0);
            result.orientation_y = pose["orientation"].value("y", 0.0);
            result.orientation_z = pose["orientation"].value("z", 0.0);
            result.orientation_w = pose["orientation"].value("w", 1.0);
        }
    }

    if (j.contains("twist") && j["twist"].contains("twist")) {
        const auto& twist = j["twist"]["twist"];
        if (twist.contains("linear")) {
            result.linear_vel_x = twist["linear"].value("x", 0.0);
            result.linear_vel_y = twist["linear"].value("y", 0.0);
            result.linear_vel_z = twist["linear"].value("z", 0.0);
        }
        if (twist.contains("angular")) {
            result.angular_vel_x = twist["angular"].value("x", 0.0);
            result.angular_vel_y = twist["angular"].value("y", 0.0);
            result.angular_vel_z = twist["angular"].value("z", 0.0);
        }
    }

    result.valid = true;
    return result;
}

inline ParsedBumper parse_bumper(const json& j) {
    ParsedBumper result;

    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    // Kobuki format
    if (j.contains("state")) {
        result.is_pressed = (j["state"].get<int>() == 1);
        result.valid = true;
    }
    // iRobot Create format
    else if (j.contains("type")) {
        result.is_pressed = (j["type"].get<int>() == 1);
        result.valid = true;
    }
    // Simple bool format
    else if (j.contains("is_pressed")) {
        result.is_pressed = j["is_pressed"].get<bool>();
        result.valid = true;
    }
    // HazardDetectionVector format (TurtleBot4)
    else if (j.contains("detections") && j["detections"].is_array()) {
        for (const auto& det : j["detections"]) {
            if (det.contains("type") && det["type"].get<int>() == 1) {
                result.is_pressed = true;
                break;
            }
        }
        result.valid = true;
    }

    return result;
}

inline ParsedJointStates parse_joint_states(const json& j) {
    ParsedJointStates result;

    if (j.contains("header") && j["header"].contains("stamp")) {
        result.stamp_sec = j["header"]["stamp"].value("sec", 0);
        result.stamp_nanosec = j["header"]["stamp"].value("nanosec", 0u);
    }

    if (j.contains("name") && j.contains("position") &&
        j["name"].is_array() && j["position"].is_array()) {
        const auto& names = j["name"];
        const auto& positions = j["position"];

        for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
            std::string name = names[i].get<std::string>();
            double pos = positions[i].get<double>();

            if (name == "left_wheel_joint") {
                result.left_wheel_position = pos;
            } else if (name == "right_wheel_joint") {
                result.right_wheel_position = pos;
            } else if (name == "wheel_drop_left_joint") {
                result.wheel_drop_left = (pos != 0.0);
            } else if (name == "wheel_drop_right_joint") {
                result.wheel_drop_right = (pos != 0.0);
            }
        }
        result.valid = true;
    }

    return result;
}

// ============================================================================
// PARSER THREAD (generic, runs parse function in loop)
// ============================================================================

template<typename ParsedType, typename ParseFunc>
class ParserThread {
public:
    using Callback = std::function<void(const ParsedType&)>;

    ParserThread(const char* name, ThreadSafeQueue<std::string>& queue, ParseFunc parse_fn)
        : name_(name), queue_(queue), parse_fn_(parse_fn), running_(false), count_(0) {}

    ~ParserThread() { stop(); }

    void start() {
        if (running_) return;
        running_ = true;
        thread_ = std::thread([this] { run(); });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    void set_callback(Callback cb) { callback_ = cb; }
    uint64_t count() const { return count_; }

private:
    void run() {
        std::cout << "[" << name_ << "] Started\n";
        while (running_) {
            auto msg = queue_.pop_timeout(std::chrono::milliseconds(100));
            if (!msg) continue;

            try {
                json j = json::parse(*msg);
                ParsedType parsed = parse_fn_(j);
                if (parsed.valid) {
                    if (callback_) callback_(parsed);
                    count_++;
                }
            } catch (const std::exception& e) {
                std::cerr << "[" << name_ << "] Error: " << e.what() << "\n";
            }
        }
        std::cout << "[" << name_ << "] Stopped\n";
    }

    const char* name_;
    ThreadSafeQueue<std::string>& queue_;
    ParseFunc parse_fn_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> count_;
    std::thread thread_;
    Callback callback_;
};

// ============================================================================
// WHEEL ODOMETRY CALCULATOR
// ============================================================================

class WheelOdometry {
public:
    // TurtleBot4 (Create3) wheel parameters
    static constexpr double WHEEL_RADIUS = 0.036;      // meters
    static constexpr double WHEEL_SEPARATION = 0.233;  // meters (track width)

    WheelOdometry() : x_(0), y_(0), theta_(0), initialized_(false),
                      last_left_(0), last_right_(0) {}

    // Update odometry from wheel positions (radians)
    // Returns computed linear and angular velocities
    void update(double left_pos, double right_pos, double dt,
                double& vx_out, double& wz_out) {
        if (!initialized_) {
            last_left_ = left_pos;
            last_right_ = right_pos;
            initialized_ = true;
            vx_out = 0;
            wz_out = 0;
            return;
        }

        // Wheel displacement
        double delta_left = left_pos - last_left_;
        double delta_right = right_pos - last_right_;
        last_left_ = left_pos;
        last_right_ = right_pos;

        // Convert to linear displacement
        double d_left = delta_left * WHEEL_RADIUS;
        double d_right = delta_right * WHEEL_RADIUS;

        // Robot motion
        double d_center = (d_left + d_right) / 2.0;
        double d_theta = (d_right - d_left) / WHEEL_SEPARATION;

        // Update pose
        theta_ += d_theta;
        x_ += d_center * std::cos(theta_);
        y_ += d_center * std::sin(theta_);

        // Compute velocities
        if (dt > 0) {
            vx_out = d_center / dt;
            wz_out = d_theta / dt;
        } else {
            vx_out = 0;
            wz_out = 0;
        }
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }

    void reset() {
        x_ = y_ = theta_ = 0;
        initialized_ = false;
    }

private:
    double x_, y_, theta_;
    bool initialized_;
    double last_left_, last_right_;
};

// ============================================================================
// PARSER MANAGER (owns all parser threads + shared memory)
// ============================================================================

class Parsers {
public:
    Parsers(ThreadSafeQueue<std::string>& scan_q,
            ThreadSafeQueue<std::string>& joints_q,
            ThreadSafeQueue<std::string>& bumper_q)
        : scan_parser_("ScanParser", scan_q, parse_scan)
        , joints_parser_("JointsParser", joints_q, parse_joint_states)
        , bumper_parser_("BumperParser", bumper_q, parse_bumper)
        , odom_shm_(shm_names::ODOM, true)
        , odom_seq_(0)
        , last_stamp_sec_(0)
        , last_stamp_nanosec_(0) {

        // Joint states parser computes wheel odometry and writes to shared memory
        joints_parser_.set_callback([this](const ParsedJointStates& js) {
            // Compute dt
            double dt = 0;
            if (last_stamp_sec_ > 0) {
                double t1 = last_stamp_sec_ + last_stamp_nanosec_ * 1e-9;
                double t2 = js.stamp_sec + js.stamp_nanosec * 1e-9;
                dt = t2 - t1;
            }
            last_stamp_sec_ = js.stamp_sec;
            last_stamp_nanosec_ = js.stamp_nanosec;

            // Update wheel odometry
            double vx, wz;
            wheel_odom_.update(js.left_wheel_position, js.right_wheel_position, dt, vx, wz);

            // Write to shared memory
            odom_shm_.update([this, &js, vx, wz](SharedOdometry& shm) {
                shm.position_x = wheel_odom_.x();
                shm.position_y = wheel_odom_.y();
                shm.position_z = 0;

                // Quaternion from yaw angle
                double half_theta = wheel_odom_.theta() / 2.0;
                shm.orientation_x = 0;
                shm.orientation_y = 0;
                shm.orientation_z = std::sin(half_theta);
                shm.orientation_w = std::cos(half_theta);

                shm.linear_velocity_x = vx;
                shm.linear_velocity_y = 0;
                shm.linear_velocity_z = 0;
                shm.angular_velocity_x = 0;
                shm.angular_velocity_y = 0;
                shm.angular_velocity_z = wz;

                shm.timestamp_sec = js.stamp_sec;
                shm.timestamp_nanosec = js.stamp_nanosec;
                shm.sequence = ++odom_seq_;
            });

            // Call user callback with converted odom
            if (odom_callback_) {
                ParsedOdom o;
                o.position_x = wheel_odom_.x();
                o.position_y = wheel_odom_.y();
                o.orientation_z = std::sin(wheel_odom_.theta() / 2.0);
                o.orientation_w = std::cos(wheel_odom_.theta() / 2.0);
                o.linear_vel_x = vx;
                o.angular_vel_z = wz;
                o.stamp_sec = js.stamp_sec;
                o.stamp_nanosec = js.stamp_nanosec;
                o.valid = true;
                odom_callback_(o);
            }
        });
    }

    void start() {
        scan_parser_.start();
        joints_parser_.start();
        bumper_parser_.start();
    }

    void stop() {
        scan_parser_.stop();
        joints_parser_.stop();
        bumper_parser_.stop();
    }

    // Callbacks
    void on_scan(std::function<void(const ParsedScan&)> cb) { scan_parser_.set_callback(cb); }
    void on_odom(std::function<void(const ParsedOdom&)> cb) { odom_callback_ = cb; }
    void on_bumper(std::function<void(const ParsedBumper&)> cb) { bumper_parser_.set_callback(cb); }

    // Stats
    uint64_t scan_count() const { return scan_parser_.count(); }
    uint64_t odom_count() const { return joints_parser_.count(); }  // Now from joint_states
    uint64_t bumper_count() const { return bumper_parser_.count(); }

    // Shared memory access
    SharedMemory<SharedOdometry>& odom_shm() { return odom_shm_; }

private:
    ParserThread<ParsedScan, decltype(&parse_scan)> scan_parser_;
    ParserThread<ParsedJointStates, decltype(&parse_joint_states)> joints_parser_;
    ParserThread<ParsedBumper, decltype(&parse_bumper)> bumper_parser_;

    SharedMemory<SharedOdometry> odom_shm_;
    uint64_t odom_seq_;
    int32_t last_stamp_sec_;
    uint32_t last_stamp_nanosec_;
    WheelOdometry wheel_odom_;
    std::function<void(const ParsedOdom&)> odom_callback_;
};

} // namespace turtlebot4

#endif // PARSERS_HPP
