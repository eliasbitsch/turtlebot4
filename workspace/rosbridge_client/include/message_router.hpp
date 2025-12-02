#ifndef MESSAGE_ROUTER_HPP
#define MESSAGE_ROUTER_HPP

#include "rosbridge_client.hpp"
#include "thread_safe_queue.hpp"
#include "datatypes.hpp"
#include "shared_memory.hpp"

#include <thread>
#include <atomic>
#include <functional>
#include <memory>

namespace turtlebot4 {

// Message with topic information for routing
struct TopicMessage {
    std::string topic;
    std::string payload;  // Raw JSON string
};

class MessageRouter {
public:
    MessageRouter();
    ~MessageRouter();

    // Start the router (connects to rosbridge, starts all threads)
    bool start();

    // Stop the router
    void stop();

    // Check if running
    bool is_running() const { return running_; }

    // Get queues for parsers to consume
    ThreadSafeQueue<std::string>& scan_queue() { return scan_queue_; }
    ThreadSafeQueue<std::string>& odom_queue() { return odom_queue_; }
    ThreadSafeQueue<std::string>& bumper_queue() { return bumper_queue_; }

    // Get shared memory for cmd_vel (controller writes, router reads and publishes)
    SharedMemory<SharedCmdVel>& cmd_vel_shm() { return *cmd_vel_shm_; }

    // Callbacks for parsed data (optional, for debugging/logging)
    using ScanCallback = std::function<void(const ParsedScan&)>;
    using OdomCallback = std::function<void(const ParsedOdom&)>;
    using BumperCallback = std::function<void(const ParsedBumper&)>;

    void set_scan_callback(ScanCallback cb) { scan_callback_ = cb; }
    void set_odom_callback(OdomCallback cb) { odom_callback_ = cb; }
    void set_bumper_callback(BumperCallback cb) { bumper_callback_ = cb; }

private:
    void rosbridge_thread_func();
    void cmd_vel_publisher_thread_func();

    void on_scan_message(const std::string& topic, const json& msg);
    void on_odom_message(const std::string& topic, const json& msg);
    void on_bumper_message(const std::string& topic, const json& msg);

    std::unique_ptr<RosbridgeClient> client_;
    std::atomic<bool> running_;

    // Queues for parser threads
    ThreadSafeQueue<std::string> scan_queue_;
    ThreadSafeQueue<std::string> odom_queue_;
    ThreadSafeQueue<std::string> bumper_queue_;

    // Shared memory
    std::unique_ptr<SharedMemory<SharedCmdVel>> cmd_vel_shm_;

    // Threads
    std::thread rosbridge_thread_;
    std::thread cmd_vel_thread_;

    // Optional callbacks
    ScanCallback scan_callback_;
    OdomCallback odom_callback_;
    BumperCallback bumper_callback_;

    // For cmd_vel publishing rate limiting
    uint64_t last_cmd_vel_seq_;
};

} // namespace turtlebot4

#endif // MESSAGE_ROUTER_HPP
