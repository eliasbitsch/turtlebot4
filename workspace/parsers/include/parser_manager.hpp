#ifndef PARSER_MANAGER_HPP
#define PARSER_MANAGER_HPP

#include "odom_parser.hpp"
#include "scan_parser.hpp"
#include "bumper_parser.hpp"
#include "shared_memory.hpp"
#include "datatypes.hpp"
#include "thread_safe_queue.hpp"

#include <memory>
#include <functional>

namespace turtlebot4 {

class ParserManager {
public:
    ParserManager();
    ~ParserManager();

    // Initialize with queues from MessageRouter
    void init(ThreadSafeQueue<std::string>& scan_queue,
              ThreadSafeQueue<std::string>& odom_queue,
              ThreadSafeQueue<std::string>& bumper_queue);

    // Start all parser threads
    void start();

    // Stop all parser threads
    void stop();

    // Get shared memory for odom (other processes can read)
    SharedMemory<SharedOdometry>& odom_shm() { return *odom_shm_; }

    // Set callbacks for parsed data
    void set_scan_callback(std::function<void(const ParsedScan&)> cb);
    void set_odom_callback(std::function<void(const ParsedOdom&)> cb);
    void set_bumper_callback(std::function<void(const ParsedBumper&)> cb);

    // Statistics
    uint64_t scan_count() const { return scan_parser_ ? scan_parser_->messages_processed() : 0; }
    uint64_t odom_count() const { return odom_parser_ ? odom_parser_->messages_processed() : 0; }
    uint64_t bumper_count() const { return bumper_parser_ ? bumper_parser_->messages_processed() : 0; }

private:
    std::unique_ptr<SharedMemory<SharedOdometry>> odom_shm_;

    std::unique_ptr<ScanParser> scan_parser_;
    std::unique_ptr<OdomParser> odom_parser_;
    std::unique_ptr<BumperParser> bumper_parser_;

    bool initialized_;
};

} // namespace turtlebot4

#endif // PARSER_MANAGER_HPP
