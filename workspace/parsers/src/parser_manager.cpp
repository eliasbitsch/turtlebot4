#include "parser_manager.hpp"
#include <iostream>

namespace turtlebot4 {

ParserManager::ParserManager() : initialized_(false) {
}

ParserManager::~ParserManager() {
    stop();
}

void ParserManager::init(ThreadSafeQueue<std::string>& scan_queue,
                         ThreadSafeQueue<std::string>& odom_queue,
                         ThreadSafeQueue<std::string>& bumper_queue) {
    if (initialized_) return;

    // Create shared memory for odom
    try {
        odom_shm_ = std::make_unique<SharedMemory<SharedOdometry>>(shm_names::ODOM, true);
        std::cout << "[ParserManager] Created shared memory: " << shm_names::ODOM << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ParserManager] Failed to create odom shared memory: " << e.what() << std::endl;
        throw;
    }

    // Create parsers
    scan_parser_ = std::make_unique<ScanParser>(scan_queue);
    odom_parser_ = std::make_unique<OdomParser>(odom_queue);
    bumper_parser_ = std::make_unique<BumperParser>(bumper_queue);

    // Attach shared memory to odom parser
    odom_parser_->attach_shared_memory(odom_shm_.get());

    initialized_ = true;
    std::cout << "[ParserManager] Initialized" << std::endl;
}

void ParserManager::start() {
    if (!initialized_) {
        std::cerr << "[ParserManager] Not initialized!" << std::endl;
        return;
    }

    scan_parser_->start();
    odom_parser_->start();
    bumper_parser_->start();

    std::cout << "[ParserManager] All parsers started" << std::endl;
}

void ParserManager::stop() {
    if (scan_parser_) scan_parser_->stop();
    if (odom_parser_) odom_parser_->stop();
    if (bumper_parser_) bumper_parser_->stop();

    std::cout << "[ParserManager] All parsers stopped" << std::endl;
}

void ParserManager::set_scan_callback(std::function<void(const ParsedScan&)> cb) {
    if (scan_parser_) scan_parser_->set_callback(cb);
}

void ParserManager::set_odom_callback(std::function<void(const ParsedOdom&)> cb) {
    if (odom_parser_) odom_parser_->set_callback(cb);
}

void ParserManager::set_bumper_callback(std::function<void(const ParsedBumper&)> cb) {
    if (bumper_parser_) bumper_parser_->set_callback(cb);
}

} // namespace turtlebot4
