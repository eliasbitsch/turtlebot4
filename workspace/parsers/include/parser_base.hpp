#ifndef PARSER_BASE_HPP
#define PARSER_BASE_HPP

#include "thread_safe_queue.hpp"
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <iostream>

namespace turtlebot4 {

template<typename ParsedType>
class ParserBase {
public:
    using ParsedCallback = std::function<void(const ParsedType&)>;

    ParserBase(const std::string& name, ThreadSafeQueue<std::string>& input_queue)
        : name_(name), input_queue_(input_queue), running_(false) {}

    virtual ~ParserBase() {
        stop();
    }

    void start() {
        if (running_) return;
        running_ = true;
        thread_ = std::thread(&ParserBase::thread_func, this);
        std::cout << "[" << name_ << "] Parser started" << std::endl;
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        std::cout << "[" << name_ << "] Parser stopped" << std::endl;
    }

    bool is_running() const { return running_; }

    void set_callback(ParsedCallback cb) { callback_ = cb; }

    uint64_t messages_processed() const { return messages_processed_; }

protected:
    virtual ParsedType parse(const std::string& json_str) = 0;

    virtual void on_parsed(const ParsedType& data) {
        if (callback_) {
            callback_(data);
        }
    }

    std::string name_;
    ThreadSafeQueue<std::string>& input_queue_;
    std::atomic<bool> running_;
    std::thread thread_;
    ParsedCallback callback_;
    std::atomic<uint64_t> messages_processed_{0};

private:
    void thread_func() {
        while (running_) {
            auto msg_opt = input_queue_.pop_timeout(std::chrono::milliseconds(100));
            if (!msg_opt) continue;

            try {
                ParsedType parsed = parse(*msg_opt);
                if (parsed.valid) {
                    on_parsed(parsed);
                    messages_processed_++;
                }
            } catch (const std::exception& e) {
                std::cerr << "[" << name_ << "] Parse error: " << e.what() << std::endl;
            }
        }
    }
};

} // namespace turtlebot4

#endif // PARSER_BASE_HPP
