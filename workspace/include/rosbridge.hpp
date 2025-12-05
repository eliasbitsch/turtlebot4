#ifndef ROSBRIDGE_HPP
#define ROSBRIDGE_HPP

#include "datatypes.hpp"
#include "thread_safe_queue.hpp"
#include "shared_memory.hpp"
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <functional>
#include <iostream>

namespace turtlebot4 {

using json = nlohmann::json;
using WsClient = websocketpp::client<websocketpp::config::asio_client>;

// ============================================================================
// ROSBRIDGE CLIENT (WebSocket connection to rosbridge_server)
// ============================================================================

class RosbridgeClient {
public:
    using Callback = std::function<void(const std::string&, const json&)>;

    explicit RosbridgeClient(const std::string& uri = "ws://localhost:9090")
        : uri_(uri), connected_(false), running_(false) {
        // Enable error logging for debugging
        client_.clear_access_channels(websocketpp::log::alevel::all);
        client_.set_error_channels(websocketpp::log::elevel::all);
        client_.init_asio();

        client_.set_open_handler([this](auto hdl) {
            conn_ = hdl;
            connected_ = true;
            std::cout << "[Rosbridge] Connected to " << uri_ << "\n";
            // Re-subscribe all with QoS
            std::lock_guard<std::mutex> lock(mtx_);
            for (const auto& [topic, info] : subscriptions_) {
                json msg = {{"op", "subscribe"}, {"topic", topic}};
                if (info.best_effort) {
                    msg["qos"] = {{"reliability", "best_effort"}, {"durability", "volatile"}};
                }
                send(msg);
            }
            // Advertise cmd_vel directly (workaround for map iteration issue)
            json adv_msg = {{"op", "advertise"}, {"topic", "/cmd_vel"}, {"type", "geometry_msgs/msg/Twist"}};
            send(adv_msg);
        });

        client_.set_close_handler([this](auto) {
            connected_ = false;
            std::cout << "[Rosbridge] Disconnected\n";
        });

        client_.set_fail_handler([this](auto hdl) {
            connected_ = false;
            auto con = client_.get_con_from_hdl(hdl);
            std::cerr << "[Rosbridge] Connection failed: " << con->get_ec().message() << "\n";
        });

        client_.set_message_handler([this](auto, auto msg) {
            try {
                json j = json::parse(msg->get_payload());
                if (j.value("op", "") == "publish" && j.contains("topic")) {
                    std::string topic = j["topic"];
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (auto it = subscriptions_.find(topic); it != subscriptions_.end() && it->second.callback) {
                        it->second.callback(topic, j.value("msg", json{}));
                    }
                }
            } catch (...) {}
        });
    }

    ~RosbridgeClient() { stop(); }

    bool connect() {
        websocketpp::lib::error_code ec;
        auto con = client_.get_connection(uri_, ec);
        if (ec) {
            std::cerr << "[Rosbridge] Get connection failed: " << ec.message() << "\n";
            return false;
        }
        client_.connect(con);
        return true;
    }

    void run() {
        running_ = true;
        client_.run();
        running_ = false;
    }

    void stop() {
        if (running_) {
            if (connected_) {
                websocketpp::lib::error_code ec;
                client_.close(conn_, websocketpp::close::status::normal, "", ec);
            }
            client_.stop();
        }
    }

    void subscribe(const std::string& topic, Callback cb, bool best_effort = false) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            subscriptions_[topic] = {cb, best_effort};
        }
        if (connected_) {
            json msg = {{"op", "subscribe"}, {"topic", topic}};
            if (best_effort) {
                msg["qos"] = {{"reliability", "best_effort"}, {"durability", "volatile"}};
            }
            send(msg);
        }
    }

    void advertise(const std::string& topic, const std::string& type, bool best_effort = false) {
        std::cout << "[Rosbridge] advertise() called: " << topic << " connected=" << connected_ << "\n";
        {
            std::lock_guard<std::mutex> lock(mtx_);
            publishers_[topic] = {type, best_effort};
            std::cout << "[Rosbridge] publishers_ size=" << publishers_.size() << "\n";
        }
        if (connected_) {
            json msg = {{"op", "advertise"}, {"topic", topic}, {"type", type}};
            if (best_effort) {
                msg["qos"] = {{"reliability", "best_effort"}, {"durability", "volatile"}};
            }
            send(msg);
        }
    }

    void publish(const std::string& topic, const json& msg) {
        if (connected_) send({{"op", "publish"}, {"topic", topic}, {"msg", msg}});
    }

    bool is_connected() const { return connected_; }

private:
    void send(const json& j) {
        std::lock_guard<std::mutex> lock(send_mtx_);
        websocketpp::lib::error_code ec;
        client_.send(conn_, j.dump(), websocketpp::frame::opcode::text, ec);
    }

    struct SubInfo {
        Callback callback;
        bool best_effort = false;
    };

    struct PubInfo {
        std::string type;
        bool best_effort = false;
    };

    std::string uri_;
    WsClient client_;
    websocketpp::connection_hdl conn_;
    std::atomic<bool> connected_, running_;
    std::mutex mtx_, send_mtx_;
    std::map<std::string, SubInfo> subscriptions_;
    std::map<std::string, PubInfo> publishers_;
};

// ============================================================================
// ROBOT BRIDGE (combines WebSocket + Queues + Shared Memory)
// ============================================================================

class RobotBridge {
public:
    RobotBridge(const std::string& uri = "ws://localhost:9090")
        : client_(uri), cmd_vel_shm_(shm_names::CMD_VEL, true), running_(false), last_seq_(0) {}

    ~RobotBridge() { stop(); }

    bool start() {
        // Subscribe to topics → push JSON to queues
        client_.subscribe(topics::SCAN, [this](auto&, const json& m) {
            scan_queue_.push(m.dump());
        }, true);  // best_effort QoS for scan (matches LIDAR default)
        client_.subscribe(topics::ODOM, [this](auto&, const json& m) {
            odom_queue_.push(m.dump());
        }, false);  // Subscribe to /odom directly for robot pose
        client_.subscribe(topics::JOINT_STATES, [this](auto&, const json& m) {
            joints_queue_.push(m.dump());
        }, false);  // reliable QoS for joint_states
        client_.subscribe(topics::HAZARD_DETECTION, [this](auto&, const json& m) {
            bumper_queue_.push(m.dump());
        }, true);  // best_effort for hazard detection

        running_ = true;

        // Register cmd_vel publisher BEFORE starting WebSocket thread
        client_.advertise(topics::CMD_VEL, "geometry_msgs/msg/Twist", true);  // best_effort QoS

        // WebSocket thread
        ws_thread_ = std::thread([this] {
            while (running_) {
                if (!client_.connect()) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                client_.run();
                if (running_) std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });

        // Cmd_vel publisher thread
        cmd_thread_ = std::thread([this] {
            bool first_debug = true;
            while (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                if (!client_.is_connected()) continue;

                // Zero-Copy read: only copy if we need to publish
                bool should_publish = false;
                SharedCmdVel cmd;

                cmd_vel_shm_.read([&](const SharedCmdVel& c) {
                    if (first_debug && c.sequence > 0) {
                        std::cout << "[DEBUG] First read: new_cmd=" << c.new_command << " seq=" << c.sequence 
                                  << " last=" << last_seq_ << " lin=" << c.linear_x << "\n";
                        first_debug = false;
                    }
                    if (c.new_command && c.sequence != last_seq_) {
                        cmd = c;  // Copy only when needed
                        should_publish = true;
                    }
                });

                if (should_publish) {
                    last_seq_ = cmd.sequence;
                    std::cout << "[CMD_VEL] Publishing: lin=" << cmd.linear_x << " ang=" << cmd.angular_z << " seq=" << cmd.sequence << "\n";
                    client_.publish(topics::CMD_VEL, {
                        {"linear", {{"x", cmd.linear_x}, {"y", cmd.linear_y}, {"z", cmd.linear_z}}},
                        {"angular", {{"x", cmd.angular_x}, {"y", cmd.angular_y}, {"z", cmd.angular_z}}}
                    });
                    cmd_vel_shm_.update([](SharedCmdVel& c) { c.new_command = false; });
                }
            }
        });

        std::cout << "[RobotBridge] Started\n";
        return true;
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        scan_queue_.shutdown();
        odom_queue_.shutdown();
        joints_queue_.shutdown();
        bumper_queue_.shutdown();
        client_.stop();
        if (ws_thread_.joinable()) ws_thread_.join();
        if (cmd_thread_.joinable()) cmd_thread_.join();
        std::cout << "[RobotBridge] Stopped\n";
    }

    bool is_running() const { return running_ && client_.is_connected(); }

    // Queues for parsers
    ThreadSafeQueue<std::string>& scan_queue() { return scan_queue_; }
    ThreadSafeQueue<std::string>& odom_queue() { return odom_queue_; }
    ThreadSafeQueue<std::string>& joints_queue() { return joints_queue_; }
    ThreadSafeQueue<std::string>& bumper_queue() { return bumper_queue_; }

    // Shared memory for cmd_vel
    SharedMemory<SharedCmdVel>& cmd_vel_shm() { return cmd_vel_shm_; }

private:
    RosbridgeClient client_;
    SharedMemory<SharedCmdVel> cmd_vel_shm_;
    std::atomic<bool> running_;
    uint64_t last_seq_;

    ThreadSafeQueue<std::string> scan_queue_;
    ThreadSafeQueue<std::string> odom_queue_;
    ThreadSafeQueue<std::string> joints_queue_;
    ThreadSafeQueue<std::string> bumper_queue_;

    std::thread ws_thread_;
    std::thread cmd_thread_;
};

} // namespace turtlebot4

#endif // ROSBRIDGE_HPP
